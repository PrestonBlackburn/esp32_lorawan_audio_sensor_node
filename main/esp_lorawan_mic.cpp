/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * Merged LoRaWAN + I2S mic (ICS-43432) app for XIAO ESP32-S3 Plus.
 * Three FreeRTOS tasks:
 *   - i2s_reader_task : pulls raw I2S frames, converts to float32, pushes to ring buffer
 *   - dsp_task         : drains ring buffer, runs windowed FFT, computes per-band dBFS,
 *                        pushes the latest 10s aggregate report onto a length-1 queue
 *   - lorawan_task     : joins OTAA, then periodically pulls the latest report (non-blocking)
 *                        and uplinks it
 *
 * Data handoff between dsp_task and lorawan_task uses xQueueOverwrite on a
 * length-1 queue, so the radio task always sees the most recent report and
 * never blocks the DSP task.
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"

#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_dsp.h"

#include <RadioLib.h>
#include "hal/ESP32S3Hal/Esp32S3Hal.hpp"
// #include "config.h"
#include "config_b.h"

/* ─────────────────────────────────────────────────────────────────── */
/*  Pin definitions                                                     */
/* ─────────────────────────────────────────────────────────────────── */

/* --- SX1262 hat (bottom pinout, per board) --- */
#define RADIO_NSS   (5)
#define RADIO_IRQ   (2)
#define RADIO_RST   (3)
#define RADIO_GPIO  (4)   // Busy
#define RADIO_SCK   (7)
#define RADIO_MISO  (8)
#define RADIO_MOSI  (9)
/* TODO: confirm the real RF-switch GPIO against the hat schematic —
 * see note above about setRfSwitchPins(38, ...) in the original code. */
#define RADIO_RF_SWITCH_PIN (38)

/* --- ICS-43432 mic (extra castellated pins, unused by the hat) --- */
#define GPIO_BCLK   GPIO_NUM_40
#define GPIO_WS     GPIO_NUM_41   /* LRCL on the breakout */
#define GPIO_DIN    GPIO_NUM_39   /* DOUT on the breakout -> DIN on the S3 */

static const char *TAG = "app";

/* ─────────────────────────────────────────────────────────────────── */
/*  Audio / DSP parameters                                              */
/* ─────────────────────────────────────────────────────────────────── */
#define SAMPLE_RATE      16000
#define FFT_SIZE         1024
#define HOP_SIZE         (FFT_SIZE / 2)
#define I2S_READ_FRAMES  256

typedef struct {
    const char *label;
    float       low_hz;
    float       high_hz;
} Band;

static const Band BANDS[] = {
    { "110 Hz", 60,   160  },
    { "440 Hz", 390,  490  },
    { "1 kHz",  950,  1050 },
    { "4 kHz",  3950, 4050 },
};
#define NUM_BANDS (sizeof(BANDS) / sizeof(BANDS[0]))

#define REPORT_INTERVAL_S   10
#define FRAMES_PER_REPORT   (REPORT_INTERVAL_S * SAMPLE_RATE / HOP_SIZE)  /* ~312 */

#define RING_BUF_BYTES  (4 * FFT_SIZE * sizeof(float))
static RingbufHandle_t s_ring_buf;

static float s_window[FFT_SIZE];
static float s_fft_buf[FFT_SIZE * 2];
static float s_overlap_buf[FFT_SIZE];
static int   s_overlap_fill = 0;

static float s_band_sum_sq[NUM_BANDS];
static int   s_frame_count = 0;

/* ─────────────────────────────────────────────────────────────────── */
/*  Shared data: DSP task -> LoRaWAN task                               */
/* ─────────────────────────────────────────────────────────────────── */

typedef struct {
    float    dbfs[NUM_BANDS];
    uint32_t seq;          /* increments each new report, lets consumer detect staleness */
} BandReport;

/* Length-1 queue: xQueueOverwrite always keeps only the newest report.
 * The DSP task never blocks writing; the radio task never blocks reading
 * (uses a short timeout / non-blocking receive). */
static QueueHandle_t s_report_queue;
static uint32_t      s_report_seq = 0;

/* ─────────────────────────────────────────────────────────────────── */
/*  DSP helpers                                                         */
/* ─────────────────────────────────────────────────────────────────── */

static inline int freq_to_bin(float hz)
{
    int bin = (int)roundf(hz * FFT_SIZE / (float)SAMPLE_RATE);
    if (bin < 0)             bin = 0;
    if (bin >= FFT_SIZE / 2) bin = FFT_SIZE / 2 - 1;
    return bin;
}

static void run_fft_and_report(void)
{
    for (int i = 0; i < FFT_SIZE; i++) {
        s_fft_buf[2 * i]     = s_overlap_buf[i] * s_window[i];
        s_fft_buf[2 * i + 1] = 0.0f;
    }

    dsps_fft2r_fc32(s_fft_buf, FFT_SIZE);
    dsps_bit_rev_fc32(s_fft_buf, FFT_SIZE);

    for (int k = 0; k < FFT_SIZE / 2; k++) {
        float re = s_fft_buf[2 * k];
        float im = s_fft_buf[2 * k + 1];
        s_fft_buf[k] = sqrtf(re * re + im * im) / (float)FFT_SIZE;
    }

    for (int b = 0; b < (int)NUM_BANDS; b++) {
        int lo_bin = freq_to_bin(BANDS[b].low_hz);
        int hi_bin = freq_to_bin(BANDS[b].high_hz);
        int n_bins = hi_bin - lo_bin + 1;

        float sum_sq = 0.0f;
        for (int k = lo_bin; k <= hi_bin; k++) {
            float mag = s_fft_buf[k];
            sum_sq += mag * mag;
        }
        s_band_sum_sq[b] += sum_sq / (float)n_bins;
    }

    s_frame_count++;

    if (s_frame_count >= FRAMES_PER_REPORT) {
        BandReport report;
        report.seq = ++s_report_seq;

        ESP_LOGI(TAG, "-- 10 s aggregate --------------------");
        for (int b = 0; b < (int)NUM_BANDS; b++) {
            float rms  = sqrtf(s_band_sum_sq[b] / (float)s_frame_count);
            float dbfs = 20.0f * log10f(rms + 1e-9f);
            report.dbfs[b] = dbfs;

            ESP_LOGI(TAG, "  %s  (%.0f-%.0f Hz)  ->  %+.1f dBFS",
                     BANDS[b].label, BANDS[b].low_hz, BANDS[b].high_hz, dbfs);
            s_band_sum_sq[b] = 0.0f;
        }
        s_frame_count = 0;

        /* Overwrite is safe even if the radio task hasn't consumed the
         * previous report yet -- we only ever care about the latest. */
        xQueueOverwrite(s_report_queue, &report);
    }
}

/* ─────────────────────────────────────────────────────────────────── */
/*  Task: I2S reader (core 0)                                           */
/* ─────────────────────────────────────────────────────────────────── */

static void i2s_reader_task(void *arg)
{
    i2s_chan_handle_t rx_chan = (i2s_chan_handle_t)arg;

    static int32_t raw_buf[I2S_READ_FRAMES];
    static float   f32_buf[I2S_READ_FRAMES];
    size_t bytes_read = 0;

    while (1) {
        esp_err_t err = i2s_channel_read(rx_chan, raw_buf, sizeof(raw_buf),
                                          &bytes_read, pdMS_TO_TICKS(1000));
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "I2S read error: %s", esp_err_to_name(err));
            continue;
        }

        int n_frames = (int)(bytes_read / sizeof(int32_t));

        for (int i = 0; i < n_frames; i++) {
            int32_t s24 = raw_buf[i] >> 8;
            f32_buf[i]  = (float)s24 / 8388608.0f;
        }

        size_t bytes_to_send = n_frames * sizeof(float);
        if (xRingbufferSend(s_ring_buf, f32_buf, bytes_to_send, 0) != pdTRUE) {
            ESP_LOGW(TAG, "Ring buffer full - dropped %d samples", n_frames);
        }
    }
}

/* ─────────────────────────────────────────────────────────────────── */
/*  Task: DSP (core 1)                                                  */
/* ─────────────────────────────────────────────────────────────────── */

static void dsp_task(void *arg)
{
    dsps_wind_hann_f32(s_window, FFT_SIZE);
    ESP_ERROR_CHECK(dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE));

    while (1) {
        size_t bytes_received = 0;
        float *chunk = (float *)xRingbufferReceiveUpTo(
            s_ring_buf, &bytes_received, pdMS_TO_TICKS(100),
            HOP_SIZE * sizeof(float));

        if (chunk == NULL) continue;

        int n_new = (int)(bytes_received / sizeof(float));
        int consumed = 0;

        while (consumed < n_new) {
            int space   = FFT_SIZE - s_overlap_fill;
            int to_copy = n_new - consumed;
            if (to_copy > space) to_copy = space;

            memcpy(&s_overlap_buf[s_overlap_fill], &chunk[consumed],
                   to_copy * sizeof(float));

            s_overlap_fill += to_copy;
            consumed       += to_copy;

            if (s_overlap_fill == FFT_SIZE) {
                run_fft_and_report();
                memmove(s_overlap_buf, &s_overlap_buf[HOP_SIZE],
                        HOP_SIZE * sizeof(float));
                s_overlap_fill = HOP_SIZE;
            }
        }

        vRingbufferReturnItem(s_ring_buf, chunk);
    }
}

/* ─────────────────────────────────────────────────────────────────── */
/*  Task: LoRaWAN (core 0, higher priority than i2s_reader)             */
/* ─────────────────────────────────────────────────────────────────── */

static void lorawan_task(void *arg)
{
    Esp32S3Hal hal(RADIO_SCK, RADIO_MISO, RADIO_MOSI);
    static Module mod(&hal, RADIO_NSS, RADIO_IRQ, RADIO_RST, RADIO_GPIO);
    static SX1262 radio(&mod);
    static LoRaWANNode node(&radio, &Region, subBand);

    uint64_t joinEUI = RADIOLIB_LORAWAN_JOIN_EUI;
    uint64_t devEUI  = RADIOLIB_LORAWAN_DEV_EUI;
    uint8_t  appKey[] = { RADIOLIB_LORAWAN_APP_KEY };
    uint8_t  nwkKey[] = { RADIOLIB_LORAWAN_NWK_KEY };

    int16_t state = radio.begin();
    if (state == RADIOLIB_ERR_NONE) {
        ESP_LOGI(TAG, "Radio initialized OK");
        radio.setRfSwitchPins(RADIO_RF_SWITCH_PIN, RADIOLIB_NC);
    } else {
        ESP_LOGE(TAG, "Radio init failed: %d", state);
        while (true) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    vTaskDelay(pdMS_TO_TICKS(1500));
    radio.setDio2AsRfSwitch(true);

    state = node.beginOTAA(joinEUI, devEUI, nwkKey, appKey);
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "beginOTAA failed: %d", state);
        while (true) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "Joining LoRaWAN network...");
    int joinAttempts = 0;
    do {
        state = node.activateOTAA();
        if (state != RADIOLIB_LORAWAN_NEW_SESSION) {
            ESP_LOGE(TAG, "Join attempt %d failed: %d", ++joinAttempts, state);
            vTaskDelay(pdMS_TO_TICKS(10000));
        }
    } while (state != RADIOLIB_LORAWAN_NEW_SESSION);

    if (state != RADIOLIB_LORAWAN_NEW_SESSION) {
        // should never hit this now...
        ESP_LOGE(TAG, "Join failed after retries");
        while (true) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "Joined!");

    /* Fix the uplink data rate rather than letting ADR pick one on the
     * fly -- keeps your payload-size budget predictable. */
    node.setDatarate(LORAWAN_UPLINK_DATA_RATE);

    while (true) {
        BandReport report;
        bool have_report = (xQueuePeek(s_report_queue, &report, 0) == pdTRUE);

        /* ── Message format ────────────────────────────────────────
         * byte 0    : version    (uint8)  - bump if you change this layout
         * byte 1    : type       (uint8)  - MSG_TYPE_BAND_REPORT for now,
         *                                   leaves room for future message
         *                                   kinds on the same port
         * byte 2    : length     (uint8)  - total message length in bytes,
         *                                   including this header
         * byte 3-4  : band[0] dBFS x10, int16, little-endian
         * byte 5-6  : band[1] dBFS x10, int16, little-endian
         * byte 7-8  : band[2] dBFS x10, int16, little-endian
         * byte 9-10 : band[3] dBFS x10, int16, little-endian
         *
         * Total: 11 bytes. Values are dBFS * 10 so the decoder just
         * divides by 10 to recover one decimal place of precision,
         * e.g. -234 -> -23.4 dBFS. int16 range (+-32767) means this
         * scaling only overflows around +-3276.7 dBFS, nowhere near
         * anything you'll see in practice.
         *
         * Serialized by hand (not a C struct) so there's no risk of
         * compiler-inserted padding changing the on-wire layout --
         * what you see below is exactly what goes over the air. Bytes
         * are little-endian; make sure your network-server payload
         * decoder (TTN/ChirpStack function) reads them the same way. */
        #define MSG_VERSION           1
        #define MSG_TYPE_BAND_REPORT  1

        uint8_t uplinkPayload[3 + NUM_BANDS * 2];

        static_assert(sizeof(uplinkPayload) <= LORAWAN_UPLINK_DATA_MAX,
                      "uplinkPayload exceeds LORAWAN_UPLINK_DATA_MAX");

        uplinkPayload[0] = MSG_VERSION;
        uplinkPayload[1] = MSG_TYPE_BAND_REPORT;
        uplinkPayload[2] = (uint8_t)sizeof(uplinkPayload);

        for (int b = 0; b < (int)NUM_BANDS; b++) {
            float dbfs = have_report ? report.dbfs[b] : -100.0f;

            /* Scale by 10 and clamp to int16 range before narrowing --
             * do the clamp on a wider type so the cast itself can
             * never overflow/wrap. */
            int32_t scaled = (int32_t)lroundf(dbfs * 10.0f);
            if (scaled < INT16_MIN) scaled = INT16_MIN;
            if (scaled > INT16_MAX) scaled = INT16_MAX;
            int16_t value = (int16_t)scaled;

            int off = 3 + b * 2;
            uplinkPayload[off]     = (uint8_t)(value & 0xFF);         /* low byte  */
            uplinkPayload[off + 1] = (uint8_t)((value >> 8) & 0xFF);  /* high byte */
        }

        ESP_LOGI(TAG, "Sending band report (%d bytes, have_report=%d)",
                 (int)sizeof(uplinkPayload), have_report ? 1 : 0);

        int16_t txState = node.sendReceive(uplinkPayload, sizeof(uplinkPayload),
                                            LORAWAN_UPLINK_USER_PORT);

        if (txState < RADIOLIB_ERR_NONE) {
            ESP_LOGE(TAG, "sendReceive failed: %d", txState);
        } else if (txState > 0) {
            ESP_LOGI(TAG, "Downlink received in Rx%d", txState);
        } else {
            ESP_LOGI(TAG, "No downlink received");
        }

        /* LoRaWAN duty cycle / fair use - keep this at or above your
         * region's minimum for the payload size / data rate you're using. */
        vTaskDelay(pdMS_TO_TICKS(LORAWAN_UPLINK_PERIOD));
    }
}

/* ─────────────────────────────────────────────────────────────────── */
/*  app_main                                                            */
/* ─────────────────────────────────────────────────────────────────── */

extern "C" void app_main(void)
{
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "GPIO ISR service installation failed");
    }

    /* ── I2S channel ────────────────────────────────────────────── */
    i2s_chan_handle_t rx_chan;
    i2s_chan_config_t chan_cfg =
        I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_chan));

    /* Use the IDF-provided default-config macros rather than a bare
     * designated-initializer literal for slot_cfg/gpio_cfg. Newer IDF
     * versions add fields to these structs (e.g. left_align, big_endian,
     * bit_order_lsb, invert_flags) and -Werror=missing-field-initializers
     * will fail the build if a literal doesn't set every member. The
     * macros zero-init everything correctly for your IDF version, then
     * we override just the fields we care about. */
    i2s_std_slot_config_t slot_cfg =
        I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT,
                                             I2S_SLOT_MODE_MONO);
    slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;  /* SEL tied to GND */
    slot_cfg.ws_width  = 32;
    slot_cfg.ws_pol    = false;
    slot_cfg.bit_shift = true;               /* Philips format */

    i2s_std_gpio_config_t gpio_cfg = {};     /* value-init zeros invert_flags etc. */
    gpio_cfg.mclk = I2S_GPIO_UNUSED;
    gpio_cfg.bclk = GPIO_BCLK;
    gpio_cfg.ws   = GPIO_WS;
    gpio_cfg.dout = I2S_GPIO_UNUSED;
    gpio_cfg.din  = GPIO_DIN;

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = slot_cfg,
        .gpio_cfg = gpio_cfg,
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));

    /* ── Ring buffer + report queue ────────────────────────────── */
    s_ring_buf = xRingbufferCreate(RING_BUF_BYTES, RINGBUF_TYPE_BYTEBUF);
    configASSERT(s_ring_buf != NULL);

    s_report_queue = xQueueCreate(1, sizeof(BandReport));
    configASSERT(s_report_queue != NULL);

    ESP_LOGI(TAG, "Ring buffer: %u bytes (%.0f ms headroom)",
             (unsigned)RING_BUF_BYTES,
             (double)RING_BUF_BYTES / (sizeof(float) * SAMPLE_RATE) * 1000.0);

    /* ── Tasks ──────────────────────────────────────────────────
     * Core 0: i2s_reader (mostly blocked on DMA, low CPU) + lorawan
     *         (radio ISR/SPI is latency sensitive, give it higher
     *         priority than i2s_reader so it isn't starved)
     * Core 1: dsp (steady FFT load, ~1 FFT every 32 ms)
     *
        * Verify this placement on your hardware -- if you see dropped
     * I2S samples or missed RX windows, try swapping cores/priorities.
     */
    xTaskCreatePinnedToCore(i2s_reader_task, "i2s_reader",
                             4096, rx_chan, 5, NULL, 0);

    xTaskCreatePinnedToCore(lorawan_task, "lorawan",
                             8192, NULL, 6, NULL, 0);

    xTaskCreatePinnedToCore(dsp_task, "dsp",
                             8192, NULL, 4, NULL, 1);

    ESP_LOGI(TAG, "Tasks started - FFT size %d, hop %d, %d bands",
             FFT_SIZE, HOP_SIZE, (int)NUM_BANDS);
}