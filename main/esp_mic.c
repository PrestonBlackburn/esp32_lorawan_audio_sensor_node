
// ICS-43432 I2S Audio Logger — ESP-IDF v5+
//   ICS-43432  →  ESP32-S3
//   BCLK       →  GPIO_BCLK  (bit clock)
//   LRCL (WS)  →  GPIO_WS    (word select / LR clock)
//   DOUT       →  GPIO_DIN   (data into the ESP32)
//   SEL        →  GND        (selects left channel)

// The ICS-43432 outputs 24-bit audio packed into a 32-bit I2S frame
// (MSB-justified, Philips format). We read 32-bit words and right-shift
// by 8 to recover the signed 24-bit sample.

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include <math.h>
#include "freertos/ringbuf.h"
#include "esp_dsp.h"

// #define GPIO_BCLK   GPIO_NUM_7
// #define GPIO_WS     GPIO_NUM_8   /* LRCL on the breakout */
// #define GPIO_DIN    GPIO_NUM_9   /* DOUT on the breakout → DIN on the S3 */

#define GPIO_BCLK   GPIO_NUM_40
#define GPIO_WS     GPIO_NUM_38   /* LRCL on the breakout */
#define GPIO_DIN    GPIO_NUM_39   /* DOUT on the breakout → DIN on the S3 */
 
 
static const char *TAG = "ics43432";

/* ── Audio / DSP parameters ──────────────────────────────────────── */
#define SAMPLE_RATE      16000
#define FFT_SIZE         1024          /* must be power-of-2              */
#define HOP_SIZE         (FFT_SIZE/2)  /* 50 % overlap, matches Python    */
#define I2S_READ_FRAMES  256           /* frames per I2S read call        */
 
/* ── Frequency bands (mirrors BANDS in the Python script) ─────────── */
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

static float s_band_sum_sq[NUM_BANDS];   /* accumulated power per band  */
static int   s_frame_count = 0;          /* FFT frames since last report */

/* ── Ring buffer between the two tasks ───────────────────────────── */
/*    Holds float32 samples.  Size = 4 × FFT_SIZE gives plenty of
 *    headroom (≈ 0.25 s at 16 kHz).                                  */
#define RING_BUF_BYTES  (4 * FFT_SIZE * sizeof(float))
static RingbufHandle_t s_ring_buf;
 
/* ── DSP scratch buffers (static to avoid stack pressure) ─────────── */
/*    fft_buf: interleaved complex [re0,im0,re1,im1,...] for esp-dsp   */
static float s_window[FFT_SIZE];
static float s_fft_buf[FFT_SIZE * 2];
static float s_overlap_buf[FFT_SIZE]; /* sliding window of samples     */
static int   s_overlap_fill = 0;       /* how many samples are loaded   */
 
/* ─────────────────────────────────────────────────────────────────── */
/*  Helpers                                                             */
/* ─────────────────────────────────────────────────────────────────── */
 
/**
 * Frequency bin index for a given Hz value.
 * bin = round(freq * FFT_SIZE / sample_rate)
 * The one-sided spectrum has FFT_SIZE/2 bins (indices 0 … FFT_SIZE/2-1).
 */
static inline int freq_to_bin(float hz)
{
    int bin = (int)roundf(hz * FFT_SIZE / (float)SAMPLE_RATE);
    if (bin < 0)             bin = 0;
    if (bin >= FFT_SIZE / 2) bin = FFT_SIZE / 2 - 1;
    return bin;
}
 

/**
 * Run a Hann-windowed FFT on the FFT_SIZE samples currently in
 * s_overlap_buf and compute per-band RMS dBFS.
 */


static void run_fft_and_log(void)
{
    /* 1. Apply Hann window and pack into interleaved complex buffer.
     *    imaginary parts are zero (real-valued input).               */
    for (int i = 0; i < FFT_SIZE; i++) {
        s_fft_buf[2 * i]     = s_overlap_buf[i] * s_window[i]; /* Re */
        s_fft_buf[2 * i + 1] = 0.0f;                            /* Im */
    }
 
    /* 2. FFT (in-place, bit-reversal included) */
    dsps_fft2r_fc32(s_fft_buf, FFT_SIZE);
    dsps_bit_rev_fc32(s_fft_buf, FFT_SIZE);
 
    /* 3. One-sided magnitude spectrum.
     *    |X[k]| = sqrt(Re² + Im²)
     *    Normalise by FFT_SIZE so the scale is independent of window size. */
    for (int k = 0; k < FFT_SIZE / 2; k++) {
        float re = s_fft_buf[2 * k];
        float im = s_fft_buf[2 * k + 1];
        s_fft_buf[k] = sqrtf(re * re + im * im) / (float)FFT_SIZE;
    }
    /* s_fft_buf[0 .. FFT_SIZE/2-1] now holds the magnitude spectrum   */
 
    /* 4. Per-band RMS → dBFS */
    for (int b = 0; b < NUM_BANDS; b++) {
        int lo_bin = freq_to_bin(BANDS[b].low_hz);
        int hi_bin = freq_to_bin(BANDS[b].high_hz);
        int n_bins = hi_bin - lo_bin + 1;
 
        float sum_sq = 0.0f;
        for (int k = lo_bin; k <= hi_bin; k++) {
            float mag = s_fft_buf[k];
            sum_sq += mag * mag;
        }
        float rms  = sqrtf(sum_sq / (float)n_bins);
        float dbfs = 20.0f * log10f(rms + 1e-9f);
 
        ESP_LOGI(TAG, "  %s  (%.0f-%.0f Hz, %d bins)  →  %+.1f dBFS",
                 BANDS[b].label, BANDS[b].low_hz, BANDS[b].high_hz,
                 n_bins, dbfs);
    }
}
 

static void run_fft_and_log_aggregate(void)
{
    /* Apply Hann window, pack interleaved complex */
    for (int i = 0; i < FFT_SIZE; i++) {
        s_fft_buf[2 * i]     = s_overlap_buf[i] * s_window[i];
        s_fft_buf[2 * i + 1] = 0.0f;
    }

    dsps_fft2r_fc32(s_fft_buf, FFT_SIZE);
    dsps_bit_rev_fc32(s_fft_buf, FFT_SIZE);

    /* One-sided magnitude spectrum */
    for (int k = 0; k < FFT_SIZE / 2; k++) {
        float re = s_fft_buf[2 * k];
        float im = s_fft_buf[2 * k + 1];
        s_fft_buf[k] = sqrtf(re * re + im * im) / (float)FFT_SIZE;
    }

    /* Accumulate power per band (sum of squared magnitudes across bins) */
    for (int b = 0; b < NUM_BANDS; b++) {
        int lo_bin = freq_to_bin(BANDS[b].low_hz);
        int hi_bin = freq_to_bin(BANDS[b].high_hz);
        int n_bins = hi_bin - lo_bin + 1;

        float sum_sq = 0.0f;
        for (int k = lo_bin; k <= hi_bin; k++) {
            float mag = s_fft_buf[k];
            sum_sq += mag * mag;
        }
        /* Normalise by bin count so band width doesn't bias the level */
        s_band_sum_sq[b] += sum_sq / (float)n_bins;
    }

    s_frame_count++;

    /* Report every FRAMES_PER_REPORT frames (~10 s) */
    if (s_frame_count >= FRAMES_PER_REPORT) {
        ESP_LOGI(TAG, "── 10 s aggregate ──────────────────────");
        for (int b = 0; b < NUM_BANDS; b++) {
            float rms  = sqrtf(s_band_sum_sq[b] / (float)s_frame_count);
            float dbfs = 20.0f * log10f(rms + 1e-9f);
            ESP_LOGI(TAG, "  %s  (%.0f-%.0f Hz)  →  %+.1f dBFS",
                     BANDS[b].label, BANDS[b].low_hz, BANDS[b].high_hz, dbfs);
            s_band_sum_sq[b] = 0.0f;  /* reset for next window */
        }
        s_frame_count = 0;
    }
}
/* ─────────────────────────────────────────────────────────────────── */
/*  Tasks                                                               */
/* ─────────────────────────────────────────────────────────────────── */
 
/**
 * i2s_reader_task
 *
 * Reads I2S frames, converts to float32 [-1, 1], and pushes them onto
 * the ring buffer for the DSP task to consume.
 */
static void i2s_reader_task(void *arg)
{
    i2s_chan_handle_t rx_chan = (i2s_chan_handle_t)arg;
 
    static int32_t  raw_buf[I2S_READ_FRAMES];
    static float    f32_buf[I2S_READ_FRAMES];
    size_t bytes_read = 0;
 
    while (1) {
        esp_err_t err = i2s_channel_read(rx_chan, raw_buf,
                                          sizeof(raw_buf),
                                          &bytes_read,
                                          pdMS_TO_TICKS(1000));
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "I2S read error: %s", esp_err_to_name(err));
            continue;
        }
 
        int n_frames = (int)(bytes_read / sizeof(int32_t));
 
        /* ICS-43432: 24-bit audio in top 3 bytes of each 32-bit word.
         * Shift right by 8, then normalise to [-1, 1] using the
         * 24-bit full-scale value (8 388 608 = 2^23).               */
        for (int i = 0; i < n_frames; i++) {
            int32_t s24 = raw_buf[i] >> 8;   /* sign-extended 24-bit */
            f32_buf[i]  = (float)s24 / 8388608.0f;
        }
 
        /* Write to ring buffer.  Use 0-tick timeout so we never block
         * the I2S reader; if the DSP task falls behind, samples are
         * dropped rather than stalling capture.                       */
        size_t bytes_to_send = n_frames * sizeof(float);
        if (xRingbufferSend(s_ring_buf, f32_buf, bytes_to_send, 0) != pdTRUE) {
            ESP_LOGW(TAG, "Ring buffer full — dropped %d samples", n_frames);
        }
    }
}


/**
 * dsp_task
 *
 * Drains the ring buffer into a sliding overlap buffer.
 * Every HOP_SIZE new samples it fires a Hann-windowed FFT and logs
 * the per-band dBFS — exactly mirroring the Python STFT with 50 % overlap.
 */
static void dsp_task(void *arg)
{
    /* Initialise Hann window coefficients once */
    dsps_wind_hann_f32(s_window, FFT_SIZE);
 
    /* Pre-initialise the FFT tables (required by esp-dsp) */
    ESP_ERROR_CHECK(dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE));
 
    /* How many new samples we need before the next FFT frame */
    int samples_since_last_fft = 0;
 
    while (1) {
        size_t  bytes_received = 0;
        float  *chunk = (float *)xRingbufferReceiveUpTo(
            s_ring_buf,
            &bytes_received,
            pdMS_TO_TICKS(100),   /* block up to 100 ms waiting for data */
            HOP_SIZE * sizeof(float)
        );
 
        if (chunk == NULL) continue;  /* timeout — nothing available yet */
 
        int n_new = (int)(bytes_received / sizeof(float));
 
        /* Feed new samples into the overlap buffer.
         * When we have FFT_SIZE samples loaded we run the FFT,
         * then slide the window forward by HOP_SIZE (discard the
         * oldest HOP_SIZE samples and keep the newest HOP_SIZE).    */
        int consumed = 0;
        while (consumed < n_new) {
            /* How many samples to copy into the current frame */
            int space   = FFT_SIZE - s_overlap_fill;
            int to_copy = n_new - consumed;
            if (to_copy > space) to_copy = space;
 
            memcpy(&s_overlap_buf[s_overlap_fill],
                   &chunk[consumed],
                   to_copy * sizeof(float));
 
            s_overlap_fill     += to_copy;
            consumed           += to_copy;
            samples_since_last_fft += to_copy;
 
            if (s_overlap_fill == FFT_SIZE) {
                /* Full frame ready → run FFT */
                run_fft_and_log_aggregate();
 
                /* Slide window: keep the last HOP_SIZE samples */
                memmove(s_overlap_buf,
                        &s_overlap_buf[HOP_SIZE],
                        HOP_SIZE * sizeof(float));
                s_overlap_fill     = HOP_SIZE;
                samples_since_last_fft = 0;
            }
        }
 
        vRingbufferReturnItem(s_ring_buf, chunk);
    }
}


/* ─────────────────────────────────────────────────────────────────── */
/*  app_main                                                            */
/* ─────────────────────────────────────────────────────────────────── */
 
void app_main(void)
{
    /* ── 1. I2S channel ─────────────────────────────────────────── */
    i2s_chan_handle_t rx_chan;
    i2s_chan_config_t chan_cfg =
        I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_chan));
 
    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_32BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
            .slot_mode      = I2S_SLOT_MODE_MONO,
            .slot_mask      = I2S_STD_SLOT_LEFT,  /* SEL tied to GND */
            .ws_width       = 32,
            .ws_pol         = false,
            .bit_shift      = true,                /* Philips format  */
        },
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = GPIO_BCLK,
            .ws   = GPIO_WS,
            .dout = I2S_GPIO_UNUSED,
            .din  = GPIO_DIN,
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));
 
    /* ── 2. Ring buffer ─────────────────────────────────────────── */
    s_ring_buf = xRingbufferCreate(RING_BUF_BYTES, RINGBUF_TYPE_BYTEBUF);
    configASSERT(s_ring_buf != NULL);
 
    ESP_LOGI(TAG, "Ring buffer: %u bytes (%.0f ms headroom)",
             (unsigned)RING_BUF_BYTES,
             (double)RING_BUF_BYTES / (sizeof(float) * SAMPLE_RATE) * 1000.0);
 
    /* ── 3. Tasks ───────────────────────────────────────────────── */
    /*  Pin the I2S reader to core 0 (handles DMA callbacks).
     *  Pin the DSP task  to core 1 (floating-point heavy).          */
    xTaskCreatePinnedToCore(i2s_reader_task, "i2s_reader",
                             4096, rx_chan,
                             5,             /* priority */
                             NULL, 0);      /* core 0   */
 
    xTaskCreatePinnedToCore(dsp_task, "dsp",
                             8192, NULL,
                             4,             /* priority */
                             NULL, 1);      /* core 1   */
 
    ESP_LOGI(TAG, "Tasks started — FFT size %d, hop %d, %d bands",
             FFT_SIZE, HOP_SIZE, (int)NUM_BANDS);
}
