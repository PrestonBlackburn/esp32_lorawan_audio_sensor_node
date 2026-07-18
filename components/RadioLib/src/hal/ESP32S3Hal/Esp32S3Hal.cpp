#include "Esp32S3Hal.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_intr_alloc.h"

Esp32S3Hal::Esp32S3Hal(int8_t sck, int8_t miso, int8_t mosi)
    : RadioLibHal(GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, 0, 1, GPIO_INTR_POSEDGE, GPIO_INTR_NEGEDGE),
      spiSCK_(sck), spiMISO_(miso), spiMOSI_(mosi) {
    (void)gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
}


void Esp32S3Hal::init() {
    spiBegin();
}

void Esp32S3Hal::term() {
    spiEnd();
}

// GPIO
void Esp32S3Hal::pinMode(uint32_t pin, uint32_t mode) {
    if (pin == RADIOLIB_NC) return;
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = (1ULL << pin);
    cfg.mode = static_cast<gpio_mode_t>(mode);
    cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&cfg);
}

void Esp32S3Hal::digitalWrite(uint32_t pin, uint32_t value) {
    if (pin == RADIOLIB_NC) return;
    gpio_set_level(static_cast<gpio_num_t>(pin), value ? 1 : 0);
}

uint32_t Esp32S3Hal::digitalRead(uint32_t pin) {
    if (pin == RADIOLIB_NC) return 0;
    return static_cast<uint32_t>(gpio_get_level(static_cast<gpio_num_t>(pin)));
}

void Esp32S3Hal::attachInterrupt(uint32_t pin, void (*cb)(void), uint32_t mode) {
    if (pin == RADIOLIB_NC) return;
    gpio_set_intr_type(static_cast<gpio_num_t>(pin),
                       static_cast<gpio_int_type_t>(mode & 0x7));
    gpio_isr_handler_add(static_cast<gpio_num_t>(pin),
                         reinterpret_cast<gpio_isr_t>(cb), nullptr);
}

void Esp32S3Hal::detachInterrupt(uint32_t pin) {
    if (pin == RADIOLIB_NC) return;
    gpio_isr_handler_remove(static_cast<gpio_num_t>(pin));
    gpio_set_intr_type(static_cast<gpio_num_t>(pin), GPIO_INTR_DISABLE);
}

// Timing
void Esp32S3Hal::delay(RadioLibTime_t ms) {
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

void Esp32S3Hal::delayMicroseconds(RadioLibTime_t us) {
    //Note: vTaskDelay() has 1ms resolution, so we need to use a busy-wait loop here
    //Might be a better way to do this? TODO
    uint64_t start = esp_timer_get_time();
    while ((esp_timer_get_time() - start) < (uint64_t)us) { /* spin */ }
}

RadioLibTime_t Esp32S3Hal::millis() {
    return static_cast<RadioLibTime_t>(esp_timer_get_time() / 1000ULL);
}

RadioLibTime_t Esp32S3Hal::micros() {
    return static_cast<RadioLibTime_t>(esp_timer_get_time());
}

long Esp32S3Hal::pulseIn(uint32_t pin, uint32_t state, RadioLibTime_t timeout) {
    if (pin == RADIOLIB_NC) return 0;
    const RadioLibTime_t t0 = micros();

    while (digitalRead(pin) != state) {
        if ((micros() - t0) > timeout) return 0;
    }

    const RadioLibTime_t tStart = micros();
    while (digitalRead(pin) == state) {
        if ((micros() - tStart) > timeout) return 0;
    }
    return static_cast<long>(micros() - tStart);
}

// SPI
void Esp32S3Hal::spiBegin() {
    spi_bus_config_t bus = {};
    bus.mosi_io_num = spiMOSI_;
    bus.miso_io_num = spiMISO_;
    bus.sclk_io_num = spiSCK_;
    bus.quadwp_io_num = -1;
    bus.quadhd_io_num = -1;

    esp_err_t err = spi_bus_initialize(SPI3_HOST, &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "spi_bus_initialize: %s", esp_err_to_name(err));
    }

    spi_device_interface_config_t dev = {};
    dev.mode = 0;
    dev.clock_speed_hz = 4 * 1000 * 1000;
    dev.spics_io_num = -1;
    dev.queue_size = 1;

    err = spi_bus_add_device(SPI3_HOST, &dev, &spi_);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "spi_bus_add_device: %s", esp_err_to_name(err));
    }
}

void Esp32S3Hal::spiBeginTransaction() {
    // No-op
}

uint8_t Esp32S3Hal::spiTransferByte(uint8_t b) {
    spi_transaction_t t = {};
    t.length = 8;
    t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    t.tx_data[0] = b;
    esp_err_t err = spi_device_polling_transmit(spi_, &t);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi tx byte: %s", esp_err_to_name(err));
    }
    return t.rx_data[0];
}

void Esp32S3Hal::spiTransfer(uint8_t* out, size_t len, uint8_t* in) {
    if (len == 0) return;

    if (len == 1) {
        uint8_t v = spiTransferByte(out ? out[0] : 0x00);
        if (in) in[0] = v;
        return;
    }

    spi_transaction_t t = {};
    t.length = len * 8;
    t.tx_buffer = out;
    t.rx_buffer = in;
    esp_err_t err = spi_device_polling_transmit(spi_, &t);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi xfer len=%u: %s", (unsigned)len, esp_err_to_name(err));
    }
}

void Esp32S3Hal::spiEndTransaction() {
    // No-op
}

void Esp32S3Hal::spiEnd() {
    if (spi_) {
        (void)spi_bus_remove_device(spi_);
        spi_ = nullptr;
    }
    (void)spi_bus_free(SPI3_HOST);
}