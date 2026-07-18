#pragma once

#include <RadioLib.h>
#include "driver/spi_master.h"

// --- Esp32S3Hal -------------------------------------------------------------

class Esp32S3Hal : public RadioLibHal {
public:
    explicit Esp32S3Hal(int8_t sck, int8_t miso, int8_t mosi);

    // Lifecycle
    void init() override;
    void term() override;

    // GPIO
    void pinMode(uint32_t pin, uint32_t mode) override;
    void digitalWrite(uint32_t pin, uint32_t value) override;
    uint32_t digitalRead(uint32_t pin) override;
    void attachInterrupt(uint32_t pin, void (*cb)(void), uint32_t mode) override;
    void detachInterrupt(uint32_t pin) override;

    // Timing
    void delay(RadioLibTime_t ms) override;
    void delayMicroseconds(RadioLibTime_t us) override;
    RadioLibTime_t millis() override;
    RadioLibTime_t micros() override;
    long pulseIn(uint32_t pin, uint32_t state, RadioLibTime_t timeout) override;

    // SPI
    void spiBegin() override;
    void spiBeginTransaction() override;
    void spiTransfer(uint8_t* out, size_t len, uint8_t* in) override;
    void spiEndTransaction() override;
    void spiEnd() override;

private:
    uint8_t spiTransferByte(uint8_t b);

    static constexpr const char* TAG = "RadioLibHAL";

    int8_t spiSCK_;
    int8_t spiMISO_;
    int8_t spiMOSI_;
    spi_device_handle_t spi_ = nullptr;
};