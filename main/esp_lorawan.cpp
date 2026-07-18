/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

// include the library
#include <RadioLib.h>
#include "hal/ESP32S3Hal/Esp32S3Hal.hpp"

#include "esp_log.h"
#include "config.h"

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "driver/gpio.h"

static const char *TAG = "LoRaWAN";

#define RADIO_NSS   (5)   // LoRa CS
#define RADIO_IRQ   (2)  // DIO1
#define RADIO_RST   (3)  // Reset
#define RADIO_GPIO  (4)  // Busy

#define RADIO_SCK   (7)
#define RADIO_MISO  (8)
#define RADIO_MOSI  (9)

// HAL + radio
Esp32S3Hal hal(RADIO_SCK, RADIO_MISO, RADIO_MOSI);
Module mod(&hal, RADIO_NSS, RADIO_IRQ, RADIO_RST, RADIO_GPIO);
SX1262 radio(&mod);

// LoRaWAN node — Region and subBand come from config.h
LoRaWANNode node(&radio, &Region, subBand);

// Credentials from config.h
uint64_t joinEUI = RADIOLIB_LORAWAN_JOIN_EUI;
uint64_t devEUI  = RADIOLIB_LORAWAN_DEV_EUI;
uint8_t  appKey[] = { RADIOLIB_LORAWAN_APP_KEY };
uint8_t  nwkKey[] = { RADIOLIB_LORAWAN_NWK_KEY };

// the entry point for the program
// it must be declared as "extern C" because the compiler assumes this will be a C function
extern "C" void app_main(void) {
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) { 
        // ESP_ERR_INVALID_STATE means it was already installed elsewhere, which is fine
        ESP_LOGE(TAG, "GPIO ISR service installation failed");
    }
    // ==== Radio Init ====
    int16_t state = radio.begin();
    if (state == RADIOLIB_ERR_NONE) {
        ESP_LOGI(TAG, "Radio initialized OK");
        // 2. Route the physical external RF switch pin for the Wio layout
        radio.setRfSwitchPins(38, RADIOLIB_NC);
    } else {
        ESP_LOGE(TAG, "Radio init failed: %d", state);
        while (true) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    vTaskDelay(pdMS_TO_TICKS(1500));

    radio.setDio2AsRfSwitch(true);

    // ==== LoRaWAN OTAA setup ====
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
            vTaskDelay(pdMS_TO_TICKS(10000));  // wait 10s before retry
        }
    } while (state != RADIOLIB_LORAWAN_NEW_SESSION);

    if (state != RADIOLIB_LORAWAN_NEW_SESSION) {
        ESP_LOGE(TAG, "Join failed after retries");
        while (true) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "Joined!");
    // ==== Uplink loop ====
    uint32_t counter = 0;

    while (true) {
        uint8_t uplinkPayload[4];
        uplinkPayload[0] = (counter >> 8) & 0xFF;
        uplinkPayload[1] = counter & 0xFF;
        uplinkPayload[2] = 0x00;
        uplinkPayload[3] = 0x00;

        ESP_LOGI(TAG, "Sending uplink #%lu", (unsigned long)counter++);

        int16_t txState = node.sendReceive(uplinkPayload, sizeof(uplinkPayload));

        if (txState < RADIOLIB_ERR_NONE) {
            ESP_LOGE(TAG, "sendReceive failed: %d", txState);
        } else if (txState > 0) {
            ESP_LOGI(TAG, "Downlink received in Rx%d", txState);
        } else {
            ESP_LOGI(TAG, "No downlink received");
        }

        // LoRaWAN requires respecting duty cycle / fair use policy
        // 60+ seconds between uplinks is typical for testing
        vTaskDelay(pdMS_TO_TICKS(LORAWAN_UPLINK_PERIOD));
    }


}
