idf_component_register(SRCS "esp_lorawan_mic.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES 
                        driver 
                        esp_driver_i2s 
                        esp_driver_gpio 
                        freertos 
                        esp_ringbuf 
                        esp-dsp
                        RadioLib 
                        spi_flash 
                    )
target_compile_options(${COMPONENT_LIB} PRIVATE -lm)