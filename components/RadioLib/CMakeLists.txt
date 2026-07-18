
cmake_minimum_required(VERSION 3.13)

# first gather the files since this is needed for both ESP-IDF as well as other builds
file(GLOB_RECURSE RADIOLIB_SOURCES
  "src/*.cpp"
)

# exclude all HAL source files
list(FILTER RADIOLIB_SOURCES EXCLUDE REGEX "src/hal/.*\\.cpp")

#Include HAL for ESP32-S3
list(APPEND RADIOLIB_SOURCES "src/hal/ESP32S3Hal/Esp32S3Hal.cpp")



  # Build RadioLib as an ESP-IDF component
  # required because ESP-IDF runs cmake in script mode
  # and needs idf_component_register()

  idf_component_register(
    SRCS ${RADIOLIB_SOURCES}
    INCLUDE_DIRS . src 
    REQUIRES driver freertos spi_flash esp_timer esp_rom esp_driver_spi esp_driver_gpio

  )

