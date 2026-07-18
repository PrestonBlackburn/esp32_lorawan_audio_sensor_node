# ESP32 I2S Mic + LoRaWAN 


Setup esp-idf link
```bash
source ~/esp/esp-idf/export.sh
idf.py create-project esp_lorawan_mic .
idf.py set-target esp32s3
```

Add Libraries
```bash
git init
git submodule add https://github.com/espressif/esp-dsp.git ./components/esp-dsp
git submodule add https://github.com/jgromes/RadioLib.git ./components/RadioLib
```
*note - radiolib hal must be udpated too

Needed updates for RadioLib (so update this project to match)
```bash
idf.py menuconfig
```

Update:
Component config → FreeRTOS → Kernel → configTICK_RATE_HZ
100 -> 1000


Install
```bash
idf.py build
idf.py -p /dev/ttyACM0 flash
```

Also need gateway setup
see - https://www.elecrow.com/wiki/lr1302-lorawan-gateway-module.html

```bash
idf.py monitor
```
