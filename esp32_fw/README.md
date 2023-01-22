# RTK1010Board ESP32 Firmware

## Toolchain

- [ESP32 Compiler V4.4.* ](https://docs.espressif.com/projects/esp-idf/en/v4.4.3/esp32/get-started/index.html)

## Build

- `. ~/workspace/esp32/esp-idf-v4.4/export.sh`

- `esp32_fw$ idf.py menuconfig`

- `esp32_fw$ idf.py build`

- `esp32_fw$ idf.py dfu`

- connect BOOT to GND

- `esp32_fw$ idf.py dfu-flash`

- disconnect BOOT from GND and reset
