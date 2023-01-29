# RTK1010Board ESP32 Firmware
## Toolchain
- [ESP32 Compiler V4.4.* ](https://docs.espressif.com/projects/esp-idf/en/v4.4.3/esp32/get-started/index.html)

## Variants
### RTK1010_NODE_ROVER_NTRIP_CLIENT
Standalone NTRIB client. Receiving from a RTCM caster (e.g. german SAPOS), passing the data to the RTK/GPS receiver
and providing the corrected data on a TCP server port.

### RTK1010_NODE_BASE_NTRIP_CASTER
Own RTCM caster
t.b.d.
### RTK1010_NODE_SERIAL_ADAPTER
Simple UART <=> USB/serial adapter
t.b.d.
### RTK1010_NODE_TCP_ADAPTER
Simple UART <=> TCP server port adapter
 

## Build
```sh
. ~/workspace/esp32/esp-idf-v4.4/export.sh
idf.py set-target esp32s2
idf.py menuconfig
```
```sh
idf.py build
idf.py dfu
# connect BOOT to GND
idf.py dfu-flash
# disconnect BOOT from GND and reset
```
```sh
cu -l /dev/ttyACM0 -s 115200
```
## OTA FW Update
```sh
idf.py build
curl 192.168.0.191:8032 --data-binary @- < build/rtknode.bin
```
## UDP Console
```sh
nc -k -l -u 42007
```

