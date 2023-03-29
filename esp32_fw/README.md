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

# Example
## SAPOS Rover
```sh
$ git clone https://github.com/MS71/RTK1010Board.git rover
Cloning into 'rover'...
remote: Enumerating objects: 486, done.
remote: Counting objects: 100% (122/122), done.
remote: Compressing objects: 100% (50/50), done.
remote: Total 486 (delta 97), reused 88 (delta 72), pack-reused 364
Receiving objects: 100% (486/486), 25.11 MiB | 18.16 MiB/s, done.
Resolving deltas: 100% (235/235), done.

$ cd rover/esp32_fw
rover/esp32_fw$ . ~/workspace/esp32/esp-idf-v4.4/export.sh
Setting IDF_PATH to 'workspace/esp32/esp-idf-v4.4'
Detecting the Python interpreter
Checking "python" ...
Checking "python3" ...
...
Done! You can now compile ESP-IDF projects.
Go to the project directory and run:

  idf.py build

rover/esp32_fw$ idf.py set-target esp32s2
Adding "set-target"'s dependency "fullclean" to list of commands with default set of options.
Executing action: fullclean
...
-- Configuring done
-- Generating done
-- Build files have been written to: rover/esp32_fw/build

rover/esp32_fw$ idf.py menuconfig
-> Partition Table -> "Factory app, two OTA definitions"
-> Serial flasher config -> "Flash size (4 MB)"
-> RTK1010 Node Config -> WIFI Settings -> fill Hostname=rtkrover, SSID, PASSWORD
-> RTK1010 Node Config -> Console -> "[*] Enable UDP Console"
-> RTK1010 Node Config -> Console -> UDP Console Host = IP of your linux development host
-> RTK1010 Node Config -> Node Variant (Rover NTRIP Client)
-> RTK1010 Node Config -> NTRIP Client Settings -> SAPOS user data
  (openservice-sapos.niedersachsen.de) NTRIP Host
  (2101) NTRIP Port
  (myusername) NTRIP Username
  (mypassword) NTRIP Password
  (VRS_3_2G_NI) NTRIP Mountpoint
-> Component config -> TinyUSB Stack -> Enable
-> Component config -> TinyUSB Stack -> Use TinyUSB Stack -> Communication Device Class (CDC) -> Enable
=> EXIT + SAVE
  
rover/esp32_fw$ idf.py build
...
Done
rover/esp32_fw$ idf.py dfu
=> connect board with BOOT0 jumper
rover/esp32_fw$ idf.py dfu-flash
remove BOOT0 jumper and restart
rover/esp32_fw$ ping rtkrover
```
## TCP RTK Base
```sh
$ git clone https://github.com/MS71/RTK1010Board.git tcpbase
Cloning into 'rover'...
remote: Enumerating objects: 486, done.
remote: Counting objects: 100% (122/122), done.
remote: Compressing objects: 100% (50/50), done.
remote: Total 486 (delta 97), reused 88 (delta 72), pack-reused 364
Receiving objects: 100% (486/486), 25.11 MiB | 18.16 MiB/s, done.
Resolving deltas: 100% (235/235), done.

$ cd tcpbase/esp32_fw
tcpbase/esp32_fw$ . ~/workspace/esp32/esp-idf-v4.4/export.sh
Setting IDF_PATH to 'workspace/esp32/esp-idf-v4.4'
Detecting the Python interpreter
Checking "python" ...
Checking "python3" ...
...
Done! You can now compile ESP-IDF projects.
Go to the project directory and run:

  idf.py build

tcpbase/esp32_fw$ idf.py set-target esp32s2
Adding "set-target"'s dependency "fullclean" to list of commands with default set of options.
Executing action: fullclean
...
-- Configuring done
-- Generating done
-- Build files have been written to: rover/esp32_fw/build

tcpbase/esp32_fw$ idf.py menuconfig
-> Partition Table -> "Factory app, two OTA definitions"
-> Serial flasher config -> "Flash size (4 MB)"
-> RTK1010 Node Config -> WIFI Settings -> fill Hostname=tcpbase, SSID, PASSWORD
-> RTK1010 Node Config -> Console -> "[*] Enable UDP Console"
-> RTK1010 Node Config -> Console -> UDP Console Host = IP of your linux development host
-> RTK1010 Node Config -> Node Variant (Base RTCM TCP Server)
-> Component config -> TinyUSB Stack -> Enable
-> Component config -> TinyUSB Stack -> Use TinyUSB Stack -> Communication Device Class (CDC) -> Enable
=> EXIT + SAVE
  
tcpbase/esp32_fw$ idf.py build
...
Done
tcpbase/esp32_fw$ idf.py dfu
=> connect board with BOOT0 jumper
tcpbase/esp32_fw$ idf.py dfu-flash
remove BOOT0 jumper and restart
tcpbase/esp32_fw$ ping tcpbase
```
## TCP RTK Rover
```sh
```
