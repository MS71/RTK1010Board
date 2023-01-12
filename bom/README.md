# RTK1010-PCB-BOM

#### A) Minimum RTK-1010 receiver with arduino form factor

  - RTK-1010 <-> RX/TX arduino pins
  - SMA or IPEX connector
  - LNA power by RTK-1010
  - RTK-1010 backup battery
  - *(optional)* LNA LDO

#### B) Minimum RTK base station with RTK-1010 & ESP32-S2 powered by USB

  - ESP32: RTK <=> TCP bridge
  - powered by USB
  - SMA or IPEX connector
  - LNA power by RTK-1010
  - RTK-1010 backup battery
  - *(optional)* LNA LDO

#### C) Mowgli HEAD with RTK-1010, ESP32-S2 and BNO055 IMU *(TODO)*

  - ESP32: running microROS
  - VCC, GND, UART interface to YF mainboard
  - low power, sleep mode
  - SMA or IPEX connector
  - LNA power by RTK-1010
  - RTK-1010 backup battery
  - *(optional)* LNA LDO

## BOM-Tables

#### A) Minimum RTK-1010 receiver with arduino form factor

| Amount | Placement | Description | Reichelt Part# | Mouser Part# |
| :----: | :-------: | :---------: | :------------: | :----------: |
| 1 | U1             | RTK-1010 | - | - |
| 1 | C5             | 0805 capacitor 100nF | [KEM X7R0805 100N](https://reichelt.com/multi-layer-ceramic-capacitor-100nf-50v-125-c-kem-x7r0805-100n-p207073.html) | |
| 2 | C11,C13        | tantalum 100µF/10V | [T491C 100U 10](https://reichelt.com/smd-tantalum-100-f-10v-125-c-t491c-100u-10-p206478.html) | |
| 5 | C4,C5,C7,C8,C9 | 100nF/50V | [KEM Z5U0805 100N](https://www.reichelt.com/de/en/multi-layer-ceramic-capacitor-100nf-50v-85-c-kem-z5u0805-100n-p207084.html?search=100nF+0805) | |
| 2 | R7, R8         | 0805 resistor 100Ω | [SMD-0805 100](https://reichelt.com/smd-chip-resistor-type-0805-100-ohm-smd-0805-100-p32874.html) | |
| 1 | R15            | 0805 resistor 10kΩ | [SMD-0805 10,0K](https://reichelt.com/smd-chip-resistor-type-0805-10-k-ohm-smd-0805-10-0k-p32898.html) | |
| 2 | L1,L3          | 0804 Inductor 39nH | [L-0805AS 39N](https://www.reichelt.de/de/de/smd-induktivitaet-0805-keramik-39-nh-l-0805as-39n-p255488.html) | |
| 1 | J2             | | - | [73412-0110](https://mouser.com/ProductDetail/Molex/73412-0110?qs=NlNVDDZd7xQHV8e0ilpSdQ%3D%3D) |
| 1 | L1/L5 GPS      | Linx active GPS antenna | - | [ANT-GNCP-C25L15100](https://mouser.com/ProductDetail/Linx-Technologies/ANT-GNCP-C25L15100?qs=7D1LtPJG0i2oOR5Ka99M8Q%3D%3D) |
| 1 | BT1            | CR2032 holder | [KEYSTONE 1058](https://reichelt.com/button-cell-holder-for-1x-20mm-keystone-1058-p213351.html) | |
| 1 | (BT1)          | CR2032 | [CR2032](https://reichelt.com/lithium-button-cell-battery-3-volt-210-mah-20-0x3-2-mm-cr-2032-p26550.html) |  |

---

#### B) Minimum RTK base station with RTK-1010 & ESP32-S2 powered by USB

| Amount | Placement | Description | Reichelt Part# | Mouser Part# |
| :----: | :-------: | :---------: | :------------: | :----------: |
| 1 | U1             | RTK-1010 | - | - |
| 1 | C5             | 0805 capacitor 100nF | [KEM X7R0805 100N](https://reichelt.com/multi-layer-ceramic-capacitor-100nf-50v-125-c-kem-x7r0805-100n-p207073.html) | |
| 2 | C11,C13        | tantalum 100µF/10V | [T491C 100U 10](https://reichelt.com/smd-tantalum-100-f-10v-125-c-t491c-100u-10-p206478.html) | |
| 5 | C4,C5,C7,C8,C9 | 100nF/50V | [KEM Z5U0805 100N](https://www.reichelt.com/de/en/multi-layer-ceramic-capacitor-100nf-50v-85-c-kem-z5u0805-100n-p207084.html?search=100nF+0805) | |
| 4 | R4,R5,R13,R14  | 0805 resistor 100Ω | [SMD-0805 100](https://reichelt.com/smd-chip-resistor-type-0805-100-ohm-smd-0805-100-p32874.html) | |
| 1 | R15            | 0805 resistor 10kΩ | [SMD-0805 10,0K](https://reichelt.com/smd-chip-resistor-type-0805-10-k-ohm-smd-0805-10-0k-p32898.html) | |
| 2 | L1,L3          | 0804 Inductor 39nH | [L-0805AS 39N](https://www.reichelt.de/de/de/smd-induktivitaet-0805-keramik-39-nh-l-0805as-39n-p255488.html) | |
| 1 | J2             | | - | [73412-0110](https://mouser.com/ProductDetail/Molex/73412-0110?qs=NlNVDDZd7xQHV8e0ilpSdQ%3D%3D) |
| 1 | L1/L5 GPS      | Linx active GPS antenna | - | [ANT-GNCP-C25L15100](https://mouser.com/ProductDetail/Linx-Technologies/ANT-GNCP-C25L15100?qs=7D1LtPJG0i2oOR5Ka99M8Q%3D%3D) |
| 1 | BT1            | CR2032 holder | [KEYSTONE 1058](https://reichelt.com/button-cell-holder-for-1x-20mm-keystone-1058-p213351.html) | |
| 1 | (BT1)          | CR2032 | [CR2032](https://reichelt.com/lithium-button-cell-battery-3-volt-210-mah-20-0x3-2-mm-cr-2032-p26550.html) |  |
| 2 | C2, C3         | 0805 capacitor 100pF | [KEM C0G0805 100P](https://reichelt.com/multi-layer-ceramic-capacitor-100pf-50v-125-c-kem-c0g0805-100p-p207035.html) | |
| 1 | J3             | USB socket | - | [649-10118194-0001LF](https://eu.mouser.com/ProductDetail/Amphenol-FCI/10118194-0001LF?qs=Ywefl8B65e4FIdY8OWfRQA%3D%3D) |
| 1 | U4             | ESP32-S2 | [ESP32S2WROVERI4 (sma antenna)](https://reichelt.com/wifi-modul-802-11-bt-2-4-2-5ghz-150mb-s-esp32s2wroveri4-p311738.html)<br>***or***<br>[ESP32-S2-WROVER (pcb antenna)](https://reichelt.com/wifi-smd-module-esp32-s2-4-mb-spi-2mb-psram-18x31x3-3-mm-esp32-s2-wrover-p300188.html) | [ESP32-S2-WROVER-I (sma antenna)](https://mouser.com/ProductDetail/Espressif-Systems/ESP32-S2-WROVER-IM22S2H3216UH3Q0?qs=sGAEpiMZZMu3sxpa5v1qrl%2FYtpu2q02Iuga2xwvMqqs%3D)<br>***or***<br>[ESP32-S2-WROVER (pcb antenna)](https://mouser.com/ProductDetail/Espressif-Systems/ESP32-S2-WROVERM22S2H3216PH3Q0?qs=sGAEpiMZZMu3sxpa5v1qrl%2FYtpu2q02IcDxUAUeVSag%3D) |

---

#### C) Mowgli HEAD with RTK-1010, ESP32-S2 and BNO055 IMU *(TODO)*

| Amount | Placement | Description | Reichelt Part# | Mouser Part# |
| :----: | :-------: | :---------: | :------------: | :----------: |
| 1 | U1             | RTK-1010 | - | - |
| 1 | C5             | 0805 capacitor 100nF | [KEM X7R0805 100N](https://reichelt.com/multi-layer-ceramic-capacitor-100nf-50v-125-c-kem-x7r0805-100n-p207073.html) | |
| 2 | C11,C13        | tantalum 100µF/10V | [T491C 100U 10](https://reichelt.com/smd-tantalum-100-f-10v-125-c-t491c-100u-10-p206478.html) | |
| 5 | C4,C5,C7,C8,C9 | 100nF/50V | [KEM Z5U0805 100N](https://www.reichelt.com/de/en/multi-layer-ceramic-capacitor-100nf-50v-85-c-kem-z5u0805-100n-p207084.html?search=100nF+0805) | |
| 4 | R4,R5,R13,R14  | 0805 resistor 100Ω | [SMD-0805 100](https://reichelt.com/smd-chip-resistor-type-0805-100-ohm-smd-0805-100-p32874.html) | |
| 3 | R15,R17,R18    | 0805 resistor 10kΩ | [SMD-0805 10,0K](https://reichelt.com/smd-chip-resistor-type-0805-10-k-ohm-smd-0805-10-0k-p32898.html) | |
| 2 | L1,L3          | 0804 Inductor 39nH | [L-0805AS 39N](https://www.reichelt.de/de/de/smd-induktivitaet-0805-keramik-39-nh-l-0805as-39n-p255488.html) | |
| 1 | J2             | | - | [73412-0110](https://mouser.com/ProductDetail/Molex/73412-0110?qs=NlNVDDZd7xQHV8e0ilpSdQ%3D%3D) |
| 1 | L1/L5 GPS      | Linx active GPS antenna | - | [ANT-GNCP-C25L15100](https://mouser.com/ProductDetail/Linx-Technologies/ANT-GNCP-C25L15100?qs=7D1LtPJG0i2oOR5Ka99M8Q%3D%3D) |
| 1 | BT1            | CR2032 holder | [KEYSTONE 1058](https://reichelt.com/button-cell-holder-for-1x-20mm-keystone-1058-p213351.html) | |
| 1 | (BT1)          | CR2032 | [CR2032](https://reichelt.com/lithium-button-cell-battery-3-volt-210-mah-20-0x3-2-mm-cr-2032-p26550.html) |  |
| 2 | C2, C3         | 0805 capacitor 100pF | [KEM C0G0805 100P](https://reichelt.com/multi-layer-ceramic-capacitor-100pf-50v-125-c-kem-c0g0805-100p-p207035.html) | |
| 1 | U4             | ESP32-S2 | [ESP32S2WROVERI4 (sma antenna)](https://reichelt.com/wifi-modul-802-11-bt-2-4-2-5ghz-150mb-s-esp32s2wroveri4-p311738.html)<br>***or***<br>[ESP32-S2-WROVER (pcb antenna)](https://reichelt.com/wifi-smd-module-esp32-s2-4-mb-spi-2mb-psram-18x31x3-3-mm-esp32-s2-wrover-p300188.html) | [ESP32-S2-WROVER-I (sma antenna)](https://mouser.com/ProductDetail/Espressif-Systems/ESP32-S2-WROVER-IM22S2H3216UH3Q0?qs=sGAEpiMZZMu3sxpa5v1qrl%2FYtpu2q02Iuga2xwvMqqs%3D)<br>***or***<br>[ESP32-S2-WROVER (pcb antenna)](https://mouser.com/ProductDetail/Espressif-Systems/ESP32-S2-WROVERM22S2H3216PH3Q0?qs=sGAEpiMZZMu3sxpa5v1qrl%2FYtpu2q02IcDxUAUeVSag%3D) |
| 1 | J5             | Micro Match SMD | [MPE 374-2-008](https://www.reichelt.de/buchsenleisten-micro-match-smd-1-27-mm-2x04-l-mpe-374-2-008-p120017.html?&trstct=pol_0&nbc=1) | |
| 1 | J4             | BNO055 Connector | [2,54 mm, 1X08](https://www.reichelt.de/buchsenleisten-2-54-mm-1x08-gerade-mpe-094-1-008-p119917.html?&trstct=pol_0&nbc=1) | |


