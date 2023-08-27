# RTK1010-PCB-BOM

<h2 id="notes">
  Note regarding optional components:
</h2>

- U2 is the 5V to 3.3V LDO - *C1 should be populated too*

    If you like easy USB flashing of the ESP (only USB connected) U2 is required otherwise the ESP does not gets 3.3V power *(JP2 must be cloosed too)*.

    The alternative flashing/power method is to supply 3.3V and solder your serial adapter to the uart probe points.

    If U2 is populated the IOREF (3.3V) at the arduino pin header should not be connected !

- C6 & U3 are required if the antenna requires more than 30mA LNA.

    This way you can supply up to 250mA LNA to active antennas.

- C10 can stabelize the LNA voltage if it flucturates. *(maybe required if you want to use a different antenna)*

If you like to tinker more with the board you may want to populate more 0805 100Ω resistors - check the schematics to decide if you need them.

<br>

<hr>

<br>

<h3 id="bom-a">
  A) Minimum RTK-1010 receiver with arduino form factor
</h3>

<details>
<summary>BOM A)</summary>

| Amount | Placement | Description | Reichelt Part# | Mouser Part# |
| :----: | :-------: | :---------: | :------------: | :----------: |
| 1 | U1             | [RTK-1010](https://www.locosystech.com/product/rtk-module-1010.html) | ❌ | ❌ |
| 2 | C11,C13        | tantalum 47µF/10V | [SMD TAN.47/10](https://reichelt.com/smd-tantalum-47-f-10-10-v-case-d-125-c-ve-500-smd-tan-47-10-p18859.html) | |
| 5 | C4,C5,C7,C8,C9 | 0805 capacitor 100nF/50V | [KEM Z5U0805 100N](https://reichelt.com/multi-layer-ceramic-capacitor-100nf-50v-85-c-kem-z5u0805-100n-p207084.html) | |
| 2 | R7,R8          | 0805 resistor 100Ω | [SMD-0805 100](https://reichelt.com/smd-chip-resistor-type-0805-100-ohm-smd-0805-100-p32874.html) | |
| 1 | R15            | 0805 resistor 10kΩ | [SMD-0805 10,0K](https://reichelt.com/smd-chip-resistor-type-0805-10-k-ohm-smd-0805-10-0k-p32898.html) | |
| 2 | L1,L3          | 0805 inductor 39nH | [L-0805AS 39N](https://reichelt.com/smd-induktivitaet-0805-keramik-39-nh-l-0805as-39n-p255488.html) | |
| 1 | J2             | U.FL/I-PEX/SMA connector | ❌ | [73412-0110](https://mouser.com/ProductDetail/Molex/73412-0110?qs=NlNVDDZd7xQHV8e0ilpSdQ%3D%3D) |
| 1 | L1/L5 GPS      | Linx active GPS antenna | ❌ | [ANT-GNCP-C25L15100](https://mouser.com/ProductDetail/Linx-Technologies/ANT-GNCP-C25L15100?qs=7D1LtPJG0i2oOR5Ka99M8Q%3D%3D) |
| 1 | BT1            | CR2032 holder | [KEYSTONE 1058](https://reichelt.com/button-cell-holder-for-1x-20mm-keystone-1058-p213351.html) | |
| 1 | (BT1)          | CR2032 | [CR2032](https://reichelt.com/lithium-button-cell-battery-3-volt-210-mah-20-0x3-2-mm-cr-2032-p26550.html) | |
| | **Optional:** |
| 1 | U3             | GPS LNA 3.3V LDO  | [MCP 1700-3302](https://reichelt.com/ldo-fixed-voltage-regulator-3-3v-250ma-sot-23-3-pin-mcp-1700-3302-p200923.html)<br>[MCP 1702T-3302E](https://reichelt.com/de/en/ldo-voltage-regulator-3-3v-sot-23a-3-mcp-1702t-3302e-p216942.html) | |
| 1 | C6             | 0805 capacitor 100nF/50V | [KEM Z5U0805 100N](https://reichelt.com/multi-layer-ceramic-capacitor-100nf-50v-85-c-kem-z5u0805-100n-p207084.html) | |
||
| 1 | C10            | tantalum 47µF/10V | [SMD TAN.47/10](https://reichelt.com/smd-tantalum-47-f-10-10-v-case-d-125-c-ve-500-smd-tan-47-10-p18859.html) | |

</details>

<br>

<hr>

<br>

<h3 id="bom-b">
  B) Minimum RTK base station with RTK-1010 & ESP32-S2 powered by USB
</h3>

<details>
<summary>BOM B)</summary>

| Amount | Placement | Description | Reichelt Part# | Mouser Part# |
| :----: | :-------: | :---------: | :------------: | :----------: |
| 1 | U1             | [RTK-1010](https://www.locosystech.com/product/rtk-module-1010.html) | ❌ | ❌ |
| 1 | U2             | 3.3V LDO | [NCP 1117 ST33T3G](https://reichelt.com/ldo-voltage-regulator-3-3vdc-sot-223-ncp-1117-st33t3g-p188925.html) | |
| 3 | C11,C12,C13    | tantalum 47µF/10V | [SMD TAN.47/10](https://reichelt.com/smd-tantalum-47-f-10-10-v-case-d-125-c-ve-500-smd-tan-47-10-p18859.html) | |
| 2 | C2,C3          | 0805 capacitor 100pF/50V | [KEM C0G0805 100P](https://reichelt.com/multi-layer-ceramic-capacitor-100pf-50v-125-c-kem-c0g0805-100p-p207035.html) | |
| 6 | C1,C4,C5,C7,C8,C9 | 0805 capacitor 100nF/50V | [KEM Z5U0805 100N](https://reichelt.com/multi-layer-ceramic-capacitor-100nf-50v-85-c-kem-z5u0805-100n-p207084.html) | |
| 4 | R4,R5,R13,R14  | 0805 resistor 100Ω | [SMD-0805 100](https://reichelt.com/smd-chip-resistor-type-0805-100-ohm-smd-0805-100-p32874.html) | |
| 2 | R15,R16        | 0805 resistor 10kΩ | [SMD-0805 10,0K](https://reichelt.com/smd-chip-resistor-type-0805-10-k-ohm-smd-0805-10-0k-p32898.html) | |
| 2 | L1,L3          | 0805 inductor 39nH | [L-0805AS 39N](https://reichelt.com/smd-induktivitaet-0805-keramik-39-nh-l-0805as-39n-p255488.html) | |
| 1 | J2             | U.FL/I-PEX/SMA connector | ❌ | [73412-0110](https://mouser.com/ProductDetail/Molex/73412-0110?qs=NlNVDDZd7xQHV8e0ilpSdQ%3D%3D) |
| 1 | L1/L5 GPS      | Linx active GPS antenna | ❌ | [ANT-GNCP-C25L15100](https://mouser.com/ProductDetail/Linx-Technologies/ANT-GNCP-C25L15100?qs=7D1LtPJG0i2oOR5Ka99M8Q%3D%3D) |
| 1 | BT1            | CR2032 holder | [KEYSTONE 1058](https://reichelt.com/button-cell-holder-for-1x-20mm-keystone-1058-p213351.html) | |
| 1 | (BT1)          | CR2032 | [CR2032](https://reichelt.com/lithium-button-cell-battery-3-volt-210-mah-20-0x3-2-mm-cr-2032-p26550.html) | |
| 1 | J3             | USB socket | ❌ | [649-10118194-0001LF](https://eu.mouser.com/ProductDetail/Amphenol-FCI/10118194-0001LF?qs=Ywefl8B65e4FIdY8OWfRQA%3D%3D) |
| 1 | U4             | ESP32-S2 | [ESP32S2WROVERI4 (sma antenna)](https://reichelt.com/wifi-modul-802-11-bt-2-4-2-5ghz-150mb-s-esp32s2wroveri4-p311738.html)<br>***or***<br>[ESP32-S2-WROVER (pcb antenna)](https://reichelt.com/wifi-smd-module-esp32-s2-4-mb-spi-2mb-psram-18x31x3-3-mm-esp32-s2-wrover-p300188.html) | [ESP32-S2-WROVER-I (sma antenna)](https://mouser.com/ProductDetail/Espressif-Systems/ESP32-S2-WROVER-IM22S2H3216UH3Q0?qs=sGAEpiMZZMu3sxpa5v1qrl%2FYtpu2q02Iuga2xwvMqqs%3D)<br>***or***<br>[ESP32-S2-WROVER (pcb antenna)](https://mouser.com/ProductDetail/Espressif-Systems/ESP32-S2-WROVERM22S2H3216PH3Q0?qs=sGAEpiMZZMu3sxpa5v1qrl%2FYtpu2q02IcDxUAUeVSag%3D) |
| 1 | TP_BOOT1/TP_GND1 | ESP boot header | [Header](https://reichelt.com/pin-headers-2-54-mm-1x02-straight-mpe-087-1-002-p119879.html)| |
| 1 |                | ESP boot jumper | [Jumper](https://reichelt.com/jumper-red-with-lug-jumper-2-54gl-rt-p9018.html)| |
| | **Optional:** |
| 1 | U3             | GPS LNA 3.3V LDO  | [MCP 1700-3302](https://reichelt.com/ldo-fixed-voltage-regulator-3-3v-250ma-sot-23-3-pin-mcp-1700-3302-p200923.html)<br>[MCP 1702T-3302E](https://reichelt.com/de/en/ldo-voltage-regulator-3-3v-sot-23a-3-mcp-1702t-3302e-p216942.html) | |
| 1 | C6             | 0805 capacitor 100nF/50V | [KEM Z5U0805 100N](https://reichelt.com/multi-layer-ceramic-capacitor-100nf-50v-85-c-kem-z5u0805-100n-p207084.html) | |
||
| 1 | C10            | tantalum 47µF/10V | [SMD TAN.47/10](https://reichelt.com/smd-tantalum-47-f-10-10-v-case-d-125-c-ve-500-smd-tan-47-10-p18859.html) | |

</details>

<br>

<hr>

<br>

<h3 id="bom-c">
  C) Mowgli HEAD with RTK-1010, ESP32-S2 and BNO055 IMU *(TODO)*
</h3>

<details>
<summary>BOM C)</summary>

| Amount | Placement | Description | Reichelt Part# | Mouser Part# |
| :----: | :-------: | :---------: | :------------: | :----------: |
| 1 | U1             | [RTK-1010](https://www.locosystech.com/product/rtk-module-1010.html) | ❌ | ❌ |
| 3 | C11,C12,C13    | tantalum 47µF/10V | [SMD TAN.47/10](https://reichelt.com/smd-tantalum-47-f-10-10-v-case-d-125-c-ve-500-smd-tan-47-10-p18859.html) | |
| 2 | C2,C3          | 0805 capacitor 100pF/50V | [KEM C0G0805 100P](https://reichelt.com/multi-layer-ceramic-capacitor-100pf-50v-125-c-kem-c0g0805-100p-p207035.html) | |
| 5 | C4,C5,C7,C8,C9 | 0805 capacitor 100nF/50V | [KEM Z5U0805 100N](https://reichelt.com/multi-layer-ceramic-capacitor-100nf-50v-85-c-kem-z5u0805-100n-p207084.html) | |
| 4 | R4,R5,R13,R14  | 0805 resistor 100Ω | [SMD-0805 100](https://reichelt.com/smd-chip-resistor-type-0805-100-ohm-smd-0805-100-p32874.html) | |
| 4 | R15,R16,R17,R18| 0805 resistor 10kΩ | [SMD-0805 10,0K](https://reichelt.com/smd-chip-resistor-type-0805-10-k-ohm-smd-0805-10-0k-p32898.html) | |
| 2 | L1,L3          | 0805 inductor 39nH | [L-0805AS 39N](https://reichelt.com/smd-induktivitaet-0805-keramik-39-nh-l-0805as-39n-p255488.html) | |
| 1 | J2             | U.FL/I-PEX/SMA connector | ❌ | [73412-0110](https://mouser.com/ProductDetail/Molex/73412-0110?qs=NlNVDDZd7xQHV8e0ilpSdQ%3D%3D) |
| 1 | L1/L5 GPS      | Linx active GPS antenna | ❌ | [ANT-GNCP-C25L15100](https://mouser.com/ProductDetail/Linx-Technologies/ANT-GNCP-C25L15100?qs=7D1LtPJG0i2oOR5Ka99M8Q%3D%3D) |
| 1 | BT1            | CR2032 holder | [KEYSTONE 1058](https://reichelt.com/button-cell-holder-for-1x-20mm-keystone-1058-p213351.html) | |
| 1 | (BT1)          | CR2032 | [CR2032](https://reichelt.com/lithium-button-cell-battery-3-volt-210-mah-20-0x3-2-mm-cr-2032-p26550.html) | |
| 1 | U4             | ESP32-S2 | [ESP32S2WROVERI4 (sma antenna)](https://reichelt.com/wifi-modul-802-11-bt-2-4-2-5ghz-150mb-s-esp32s2wroveri4-p311738.html)<br>***or***<br>[ESP32-S2-WROVER (pcb antenna)](https://reichelt.com/wifi-smd-module-esp32-s2-4-mb-spi-2mb-psram-18x31x3-3-mm-esp32-s2-wrover-p300188.html) | [ESP32-S2-WROVER-I (sma antenna)](https://mouser.com/ProductDetail/Espressif-Systems/ESP32-S2-WROVER-IM22S2H3216UH3Q0?qs=sGAEpiMZZMu3sxpa5v1qrl%2FYtpu2q02Iuga2xwvMqqs%3D)<br>***or***<br>[ESP32-S2-WROVER (pcb antenna)](https://mouser.com/ProductDetail/Espressif-Systems/ESP32-S2-WROVERM22S2H3216PH3Q0?qs=sGAEpiMZZMu3sxpa5v1qrl%2FYtpu2q02IcDxUAUeVSag%3D) |
| 1 | J4             | IMU connector | [2,54 mm, 1X08](https://reichelt.com/sockets-2-54-mm-1x08-straight-mpe-094-1-008-p119917.html) | |
| 1 | J5             | Micro Match SMD socket | [MPE 374-2-008](https://reichelt.com/sockets-micro-match-smd-1-27-mm-2x04-l-mpe-374-2-008-p120017.html) | |
| 1 | (J5)           | Micro Match Connector | [MPE 372-1-008](https://reichelt.com/idc-header-micro-match-8-pin-mpe-372-1-008-p120051.html)| |
| 1 | TP_BOOT1/TP_GND1 | ESP boot header | [Header](https://reichelt.com/pin-headers-2-54-mm-1x02-straight-mpe-087-1-002-p119879.html)| |
| 1 |                | ESP boot jumper | [Jumper](https://reichelt.com/jumper-red-with-lug-jumper-2-54gl-rt-p9018.html)| |
| | **Optional:** |
| 1 | U2             | 3.3V LDO | [NCP 1117 ST33T3G](https://reichelt.com/ldo-voltage-regulator-3-3vdc-sot-223-ncp-1117-st33t3g-p188925.html) | |
| 1 | C1             | 0805 capacitor 100nF/50V | [KEM Z5U0805 100N](https://reichelt.com/multi-layer-ceramic-capacitor-100nf-50v-85-c-kem-z5u0805-100n-p207084.html) | |
| 1 | J3             | USB socket | ❌ | [649-10118194-0001LF](https://eu.mouser.com/ProductDetail/Amphenol-FCI/10118194-0001LF?qs=Ywefl8B65e4FIdY8OWfRQA%3D%3D) |
||
| 1 | U3             | GPS LNA 3.3V LDO  | [MCP 1700-3302](https://reichelt.com/ldo-fixed-voltage-regulator-3-3v-250ma-sot-23-3-pin-mcp-1700-3302-p200923.html)<br>[MCP 1702T-3302E](https://reichelt.com/de/en/ldo-voltage-regulator-3-3v-sot-23a-3-mcp-1702t-3302e-p216942.html) | |
| 1 | C6             | 0805 capacitor 100nF/50V | [KEM Z5U0805 100N](https://reichelt.com/multi-layer-ceramic-capacitor-100nf-50v-85-c-kem-z5u0805-100n-p207084.html) | |
||
| 1 | C10            | tantalum 47µF/10V | [SMD TAN.47/10](https://reichelt.com/smd-tantalum-47-f-10-10-v-case-d-125-c-ve-500-smd-tan-47-10-p18859.html) | |

</details>

<br>

<hr>

<br>

<h2 id="antenna">
  Alternative antennas
</h2>

| Confirmed<br>Working | Antenna | Internal / External| U.FL/I-PEX | SMA | Source | Notes |
| :---------------: | :-----: |  :-: | :--------: | :-: | :----: | :---: |
| ✔ | Linx ANT-GNCP-C25L15100 | I | ✔ | ❌ | [Mouser](https://mouser.com/ProductDetail/Linx-Technologies/ANT-GNCP-C25L15100?qs=7D1LtPJG0i2oOR5Ka99M8Q%3D%3D)<br>[DigiKey](https://digikey.com/en/products/detail/linx-technologies-inc/ANT-GNCP-C25L15100/15294130) |
| ✔ | Linx ANT-GNRM-L15A | E | ❌ | ✔ | [Mouser](https://eu.mouser.com/ProductDetail/TE-Connectivity-Linx-Technologies/ANT-GNRM-L15A-3?qs=tlsG%2FOw5FFgeA7pANnaD7A%3D%3D)<br>[DigiKey](https://www.digikey.com/en/products/detail/te-connectivity-linx/ANT-GNRM-L15A-3/16740157) | Metallic ground plane recommended
| ❓ | u-blox ANN-MB1-00 |  E | ❌ | ✔ | [u-blox](https://www.u-blox.com/en/product/ann-mb1-antenna)<br>[Mouser](https://mouser.com/ProductDetail/u-blox/ANN-MB1-00?qs=CiayqK2gdcJPcGOJA8mMZQ%3D%3D)<br>[DigiKey](https://digikey.com/en/products/detail/u-blox/ANN-MB1-00/14835875) | |
| ❓ | ArduSimple OEM Survey (Tri-band) | I | ❌ | ✔ | [ArduSimple](https://www.ardusimple.com/product/oem-survey-tripleband-gnss-antenna) | |
| ❓ | ArduSimple Budget Survey (Tri-band) |  E | ❌ | ✔ | [ArduSimple](https://www.ardusimple.com/product/budget-survey-tripleband-gnss-antenna-ip66) | The antenna itself has a TNC connector - includes 2,5M TNC-SMA cable  |
