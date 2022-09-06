# RTK1010Board
Arduino Compatible RTK1010 Board/PCB

! Design not finished yet !

Feature:

* Arduino compatibe form factor
* RTK1010 GPS receiver
* optional SMA connector for external antennas
* IPEX connector for onboard antennas
* optional LDO for LNA power supply
* optional ESP32-S2 for operating as base station
* optional power via USB
* optional backup battery
* optional BNO055 IMU

Use Cases:

A.) Minimum RTK-1010 receiver with arduino form factor. 
* just RTK-1010 to RX/TX arduino pin
* SMA or IPEX connected
* LNA power by rtk-1010
* optional backup bat for RTK-1010
* optional LNA LDO

B.) Minimum USB powered RTK base station with RTK-1010, ESP32-S2.  
* ESP32: RTK <=> TCP bridge
* powered by USB
* SMA or IPEX connected
* LNA power
* optional backup bat for RTK-1010
* optional LNA LDO

C.) Mowgli HEAD with RTK-1010, ESP32-S2 and BNO055 IMU (todo)
* ESP32: running microROS
* VCC, GND, UART interface to YF mainboard
* low power, sleep mode

[RTK 1010 datasheet](https://www.locosystech.com/Templates/att/RTK-1010_datasheet_v0.7.pdf)

[Schematic](RTK1010Board.pdf)

![alt text](RTK1010Board.svg)
![alt text](images/RTK1010Board_3D_F.png)
![alt text](images/RTK1010Board_3D_B.png)
