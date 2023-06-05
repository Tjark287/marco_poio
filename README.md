# Marco POIo Project

The goal of the "Marco POIo" project is to assist blind people in navigating back to a known spot where a point of interest (POI) has been left behind. The wearable device provides haptic feedback in the form of vibrations to guide the user towards the POI when the user presses a button. Additionally, the stationary device emits a beeping sound when the user gets within a specified distance of the POI. Both devices are equipped with GPS, a microcontroller unit (MCU) and an ISR transceiver. The wearable device also includes a magnetic sensor to act as a compass, helping the user to navigate towards the stationary device.

**Software overview:**
The main.c has two modes depending on loading the POI or the WEARABLE. Please use either the #define IS_POI or IS_WEARABLE depending on which device should be programmed. We used the minmea library by Kosma Moczek for nmea string decoding and partially ported the RFM69 library by Felix Rusu (LowPowerLab.com) to C for the ISM communications.

**Disclaimer:**
Please note that the project was created for a UNI project and is still in its early development phase. We do not recommend relying on the system as is for critical navigation purposes. We wish to improve on many areas, especially creating a more robust system for ISM communications, processing more of the available GPS NMEA sentences (currently only RMC processed) for a more stable location and a routine to calibrate the compass for more percise heading measurement as well as a generally improved system design.

**How To Build your own:**

*Hardware Components required:*
- Main Processor: STM32F411CEU - can vary, not much processing power is required
- GPS Receiver: L76-M33
- Radio: RFM69HW
- Antenna: helical antenna used, any 868MHz antenna with SWR<2 is good
- Magnetometer IC: MMC34160PJ
- 3V3 vibration motor
- 2x 2n7000 MOSFET
- LEDSs + resistors
- ST-Link V2 programmer

Please refer to the schematic for details.
