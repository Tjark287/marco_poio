# Marco POIo Project

The goal of the "Marco POIo" project is to assist blind people in navigating back to a known spot where a point of interest (POI) has been left behind. The wearable device provides haptic feedback in the form of vibrations to guide the user towards the POI when the user presses a button. Additionally, the stationary device emits a beeping sound when the user gets within a specified distance of the POI. Both devices are equipped with GPS, a microcontroller unit (MCU) and an ISM transceiver. The wearable device also includes a magnetic sensor to act as a compass, helping the user to navigate towards the stationary device.

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

## Prototype devices
For testing and in-class presentation we made 2 crude prototypes of our devices:
*POI:*
![POI prototype](https://github.com/Tjark287/marco_poio/Hardware/POI_Prototype.jpeg?raw=true)
*Wearable:*
![WEARABLE prototype](https://github.com/Tjark287/marco_poio/Hardware/WEARABLE_Prototype.jpeg?raw=true)



## How to use Marco POIo
The "Marco POIo" device consists of two parts: the Point of Interest (POI) and the WEARABLE. The POI is placed at a specific location, such as your clothing at the beach, while the WEARABLE is carried with you, for example, when you go swimming. The goal of the device is to help you find the POI once you come out of the water. Here is a detailed guide on how to use the device:

*1. Activating the POI:*
- Locate the POI and make sure it is turned on.
- Press the special power button on the POI. This will play a unique activation sound and activate the device.

*2. Preparing the WEARABLE before swimming:*
- Take the WEARABLE with you for swimming.
- Attach it as described.

*3. Using the WEARABLE during the return:*
- After swimming, when you come out of the water, press the button on the WEARABLE to start navigation mode.
- The WEARABLE starts vibrating to provide feedback.
- Hold the WEARABLE horizontally and point it in different directions to feel the vibration.
- The vibration becomes stronger when you align the WEARABLE towards your POI.
- Move in the direction where the vibration is strongest to find your way back to the POI.

*4. Approaching the POI:*
- As you get closer to the POI and are only a few meters away, the POI automatically starts beeping to help you locate it.

*5. Arrival and turning off the POI:*
- Once you reach the POI, press the button on it.
- This will stop the beeping, and the POI will automatically turn off.

Note: Make sure to activate the POI and Wearable, wait for the ready-beep and only then leave your spot. Please note that the device is still in the development phase and is not recommended for critical navigation situations. Do not rely solely on this device; use it as a support for your orientation.


## Design 
The development of the wearable focuses on finding the best wearing position, preferably on the wrist with a wrist strap or Velcro for ease of use. The wearable should be inexpensive and have Braille on the outside describing which way the device should be properly attached to the arm. It provides tactile feedback for navigation and a user-friendly interface.
The challenge is to develop a waterproof housing that is also light enough and does not interfere with signal transmission. Special materials, seals and coatings are being considered to prevent water ingress and ensure device performance.
Work is ongoing to develop and optimize the design to ensure the wearable is practical to implement and meets requirements. Consideration will be given to wearing position, accessibility through Braille, tactile feedback, water resistance, and ease of use. The goal is to create a wearable that provides an effective navigation solution for blind people while being comfortable and easy to use.

### Mockup Idea 1:
In this design idea, the wearable is attached to an arm holder by simply clicking it into the holder. After swimming, the wearable can be removed from the holder by a mechanical mechanism. It is activated by pressing a button. The elongated wearable then moves in front of itself like a blind stick. More intense vibration signals the user which direction to go.

### Mockup Idea 2:
The second design idea involves attaching the wearable to a wristwatch. The case of the wearable can be easily attached to any watch band. Braille on the case clarifies where the front and back are located. Once on the water, you hold your arm in front of you as if you were looking at the watch and press the button on the side of the case. Intense vibration signals the user which way to go.


