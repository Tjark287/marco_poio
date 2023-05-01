# marko_poio


# Introduction 
The goal of the "Marco POIo" project is to create a system of two devices that enable users to easily navigate and locate a stationary device in a specific location. One device is stationary and the other is wearable, with both devices equipped with GPS, a microcontroller unit (MCU), LoRaWAN and an interface. The wearable device also includes a magnetic sensor to act as a compass, helping the user to navigate towards the stationary device which has auditory feedback (beeper) to guide the user on the final approach. The user can interact with the system through a button on the and haptic feedback on the wearable device.


# GPS Module 
The module provides one universal asynchronous receiver & transmitter serial port. The module is designed as a DCE (Data Communication Equipment), following the traditional DCE-DTE (Data Terminal Equipment) connection. The module and the client (DTE) are connected through the following signal shown as following figure. It supports data baud-rate from 4800bps to 115200bps.

## UART Ports
TXD1: Send data to the RXD signal line of DTE
RXD1: Receive data from the TXD signal line of DTE


This UART port has the following features:
UART port can be used for firmware upgrade, NMEA output and PMTK proprietary messages input.
The default output NMEA type setting is RMC, VTG, GGA, GSA, GSV and GLL.
UART port supports the following data rates:
4800, 9600, 14400, 19200, 38400, 57600, 115200.
The default setting is 9600bps, 8 bits, no parity bit, 1 stop bit.
Hardware flow control and synchronous operation are not supported.

The UART port does not support the RS-232 level but only CMOS level. If the module’s UART port is connected to the UART port of a computer, it is necessary to add a level shift circuit between the module and the computer.
