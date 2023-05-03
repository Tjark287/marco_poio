# marko_poio


# Introduction 
The goal of the "Marco POIo" project is to create a system of two devices that enable users to easily navigate and locate a stationary device in a specific location. One device is stationary and the other is wearable, with both devices equipped with GPS, a microcontroller unit (MCU), LoRaWAN and an interface. The wearable device also includes a magnetic sensor to act as a compass, helping the user to navigate towards the stationary device which has auditory feedback (beeper) to guide the user on the final approach. The user can interact with the system through a button on the and haptic feedback on the wearable device.


# GPS Module 
L76 Series module

## UART
The module provides one universal asynchronous receiver & transmitter serial port. The module is designed as a DCE (Data Communication Equipment), following the traditional DCE-DTE (Data Terminal Equipment) connection. The module and the client (DTE) are connected through the following signal shown as following figure. It supports data baud-rate from 4800bps to 115200bps.

### UART Ports
- TXD1: Send data to the RXD signal line of DTE
- RXD1: Receive data from the TXD signal line of DTE


This UART port has the following features:
- UART port can be used for firmware upgrade, NMEA output and PMTK proprietary messages input.
- The default output NMEA type setting is RMC, VTG, GGA, GSA, GSV and GLL.
- UART port supports the following data rates:
4800, 9600, 14400, 19200, 38400, 57600, 115200.
- The default setting is 9600bps, 8 bits, no parity bit, 1 stop bit.
- Hardware flow control and synchronous operation are not supported.

The UART port does not support the RS-232 level but only CMOS level. If the module’s UART port is connected to the UART port of a computer, it is necessary to add a level shift circuit between the module and the computer.

## NMEA-Daten in Variablen speichern
Um NMEA-Daten von Ihrem GPS-Modul in Variablen zu speichern, müssen Sie die empfangenen Daten zunächst analysieren und dann die relevanten Informationen auswählen und speichern. Hier sind einige Schritte, die Sie befolgen können, um NMEA-Daten in Variablen zu speichern:
    Empfangen Sie die NMEA-Daten von Ihrem GPS-Modul über die serielle Schnittstelle und speichern Sie sie in einem Puffer.

- Analysieren Sie den Puffer und suchen Sie nach den NMEA-Zeilen, die Informationen über die Position, Geschwindigkeit und Richtung des GPS-Empfängers enthalten. Diese Zeilen beginnen normalerweise mit "$GPGGA", "$GPRMC" oder "$GPVTG".

- Verarbeiten Sie die gefundenen NMEA-Zeilen, um die relevanten Informationen zu extrahieren. Verwenden Sie beispielsweise die "strtok()" -Funktion in C oder eine ähnliche Funktion, um die Zeile in einzelne Teile zu zerlegen, die durch Kommas getrennt sind.

- Speichern Sie die extrahierten Informationen in geeigneten Variablen. Zum Beispiel können Sie die Längen- und Breitengrad-Informationen in Fließkommazahlen speichern und die Zeitinformationen in einem Zeitstempel-Format speichern.

- Verwenden Sie die gespeicherten Variablen, um Ihre Anwendung oder Ihre Benutzeroberfläche zu aktualisieren. Sie können die Positionsinformationen zum Beispiel auf einer Karte anzeigen oder die Geschwindigkeit und Richtung anzeigen.

Es gibt auch verschiedene Bibliotheken und Frameworks, die Ihnen dabei helfen können, NMEA-Daten von Ihrem GPS-Modul zu verarbeiten und in Variablen zu speichern. Zum Beispiel gibt es die "TinyGPS++"-Bibliothek für Arduino, die Ihnen dabei hilft, GPS-Daten von Ihrem GPS-Modul zu verarbeiten und die relevanten Informationen in geeigneten Variablen zu speichern.
