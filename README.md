# WePowerBeacon

V1.2 - Implementation to trigger the POL pin, add the frame counter to the packet, advertise MAX_Limits frames and sleep for minimum_time and then update the sensor data and manufacturer data.

Data format: [50 57][4B B1 38 7E 8B D4 F8 5A AC 5E E4 0E 6D 37 8B CD][02] [03 00][ 00 4F]

[50 57]: "UUID" <br />
[4B B1 38 7E 8B D4 F8 5A AC 5E E4 0E 6D 37 8B CD]: "Encrypted data", <br />
[02]: "Frame counter", <br />
[03 00]: "device id", <br />
[00]: "status", <br />
[4F]: "rfuIndex", <br />