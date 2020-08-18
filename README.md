# MQTT Room Climate
Monitors temperature, humidity and air pressure in a room and sends the data to MQTT server. Not energy efficient device. It is supposed to be powered up by a constant usb power supply.

# Circuit
TODO

# Configuration
Please copy file `Config.h.local` into `Config.h` and define your MQTT and Network credentials there before building the firmware. IP address is configured automatically. OTA is supported.

> Make sure not to commit your credentials to Git. `Config.h` is ignored in file .gitignore.

# TODO
* Enable local AP mode to change WiFi creds without firmwaring it