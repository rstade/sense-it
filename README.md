# sense-it
basic code for ESP32 microcontroller with MQTT interface and management of configuration data via Bluetooth
- tested with ESP32-WROOM-32
- using Arduino IDE for Microchip Studio 7, but should work also with standalone Arduino IDE

can be configured by #define's in the code as
- as CO2 Sensor using MHZ19
- as pressure/humidity/temperature sensor using BME280
- as temperature sensor using one or more DS18B20

following configuration data can be managed via Bluetooth in the field
- SSID and password of the Wifi network
- address of the MQTT server to use
- root of publication topic
- root of subscription topic (unused)