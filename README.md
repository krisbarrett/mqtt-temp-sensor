# MQTT Temperature and Humidity Sensor

Arduino code for periodically publishing temperature and humidity measurements to an MQTT broker.

## Hardware

* Arduino-compatible board with WiFi connectivity. For this project, I used the [FeatherS3](https://esp32s3.com/feathers3.html) development board
* [BME280](https://www.bosch-sensortec.com/products/environmental-sensors/humidity-sensors-bme280/) Temperature, Humidity, and Pressure Sensor. Adafruit offers a [breakout board](https://www.adafruit.com/product/2652) which easily connects to the STEMMA QT connector on the FeatherS3 board
* [STEMMA QT cable](https://www.adafruit.com/product/4399)
* Lithium Polymer Battery with JST connector

## Libraries

The following libraries should be added to the Arduino IDE using the library manager.

* [Adafruit_BME280_Library](https://github.com/adafruit/Adafruit_BME280_Library)
* [MQTT](https://github.com/256dpi/arduino-mqtt)
* [ArduinoJson](https://github.com/bblanchon/ArduinoJson)

## Configuration

Create a file called "secrets.h" and set the following constants.  Note that "secrets.h" is ignored in the .gitignore file to avoid accidentally checking it in.

* WIFI_SSID - the SSID of the WiFi network to connect to.
* WIFI_PASSWORD - the password for the WiFi network.
* MQTT_CLIENT_ID - the client ID for connecting to the MQTT broker.
* MQTT_BROKER - the hostname or IP address (e.g., `IPAddress(192, 168, 1, 1)`) of the MQTT broker.
* TEMP_TOPIC - the name of the topic to publish temperature messages.
* HUMIDITY_TOPIC - the name of the topic to publish humidity messages.
* LOOP_DELAY - integer representing the delay in milliseconds at the end of loop().

## Example Output

After opening the serial monitor, you should see something similar to the following if everything works as expected:

```
Initializing BME280 sensor...OK
Connecting to Wi-Fi.....OK
Connecting to MQTT broker...OK
PUBLISH home/office/temperature - {"time":4043,"temperature":24.79000092} - OK
PUBLISH home/office/humidity - {"time":4045,"humidity":33.27246094} - OK
```
## TODO  

* [x] Reconnect to WiFi and MQTT on disconnect
* [ ] Add username/password authentication for MQTT
* [ ] Add TLS support for MQTT
