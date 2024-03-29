# MQTT-Connected DHT Temperature and Humidity Sensors Using ESP8266 Modules (for DHT12)
Based on [this cool project](https://github.com/corbanmailloux/esp-mqtt-dht), this gives a simple way to deploy temperature and humidity sensors around your house using cheap components and the MQTT protocol. I use it with [Home Assistant](https://home-assistant.io/), an amazing, extensible, open-source home automation system, but this project could be used standalone or with any platform.

#### This fork is meant as an example on how to use the DHT12 sensor with the [Wemos DHT12 library](https://github.com/wemos/WEMOS_DHT12_Arduino_Library) as it is/was not supported by the Adafruit DHT Sensor library

The sensor publishes to 8 MQTT topics, 4 per connected sensor:
- Temperature topic (example: `home/livingroom/temperature`): The sensor publishes the temperature in Celsius (example: `23.10`).
- Humidity topic (example: `home/livingroom/humidity`): The sensor publishes the relative humidity in percent (example: `37.40`).
- Absolute humidity topic (example `home/livingroom/absolute_humidity`): The sensor publishes [an approximation to the absolute humidity]((https://carnotcycle.wordpress.com/2012/08/04/how-to-convert-relative-humidity-to-absolute-humidity/) in g/m³ (example: `12.0`)
- Dew point topic (example: `home/livingroom/dew_point`): The sensor publishes [an approximation to the dew point](https://carnotcycle.wordpress.com/2012/08/04/how-to-convert-relative-humidity-to-absolute-humidity/) in °C according to air humidity and temperature (example: `8.0`)

In the sample configuration, the values are published every 30 seconds, but that is configurable.

This example was also used to control a recuperator based on the humdity and temperature difference between fresh/outside air and room/inside air.

## Installation/Configuration
I'll explain how to set the sensor up as an [MQTT sensor](https://home-assistant.io/components/sensor.mqtt/) in Home Assistant. This guide assumes that you already have Home Assistant set up and running. If not, see the installation guides [here](https://home-assistant.io/getting-started/).

### The Home Assistant Side
1. In your `configuration.yaml`, add the following:

    ```yaml
    sensor:
      - platform: mqtt
        name: "Living Room Temperature"
        state_topic: "home/livingroom/temperature"
        qos: 0
        unit_of_measurement: "°C"

      - platform: mqtt
        name: "Living Room Humidity"
        state_topic: "home/livingroom/humidity"
        qos: 0
        unit_of_measurement: "%"
   
      - platform: mqtt
        name: "Living Room Absolute Humidity"
        state_topic: "home/livingroom/absolute_humidity"
        qos: 0
        unit_of_measurement: "g/m³"
  
      - platform: mqtt
        name: "Living Room Humidity"
        state_topic: "home/livingroom/dew_point"
        qos: 0
        unit_of_measurement: "°C"
    ```
2. Set the `name` and `state_topic` to values that make sense for you.
3. Restart Home Assistant. Depending on how you installed it, the process differs. For a Raspberry Pi All-in-One install, use `sudo systemctl restart home-assistant.service` (or just restart the Pi).

### The ESP/Sensor
I'm using ESP8266-01 microcontrollers for my sensors because they are so cheap and small. The downside of the size and price is that programming them can be a bit of a hassle. There are many sites that go into detail, so I won't do it here. You'll need an ESP set up to work with the Arduino IDE. See the readme [here](https://github.com/esp8266/Arduino) for instructions.

1. Using the Library Manager in the Arduino IDE, install the [Adafruit Unified Sensor Library](https://github.com/adafruit/Adafruit_Sensor) and the [DHT Sensor Library](https://github.com/adafruit/DHT-sensor-library). You can find the Library Manager in the "Sketch" menu under "Include Library" -> "Manage Libraries..."
2. Via [the WEMOS DHT12 github](https://github.com/wemos/WEMOS_DHT12_Arduino_Library), download and install the DHT12 library as described.
3. In the `mqtt-esp8266_dht22` folder, update the `config-sample.h` file with your WiFi, MQTT, and DHT settings.
4. Ensure that the `CONFIG_MQTT_CLIENT_ID` setting is a unique value for your network.
5. Set `CONFIG_MQTT_TOPIC_TEMP` and `CONFIG_MQTT_TOPIC_HUMID` to match the values you put in your `configuration.yaml`.
6. Save the configuration file as `config.h`.
7. Open the `.ino` file in the Arduino IDE and upload to an ESP with the correct connections.

#### Wiring

The wiring should be read from the comments in the `.ino` file.
There is no proto-board layout or something similar.
Sorry.
