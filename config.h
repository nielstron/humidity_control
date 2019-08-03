/*
 * This is a sample configuration file for the "mqtt_esp8266_dht22" sensor.
 *
 * Change the settings below and save the file as "config.h"
 * You can then upload the code using the Arduino IDE.
 */

// WiFi
#define CONFIG_WIFI_SSID "wifi_ssid"
#define CONFIG_WIFI_PASS "wifi_pass"

// MQTT
#define CONFIG_MQTT_HOST "mqtt_host"
#define CONFIG_MQTT_USER "mqtt_user"
#define CONFIG_MQTT_PASS "mqtt_pass"

#define CONFIG_MQTT_CLIENT_ID "Humidity Control" // Must be unique on the MQTT network

// MQTT Topics
#define CONFIG_MQTT_TOPIC_HEAD "home/basement/"
#define CONFIG_MQTT_TOPIC_RECUPERATOR_HEAD "recuperator"
#define CONFIG_MQTT_TOPIC_TEMP "/temperature"
#define CONFIG_MQTT_TOPIC_HUMID "/humidity"
#define CONFIG_MQTT_TOPIC_ABSHUMID "/absolute_humidity"
#define CONFIG_MQTT_TOPIC_DEWPNT "/dew_point"
#define CONFIG_MQTT_TOPIC_STATE "/state"
#define CONFIG_MQTT_TOPIC_MAXTIMER "/max_timer"

// DHT
#define CONFIG_DHT_SAMPLE_DELAY 180000 // Milliseconds between readings
//define DHT sensor names
#define DHT_NAME_ONE "room_air"
#define DHT_NAME_TWO "fresh_air"
