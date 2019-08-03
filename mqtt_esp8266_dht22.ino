/*
   ESP8266 MQTT DHT for Home Assistant.
   Also toggle PIN XX to turn on/off Recuperator
*/

// Set configuration options for WiFi, MQTT, and DHT in the following file:
#include "config.h"

// Math
#include <math.h>

//Strings
#include <String.h>

// WiFi
#include <ESP8266WiFi.h>
const char* wifiSSID = CONFIG_WIFI_SSID;
const char* wifiPassword = CONFIG_WIFI_PASS;
WiFiClient espClient;

// MQTT
// http://pubsubclient.knolleary.net/
#include <PubSubClient.h>
const char* mqttServer = CONFIG_MQTT_HOST;
const char* mqttUsername = CONFIG_MQTT_USER;
const char* mqttPassword = CONFIG_MQTT_PASS;
const char* mqttClientId = CONFIG_MQTT_CLIENT_ID; // Must be unique on the MQTT network
PubSubClient client(espClient);
// MQTT Topics
const char* headTopic = CONFIG_MQTT_TOPIC_HEAD;
const char* recuperatorTopic = CONFIG_MQTT_TOPIC_RECUPERATOR_HEAD;
const char* temperatureTopic = CONFIG_MQTT_TOPIC_TEMP;
const char* humidityTopic = CONFIG_MQTT_TOPIC_HUMID;
const char* absHumidityTopic = CONFIG_MQTT_TOPIC_ABSHUMID;
const char* dewPointTopic = CONFIG_MQTT_TOPIC_DEWPNT;
const char* stateTopic = CONFIG_MQTT_TOPIC_STATE;
const char* maxTimerTopic = CONFIG_MQTT_TOPIC_MAXTIMER;

// DHT (12)
#include <WEMOS_DHT12.h>
#include "Wire.h"

//DHT (22)
#include <DHT.h>
#include <DHT_U.h>

//Max Timer
boolean mTIsOn = false;
//Fresh and room air humidity and temperature
//save globally as cache for checking past values
float roomAbsHum = 10;
float freshAbsHum = 10;
// Air humidity shall not fall below 55%
float roomRelHum = 60;
// Temperature should not fall below 18°C
float roomTemp = 20;
float freshTemp = 5;

int mTPin = 13; // Pin for toggeling the maxTimer function on the Recuperator (OUTPUT, LOW => ON, INPUT => OFF)
const int rec1Pin = 15; // Pin that reads the value of the Pin 1 of the Recupertator NOTINUSE
//const int offPin = 3; // Pin that could turn the Recuperator completely off (OUTPUT, LOW => Recuperator ON, INPUT => OFF)
const int txPin = 1; // On-board blue LED1
unsigned long lastSampleTime = 0;

/*
   Publish changes to the pin 1 voltage NOTINUSE
*/
void rec1PinChange() {
  // Publish state of recuperator pin 1 => maybe something interesting?
  int state = digitalRead(rec1Pin);
  publishMQTT(
    "Status",
    state,
    stateTopic,
    "",
    recuperatorTopic);
}

/* Callback function for mqttClient
*/
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.println(topic);
  String maxTimer_control_topic = String(headTopic) + recuperatorTopic + "/cmnd" + maxTimerTopic;
  Serial.println(maxTimer_control_topic.equals(topic));
  Serial.println(String((char*) payload));
}

void setup() {
  Serial.begin(9600);
  // pinMode(txPin, OUTPUT);
  // digitalWrite(txPin, HIGH); // Turn off the on-board LED

  turnMTOff(); //mT is initially turned off
  //subscribe to the corresponding mqtt Topic
  String maxTimer_control_topic = String(headTopic) + recuperatorTopic + "/cmnd" + maxTimerTopic;
  Serial.print(String("MaxTimerTopic: ").c_str());
  Serial.println( maxTimer_control_topic.c_str());
  client.subscribe(maxTimer_control_topic.c_str());

  //pinMode(rec1Pin, INPUT); //Rec Pin 1 reader should be input
  //Publish on pin change
  //attachInterrupt(digitalPinToInterrupt(rec1Pin), rec1PinChange, CHANGE);
  //pinMode(offPin, OUTPUT); //Recuperator should be allowed to be on
  //digitalWrite(offPin, LOW);
  pinMode(mTPin, OUTPUT);

  setupWifi();
  client.setServer(mqttServer, 1883);
  client.setCallback(callback);

}

//implementing the formula at https://carnotcycle.wordpress.com/2012/08/04/how-to-convert-relative-humidity-to-absolute-humidity/
//NOTICE: This is only correct at 0.1% for temperature ranges from -30°C to 35°C
float absoluteHumidity(float rH, float temperature) {
  float p1 = 6.112 * exp((17.67 * temperature) / (243.5 + temperature)) * rH * 2.1674;
  float p2 = (273.15 + temperature);
  float aH = p1 / p2; // split the two parts  for simplification of formula reading
  return aH;
}

//implementing the formula at https://carnotcycle.wordpress.com/2012/08/04/how-to-convert-relative-humidity-to-absolute-humidity/
//NOTICE: This is only correct at 0.1% for temperature ranges from -30°C to 35°C
float dewPointTemp(float rH, float temperature) {
  float p1 = 243.5 * ( log(rH / 100) + ((17.67 * temperature) / (243.5 + temperature)));
  float p2 = 17.67 - log(rH / 100) - ((17.67 * temperature) / (243.5 + temperature));
  float dP = p1 / p2; // split the two parts  for simplification of formula reading
  return dP;
}


void setupWifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifiSSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSSID, wifiPassword);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(mqttClientId, mqttUsername, mqttPassword)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void readAndPublish(DHT12 dht, char* dhtName) {
  float humidity, celsius, absHumidity, dewPoint;

  if (dht.get() != 0) {
    Serial.print(String("Error reading temperature DHT ") + dhtName + ": "); Serial.println(dht.get());
  }
  else {
    celsius = dht.cTemp;
    publishMQTT(
      "Temperature",
      celsius,
      temperatureTopic,
      "°C",
      dhtName);


    humidity = dht.humidity;
    publishMQTT(
      "Humidity",
      humidity,
      humidityTopic,
      "%",
      dhtName);

    absHumidity = absoluteHumidity(humidity, celsius);
    publishMQTT(
      "Absolute humidity",
      absHumidity,
      absHumidityTopic,
      "g/m³",
      dhtName);

    dewPoint = dewPointTemp(humidity, celsius);
    publishMQTT(
      "Dew point temperature",
      dewPoint,
      dewPointTopic,
      "°C",
      dhtName);
  }
}

void publishMQTT(const char* value_name, float value, const char* subtopic, const char* unit_of_measurement, const char* dht_name) {
  publishMQTT(
    value_name,
    String(value).c_str(),
    subtopic,
    unit_of_measurement,
    dht_name
  );
}

void publishMQTT(const char* value_name, const char* value, const char* subtopic, const char* unit_of_measurement, const char* dht_name) {
  String topic = String(headTopic) + dht_name + subtopic;
  client.publish(topic.c_str(), value, true); // publish with retain flag set

  Serial.print(String(value_name) + " " + dht_name + "(" + topic + ")" + ": ");
  Serial.print(value);
  Serial.println(unit_of_measurement);
}

/*
   Turn on the max Timer function
*/
void turnMTOn() {
  mTIsOn = true;
  pinMode(mTPin, OUTPUT);
  digitalWrite(mTPin, HIGH);
  publishMQTT(
    "MaxTimer",
    "ON",
    maxTimerTopic,
    "",
    recuperatorTopic);
}

/*
   Turn on the max Timer function
*/
void turnMTOff() {
  mTIsOn = false;
  pinMode(mTPin, OUTPUT);
  digitalWrite(mTPin, LOW);
  publishMQTT(
    "MaxTimer",
    "OFF",
    maxTimerTopic,
    "",
    recuperatorTopic);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long currentMillis = millis();
  if (Serial.available() > 0) {
    // read the incoming byte:
    byte incomingByte = Serial.parseInt();
    if (incomingByte != 0) {

      // say what you got:
      Serial.print("I received: ");
      Serial.println(incomingByte, DEC);

      Serial.print("Current state");
      Serial.println(digitalRead(mTPin));
      mTPin = incomingByte;
      turnMTOn();

      delay(1000);

      turnMTOff();
    }

    Serial.println("Done");
  }
  if (currentMillis - lastSampleTime >= CONFIG_DHT_SAMPLE_DELAY) {
    lastSampleTime = currentMillis;

	//Recreate DHT objects for reconfiguration of SDA and  SCL
    DHT12 dht1(0x5c, 4, 5); //0x5c is I²C adress, pin 4, 5 pins for sensor
    {
      //Lies alle daten aus und veröffentliche sie
      float humidity, celsius, absHumidity, dewPoint;

      //Printe einen Fehler, falls es einen gibt und lies nichts aus
      if (dht1.get() != 0) {
        Serial.print(String("Error reading temperature DHT ") + DHT_NAME_ONE + ": "); Serial.println(dht1.get());
      }
      else {
        celsius = dht1.cTemp;
        publishMQTT(
          "Temperature",
          celsius,
          temperatureTopic,
          "°C",
          DHT_NAME_ONE);

        roomTemp = celsius;

        humidity = dht1.humidity;
        publishMQTT(
          "Humidity",
          humidity,
          humidityTopic,
          "%",
          DHT_NAME_ONE);

        roomRelHum = humidity;

        absHumidity = absoluteHumidity(humidity, celsius);
        publishMQTT(
          "Absolute humidity",
          absHumidity,
          absHumidityTopic,
          "g/m³",
          DHT_NAME_ONE);

        roomAbsHum = absHumidity;

        dewPoint = dewPointTemp(humidity, celsius);
        publishMQTT(
          "Dew point temperature",
          dewPoint,
          dewPointTopic,
          "°C",
          DHT_NAME_ONE);
      }
    }

    DHT dht2(12, 22); //der zweite (genaue) Sensor ist an pin 12 und ein modell der nummer 22
    dht2.begin();
    {
      //Lies alle daten aus und veröffentliche sie
      float humidity, celsius, absHumidity, dewPoint;
      //Printe einen Fehler, falls es einen gibt und lies nichts aus

      if (isnan(dht2.readTemperature())) {
        Serial.print(String("Error reading temperature DHT ") + DHT_NAME_TWO );
      }
      else {
        celsius = dht2.readTemperature();
        publishMQTT(
          "Temperature",
          celsius,
          temperatureTopic,
          "°C",
          DHT_NAME_TWO);

        freshTemp = celsius;
  
        humidity = dht2.readHumidity();
        publishMQTT(
          "Humidity",
          humidity,
          humidityTopic,
          "%",
          DHT_NAME_TWO);
  
        absHumidity = absoluteHumidity(humidity, celsius);
        publishMQTT(
          "Absolute humidity",
          absHumidity,
          absHumidityTopic,
          "g/m³",
          DHT_NAME_TWO);
  
        freshAbsHum = absHumidity;
  
        dewPoint = dewPointTemp(humidity, celsius);
        publishMQTT(
          "Dew point temperature",
          dewPoint,
          dewPointTopic,
          "°C",
          DHT_NAME_TWO);
      }
    }

    //Schalte MaxTimer je nach Feuchtigkeit der Frischluft relativ zur Raumluft
    //ist die raumluft zu trocken (55%) drossele den Lüfter
    if (
      (freshAbsHum < roomAbsHum) && 
      (roomRelHum >= 55) && 
      ((freshTemp < roomTemp && roomTemp >= 18) || (freshTemp >= roomTemp && roomTemp <= 22)) {
      turnMTOff(); //kein Timer => höhere Leistung
    } else {
      turnMTOn(); // Timer => verringerte Leistung
    }
  }
}
