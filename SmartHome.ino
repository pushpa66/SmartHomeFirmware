#include <Adafruit_Sensor.h>
#include <DHT_U.h>
#include <DHT.h>
#include <ESP8266WiFi.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <WiFiUdp.h>


//-------------------MQTT variables-----------------------------------------
#define publishTopic "mqtt/status"
#define subscribeTopic "mqtt/command"
const char* mqttServer = "iot.eclipse.org";
const int serverPort = 1883;

const char* USERNAME = "SmartHome1234";
const char* PASSWORD = "SmartHome1234";
WiFiClient client;

Adafruit_MQTT_Client mqtt(&client, mqttServer, serverPort);
Adafruit_MQTT_Publish STATE = Adafruit_MQTT_Publish(&mqtt, publishTopic);
Adafruit_MQTT_Subscribe COMMAND = Adafruit_MQTT_Subscribe(&mqtt, subscribeTopic, MQTT_QOS_1);

char * WIFI_SSID = "My_Dialog_4G";
char * WIFI_PASSWORD = "1Kumara@";

bool availableWiFi = false;
String data = "";

const int led1 = D2;
const int led2 = D3;
const int fan = D8;
const int gas = D5;
const int motionPin = D6;

#define DHTPIN            D7
#define DHTTYPE           DHT11     // DHT 11 
DHT_Unified dht(DHTPIN, DHTTYPE);

String m = "";
String g = "";
float t = 0;
float h = 0;

int t1 = 0;
void setup() {

  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  digitalWrite(led1, LOW);
  digitalWrite(led2, LOW);
  pinMode(fan, OUTPUT);
  digitalWrite(fan, LOW);
  pinMode(gas, INPUT);
  pinMode(motionPin, INPUT);

  Serial.begin(9600);

  // Initialize device.
  dht.begin();
  Serial.println("DHTxx Unified Sensor Example");
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Temperature");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" *C");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" *C");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" *C");  
  Serial.println("------------------------------------");
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Humidity");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println("%");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println("%");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println("%");  
  Serial.println("------------------------------------");
 
  while (!Serial) {}
  for (int i = 0; i < 3; i++) {
    setupWiFi();
    if (availableWiFi)
      break;
  }

  mqtt.subscribe(&COMMAND);
  t1 = millis();
}

void loop() {

    checkSubscription();
    readSensors();
    readDHT();
  
   if(millis() - t1 > 0){
    if(millis() - t1 > 2000){
      t1 = millis();
      data = "{'T':'"+String(t)+"','H':'"+String(h)+"','M':'"+m+"','G':'"+g+"'}";
      transferSTATE();
    }
   } else {
    t1 = millis();
   }
}

void setLed(int led,bool state){
  digitalWrite(led, state);
}

void setFan(bool state){
  digitalWrite(fan, state);
}

void readSensors(){
  if(digitalRead(gas)){
    g = "YES";
  } else {
    g = "NO";
  }
  if(digitalRead(motionPin)){
    m = "YES";
  } else {
    m = "NO";
  }
}

void readDHT(){
  sensors_event_t event; 
  // Get temperature event and print its value. 
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println("Error reading temperature!");
  }
  else {
    t = event.temperature;
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println("Error reading humidity!");
  }
  else {
    h = event.relative_humidity;
  }
}

void checkSubscription() {
  setupWiFi();
  MQTTConnect(mqtt);
  mqtt.subscribe(&COMMAND);
  Adafruit_MQTT_Subscribe *subscription;
  if ((subscription = mqtt.readSubscription(500))) {
    if (subscription == &COMMAND) {
      Serial.print(F("COMMANDS : "));

      char* sub = (char*)COMMAND.lastread;
      data = String(sub);
      
      Serial.println(data);
      if (data.indexOf("Bulb1:ON") != -1) {
        setLed(led1, true);
        Serial.println(F("Bulb1 ON"));
      } else if (data.indexOf("Bulb1:OFF") != -1) {
        setLed(led1, false);
        Serial.println(F("Bulb1 OFF"));
      } else if (data.indexOf("Bulb2:ON") != -1) {
        setLed(led2, true);
        Serial.println(F("Bulb2 ON"));
      } else if (data.indexOf("Bulb2:OFF") != -1) {
        setLed(led2, false);
        Serial.println(F("Bulb2 OFF"));
      } else if (data.indexOf("Fan:ON") != -1) {
        setFan(true);
        Serial.println(F("Fan ON"));
      } else if (data.indexOf("Fan:OFF") != -1) {
        setFan(false);
        Serial.println(F("Fan OFF"));
      }
    }
  }
  //  if (! mqtt.ping()) {
  //    mqtt.disconnect();
  //  }
}

bool transferSTATE() {
  setupWiFi();
  if (availableWiFi) {
    MQTTConnect(mqtt);
    Serial.println(data);
    Serial.println(data.length());
    byte transmitBuffer[data.length()];
    memset(transmitBuffer, 0, sizeof transmitBuffer);
    data.getBytes(transmitBuffer, sizeof transmitBuffer + 1);

    if (!STATE.publish(transmitBuffer, sizeof transmitBuffer)) {
      Serial.println(F("Failed!"));
      return false;
    } else {
      Serial.println(F("Success!"));
      return true;
    }
    //    if (! mqtt.ping()) {
    //      mqtt.disconnect();
    //    }
  } else {
    Serial.println(F("Transmission can not start"));
    return false;
  }
}

void setupWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    for ( int i = 0; i < 15; i++) {
      if (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
      } else {
        randomSeed(micros());

        Serial.println();
        Serial.println(F("WiFi connected"));
        Serial.println(F("IP address: "));
        Serial.println(WiFi.localIP());
        availableWiFi = true;
        return;
      }
    }
    Serial.println();
    Serial.println(F("WiFi connection is failed.."));
    availableWiFi = false;
  } else {
    availableWiFi = true;
  }
}

void MQTTConnect(Adafruit_MQTT_Client & mqtt) {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    //Serial.println("Already connected to MQTT..");
    return;
  }
  Serial.print(F("Connecting to MQTT... "));

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));

    mqtt.disconnect();
    delay(500);  // wait 5 seconds
    retries--;

    if (retries == 0) {
      Serial.println(F("MQTT Not Connected!"));
      break;
    } else {
      Serial.println(F("Retrying MQTT connection in 0.5 seconds..."));
    }
  }
  Serial.println(F("MQTT Connected!"));
}

