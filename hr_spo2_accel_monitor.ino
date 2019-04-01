#include <ESP8266WiFi.h>
#include <MAX30105.h>
#include <heartRate.h>
#include <PubSubClient.h>
#include <Wire.h>

//Wifi/MGTT parameters
#define WLAN_SSID "uber_pi"
#define WLAN_PASS "notsosecretpassword"
#define BROKER_IP "192.168.8.1"

//Pin parameters
#define D0 16
#define D1 5
#define D2 4
#define D3 0
#define D4 2

//MAX30102 sensor and related values.
MAX30105 fingerSensor;
int ahr; // Average heart rate.
float bpm; // Heartbeats per minute.
long lastBeat = 0; // Time since last detected heartbeat.
const byte SAMPLE_SIZE = 8;
byte samples[SAMPLE_SIZE];
byte s = 0; // Index for samples.

WiFiClient client;
PubSubClient mqttclient(client);

void callback (char* topic, byte* payload, unsigned int length) {
  char request;
  //Serial.println(topic);
  //Serial.write(payload, length);
  //Serial.println("");
  
  //Process LED.
  /*
  if (!strcmp(topic,"/led/led_toggle")) {
    request = (char) *payload;
    if (request == '0') digitalWrite(LED,LOW);
    else if (request == '1') digitalWrite(LED,HIGH);
    else return; // Invalid request.
  }*/
}

void setup() {
  Serial.begin(115200);

  // init pins
  //pinMode(BUTTON,INPUT);
  //pinMode(LED,OUTPUT);

  // init finger sensor
  // Defaults to GPIO4 for SDA and GPIO5 for SCL.
  fingerSensor.begin(Wire, I2C_SPEED_FAST);
  fingerSensor.setup();
  fingerSensor.setPulseAmplitudeRed(0x0A);

  // connect to wifi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
  }

  Serial.println(F("WiFi Connected"));
  Serial.println(F("IP address: "));
  Serial.println(WiFi.localIP());

  // connect to mqtt server
  mqttclient.setServer(BROKER_IP, 1883);
  mqttclient.setCallback(callback);
  connect();
}

void loop() {
  long ir_value;
  int i;
  int sensor_value;
  char buffer[5];
  if ( !mqttclient.connected() ) {
    connect();
  } else {
    ir_value = fingerSensor.getIR(); // Read IR value.
    
    // Read and publish sensor value
    if (ir_value > 7000){ // Finger detected.
      if (checkForBeat(ir_value) == true) {
        long delta = millis() - lastBeat;
        lastBeat = millis();

        bpm = 60 / (delta/1000.0);
        /*
        Serial.print("heartbeat: ");
        Serial.print(bpm);
        Serial.println("");
        */
        if (bpm < 255 && bpm > 16) {
          samples[s++] = (byte) bpm;
          s %= SAMPLE_SIZE;
          // Take average of readings.
          ahr = 0; // Reset average heart rate.
          for (i = 0; i < SAMPLE_SIZE; i++) {
            ahr += samples[i];
          }
          ahr /= SAMPLE_SIZE;
        }
      }
      Serial.println(ahr);
    } else { // No finder detected.
      //Serial.println(F("No finger detected..."));
      ahr = 0;
    }
  }
  mqttclient.loop();
}

void connect(){
  while(WiFi.status() != WL_CONNECTED) {
    Serial.println(F("Wifi issue"));
    delay(3000);
  }
  Serial.print(F("Connecting to mqtt server..."));
  while(!mqttclient.connected()){
    if (mqttclient.connect(WiFi.macAddress().c_str())){
      Serial.println(F("MQTT server connected."));
      
      //mqttclient.subscribe("/led/led_toggle");
      //mqttclient.subscribe("/sensors/light/control");
      
    } else {
      Serial.print(F("MQTT server connection failed! rc="));
      Serial.print(mqttclient.state());
      Serial.println("try again in 10 seconds");
      delay(20000);
    }
  }
}
