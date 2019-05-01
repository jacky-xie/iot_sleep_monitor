#include <ESP8266WiFi.h>
#include <MAX30105.h>
#include <heartRate.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_MCP3008.h>
#include <DHT.h>

//Wifi/MGTT parameters
#define WLAN_SSID "uber_pi"
#define WLAN_PASS "notsosecretpassword"
#define BROKER_IP "192.168.8.1"

//NodeMCU 8266 Pin parameters
//Used for AM2302
#define SD2 9
//Used for MAX30102
#define D1 5
#define D2 4
//Used for MCP3008 ADC
#define D3 0  //DIN
#define D4 2  //DOUT
#define D5 14 //CLK
#define D6 12 //CS
//Used for LED
#define D7 13

//AMD2302 initialization.
//Temperature/Humidity sensor.
//NOTE: ESP8266 and DHT-22 library might have intermittent nan issues stated at: https://github.com/adafruit/DHT-sensor-library/issues/116
DHT am2302(SD2,DHT22);
float temperatureC;
float humidity;

//MCP3008 initialization.
Adafruit_MCP3008 adc;

//MAX30102 sensor and related values.
//BUFFER_SIZE used for calculating "average" heart rate.
#define BUFFER_SIZE 32
//SAMPLE_SIZE used for calculating "current" heart rate.
#define SAMPLE_SIZE 4
MAX30105 hrSensor;
int hr_sensor_state;

byte samples[BUFFER_SIZE];
float bpm;
int last_bpm;
long lastBeat = 0;
int heartRate;
byte s = 0; // Index of samples.

long last_report_delta = 0;
long last_report_time = 0;
long last_delta = 0;
long last_accel_delta = 0;
long last_accel_time = 0;

int calibrated = 0;
int buffer_full = 0;
int detected = 0;

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

  am2302.begin(55);

  //LED
  pinMode(D7, OUTPUT);
  //ADC 0,1,2 hooked to X,Y,Z.
  //CLK, MOSI, MISO, CS
  if (!adc.begin(D5,D3,D4,D6)){
    Serial.println(F("Error with initializing ADC!"));
  }
  // init hr sensor
  // Defaults to GPIO4 for SDA and GPIO5 for SCL.
  if (!hrSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println(F("Error with initializing sensor!"));
    hr_sensor_state = 1;
  } else {
    hr_sensor_state = 0;
    hrSensor.setup();
    hrSensor.setPulseAmplitudeRed(16);
  }
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
    if ( !mqttclient.connected() ) {
      connect();
    } else {
      if (hr_sensor_state == 0) getSensorData();
      else {
        Serial.println(F("Notice: hr sensor is not initialized, is disconnected, or is missing."));
        delay(3000);
        if (!hrSensor.begin(Wire, I2C_SPEED_FAST)) {
          Serial.println(F("Error with initializing sensor!"));
          hr_sensor_state = 1;
        } else {
          hr_sensor_state = 0;
          hrSensor.setup();
          hrSensor.setPulseAmplitudeRed(16);
        }
      }
    }
    mqttclient.loop();
}

void connect() {
    while(WiFi.status() != WL_CONNECTED) {
      Serial.println(F("Wifi issue"));
      delay(3000);
    }
    //Serial.print(F("Connecting to mqtt server..."));
    while(!mqttclient.connected()){
      if (mqttclient.connect(WiFi.macAddress().c_str())){
        Serial.println(F("MQTT server connected."));
        //mqttclient.subscribe("/test");
      } else {
        Serial.print(F("MQTT server connection failed! rc="));
        Serial.print(mqttclient.state());
        Serial.println("try again in 10 seconds");
        delay(10000);
      }
    }
}

void getSensorData() {
  long ir_value, delta;
  int i, x, y, z;
  float temp;
  char buf[10];

  ir_value = hrSensor.getIR(); // Read IR value.
  // Read and publish sensor value
  if (ir_value > 110000) { // Entity detected.
    if (detected == 0) { // Increase brightness.
      hrSensor.setPulseAmplitudeRed(255);
      detected = 1;
    }
    
    if (checkForBeat(ir_value) == true) {
      delta = millis() - lastBeat;
      lastBeat = millis();
      bpm = 60 / (delta / 1000.0);
      digitalWrite(D7,HIGH);

      //"Discards" any erroneous values.
      if (bpm < 192 && bpm > 16) {
        // Lessen the impact of skipped beats or extra beats on the heart rate calculation.
        if ((last_delta * 4) < delta ) {
          //Missed a beat.
          if (s != 0) {
            //Correct the sample by averaging it with the previous one.
            bpm = (samples[s-1] + bpm) / 2;
          }
        }
        if (last_delta > (delta * 4)){
          //Extra beat.
          if (s != 0) {
            //Correct the sample by averaging it with the previous one.
            bpm = (samples[s-1] + bpm) / 2;
          }
        }
        samples[s++] = (byte) bpm;
        last_bpm = (int) bpm;
      }
      
      last_delta = delta;
  
      if (calibrated == 0)
        if (s >= SAMPLE_SIZE) calibrated = 1;
      if (s == BUFFER_SIZE) {
        s = 0;
        buffer_full = 1;
      }
    }
    
    // Take average of readings.
    if (calibrated == 1) {  
      heartRate = 0; // Reset average heart rate.
      if (buffer_full != 0) {
        for (i = 0; i < BUFFER_SIZE; i++) {
          heartRate += samples[i];
        }
        heartRate /= BUFFER_SIZE;
      } else {
        for (i = 0; i < s; i++) {
          heartRate += samples[i];
        }
        heartRate /= s;
      }
    }

    // Turn off LED after 100ms of a heart beat.
    if ((millis() - lastBeat) > 100) digitalWrite(D7,LOW);

    if (calibrated == 1){
      
      //Report sensor data results every 3 seconds.
      if (last_report_delta > 3) {
        last_report_time = millis() / 1000;
        last_report_delta = 0;

        //AM2302 requires around 2 seconds to poll and calculate the data.
        temperatureC = am2302.readTemperature();
        humidity = am2302.readHumidity();
        
        itoa(heartRate, buf, 10);
        mqttclient.publish("bpm", buf, false);
        dtostrf(temperatureC, 5, 2, buf);
        mqttclient.publish("temperatureC", buf, false);
        dtostrf(humidity, 5, 2, buf);
        mqttclient.publish("humidity", buf, false);
      } else {
        last_report_delta += ((millis() / 1000) - last_report_time);
      }
      
      //Report accelerometer values every second.
      if (last_accel_delta > 1){
        last_accel_time = millis() / 1000;
        last_accel_delta = 0;

        x = adc.readADC(0);
        y = adc.readADC(1);
        z = adc.readADC(2);
        
        itoa(x, buf, 10);
        mqttclient.publish("accel_x", buf, false);
        itoa(y, buf, 10);
        mqttclient.publish("accel_y", buf, false);
        itoa(z, buf, 10);
        mqttclient.publish("accel_z", buf, false);
      } else {
        last_accel_delta += ((millis() / 1000) - last_accel_time);
      }
    }
  // ir_value < 110000
  } else {// No entity detected.
    if (detected == 1) { // Lower sensor red LED brightness.
      hrSensor.setPulseAmplitudeRed(8);
      detected = 0;
      calibrated = 0;
      s = 0;
      buffer_full = 0;
      last_delta = 0;
      last_accel_delta = 0;
      last_report_delta = 0;
    }
  }
}
