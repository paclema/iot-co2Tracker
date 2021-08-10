#include <Arduino.h>


// Main variables:
// #define DEBUG_ESP_CORE

// Enable wifi diagnostic using platformio build_glag: -D ENABLE_SERIAL_DEBUG:
#define ENABLE_SERIAL_DEBUG true


// Device configurations
unsigned long currentLoopMillis = 0;
unsigned long previousMainLoopMillis = 0;


// WebConfigServer Configuration
#include "WebConfigServer.h"
WebConfigServer config;   // <- global configuration object


// Websocket functions to publish:
String getLoopTime(){ return String(currentLoopMillis - previousMainLoopMillis);}
String getRSSI(){ return String(WiFi.RSSI());}
String getHeapFree(){ return String((float)GET_FREE_HEAP/1000);}

#include <PubSubClient.h>
PubSubClient * mqttClient;

// Co2 sensor
#include <Wire.h>
#include "SparkFun_SCD30_Arduino_Library.h"
SCD30 airSensor;

// GPS
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
static const int RXPin = 14, TXPin = 12; // RX to D6 and TX to D5
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);


void initSCD30(void){
  Wire.begin();

  //Start sensor using the Wire port and enable the auto-calibration (ASC)
  if (airSensor.begin(Wire, true) == false)
  {
      Serial.println("Air sensor not detected. Please check wiring. Freezing...");
      while (1)
          ;
  }

  airSensor.setMeasurementInterval(2); //Change number of seconds between measurements: 2 to 1800 (30 minutes)


  Serial.print("Auto calibration set to ");
  if (airSensor.getAutoSelfCalibration() == true)
      Serial.println("true");
  else
      Serial.println("false");

  //The SCD30 has data ready every two seconds

    //Read altitude compensation value
  unsigned int altitude = airSensor.getAltitudeCompensation();
  Serial.print("Current altitude: ");
  Serial.print(altitude);
  Serial.println("m");

  //My desk is ~116m above sealevel
  airSensor.setAltitudeCompensation(116); //Set altitude of the sensor in m, stored in non-volatile memory of SCD30

  //Pressure in Boulder, CO is 24.65inHg or 834.74mBar
  // airSensor.setAmbientPressure(835); //Current ambient pressure in mBar: 700 to 1200, will overwrite altitude compensation

  //Read temperature offset
  float offset = airSensor.getTemperatureOffset();
  Serial.print("Current temp offset: ");
  Serial.print(offset, 2);
  Serial.println("C");

  //airSensor.setTemperatureOffset(5); //Optionally we can set temperature offset to 5°C, stored in non-volatile memory of SCD30

}

void initGPS(void){
  Serial.print(F("TinyGPS++ library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println();

  ss.begin(GPSBaud);
}

void setup() {
  Serial.begin(115200);
  
  #ifdef ENABLE_SERIAL_DEBUG
    Serial.setDebugOutput(true);
  #endif

  config.begin();
 
  config.addDashboardObject("heap_free", getHeapFree);
  config.addDashboardObject("loop", getLoopTime);
  config.addDashboardObject("RSSI", getRSSI);

  mqttClient = config.getMQTTClient();



  // SCD30 and GPS setup:
  initGPS();
  initSCD30();


  Serial.println("###  Looping time\n");

}

void loop() {

  currentLoopMillis = millis();

  config.loop();

  // Reconnection loop:
  // if (WiFi.status() != WL_CONNECTED) {
  //   config.begin();
  //   networkRestart();
  //   config.configureServer(&server);
  // }

  // Check GPS location:
  while (ss.available() > 0)
    gps.encode(ss.read());

  if (airSensor.dataAvailable()) {
    Serial.print("co2(ppm):");
    Serial.print(airSensor.getCO2());

    Serial.print(" temp(C):");
    Serial.print(airSensor.getTemperature(), 1);

    Serial.print(" humidity(%):");
    Serial.print(airSensor.getHumidity(), 1);

    Serial.println();
    
    Serial.printf("Lat: %f - Long: %f - Date: %d - Time: %d - Spped: %f km/h\n", gps.location.lat(), gps.location.lng(), gps.date.value(), gps.time.value(), gps.speed.kmph());


    if(mqttClient->connected()) {

      String topic = config.getDeviceTopic() + "data";
      String msg_pub;
      StaticJsonDocument<256> doc;

      doc["CO2"] = airSensor.getCO2();
      doc["temp"] = airSensor.getTemperature();
      doc["humidity"] = airSensor.getHumidity();
      doc["lat"] = gps.location.lat();
      doc["lng"] = gps.location.lng();
      doc["date"] = gps.date.value();
      doc["time"] = gps.time.value();
      doc["speed"] = gps.speed.kmph();
      doc["wifiSta_rssi"] = WiFi.RSSI();

      serializeJson(doc, msg_pub);    
      mqttClient->publish(topic.c_str(), msg_pub.c_str());

    }

  }
  previousMainLoopMillis = currentLoopMillis;
}
