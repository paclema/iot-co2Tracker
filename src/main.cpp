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

#ifndef GPS_RX_PIN
  #define GPS_RX_PIN 14 // Wemos D1 mini/pro RX to D6 
#endif
#ifndef GPS_TX_PIN
  #define GPS_TX_PIN 12 // Wemos D1 mini/pro TX to D5 
#endif

#define GPS_DATA_PUBLISH_TIME 1000

static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial ss(GPS_RX_PIN, GPS_TX_PIN);


//Main variables:
uint16_t co2 = 0;
float temp = 0;
float hum = 0;

double lat = 0;
double lng = 0;
uint32_t gpsDate = 0;
uint32_t gpsTime = 0;
double gpsSpeed = 0;

uint32_t gpsSat = 0;
double gpsAltitude = 0;
double gpsHdop = 0;
double gpsCourse = 0;


String topic = "";
StaticJsonDocument<256> doc;


unsigned long lastGPSPublish = 0UL;







void initSCD30(void){
  Wire.begin();

  //Start sensor using the Wire port and enable the auto-calibration (ASC)
  if (airSensor.begin(Wire, true) == false)
  {
      Serial.println("Air sensor not detected. Please check wiring. Freezing...");
      // while (1)
      //     ;
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

  #ifdef ARDUINO_IOTPOSTBOX_V1
  pinMode(LDO2_EN_PIN, OUTPUT);
  digitalWrite(LDO2_EN_PIN, HIGH);
  #endif

  config.begin();
 
  config.addDashboardObject("heap_free", getHeapFree);
  config.addDashboardObject("loop", getLoopTime);
  config.addDashboardObject("RSSI", getRSSI);

  mqttClient = config.getMQTTClient();



  // SCD30 and GPS setup:
  initGPS();
  delay(100);
  initSCD30();


  topic = config.getDeviceTopic() + "data";

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

  if (gps.charsProcessed() < 10)
    Serial.println(F("WARNING: No GPS data.  Check wiring."));


  if (currentLoopMillis - lastGPSPublish > GPS_DATA_PUBLISH_TIME){
    lastGPSPublish = currentLoopMillis;

    // if (gps.location.isUpdated()){
    //   Serial.printf("---> NEW GPS location: %lf - %lf\n", gps.location.lat(), gps.location.lng());
    // } 
    // if (gps.speed.isUpdated()){
    //   Serial.printf("---> NEW GPS speed: %lf\n", gps.speed.kmph());
    // }


    lat = gps.location.lat();
    lng = gps.location.lng();
    gpsDate = gps.date.value();
    gpsTime = gps.time.value();
    gpsSpeed = gps.speed.kmph();

    gpsSat = gps.satellites.value();
    gpsAltitude = gps.altitude.meters();
    gpsHdop = gps.hdop.hdop();
    gpsCourse = gps.course.deg();

    
    // Serial.printf("Lat: %lf - Long: %lf - Date: %zu - Time: %zu - Spped: %lf km/h\n", lat, lng, gpsDate, gpsTime, gpsSpeed);
    // Serial.printf("Satellites: %zu - Altitude: %lf - Hdop: %lf - Course: %lf\n", gpsSat, gpsAltitude, gpsHdop, gpsCourse);


    if(mqttClient->connected()) {

      String msg_pub;
      StaticJsonDocument<256> doc;

      doc["lat"] = lat;
      doc["lng"] = lng;
      doc["date"] = gpsDate;
      doc["time"] = gpsTime;
      doc["speed"] = gpsSpeed;
      doc["satellites"] = gpsSat;
      doc["altitude"] = gpsAltitude;
      doc["hdop"] = gpsHdop;
      doc["course"] = gpsCourse;
      doc["wifiSta_rssi"] = WiFi.RSSI();

      serializeJson(doc, msg_pub);    
      mqttClient->publish(topic.c_str(), msg_pub.c_str());
      
      // Serial.println(msg_pub);
    }

  }

  if (airSensor.dataAvailable()) {

    co2 = airSensor.getCO2();
    temp = airSensor.getTemperature();
    hum = airSensor.getHumidity();
    
    Serial.print("co2(ppm):");
    Serial.print(co2);
    Serial.print(" temp(C):");
    Serial.print(temp, 1);
    Serial.print(" humidity(%):");
    Serial.print(hum, 1);
    Serial.println();

    lat = gps.location.lat();
    lng = gps.location.lng();
    gpsDate = gps.date.value();
    gpsTime = gps.time.value();
    gpsSpeed = gps.speed.kmph();

    gpsSat = gps.satellites.value();
    gpsAltitude = gps.altitude.meters();
    gpsHdop = gps.hdop.hdop();
    gpsCourse = gps.course.deg();

    
    Serial.printf("Lat: %lf - Long: %lf - Date: %zu - Time: %zu - Spped: %lf km/h\n", lat, lng, gpsDate, gpsTime, gpsSpeed);
    Serial.printf("Satellites: %zu - Altitude: %lf - Hdop: %lf - Course: %lf\n", gpsSat, gpsAltitude, gpsHdop, gpsCourse);


    if(mqttClient->connected()) {

      String msg_pub;
      StaticJsonDocument<256> doc;

      doc["CO2"] = co2;
      doc["temp"] = temp;
      doc["humidity"] = hum;
      doc["lat"] = lat;
      doc["lng"] = lng;
      doc["date"] = gpsDate;
      doc["time"] = gpsTime;
      doc["speed"] = gpsSpeed;
      doc["satellites"] = gpsSat;
      doc["altitude"] = gpsAltitude;
      doc["hdop"] = gpsHdop;
      doc["course"] = gpsCourse;
      doc["wifiSta_rssi"] = WiFi.RSSI();

      serializeJson(doc, msg_pub);    
      mqttClient->publish(topic.c_str(), msg_pub.c_str());
      
      Serial.println(msg_pub);


    }

  }
  previousMainLoopMillis = currentLoopMillis;
}
