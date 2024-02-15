#include <Arduino.h>


// Main variables:
// #define DEBUG_ESP_CORE

// Enable wifi diagnostic using platformio build_glag: -D ENABLE_SERIAL_DEBUG:
#define ENABLE_SERIAL_DEBUG true


// Device configurations
unsigned long currentLoopMillis = 0;
unsigned long previousMainLoopMillis = 0;
size_t totalBytes;
size_t usedBytes;
size_t freeBytes;
unsigned long previousSPIFFSLoopMillis = 0;
#define SPIFFS_CHECK_SPACE_TIME 5000


// WebConfigServer Configuration
#include "WebConfigServer.h"
WebConfigServer config;   // <- global configuration object

#ifdef ARDUINO_IOTPOSTBOX_V1
#include "PowerManagement.h"
PowerManagement power;
#endif



// Websocket functions to publish:
String getLoopTime(){ return String(currentLoopMillis - previousMainLoopMillis);}
String getRSSI(){ return String(WiFi.RSSI());}
String getHeapFree(){ return String((float)GET_FREE_HEAP/1000);}
String getMemoryUsageString(){ 
  String r = String("\"Used: " + config.formatBytes(usedBytes) + " Free: " +  config.formatBytes(freeBytes) + "\"");
  // Serial.println(r);
  return r;}
String getMemoryFree(){  return String(freeBytes);};
String getVBat(){ return String((float)power.vBatSense.mV/1000,4);}
String getVBus(){ return String((float)power.vBusSense.mV/1000,3);}


#include <MQTTClient.h>
MQTTClient *mqttClient;

#include <Co2Tracker.h>
Co2Tracker* co2Tracker = nullptr;

// Co2 sensor
#include <Wire.h>
#include "SparkFun_SCD30_Arduino_Library.h"
SCD30 airSensor;

// GPS
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <string.h>
#include <sstream>
const int timeZoneoffset = 2; // Madrid UTC +2

#ifndef GPS_RX_PIN
  #define GPS_RX_PIN 14 // Wemos D1 mini/pro RX to D6 
#endif
#ifndef GPS_TX_PIN
  #define GPS_TX_PIN 12 // Wemos D1 mini/pro TX to D5 
#endif

#define GPS_DATA_PUBLISH_TIME 10000

static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial ss(GPS_RX_PIN, GPS_TX_PIN);


// OLED screen
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128       // OLED display width, in pixels
#define SCREEN_HEIGHT 64       // OLED display height, in pixels
#define OLED_RESET -1          //Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C    //See datasheet for Address
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// TFT SPI screen
#include <TFT_eSPI.h>
#include <SPI.h>
TFT_eSPI tft = TFT_eSPI();       // Invoke custom library


<<<<<<< HEAD
=======
class co2TrackerConfig : public IWebConfig {
public:
  bool localLogs = false;
  bool publishGPSdata = false;
  bool publishLoraWan = false;
  void parseWebConfig(JsonObjectConst configObject){
    localLogs = configObject["localLogs"] | false;
    publishGPSdata = configObject["publishGPSdata"] | false;
    publishLoraWan = configObject["publishLoraWan"] | false;
  }
}

co2TrackerConfig co2TrackerConfig;

>>>>>>> de187d6 (Add co2Tracker configuration options to log or enable publish data)
//Main variables:
uint16_t co2 = 0;
float temp = 0;
float hum = 0;
bool airSensorFirstMeasurement = false;

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


//Lora and TTN
//------------
#include "loraFunctions.h"



void initSCD30(void){
  Wire.begin();

  //Start sensor using the Wire port and enable the auto-calibration (ASC)
  if (airSensor.begin(Wire, true) == false)
  {
      log_e("Air sensor not detected. Please check wiring. Freezing...");
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

void initOLED(void){
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)){
    log_w(F("SSD1306 allocation failed"));
    // for (;;); // Don't proceed, loop forever
  }
  display.clearDisplay();
  display.display();

}
void initGPS(void){
  Serial.print(F("TinyGPS++ library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println();

  ss.begin(GPSBaud);
}

void updateDisplay(void){
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(3);
  display.print(gpsSpeed);

  display.setCursor(75, 20);
  display.setTextSize(2);
  display.print("km/h");

  display.setTextSize(1);
  display.setCursor(0, 30);
  display.print("Co2:");
  display.setCursor(25, 30);
  display.print(co2);

  display.setTextSize(1);
  display.setCursor(0, 40);
  display.print("vBat:");
  display.setCursor(30, 40);
  display.print(String((float)power.vBatSense.mV/1000,3));

  display.setTextSize(1);
  display.setCursor(0, 50);
  display.print("vBus:");
  display.setCursor(30, 50);
  display.print(String((float)power.vBusSense.mV/1000,3));

  display.setTextSize(1);
  display.setCursor(70, 40);
  display.print("SAT:");
  display.setCursor(95, 40);
  display.print(gpsSat);

  display.setTextSize(1);
  display.setCursor(70, 50);
  display.print("ALT:");
  display.setCursor(95, 50);
  display.print(gpsAltitude, 0);

  display.display();

  tft.fillScreen(TFT_BLACK);
  tft.drawFloat(power.vBatSense.mV/1000, 3, 160, 120, 6);

}



void updateTFT(void){
  // display.clearDisplay();
  // display.setCursor(0, 0);
  // display.setTextSize(3);
  // display.print(gpsSpeed);

  // display.setCursor(75, 20);
  // display.setTextSize(2);
  // display.print("km/h");

  // display.setTextSize(1);
  // display.setCursor(0, 30);
  // display.print("Co2:");
  // display.setCursor(25, 30);
  // display.print(co2);

  // display.setTextSize(1);
  // display.setCursor(0, 40);
  // display.print("vBat:");
  // display.setCursor(30, 40);
  // display.print(String((float)power.vBatSense.mV/1000,3));

  // display.setTextSize(1);
  // display.setCursor(0, 50);
  // display.print("vBus:");
  // display.setCursor(30, 50);
  // display.print(String((float)power.vBusSense.mV/1000,3));

  // display.setTextSize(1);
  // display.setCursor(70, 40);
  // display.print("SAT:");
  // display.setCursor(95, 40);
  // display.print(gpsSat);

  // display.setTextSize(1);
  // display.setCursor(70, 50);
  // display.print("ALT:");
  // display.setCursor(95, 50);
  // display.print(gpsAltitude, 0);

  // display.display();


  /*
  TL_DATUM = Top left
  TC_DATUM = Top centre
  TR_DATUM = Top right
  ML_DATUM = Middle left
  MC_DATUM = Middle centre
  MR_DATUM = Middle right
  BL_DATUM = Bottom left
  BC_DATUM = Bottom centre
  BR_DATUM = Bottom right*/

  // tft.fillScreen(TFT_BLACK);
  // tft.drawString("Speed:", 20, 20,4);
  // tft.drawFloat(gpsSpeed, 3, 120, 20, 4);
  tft.drawString("Speed: " + String(gpsSpeed), 20, 20,4);

  tft.drawString("Co2: " + String(co2), 20, 50,4);
  // tft.drawFloat(co2, 100, 100, 4);

  tft.drawString("vBat: " + String(power.vBatSense.mV/1000,3), 20, 80,4);
  // tft.drawFloat(power.vBatSense.mV/1000, 3, 40, 120, 5);

    // Find centre of screen
  // uint16_t x = tft.width()/2;
  // uint16_t y = tft.height()/2;

  // Set datum to Middle Right
  // tft.setTextDatum(MR_DATUM);

  // Set the padding to the maximum width that the digits could occupy in font 4
  // This ensures small numbers obliterate large ones on the screen
  // tft.setTextPadding( tft.textWidth("-88.88", 4) );

  // Creat a random signed floating point number in range -15 to +15
  // float fpn = random(91)/3.0 - 15.0;

  // Draw a floating point number with 2 decimal places with right datum in font 4
  // tft.drawFloat( fpn, 2, x, y, 4);

  // Reset text padding to 0 otherwise all future rendered strings will use it!
  // tft.setTextPadding(0);
  
  // Set datum to Middle Left
  // tft.setTextDatum(ML_DATUM);

  // Draw text with left datum in font 4
  // tft.drawString(" Units", x, y, 4);

Serial.println("Updating TFT");
}


void displayNoData(){
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.setTextSize(3);
    display.print("No Data");
    display.display();
}


void logGPS(void){
  std::stringstream fileName;
  char timeBuffer[16];
  
  lat = gps.location.lat();
  lng = gps.location.lng();
  gpsDate = gps.date.value();
  gpsTime = gps.time.value() + gps.time.age()/10 + timeZoneoffset*1000000;
  gpsSpeed = gps.speed.kmph();

  gpsSat = gps.satellites.value();
  gpsAltitude = gps.altitude.meters();
  gpsHdop = gps.hdop.hdop();
  gpsCourse = gps.course.deg();

  // setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year());
  // adjustTime(timeZoneoffset * SECS_PER_HOUR);

  // updateDisplay();
  updateTFT();
  
  // Create file if it does not exists
  fileName << "/GPS_" << (int)gps.date.year() << "_" << (int)gps.date.month()<< "_" << (int)gps.date.day() << ".csv";
  if( !LittleFS.exists( fileName.str().c_str()) ) {
    File file = LittleFS.open( fileName.str().c_str(), FILE_WRITE);
    if(!file){
      log_e("Failed to create file %s", fileName.str().c_str());
      return;
    }
    log_i("Created log file %s", fileName.str().c_str());
    file.println("time,latitude,longitude,altitude,speed,hdop,satellites,course,vBat,vBus,PowerStatus,ChargingStatus,co2,temp,hum");
  }

  // Open the file to append new line
  File file = LittleFS.open(fileName.str().c_str(), FILE_APPEND);
  if(!file) {
    log_e("Failed to open file %s for appending", fileName.str().c_str());
    return;
  }

  // write file and position
  String newLine;
  newLine += String((uint32_t)gps.time.isValid()?gpsTime:0) + ",";
  newLine += String((double)gps.location.isValid()?lat:0.0f, 8) + ",";
  newLine += String((double)gps.location.isValid()?lng:0.0f, 8) + ",";
  newLine += String((double)gps.altitude.isValid()?gpsAltitude:0.0f, 1) + ",";
  newLine += String((double)gps.speed.isValid()?gpsSpeed:0.0f, 3) + ",";
  newLine += String((double)gps.hdop.isValid()?gpsHdop:0.0f, 2) + ",";
  newLine += String((int)gps.satellites.isValid()?gpsSat:0) + ",";
  newLine += String((double)gps.course.isValid()?gpsCourse:0, 1) + ",";
  newLine += String((float)(power.vBatSense.mV/1000), 5) + ",";
  newLine += String((float)(power.vBusSense.mV/1000), 5) + ",";
  newLine += String((int)power.getPowerStatus()) + ",";
  newLine += String((int)power.getChargingStatus()) + ",";
  newLine += String((uint16_t)co2) + ",";
  newLine += String((float)temp, 2) + ",";
  newLine += String((float)hum, 2);
  file.println(newLine.c_str());
  file.close();

}



void setup() {
  // esp_log_level_set("i2c.master", ESP_LOG_NONE);
  Serial.begin(115200);
  delay(8000);
  
  #ifdef ENABLE_SERIAL_DEBUG
    Serial.setDebugOutput(true);
  #endif

  #ifdef ARDUINO_IOTPOSTBOX_V1
  // while(!Serial) {}
  pinMode(LDO2_EN_PIN, OUTPUT);
  digitalWrite(LDO2_EN_PIN, HIGH);
  power.setup();
  #endif

  config.addConfig(&co2TrackerConfig, "co2Tracker");
  config.begin();

  loraSetup();
 
  config.addDashboardObject("heap_free", getHeapFree);
  config.addDashboardObject("loop", getLoopTime);
  config.addDashboardObject("RSSI", getRSSI);
  config.addDashboardObject("SPIFFS_Usage", getMemoryUsageString);
  config.addDashboardObject("SPIFFS_Free", getMemoryFree);
  config.addDashboardObject("VBat", getVBat);
  config.addDashboardObject("VBus", getVBus);

  mqttClient = config.getMQTTClient();



  // SCD30 and GPS setup:
  initOLED();
  displayNoData();
  initGPS();
  delay(100);
  initSCD30();

  tft.init();
  // tft.setRotation(3);
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.fillScreen(TFT_BLACK);
  // tft.drawFloat(4658/1000, 3, 160, 120, 6);
  tft.drawString("Starting...", 20, 20,4);
  delay(1000);
  // plotLinear("A00", 0, 160);
  // plotLinear("A1", 1 * d, 160);
  // plotLinear("A2", 2 * d, 160);
  // plotLinear("A3", 3 * d, 160);
  // plotLinear("A4", 4 * d, 160);
  // plotLinear("A5", 5 * d, 160);
  
  #ifdef ARDUINO_IOTPOSTBOX_V1
  power.update();
  #endif

  topic = config.getDeviceTopic() + "data";

  co2Tracker = new Co2Tracker();
  co2Tracker->setMQTTClient(config.getMQTTClient());

  Serial.println("###  Looping time\n");

}

void loop() {

  currentLoopMillis = millis();

   if (currentLoopMillis - previousSPIFFSLoopMillis > SPIFFS_CHECK_SPACE_TIME){
    previousSPIFFSLoopMillis = currentLoopMillis;
    totalBytes = LittleFS.totalBytes();
    usedBytes = LittleFS.usedBytes();
    freeBytes  = totalBytes - usedBytes;
   }

  config.loop();

  loraLoop();


  #ifdef ARDUINO_IOTPOSTBOX_V1
  power.update();
  #endif

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

    if(co2TrackerConfig.publishLoraWan) publish2TTN();

    if ( gps.location.isValid() && gps.location.lat() != 0 && gps.location.lng() != 0 && gps.date.isValid() && gps.time.isValid() ){

      if(co2TrackerConfig.localLogs) logGPS();

      // Serial.printf("Lat: %lf - Long: %lf - Date: %zu - Time: %zu - Spped: %lf km/h\n", lat, lng, gpsDate, gpsTime, gpsSpeed);
      // Serial.printf("****** - Time: %zu secs: %d -age %d- Time+age: %d \n", gpsTime, gps.time.second(),  gps.time.age(), (gpsTime + gps.time.age()/10));
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
        doc["rssi_STA"] = WiFi.RSSI();
        #ifdef ARDUINO_IOTPOSTBOX_V1
        doc["vBat"] = (float)power.vBatSense.mV/1000;
        doc["vBus"] = (float)power.vBusSense.mV/1000;
        doc["PowerStatus"] = (int)power.getPowerStatus();
        doc["ChargingStatus"] = (int)power.getChargingStatus();
        #endif

        serializeJson(doc, msg_pub);
        // mqttClient->setBufferSize((uint16_t)(msg_pub.length() + 100));  // Only using PubSubClient
        mqttClient->publish(topic.c_str(), msg_pub.c_str());
        // Serial.println(msg_pub);
      }

    } else displayNoData();

    

  }

  if (airSensor.dataAvailable()) {

    co2 = airSensor.getCO2();
    temp = airSensor.getTemperature();
    hum = airSensor.getHumidity();
    airSensorFirstMeasurement = true;
    
    Serial.print("co2(ppm):");
    Serial.print(co2);
    Serial.print(" temp(C):");
    Serial.print(temp, 1);
    Serial.print(" humidity(%):");
    Serial.print(hum, 1);
    Serial.println();

    bool GPSdataValid = false;
    if (gps.location.isValid() && gps.location.lat() != 0 && gps.location.lng() != 0 && gps.date.isValid() && gps.time.isValid() 
        // && gps.date.year() == 2022 && (gps.date.month() == 7 || gps.date.month() == 8)){
      	){
      logGPS();
      GPSdataValid = true;
    // Serial.printf("Lat: %lf - Long: %lf - Date: %zu - Time: %zu - Spped: %lf km/h\n", lat, lng, gpsDate, gpsTime, gpsSpeed);
    // Serial.printf("****** - Time: %zu secs: %d -age %d- Time+age: %d \n", gpsTime, gps.time.second(),  gps.time.age(), (gpsTime + gps.time.age()/10));
    // Serial.printf("Satellites: %zu - Altitude: %lf - Hdop: %lf - Course: %lf\n", gpsSat, gpsAltitude, gpsHdop, gpsCourse);
    } else {
    updateTFT();
    log_e("Failed logGPS while CO2 measurement");
    }

    updateTFT();

    if(mqttClient->connected()) {

      String msg_pub;
      StaticJsonDocument<256> doc;

      doc["CO2"] = co2;
      doc["temp"] = temp;
      doc["humidity"] = hum;

      if(GPSdataValid && co2TrackerConfig.publishGPSdata){
        doc["lat"] = lat;
        doc["lng"] = lng;
        doc["date"] = gpsDate;
        doc["time"] = gpsTime;
        doc["speed"] = gpsSpeed;
        doc["satellites"] = gpsSat;
        doc["altitude"] = gpsAltitude;
        doc["hdop"] = gpsHdop;
        doc["course"] = gpsCourse;
      }
      doc["rssi_STA"] = WiFi.RSSI();
      #ifdef ARDUINO_IOTPOSTBOX_V1
      doc["vBat"] = (float)power.vBatSense.mV/1000;
      doc["vBus"] = (float)power.vBusSense.mV/1000;
      doc["PowerStatus"] = (int)power.getPowerStatus();
      doc["ChargingStatus"] = (int)power.getChargingStatus();
      #endif
      serializeJson(doc, msg_pub);
      // mqttClient->setBufferSize((uint16_t)(msg_pub.length() + 100));  // Only using PubSubClient
      mqttClient->publish(topic.c_str(), msg_pub.c_str());
      // Serial.println(msg_pub);

    }

  }
  previousMainLoopMillis = currentLoopMillis;
}
