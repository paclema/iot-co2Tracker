#ifndef CO2TRACKER_H
#define CO2TRACKER_H
#pragma once

#include <Arduino.h>
#include <WiFi.h>

#include <IWebConfig.h>
#include <MQTTClient.h>
#include <MQTTClientCallback.h>

// Co2 sensor
#include <Wire.h>
#include "SparkFun_SCD30_Arduino_Library.h"

// GPS
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
// #include <string.h>
#ifndef GPS_RX_PIN
  #define GPS_RX_PIN 14 // Wemos D1 mini/pro RX to D6 
#endif
#ifndef GPS_TX_PIN
  #define GPS_TX_PIN 12 // Wemos D1 mini/pro TX to D5 
#endif

#define GPS_DATA_PUBLISH_TIME 10000

// OLED screen
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128       // OLED display width, in pixels
#define SCREEN_HEIGHT 64       // OLED display height, in pixels
#define OLED_RESET -1          //Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C    //See datasheet for Address

// TFT SPI screen
#include <TFT_eSPI.h>
// #include <SPI.h>

//Lora and TTN
#include <loraFunctions.h>

#ifdef ARDUINO_IOTPOSTBOX_V1
#include "PowerManagement.h"
extern PowerManagement power;
#endif

class Co2Tracker : public MQTTClientCallback, public IWebConfig {
public:
    Co2Tracker();
    void begin();
    void loop();

    void setMQTTClient(MQTTClient * client);
    void setMQTTDeviceTopic(const String& topic){ this->topic = topic; };

    // IWebConfig
    void parseWebConfig(JsonObjectConst configObject) override;

    // MQTTClientCallback
    void onConnected(MQTTClient* client) override;

    // Flags configurables
    bool localLogs = false;
    bool publishGPSdata = false;
    bool publishLoraWan = false;

private:

    MQTTClient* pMQTTClient = nullptr;
    SCD30 airSensor;
    TinyGPSPlus gps;
    const int timeZoneoffset = 2; // Madrid UTC +2
    static const uint32_t GPSBaud = 9600;
    SoftwareSerial ss;
    Adafruit_SSD1306 display;
    TFT_eSPI tft;
    // TFT_eSPI tft = TFT_eSPI();       // Invoke custom library

    //Main sensor and state variables:
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
    unsigned long lastGPSPublish = 0UL;

    // Métodos privados de inicialización y display
    void initSCD30();
    void initOLED();
    void initGPS();
    void updateDisplay();
    void updateTFT();
    void displayNoData();
    void logGPS();

};

#endif // CO2TRACKER_H
