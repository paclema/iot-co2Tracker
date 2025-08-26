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
#include <sstream>
// #include <string.h>
#ifndef GPS_RX_PIN
  #define GPS_RX_PIN 14 // Wemos D1 mini/pro RX to D6 
#endif
#ifndef GPS_TX_PIN
  #define GPS_TX_PIN 12 // Wemos D1 mini/pro TX to D5 
#endif

#define GPS_DATA_PUBLISH_TIME 10000

// LVGL UI
#include <lvgl.h>
#include "ui.h"

//Lora and TTN
// #include <loraFunctions.h>
#include <LoRaWANClient.h>
#include <LoRaWANClientCallback.h>
#include <CayenneLPP.h>
#define CAYENNE_MAX_PAYLOAD_SIZE    64


#ifdef ARDUINO_IOTPOSTBOX_V1
#include "PowerManagement.h"
extern PowerManagement power;
#endif

class Co2Tracker : public MQTTClientCallback, public IWebConfig, public LoRaWANClientCallback {
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
    void onDisconnected(MQTTClient* client) override;

    // LoRaWANClientCallback
    void onTxComplete(LoRaWANClient* client) {
        ESP_LOGW("Co2Tracker", "LoRaWAN transmission complete");
    }
    void onDownlinkReceived(LoRaWANClient* client, const lorawan_event_data* data) override {
        ESP_LOGW("Co2Tracker", "LoRaWAN Downlink received: %.*s", data->data_len, data->data);
    }
    void onJoinSuccess(LoRaWANClient* client) override {
        ESP_LOGW("Co2Tracker", "LoRaWAN Join Success");
        lv_obj_set_style_img_opa(ui_loraImg, 255, LV_PART_MAIN);
    }
    // void onEvent(LoRaWANClient* client, ev_t event) override {}
    void sendLoraCayenne();
    void sendLoraBinary();
    
    // Flags configurables
    bool localLogs = false;
    bool publishGPSdata = false;
    bool publishLoraWan = false;

private:
    LoRaWANClient lorawan;
    MQTTClient* pMQTTClient = nullptr;
    SCD30 airSensor;
    TinyGPSPlus gps;
    const int timeZoneoffset = 2; // Madrid UTC +2
    static const uint32_t GPSBaud = 9600;
    SoftwareSerial ss;

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


    void initSCD30(void);
    void initGPS(void);
    void logGPS(void);

};

#endif // CO2TRACKER_H
