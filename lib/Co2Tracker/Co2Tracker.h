#ifndef CO2TRACKER_H
#define CO2TRACKER_H
#pragma once

#include <Arduino.h>

#include <IWebConfig.h>

#include <MQTTClient.h>
#include <MQTTClientCallback.h>

class Co2Tracker : 
    public MQTTClientCallback, 
    public IWebConfig {

public:
    bool localLogs = false;
    bool publishGPSdata = false;
    bool publishLoraWan = false;

    Co2Tracker() {}
    virtual ~Co2Tracker() {}

    void parseWebConfig(JsonObjectConst configObject){
        localLogs = configObject["local_logs"] | false;
        publishGPSdata = configObject["publish_gps_data"] | false;
        publishLoraWan = configObject["publish_LoRaWAN"] | true;

        Serial.printf("Co2Tracker: localLogs: %s, publishGPSdata: %s, publishLoraWan: %s\n", 
                      localLogs ? "true" : "false", 
                      publishGPSdata ? "true" : "false", 
                      publishLoraWan ? "true" : "false");
    }

    void setMQTTClient(MQTTClient * client) {
        pMQTTClient = client;
        pMQTTClient->addCallback(this);
    }

    void onConnected(MQTTClient* client) override {
        // String msg = "{\"status\":\"connected\"}";
        // String topic = String(pMQTTClient->getBaseTopic().c_str()) + "/status";

        String msg = "connected = true";
        client->publish(pMQTTClient->getBaseTopic().c_str(), msg.c_str());

    }


private:
    MQTTClient* pMQTTClient = nullptr;
};

#endif // CO2TRACKER_H
