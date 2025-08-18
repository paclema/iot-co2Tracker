#ifndef CO2TRACKER_H
#define CO2TRACKER_H

#include <Arduino.h>

#include <MQTTClient.h>
#include <MQTTClientCallback.h>

class Co2Tracker : public MQTTClientCallback {
public:
    Co2Tracker() {}
    virtual ~Co2Tracker() {}

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
