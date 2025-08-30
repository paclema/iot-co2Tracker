#ifndef LORAWANCLIENT_H
#define LORAWANCLIENT_H

#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <list>
#include "LoRaWANClientCallback.h"

// Pinmap para tu hardware (ajusta los pines a tu placa)
// #define RFM95W_SCK_PIN   18
// #define RFM95W_MISO_PIN  19
// #define RFM95W_MOSI_PIN  23
// #define RFM95W_NSS_PIN    5
// #define RFM95W_RESET_PIN 14
// #define RFM95W_DIO0_PIN  26
// #define RFM95W_DIO1_PIN  33
// #define RFM95W_DIO2_PIN  32

class LoRaWANClient {
public:
    static LoRaWANClient* instance;
    static LoRaWANClient* getInstance() { return instance; }
    void addCallback(LoRaWANClientCallback* callback);
    LoRaWANClient();
    void onEvent(ev_t ev);

    void setAppEui(const uint8_t* key);
    void setDevEui(const uint8_t* key);
    void setAppKey(const uint8_t* key);

    void begin();
    void loop();
    
    bool send(uint8_t* payload, size_t len, uint8_t port = 1, bool confirmed = false);

private:
    std::list<LoRaWANClientCallback*> callbacks;
    osjob_t sendjob;

    // Pinmap and HAL configuration
    class MyHalConfig_t : public Arduino_LMIC::HalConfiguration_t {
    public:
        MyHalConfig_t() {}
        virtual void begin(void) override {
            SPI.begin(RFM95W_SCK_PIN, RFM95W_MISO_PIN, RFM95W_MOSI_PIN, RFM95W_NSS_PIN);
        }
    };
    static MyHalConfig_t myHalConfig;
    static const lmic_pinmap myPinmap;
};

#endif // LORAWANCLIENT_H