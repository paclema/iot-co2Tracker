#ifndef LORAWANCLIENTCALLBACK_H
#define LORAWANCLIENTCALLBACK_H

#include <lmic.h>
#include <stdint.h>
#include <stddef.h>

struct lorawan_event_data {
    ev_t event;
    const uint8_t* data;
    size_t data_len;
    int rssi;
    int snr;
};

class LoRaWANClient;
class LoRaWANClientCallback {
public:
    virtual void onJoinSuccess(LoRaWANClient* client) {}
    virtual void onJoinFailure(LoRaWANClient* client) {}
    virtual void onTxComplete(LoRaWANClient* client) {}
    virtual void onDownlinkReceived(LoRaWANClient* client, const lorawan_event_data* data) {}
    virtual void onEvent(LoRaWANClient* client, ev_t event) {}
    virtual ~LoRaWANClientCallback() {}
};

#endif // LORAWANCLIENTCALLBACK_H