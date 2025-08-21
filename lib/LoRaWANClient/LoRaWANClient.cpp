#include "LoRaWANClient.h"


// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]={ PASTE_LSB };

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ PASTE_LSB };

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { PASTE_MSB };

extern "C" void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8); }
extern "C" void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }
extern "C" void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16); }

// // Pinmap and HAL config
LoRaWANClient::MyHalConfig_t LoRaWANClient::myHalConfig{};

const lmic_pinmap LoRaWANClient::myPinmap = {
    .nss = RFM95W_NSS_PIN,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = RFM95W_RESET_PIN,
    .dio = {RFM95W_DIO0_PIN, RFM95W_DIO1_PIN, RFM95W_DIO2_PIN},
    .pConfig = &myHalConfig
};


LoRaWANClient::LoRaWANClient() {
}

void LoRaWANClient::begin() {
	log_d("Setting up lora...");
	// LMIC init
  	os_init_ex(&myPinmap);

	// Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Disable link check validation
    // Disable link-check mode and ADR, because ADR tends to complicate testing.
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink
    // Set the data rate to Spreading Factor 7.  This is the fastest supported rate for 125 kHz channels, and it
    // minimizes air time and battery power. Set the transmission power to 14 dBi (25 mW).
    // LMIC_setDrTxpow(DR_SF7,14);

    // Start job (sending automatically starts OTAA too)
    // do_send(&sendjob);

	log_d("Setting up lora done");

}

void LoRaWANClient::loop() {
    os_runloop_once();
}

bool LoRaWANClient::send(uint8_t* payload, size_t len, uint8_t port, bool confirmed) {
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
        return false;
    }
    LMIC_setTxData2(port, payload, len, confirmed ? 1 : 0);
    Serial.print(F("Packet queued: size="));
    Serial.println(len);
    Serial.print("Payload: ");
    for (size_t i = 0; i < len; i++) {
        Serial.print(payload[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    return true;
}

void LoRaWANClient::addCallback(LoRaWANClientCallback* callback) {
    callbacks.push_back(callback);
}


void LoRaWANClient::onEvent(ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                uint8_t v = artKey[i] & 0xff;
                if (v < 16) Serial.print('0');
                Serial.print(v, HEX);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                if (i != 0) Serial.print("-");
                uint8_t v = nwkKey[i] & 0xff;
                if (v < 16) Serial.print('0');
                Serial.print(v, HEX);
              }
              Serial.println();
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
	    	// size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            for (auto cb : callbacks) cb->onJoinSuccess(this);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            for (auto cb : callbacks) cb->onJoinFailure(this);
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            for (auto cb : callbacks) cb->onTxComplete(this);
            if (LMIC.dataLen) {
                Serial.print(F("Received "));
                Serial.print(LMIC.dataLen);
                Serial.println(F(" bytes of payload"));
                
                char TTN_response[30];
                int i;
                for ( i = 0 ; i < LMIC.dataLen ; i++ )
                    TTN_response[i] = LMIC.frame[LMIC.dataBeg+i];
                TTN_response[i] = 0;
                log_d("LMIC.rssi: %d LMIC.snr: %d TTN_response: %s\n",LMIC.rssi, LMIC.snr, TTN_response);
                
                lorawan_event_data data{
                    .event = ev,
                    .data = LMIC.frame + LMIC.dataBeg,
                    .data_len = LMIC.dataLen,
                    .rssi = LMIC.rssi,
                    .snr = LMIC.snr
                };
                for (auto cb : callbacks) cb->onDownlinkReceived(this, &data);
            }
            // Schedule next transmission
            // Schedule TX every this many seconds (might become longer due to duty
            // cycle limitations).
            // const unsigned TX_INTERVAL = 10;
            // os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
    for (auto cb : callbacks) cb->onEvent(this, ev);
}