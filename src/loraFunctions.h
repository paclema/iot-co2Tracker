#ifndef LoraFunctions_H
#define LoraFunctions_H
#pragma once

#include "Arduino.h"
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>


// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]={ PASTE_LSB };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ PASTE_LSB };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { PASTE_MSB };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static uint8_t mydata[] = "Hello, world from an iot-postbox_v1 board!";
static osjob_t sendjob;
uint8_t payload[34];

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 10;

char TTN_response[30];


#include <CayenneLPP.h>
#define CAYENNE_MAX_PAYLOAD_SIZE    64
CayenneLPP lpp(CAYENNE_MAX_PAYLOAD_SIZE);


class MyHalConfig_t : public Arduino_LMIC::HalConfiguration_t {

public:
  MyHalConfig_t(){};

  // set SPI pins to board configuration, pins may come from pins_arduino.h
  virtual void begin(void) override {
    SPI.begin(RFM95W_SCK_PIN, RFM95W_MISO_PIN, RFM95W_MOSI_PIN, RFM95W_NSS_PIN);
  }
};

static MyHalConfig_t myHalConfig{};
static const lmic_pinmap myPinmap = {
    .nss = RFM95W_NSS_PIN,
    .rxtx = LMIC_UNUSED_PIN,
  	.rst = RFM95W_RESET_PIN,
	.dio = {/*dio0*/ RFM95W_DIO0_PIN, /*dio1*/ RFM95W_DIO1_PIN, /*dio2*/ RFM95W_DIO2_PIN},
    // .rxtx_rx_active = LMIC_UNUSED_PIN,
    // .rssi_cal = 10,
    // .spi_freq = 8000000, // 8MHz
    .pConfig = &myHalConfig
};



void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.

        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}


void onEvent (ev_t ev) {
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
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
              Serial.println();
            }

            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
	    	// size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
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
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
              int i = 0;
              for ( i = 0 ; i < LMIC.dataLen ; i++ )
                TTN_response[i] = LMIC.frame[LMIC.dataBeg+i];
              TTN_response[i] = 0;
			  log_d("LMIC.rssi: %d LMIC.snr: %d TTN_response: %s\n",LMIC.rssi, LMIC.snr, TTN_response);
            
            }
            // Schedule next transmission
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
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
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
}


void loraSetup(){

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

void loraLoop() {
    os_runloop_once();
}

void publish2TTNCayenne(void){
    
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.printf("*** OP_TXRXPEND, not sending\n");
    } else {
        // Prepare upstream data transmission at the next possible time.


        lpp.reset();

        // lpp.addAnalogInput(1, (float)wakeUpGPIO);
        // lpp.addAnalogInput(2, (float)bootCount);

        // lpp.addVoltage(1, power.vBatSense.mV/1000:F); //VCC
        lpp.addVoltage(2, power.vBatSense.mV/1000.F); //vBat
        lpp.addVoltage(3, power.vBusSense.mV/1000.F); //vBus


        // CO2 sensor:
        //------------
        // if(airSensor.dataAvailable()){
        //   lpp.addConcentration(1, (uint32_t)airSensor.getCO2());
        //   lpp.addTemperature(1, airSensor.getTemperature());
        //   lpp.addRelativeHumidity(1, airSensor.getHumidity());
        // }
        if(airSensorFirstMeasurement){
          lpp.addConcentration(1, (uint32_t)co2);
          lpp.addTemperature(1, temp);
          lpp.addRelativeHumidity(1, hum);
        }


        // GPS sensor:
        //------------
        if(gps.location.isValid() && gps.altitude.isValid()){
          lpp.addGPS(1, float(gps.location.lat()), float(gps.location.lng()), float(gps.altitude.meters()));
          Serial.printf("GPS lat: %lf lng: %lf\n", gps.location.lat(),  gps.location.lng());
        }
        if(gps.speed.isValid()) lpp.addGenericSensor(1, gps.speed.kmph());
        if(gps.satellites.isValid()) lpp.addGenericSensor(2, gps.satellites.value());
        if(gps.hdop.isValid()) lpp.addPercentage(1, gps.hdop.hdop());
        if(gps.course.isValid()) lpp.addDirection(1, (float)gps.course.deg());
        if(gps.time.isValid()) lpp.addUnixTime(1, gps.time.value() + gps.time.age()/10 + timeZoneoffset*1000000);




        // Serial.printf("lpp buffer: %.*s\n", lpp.getSize(), (char*)lpp.getBuffer());
        uint8_t *payload = lpp.getBuffer();

        char buffer[128];
        String payloadString;

        for (int i = 0; i < lpp.getSize(); i++) {
        sprintf(buffer, "%02x", payload[i]);
        payloadString += buffer;
        }
        Serial.print("HEX: ");
        Serial.print(payloadString);
        Serial.print(" | SIZE: ");
        Serial.println(payloadString.length());

        Serial.printf("*** Packet queued:\n");
        DynamicJsonDocument jsonBuffer(1024);
        JsonObject root = jsonBuffer.to<JsonObject>();
        lpp.decodeTTN(lpp.getBuffer(), lpp.getSize(), root);
        serializeJsonPretty(root, Serial);
        Serial.println();

        LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
        // LMIC_setTxData2(1, mydataPostbox, sizeof(mydataPostbox)-1, 0);
        // Serial.printf("*** Packet queued: %s\n", mydataPostbox);
    }
    // Next TX is scheduled after TX_COMPLETE event.
}


void publish2TTN(void){
    
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.printf("*** OP_TXRXPEND, not sending\n");
    } else {
        // Prepare upstream data transmission at the next possible time.
        int index = 0;


        // float vBat = power.vBatSense.mV/1000.F;
        // float vBus = power.vBusSense.mV/1000.F;

        // CO2 sensor:
        //------------
        // uint16_t co2 = co2;
        // float temp = temp;
        // float hum = hum;

        // GPS sensor:
        //------------
        // double lat = gps.location.lat();
        // double lng = gps.location.lng();
        // double altitude = gps.altitude.meters();
        // double gpsSpeed = gps.speed.kmph();
        // uint32_t gpsSat = gps.satellites.value();
        // double gpsHdop = gps.hdop.hdop();
        // double gpsCourse = gps.course.deg();
        // uint32_t gpsTime = gps.time.value() + gps.time.age() / 10 + timeZoneoffset * 1000000;

        /*
        uint32_t latitudeBinary = ((gps.location.lat() + 90) / 180.0) * 16777215;
        uint32_t longitudeBinary = ((gps.location.lng() + 180) / 360.0) * 16777215;
  

        payload[0] = (latitudeBinary >> 16) & 0xFF;
        payload[1] = (latitudeBinary >> 8) & 0xFF;
        payload[2] = latitudeBinary & 0xFF;

        payload[3] = (longitudeBinary >> 16) & 0xFF;
        payload[4] = (longitudeBinary >> 8) & 0xFF;
        payload[5] = longitudeBinary & 0xFF;

        uint16_t altitudeGps = gps.altitude.meters();
        payload[6] = (altitudeGps >> 8) & 0xFF;
        payload[7] = altitudeGps & 0xFF;

        uint8_t hdopGps = gps.hdop.hdop()/10;
        payload[8] = hdopGps & 0xFF;

        */

        uint32_t latitudeBinary = ((gps.location.lat() + 90) / 180.0) * 16777215;
        payload[index++] = (latitudeBinary >> 24) & 0xFF;
        payload[index++] = (latitudeBinary >> 16) & 0xFF;
        payload[index++] = (latitudeBinary >> 8) & 0xFF;
        payload[index++] = latitudeBinary & 0xFF;

        uint32_t longitudeBinary = ((gps.location.lng() + 180) / 360.0) * 16777215;
        payload[index++] = (longitudeBinary >> 24) & 0xFF;
        payload[index++] = (longitudeBinary >> 16) & 0xFF;
        payload[index++] = (longitudeBinary >> 8) & 0xFF;
        payload[index++] = longitudeBinary & 0xFF;

        uint16_t altitudeGps = gps.altitude.meters();
        payload[index++] = (altitudeGps >> 8) & 0xFF;
        payload[index++] = altitudeGps & 0xFF;

        uint16_t hdopGps = gps.hdop.hdop();
        payload[index++] = (hdopGps >> 8) & 0xFF;
        payload[index++] = hdopGps & 0xFF;


        double gpsSpeed = gps.speed.kmph();
        uint16_t gpsSpeedInt = gpsSpeed*100;
        payload[index++] = (gpsSpeedInt >> 8) & 0xFF;
        payload[index++] = gpsSpeedInt & 0xFF;


        uint8_t gpsSat8 = (uint8_t)(gps.satellites.value() & 0xFF);
        payload[index++] = gpsSat8;

        uint16_t gpsCourse = gps.course.deg();
        payload[index++] = (gpsCourse >> 8) & 0xFF;
        payload[index++] = gpsCourse & 0xFF;

        uint32_t gpsTime = gps.time.value() + gps.time.age()/10 + timeZoneoffset*1000000; // Raw time in HHMMSSCC format (u32)
        payload[index++] = (gpsTime >> 24) & 0xFF;
        payload[index++] = (gpsTime >> 16) & 0xFF;
        payload[index++] = (gpsTime >> 8) & 0xFF;
        payload[index++] = gpsTime & 0xFF;


        uint16_t vBat = power.vBatSense.mV;
        payload[index++] = (vBat >> 8) & 0xFF;
        payload[index++] = vBat & 0xFF;

        uint16_t vBus = power.vBusSense.mV;
        payload[index++] = (vBus >> 8) & 0xFF;
        payload[index++] = vBus & 0xFF;

        payload[index++] = (co2 >> 8) & 0xFF;
        payload[index++] = co2 & 0xFF;


        uint16_t tempUint = (uint16_t)(temp * 100);  // 2 decimals. Max number: 655.35
        payload[index++] = (tempUint >> 8) & 0xFF;
        payload[index++] = tempUint & 0xFF;


        uint16_t humUint = (uint16_t)(hum * 100);  // 2 decimals. Max number: 655.35
        payload[index++] = (humUint >> 8) & 0xFF;
        payload[index++] = humUint & 0xFF;
        
        payload[index++] = (uint8_t)power.getPowerStatus();
        payload[index++] = (uint8_t)power.getChargingStatus();

        LMIC_setTxData2(1, payload, index, 0);
        Serial.print("Packet queued: size=");
        Serial.println(index);


        // Generate payload from message:


        // Print the payload and its size to the serial port
        Serial.print("Payload: ");
        for (int i = 0; i < sizeof(payload); i++) {
            Serial.print(payload[i], HEX);
            Serial.print(" ");
        }


    }
    // Next TX is scheduled after TX_COMPLETE event.
}

#endif