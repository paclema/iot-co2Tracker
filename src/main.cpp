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

#include <Co2Tracker.h>
Co2Tracker* co2Tracker = nullptr;


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



void setup() {
  // esp_log_level_set("i2c.master", ESP_LOG_NONE);
  Serial.begin(115200);
  delay(8000);
  
  #ifdef ENABLE_SERIAL_DEBUG
    Serial.setDebugOutput(true);
  #endif

  #ifdef ARDUINO_IOTPOSTBOX_V1
  // // while(!Serial) {}
  power.setup();
  #endif
  
  co2Tracker = new Co2Tracker();
  co2Tracker->setMQTTClient(config.getMQTTClient());
  
  config.addConfig(co2Tracker, "co2Tracker");
  config.begin();

  // topic = config.getDeviceTopic() + "data";
  co2Tracker->setMQTTDeviceTopic(config.getDeviceTopic() + "data");
  // mqttClient = config.getMQTTClient();
 
  config.addDashboardObject("heap_free", getHeapFree);
  config.addDashboardObject("loop", getLoopTime);
  config.addDashboardObject("RSSI", getRSSI);
  config.addDashboardObject("SPIFFS_Usage", getMemoryUsageString);
  config.addDashboardObject("SPIFFS_Free", getMemoryFree);
  config.addDashboardObject("VBat", getVBat);
  config.addDashboardObject("VBus", getVBus);

  
  #ifdef ARDUINO_IOTPOSTBOX_V1
  power.update();
  #endif

  co2Tracker->begin();

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
  co2Tracker->loop();


  #ifdef ARDUINO_IOTPOSTBOX_V1
  power.update();
  #endif



  /// TODO. SET RECONECTION CALLBACK!!
  ///********************************* */
  // Reconnection loop:
  // if (WiFi.status() != WL_CONNECTED) {
  //   config.begin();
  //   networkRestart();
  //   config.configureServer(&server);
  // }

  previousMainLoopMillis = currentLoopMillis;
}
