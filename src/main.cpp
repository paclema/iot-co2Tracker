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

#include <Co2Tracker.h>
Co2Tracker* co2Tracker = nullptr;

// TFT screen and LVGL UI
#include <lvgl.h>
#include <TFT_eSPI.h>
#include <ui.h>

enum { SCREENBUFFER_SIZE_PIXELS = TFT_WIDTH * TFT_HEIGHT / 10 };
static lv_color_t buf [SCREENBUFFER_SIZE_PIXELS];

TFT_eSPI tft = TFT_eSPI( TFT_WIDTH, TFT_HEIGHT ); /* TFT instance */

#if LV_USE_LOG != 0
void my_print(const char * buf) {
  Serial.printf(buf);
  Serial.flush();
}
#endif

void my_disp_flush (lv_display_t *disp, const lv_area_t *area, uint8_t *pixelmap){
  uint32_t w = ( area->x2 - area->x1 + 1 );
  uint32_t h = ( area->y2 - area->y1 + 1 );

  if (LV_COLOR_16_SWAP) {
      size_t len = lv_area_get_size( area );
      lv_draw_sw_rgb565_swap( pixelmap, len );
  }

  tft.startWrite();
  tft.setAddrWindow( area->x1, area->y1, w, h );
  tft.pushColors( (uint16_t*) pixelmap, w * h, true );
  tft.endWrite();

  lv_disp_flush_ready( disp );
}

static uint32_t my_tick_get_cb (void) { return millis(); }

#ifdef ARDUINO_IOTPOSTBOX_V1
#include "PowerManagement.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#define POWER_UI_UPDATE_DELAY_MS 1000
PowerManagement power;
TaskHandle_t powerChargeCbTaskHandle = nullptr;

// Task to update the battery icon alpha on charging status change
void powerChargeCbTask(void *pvParameters) {
  (void)pvParameters;
  PowerStatus lastStatus = power.getPowerStatus();
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait for notification
    PowerStatus currentPowerStatus = power.getPowerStatus();
    if (currentPowerStatus != lastStatus) {
      lastStatus = currentPowerStatus;
      lv_lock();
      if (currentPowerStatus != PowerStatus::BatteryPowered) {
          lv_obj_set_style_opa(ui_batImg, 255, LV_PART_MAIN | LV_STATE_DEFAULT); // Brillante
          
      } else {
          lv_obj_set_style_opa(ui_batImg, 100, LV_PART_MAIN | LV_STATE_DEFAULT); // Menos brillo
      }
      lv_unlock();
    }
  }
}

// Callback to notify the icon task from PowerManagement
void onChargingStatusChanged(void) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (powerChargeCbTaskHandle) {
    vTaskNotifyGiveFromISR(powerChargeCbTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

// FreeRTOS task to update power and LVGL label
void powerUpdateTask(void *pvParameters) {
  (void)pvParameters;
  char batStr[8];
  for (;;) {
    power.update();
    snprintf(batStr, sizeof(batStr), "%1.2fv", (float)power.vBatSense.mV / 1000.0);
    lv_lock();
    lv_label_set_text(ui_batVal, batStr);
    lv_unlock();
    vTaskDelay(pdMS_TO_TICKS(POWER_UI_UPDATE_DELAY_MS));
  }
}
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



void setup() {
  // esp_log_level_set("i2c.master", ESP_LOG_NONE);
  Serial.begin(115200);


  Serial.println("###  LVGL setup\n");
  Serial.printf("LVGL V%d.%d.%d\n", lv_version_major(), lv_version_minor(), lv_version_patch());
  lv_init();

#if LV_USE_LOG != 0
  lv_log_register_print_cb( my_print ); /* register print function for debugging */
#endif

  tft.begin();          /* TFT init */
  tft.setRotation( TFT_ROTATION ); /* Landscape orientation, flipped */

  static lv_disp_t* disp;
  disp = lv_display_create( TFT_WIDTH, TFT_HEIGHT );
  lv_display_set_buffers( disp, buf, NULL, SCREENBUFFER_SIZE_PIXELS * sizeof(lv_color_t), LV_DISPLAY_RENDER_MODE_PARTIAL );
  lv_display_set_flush_cb( disp, my_disp_flush );

  lv_tick_set_cb( my_tick_get_cb );

  ui_init();
  lv_timer_handler(); // Call LVGL timer handler first time to refresh the screen earlier, before setup completes ?

  // delay(8000);
  
  #ifdef ENABLE_SERIAL_DEBUG
    Serial.setDebugOutput(true);
  #endif

  #ifdef ARDUINO_IOTPOSTBOX_V1
  // // while(!Serial) {}
  power.setup();
  power.update();
  power.addChargingStatusCallback(onChargingStatusChanged);
  xTaskCreatePinnedToCore(
    powerUpdateTask,
    "PowerUpdateTask",
    4096,
    NULL,
    4,
    NULL,
    0
  );
  xTaskCreatePinnedToCore(
    powerChargeCbTask,
    "powerChargeCbTask",
    2048,
    NULL,
    4,
    &powerChargeCbTaskHandle,
    0
  );
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

  

  co2Tracker->begin();

  Serial.println("###  Looping time\n");

}

void loop() {

  lv_timer_handler();

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
