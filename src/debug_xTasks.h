
#include <Arduino.h>

#ifdef ESP32
  #include "esp_heap_caps.h"
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
  #include "esp_system.h"

  static unsigned long prevMemLogMs = 0;
  static const unsigned long MEM_LOG_EVERY_MS = 5000;
  static unsigned long lastLoopTickMs = 0;
#endif



static void logMemoryStats(){
#ifdef ESP32
  size_t free8     = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  size_t minfree8  = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
  size_t largest   = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  size_t freeHeap  = ESP.getFreeHeap();
  size_t minHeap   = ESP.getMinFreeHeap();
  size_t freePsram = ESP.getFreePsram();
  UBaseType_t hwm  = uxTaskGetStackHighWaterMark(NULL); // palabras (4B)

  Serial.printf("[mem] heap_free=%uB heap_min=%uB largest=%uB psram_free=%uB stack_hwm=%u words\n",
                (unsigned)freeHeap, (unsigned)minHeap, (unsigned)largest, (unsigned)freePsram, (unsigned)hwm);

  Serial.printf( "Task Name\tStatus\tPrio\tHWM\tTask\tAffinity\n");
  char stats_buffer[1024];
  vTaskList(stats_buffer);
  // Print in chunks of 200 characters to avoid overflowing Serial buffer
  const size_t chunkSize = 20;
  size_t len = strlen(stats_buffer);
  for (size_t i = 0; i < len; i += chunkSize) {
    Serial.write(stats_buffer + i, min(chunkSize, len - i));
  }
  // Serial.printf("%s\n", stats_buffer);
  Serial.println();
  // free(buffer);

  char statsTime_buffer[1024];
  vTaskGetRunTimeStats(statsTime_buffer);
  // Print in chunks of 200 characters to avoid overflowing Serial buffer
  size_t len2 = strlen(statsTime_buffer);
  for (size_t i = 0; i < len2; i += chunkSize) {
    Serial.write(statsTime_buffer + i, min(chunkSize, len2 - i));
  }
  Serial.println();



#else
  Serial.printf("[mem] heap_free=%uB\n", (unsigned)ESP.getFreeHeap());
#endif
}


void calculateLoopLag() {
#ifdef ESP32
  unsigned long now = millis();
  if (lastLoopTickMs != 0) {
    unsigned long lag = now - lastLoopTickMs;
    if (lag > 500) {
      Serial.printf("[loop] lag=%lums\n", lag);
    }
  }
  lastLoopTickMs = now;
#endif
}