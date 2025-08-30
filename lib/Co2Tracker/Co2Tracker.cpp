#include "Co2Tracker.h"


void Co2Tracker::gpsTask(void* pvParameters) {
    Co2Tracker* self = static_cast<Co2Tracker*>(pvParameters);
    for (;;) {
        while (self->ss.available() > 0) {
            self->gps.encode(self->ss.read());
        }
        vTaskDelay(1);
    }
}

Co2Tracker::Co2Tracker()
    : ss(GPS_RX_PIN, GPS_TX_PIN)
{
    lorawan.addCallback(this);

    lv_obj_set_style_img_opa(ui_loraImg, 100, LV_PART_MAIN);
    lv_obj_set_style_img_opa(ui_mqttImg, 100, LV_PART_MAIN);

}

void Co2Tracker::initSCD30(void){
  Wire.begin();

  //Start sensor using the Wire port and enable the auto-calibration (ASC)
  if (airSensor.begin(Wire, true) == false)
  {
      ESP_LOGE("SCD30", "Air sensor not detected. Please check wiring. Freezing...");
      // while (1)
      //     ;
  }

  airSensor.setMeasurementInterval(2); //Change number of seconds between measurements: 2 to 1800 (30 minutes)


  Serial.print("Auto calibration set to ");
  if (airSensor.getAutoSelfCalibration() == true)
      Serial.println("true");
  else
      Serial.println("false");

  //The SCD30 has data ready every two seconds

  //Read altitude compensation value
  unsigned int altitude = airSensor.getAltitudeCompensation();
  Serial.print("Current altitude: ");
  Serial.print(altitude);
  Serial.println("m");

  //My desk is ~116m above sealevel
  airSensor.setAltitudeCompensation(116); //Set altitude of the sensor in m, stored in non-volatile memory of SCD30

  //Pressure in Boulder, CO is 24.65inHg or 834.74mBar
  // airSensor.setAmbientPressure(835); //Current ambient pressure in mBar: 700 to 1200, will overwrite altitude compensation

  //Read temperature offset
  float offset = airSensor.getTemperatureOffset();
  Serial.print("Current temp offset: ");
  Serial.print(offset, 2);
  Serial.println("C");

  //airSensor.setTemperatureOffset(5); //Optionally we can set temperature offset to 5°C, stored in non-volatile memory of SCD30

}

void Co2Tracker::initGPS(void){
  Serial.print(F("TinyGPS++ library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println();

  ss.begin(GPSBaud);
}

void Co2Tracker::setMQTTClient(MQTTClient * client) {
    pMQTTClient = client;
    if (pMQTTClient) pMQTTClient->addCallback(this);
}

void Co2Tracker::begin() {
  #ifdef ARDUINO_IOTPOSTBOX_V1
      // while(!Serial) {}
      pinMode(LDO2_EN_PIN, OUTPUT);
      digitalWrite(LDO2_EN_PIN, HIGH);
      // power.setup();
  #endif

  // SCD30 and GPS setup:
  initGPS();
  initSCD30();

  if (gpsTaskHandle == nullptr) {
    xTaskCreatePinnedToCore(
      gpsTask,
      "gpsTask",
      2048,
      this,
      2,
      &gpsTaskHandle,
      0
    );
  }

  lorawan.begin();
}

void Co2Tracker::parseWebConfig(JsonObjectConst configObject) {
  localLogs = configObject["local_logs"] | false;
  publishGPSdata = configObject["publish_gps_data"] | false;
  publishLoraWan = configObject["publish_LoRaWAN"] | true;

  Serial.printf("Co2Tracker: localLogs: %s, publishGPSdata: %s, publishLoraWan: %s\n", 
                localLogs ? "true" : "false", 
                publishGPSdata ? "true" : "false", 
                publishLoraWan ? "true" : "false");
}

void Co2Tracker::onConnected(MQTTClient* client) {
  // String msg = "{\"status\":\"connected\"}";
  // String topic = String(pMQTTClient->getBaseTopic().c_str()) + "/status";

  String msg = "connected = true";
  client->publish(pMQTTClient->getBaseTopic().c_str(), msg.c_str());

  lv_obj_set_style_img_opa(ui_mqttImg, 255, LV_PART_MAIN);

}

void Co2Tracker::onDisconnected(MQTTClient* client) {
  lv_obj_set_style_img_opa(ui_mqttImg, 100, LV_PART_MAIN);
}

bool Co2Tracker::GPSDataValid() {
  if (gps.location.isValid() 
    && gps.location.lat() != 0 && gps.location.lng() != 0
    && gps.date.isValid() && gps.time.isValid()
    // && gps.date.year() == 2022 && ( gps.date.month() == 7 || gps.date.month() == 8 )
  ){
      return true;
  }
  return false;
}

void Co2Tracker::publishGPSData(void) {
  if (GPSDataValid()) {
    if (publishLoraWan) sendLoraBinary();
    if (localLogs) logGPS();
    if (pMQTTClient && pMQTTClient->connected()) publishMQTT(false, true);
  }
}


void Co2Tracker::publishMQTT(bool publishCo2, bool publishGPS) {
  String msg_pub;
  StaticJsonDocument<256> doc;

  if (publishCo2) {
    doc["CO2"] = co2Data.co2;
    doc["temp"] = co2Data.temp;
    doc["humidity"] = co2Data.hum;
  }
  if (publishGPS) {
    doc["lat"] = gps.location.lat();
    doc["lng"] = gps.location.lng();
    doc["date"] = gps.date.value();
    doc["time"] = gps.time.value() + gps.time.age()/10 + timeZoneoffset*1000000;
    doc["speed"] = gps.speed.kmph();
    doc["satellites"] = gps.satellites.value();
    doc["altitude"] = gps.altitude.meters();
    doc["hdop"] = gps.hdop.hdop();
    doc["course"] = gps.course.deg();
  }
  doc["rssi_STA"] = WiFi.RSSI();
  #ifdef ARDUINO_IOTPOSTBOX_V1
  doc["vBat"] = (float)power.vBatSense.mV/1000;
  doc["vBus"] = (float)power.vBusSense.mV/1000;
  doc["PowerStatus"] = (int)power.getPowerStatus();
  doc["ChargingStatus"] = (int)power.getChargingStatus();
  #endif
  serializeJson(doc, msg_pub);
  // mqttClient->setBufferSize((uint16_t)(msg_pub.length() + 100));  // Only using PubSubClient
  pMQTTClient->publish(topic.c_str(), msg_pub.c_str());
  // Serial.println(msg_pub);
}

void Co2Tracker::loop() {
  unsigned long currentMillis = millis();

  // GPS UI Update
  bool satellitesUpdated = gps.satellites.isUpdated();
  bool speedUpdated = gps.speed.isUpdated();
  bool altitudeUpdated = gps.altitude.isUpdated();
  bool courseUpdated = gps.course.isUpdated();

  // Update UI GPS elements only if their respective values are updated
  if (satellitesUpdated)  lv_label_set_text(ui_gpsSat, String(gps.satellites.value()).c_str());
  if (speedUpdated)       lv_label_set_text(ui_speedValue, String(gps.speed.kmph()).c_str());
  if (altitudeUpdated)    lv_label_set_text(ui_altitudeValue, String(gps.altitude.meters()).c_str());
  if (courseUpdated) {
    lv_label_set_text(ui_courseValue, String(gps.course.deg()).c_str());
    lv_img_set_angle(ui_gpsCourseArrow, (int16_t)(gps.course.deg() * 10));
  }

  if ( satellitesUpdated || speedUpdated || altitudeUpdated || courseUpdated) {
    String updatedFields;
    if (satellitesUpdated)  updatedFields += "satellites ";
    if (speedUpdated)       updatedFields += "speed ";
    if (altitudeUpdated)    updatedFields += "altitude ";
    if (courseUpdated)      updatedFields += "course ";
    ESP_LOGD("Co2Tracker", "GPS UI Updates: %s", updatedFields.c_str());
  }

  
  // GPS data publishing
  if (currentMillis - lastGPSPublish > GPS_DATA_PUBLISH_TIME) {
      lastGPSPublish = currentMillis;
      Co2Tracker::publishGPSData();
  }


  // Co2 Sensor update
  if (airSensor.dataAvailable()) {
    co2Data.co2 = airSensor.getCO2();
    co2Data.temp = airSensor.getTemperature();
    co2Data.hum = airSensor.getHumidity();
    airSensorFirstMeasurement = true;

    Serial.print("co2(ppm):");
    Serial.print(co2Data.co2);
    Serial.print(" temp(C):");
    Serial.print(co2Data.temp, 1);
    Serial.print(" humidity(%):");
    Serial.print(co2Data.hum, 1);
    Serial.println();

    lv_label_set_text(ui_co2Value, String(co2Data.co2).c_str());
    lv_label_set_text(ui_tempValue, String(co2Data.temp).c_str());
    lv_label_set_text(ui_humValue, String(co2Data.hum).c_str());

    if (pMQTTClient && pMQTTClient->connected()) publishMQTT(true, GPSDataValid());
  }

  lorawan.loop();
}


void Co2Tracker::logGPS(void){
  std::stringstream fileName;
  char timeBuffer[16];

  // setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year());
  // adjustTime(timeZoneoffset * SECS_PER_HOUR);

  // updateDisplay();
  // updateTFT();
  
  // Create file if it does not exists
  fileName << "/GPS_" << (int)gps.date.year() << "_" << (int)gps.date.month()<< "_" << (int)gps.date.day() << ".csv";
  if( !LittleFS.exists( fileName.str().c_str()) ) {
    File file = LittleFS.open( fileName.str().c_str(), FILE_WRITE);
    if(!file){
      ESP_LOGE("logGPS","logGPS", "Failed to create file %s", fileName.str().c_str());
      return;
    }
    ESP_LOGI("logGPS", "Created log file %s", fileName.str().c_str());
    file.println("time,latitude,longitude,altitude,speed,hdop,satellites,course,vBat,vBus,PowerStatus,ChargingStatus,co2,temp,hum");
  }

  // Open the file to append new line
  File file = LittleFS.open(fileName.str().c_str(), FILE_APPEND);
  if(!file) {
    ESP_LOGE("logGPS","Failed to open file %s for appending", fileName.str().c_str());
    return;
  }

  // write file and position
  String newLine;
  newLine += String((uint32_t)gps.time.isValid() ? (gps.time.value() + gps.time.age()/10 + timeZoneoffset*1000000) : 0) + ",";
  newLine += String((double)gps.location.isValid()?gps.location.lat():0.0f, 8) + ",";
  newLine += String((double)gps.location.isValid()?gps.location.lng():0.0f, 8) + ",";
  newLine += String((double)gps.altitude.isValid()?gps.altitude.meters():0.0f, 1) + ",";
  newLine += String((double)gps.speed.isValid()?gps.speed.kmph():0.0f, 3) + ",";
  newLine += String((double)gps.hdop.isValid()?gps.hdop.hdop():0.0f, 2) + ",";
  newLine += String((int)gps.satellites.isValid()?gps.satellites.value():0) + ",";
  newLine += String((double)gps.course.isValid()?gps.course.deg():0, 1) + ",";
  newLine += String((float)(power.vBatSense.mV/1000), 5) + ",";
  newLine += String((float)(power.vBusSense.mV/1000), 5) + ",";
  newLine += String((int)power.getPowerStatus()) + ",";
  newLine += String((int)power.getChargingStatus()) + ",";
  newLine += String((uint16_t)co2Data.co2) + ",";
  newLine += String((float)co2Data.temp, 2) + ",";
  newLine += String((float)co2Data.hum, 2);
  file.println(newLine.c_str());
  file.close();

}


void Co2Tracker::sendLoraBinary() {
  uint8_t payload[34];
  int index = 0;

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

  payload[index++] = (co2Data.co2 >> 8) & 0xFF;
  payload[index++] = co2Data.co2 & 0xFF;

  uint16_t tempUint = (uint16_t)(co2Data.temp * 100);  // 2 decimals. Max number: 655.35
  payload[index++] = (tempUint >> 8) & 0xFF;
  payload[index++] = tempUint & 0xFF;

  uint16_t humUint = (uint16_t)(co2Data.hum * 100);  // 2 decimals. Max number: 655.35
  payload[index++] = (humUint >> 8) & 0xFF;
  payload[index++] = humUint & 0xFF;
    
  payload[index++] = (uint8_t)power.getPowerStatus();
  payload[index++] = (uint8_t)power.getChargingStatus();

  if(lorawan.send(payload, index)) {
    // lv_obj_set_style_img_opa(ui_loraImg, 255, LV_PART_MAIN);
    ESP_LOGI("Co2Tracker", "LoRaWAN packet queued");
  } else {
    lv_obj_set_style_img_opa(ui_loraImg, 100, LV_PART_MAIN);
    ESP_LOGW("Co2Tracker", "LoRaWAN transmission failed");
  }
  Serial.printf("Packet queued: size=%d (must be <= %zu)\n", index, sizeof(payload));
  Serial.print("Payload: ");
  for (int i = 0; i < sizeof(payload); i++) {
      Serial.print(payload[i], HEX);
      Serial.print(" ");
  }
  Serial.println();
}