#include "Co2Tracker.h"

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

void Co2Tracker::loop() {

  // Check GPS location:
  while (ss.available() > 0)
      gps.encode(ss.read());

  if (gps.charsProcessed() < 10)
      Serial.println(F("WARNING: No GPS data.  Check wiring."));

  unsigned long currentMillis = millis();

  if (currentMillis - lastGPSPublish > GPS_DATA_PUBLISH_TIME) {
      lastGPSPublish = currentMillis;

      // if (gps.location.isUpdated()){
      //   Serial.printf("---> NEW GPS location: %lf - %lf\n", gps.location.lat(), gps.location.lng());
      // } 
      // if (gps.speed.isUpdated()){
      //   Serial.printf("---> NEW GPS speed: %lf\n", gps.speed.kmph());
      // }

      if (publishLoraWan) sendLoraBinary();

      if (gps.location.isValid() && gps.location.lat() != 0 && gps.location.lng() != 0 && gps.date.isValid() && gps.time.isValid()) {
          if (localLogs) logGPS();

          // Serial.printf("Lat: %lf - Long: %lf - Date: %zu - Time: %zu - Spped: %lf km/h\n", lat, lng, gpsDate, gpsTime, gpsSpeed);
          // Serial.printf("****** - Time: %zu secs: %d -age %d- Time+age: %d \n", gpsTime, gps.time.second(),  gps.time.age(), (gpsTime + gps.time.age()/10));
          // Serial.printf("Satellites: %zu - Altitude: %lf - Hdop: %lf - Course: %lf\n", gpsSat, gpsAltitude, gpsHdop, gpsCourse);

          if (pMQTTClient && pMQTTClient->connected()) {
              String msg_pub;
              StaticJsonDocument<256> doc;

              doc["lat"] = lat;
              doc["lng"] = lng;
              doc["date"] = gpsDate;
              doc["time"] = gpsTime;
              doc["speed"] = gpsSpeed;
              doc["satellites"] = gpsSat;
              doc["altitude"] = gpsAltitude;
              doc["hdop"] = gpsHdop;
              doc["course"] = gpsCourse;
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
      } else {
          // displayNoData();
      }
  }

  if (airSensor.dataAvailable()) {
      co2 = airSensor.getCO2();
      temp = airSensor.getTemperature();
      hum = airSensor.getHumidity();
      airSensorFirstMeasurement = true;

      Serial.print("co2(ppm):");
      Serial.print(co2);
      Serial.print(" temp(C):");
      Serial.print(temp, 1);
      Serial.print(" humidity(%):");
      Serial.print(hum, 1);
      Serial.println();

      bool GPSdataValid = false;
      if (gps.location.isValid() && gps.location.lat() != 0 && gps.location.lng() != 0 && gps.date.isValid() && gps.time.isValid() 
      // && gps.date.year() == 2022 && (gps.date.month() == 7 || gps.date.month() == 8)){
      ){
          logGPS();
          GPSdataValid = true;
          // Serial.printf("Lat: %lf - Long: %lf - Date: %zu - Time: %zu - Spped: %lf km/h\n", lat, lng, gpsDate, gpsTime, gpsSpeed);
          // Serial.printf("****** - Time: %zu secs: %d -age %d- Time+age: %d \n", gpsTime, gps.time.second(),  gps.time.age(), (gpsTime + gps.time.age()/10));
          // Serial.printf("Satellites: %zu - Altitude: %lf - Hdop: %lf - Course: %lf\n", gpsSat, gpsAltitude, gpsHdop, gpsCourse);
      } else {
          // updateTFT();
          ESP_LOGE("SCD30", "Failed logGPS while CO2 measurement");
      }

      // updateTFT();

      if (pMQTTClient && pMQTTClient->connected()) {
          String msg_pub;
          StaticJsonDocument<256> doc;

          doc["CO2"] = co2;
          doc["temp"] = temp;
          doc["humidity"] = hum;

          if (GPSdataValid && publishGPSdata) {
              doc["lat"] = lat;
              doc["lng"] = lng;
              doc["date"] = gpsDate;
              doc["time"] = gpsTime;
              doc["speed"] = gpsSpeed;
              doc["satellites"] = gpsSat;
              doc["altitude"] = gpsAltitude;
              doc["hdop"] = gpsHdop;
              doc["course"] = gpsCourse;

              String text = "Sat: " + String(gpsSat);
              lv_label_set_text(ui_sat, text.c_str());

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

          lv_label_set_text(ui_leftMainDataValue, String(co2).c_str());

      }
  }

  lorawan.loop();
}


void Co2Tracker::logGPS(void){
  std::stringstream fileName;
  char timeBuffer[16];
  
  lat = gps.location.lat();
  lng = gps.location.lng();
  gpsDate = gps.date.value();
  gpsTime = gps.time.value() + gps.time.age()/10 + timeZoneoffset*1000000;
  gpsSpeed = gps.speed.kmph();

  gpsSat = gps.satellites.value();
  gpsAltitude = gps.altitude.meters();
  gpsHdop = gps.hdop.hdop();
  gpsCourse = gps.course.deg();

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
  newLine += String((uint32_t)gps.time.isValid()?gpsTime:0) + ",";
  newLine += String((double)gps.location.isValid()?lat:0.0f, 8) + ",";
  newLine += String((double)gps.location.isValid()?lng:0.0f, 8) + ",";
  newLine += String((double)gps.altitude.isValid()?gpsAltitude:0.0f, 1) + ",";
  newLine += String((double)gps.speed.isValid()?gpsSpeed:0.0f, 3) + ",";
  newLine += String((double)gps.hdop.isValid()?gpsHdop:0.0f, 2) + ",";
  newLine += String((int)gps.satellites.isValid()?gpsSat:0) + ",";
  newLine += String((double)gps.course.isValid()?gpsCourse:0, 1) + ",";
  newLine += String((float)(power.vBatSense.mV/1000), 5) + ",";
  newLine += String((float)(power.vBusSense.mV/1000), 5) + ",";
  newLine += String((int)power.getPowerStatus()) + ",";
  newLine += String((int)power.getChargingStatus()) + ",";
  newLine += String((uint16_t)co2) + ",";
  newLine += String((float)temp, 2) + ",";
  newLine += String((float)hum, 2);
  file.println(newLine.c_str());
  file.close();

}

void Co2Tracker::sendLoraCayenne() {
  CayenneLPP lpp(CAYENNE_MAX_PAYLOAD_SIZE);
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



  if(lorawan.send(lpp.getBuffer(), lpp.getSize()))
    // lv_obj_set_style_img_opa(ui_loraImg, 255, LV_PART_MAIN);
    ESP_LOGI("Co2Tracker", "LoRaWAN packet queued");
  else {
    lv_obj_set_style_img_opa(ui_loraImg, 100, LV_PART_MAIN);
    ESP_LOGW("Co2Tracker", "LoRaWAN transmission failed");
  }


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
    
}

void Co2Tracker::sendLoraBinary() {
  uint8_t payload[34];
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



  // Codifica los datos como hacías en publish2TTN()
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

  if(lorawan.send(payload, index)) {
    // lv_obj_set_style_img_opa(ui_loraImg, 255, LV_PART_MAIN);
    ESP_LOGI("Co2Tracker", "LoRaWAN packet queued");
  else {
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