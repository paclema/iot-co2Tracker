#include "Co2Tracker.h"

Co2Tracker::Co2Tracker()
    : ss(GPS_RX_PIN, GPS_TX_PIN),
      display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET),
      // tft()
{
    // TFT_eSPI tft = TFT_eSPI(); 
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

void Co2Tracker::initOLED(void){
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)){
    ESP_LOGE("OLED", "SSD1306 allocation failed");
    // for (;;); // Don't proceed, loop forever
  }
  display.clearDisplay();
  display.display();

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
    initOLED();
    displayNoData();
    initGPS();
    delay(100);
    initSCD30();

    tft.init();
    // tft.setRotation(3);
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.fillScreen(TFT_BLACK);
    // tft.drawFloat(4658/1000, 3, 160, 120, 6);
    tft.drawString("Starting...", 20, 20,4);
    delay(1000);
    // plotLinear("A00", 0, 160);
    // plotLinear("A1", 1 * d, 160);
    // plotLinear("A2", 2 * d, 160);
    // plotLinear("A3", 3 * d, 160);
    // plotLinear("A4", 4 * d, 160);
    // plotLinear("A5", 5 * d, 160);


    loraSetup();

    // #ifdef ARDUINO_IOTPOSTBOX_V1
    //     power.update();
    // #endif


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

        if (publishLoraWan) publish2TTN();

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
                doc["vBat"] = (float)power->vBatSense.mV/1000;
                doc["vBus"] = (float)power->vBusSense.mV/1000;
                doc["PowerStatus"] = (int)power->getPowerStatus();
                doc["ChargingStatus"] = (int)power->getChargingStatus();
                #endif
                serializeJson(doc, msg_pub);
                // mqttClient->setBufferSize((uint16_t)(msg_pub.length() + 100));  // Only using PubSubClient
                pMQTTClient->publish(topic.c_str(), msg_pub.c_str());
                // Serial.println(msg_pub);
            }
        } else {
            displayNoData();
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
            updateTFT();
            ESP_LOGE("SCD30", "Failed logGPS while CO2 measurement");
        }

        updateTFT();

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
            }
            doc["rssi_STA"] = WiFi.RSSI();
            #ifdef ARDUINO_IOTPOSTBOX_V1
            doc["vBat"] = (float)power->vBatSense.mV/1000;
            doc["vBus"] = (float)power->vBusSense.mV/1000;
            doc["PowerStatus"] = (int)power->getPowerStatus();
            doc["ChargingStatus"] = (int)power->getChargingStatus();
            #endif
            serializeJson(doc, msg_pub);
            // mqttClient->setBufferSize((uint16_t)(msg_pub.length() + 100));  // Only using PubSubClient
            pMQTTClient->publish(topic.c_str(), msg_pub.c_str());
            // Serial.println(msg_pub);

        }
    }
}

void Co2Tracker::updateDisplay(void){
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(3);
  display.print(gpsSpeed);

  display.setCursor(75, 20);
  display.setTextSize(2);
  display.print("km/h");

  display.setTextSize(1);
  display.setCursor(0, 30);
  display.print("Co2:");
  display.setCursor(25, 30);
  display.print(co2);

  display.setTextSize(1);
  display.setCursor(0, 40);
  display.print("vBat:");
  display.setCursor(30, 40);
  display.print(String((float)power.vBatSense.mV/1000,3));

  display.setTextSize(1);
  display.setCursor(0, 50);
  display.print("vBus:");
  display.setCursor(30, 50);
  display.print(String((float)power.vBusSense.mV/1000,3));

  display.setTextSize(1);
  display.setCursor(70, 40);
  display.print("SAT:");
  display.setCursor(95, 40);
  display.print(gpsSat);

  display.setTextSize(1);
  display.setCursor(70, 50);
  display.print("ALT:");
  display.setCursor(95, 50);
  display.print(gpsAltitude, 0);

  display.display();

  tft.fillScreen(TFT_BLACK);
  tft.drawFloat(power.vBatSense.mV/1000, 3, 160, 120, 6);

}



void Co2Tracker::updateTFT(void){
  // display.clearDisplay();
  // display.setCursor(0, 0);
  // display.setTextSize(3);
  // display.print(gpsSpeed);

  // display.setCursor(75, 20);
  // display.setTextSize(2);
  // display.print("km/h");

  // display.setTextSize(1);
  // display.setCursor(0, 30);
  // display.print("Co2:");
  // display.setCursor(25, 30);
  // display.print(co2);

  // display.setTextSize(1);
  // display.setCursor(0, 40);
  // display.print("vBat:");
  // display.setCursor(30, 40);
  // display.print(String((float)power.vBatSense.mV/1000,3));

  // display.setTextSize(1);
  // display.setCursor(0, 50);
  // display.print("vBus:");
  // display.setCursor(30, 50);
  // display.print(String((float)power.vBusSense.mV/1000,3));

  // display.setTextSize(1);
  // display.setCursor(70, 40);
  // display.print("SAT:");
  // display.setCursor(95, 40);
  // display.print(gpsSat);

  // display.setTextSize(1);
  // display.setCursor(70, 50);
  // display.print("ALT:");
  // display.setCursor(95, 50);
  // display.print(gpsAltitude, 0);

  // display.display();


  /*
  TL_DATUM = Top left
  TC_DATUM = Top centre
  TR_DATUM = Top right
  ML_DATUM = Middle left
  MC_DATUM = Middle centre
  MR_DATUM = Middle right
  BL_DATUM = Bottom left
  BC_DATUM = Bottom centre
  BR_DATUM = Bottom right*/

  // tft.fillScreen(TFT_BLACK);
  // tft.drawString("Speed:", 20, 20,4);
  // tft.drawFloat(gpsSpeed, 3, 120, 20, 4);
  tft.drawString("Speed: " + String(gpsSpeed), 20, 20,4);

  tft.drawString("Co2: " + String(co2), 20, 50,4);
  // tft.drawFloat(co2, 100, 100, 4);

  tft.drawString("vBat: " + String(power.vBatSense.mV/1000,3), 20, 80,4);
  // tft.drawFloat(power.vBatSense.mV/1000, 3, 40, 120, 5);

    // Find centre of screen
  // uint16_t x = tft.width()/2;
  // uint16_t y = tft.height()/2;

  // Set datum to Middle Right
  // tft.setTextDatum(MR_DATUM);

  // Set the padding to the maximum width that the digits could occupy in font 4
  // This ensures small numbers obliterate large ones on the screen
  // tft.setTextPadding( tft.textWidth("-88.88", 4) );

  // Creat a random signed floating point number in range -15 to +15
  // float fpn = random(91)/3.0 - 15.0;

  // Draw a floating point number with 2 decimal places with right datum in font 4
  // tft.drawFloat( fpn, 2, x, y, 4);

  // Reset text padding to 0 otherwise all future rendered strings will use it!
  // tft.setTextPadding(0);
  
  // Set datum to Middle Left
  // tft.setTextDatum(ML_DATUM);

  // Draw text with left datum in font 4
  // tft.drawString(" Units", x, y, 4);

Serial.println("Updating TFT");
}

void Co2Tracker::displayNoData(){
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.setTextSize(3);
    display.print("No Data");
    display.display();
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
  updateTFT();
  
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