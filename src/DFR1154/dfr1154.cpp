#include <Arduino.h>
#include <DHT.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <mbedtls/md.h>
#include <mbedtls/sha256.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include "esp_camera.h"
#include "esp_wifi.h"
#include "image_analyzer.h"
#include "secrets.h"  // WiFi credentials (not in git)
#include "status_led.h"
#include "common.h"
#include "aws_iot.h"
#include "app_httpd.h"
#include "json_config.h"
#include "ambient.h"


#ifdef DFR1154

const char* deviceName = "DFR";

static DebounceTimer cameraAction;
static StatusLed statusLed(STATUS_LED_PIN);

void WiFiStatusCallback(WiFiEvent_t event, WiFiEventInfo_t info) {
  
  switch(event) {
    case WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      statusLed.Start(std::vector<int>{20, 180}, true);
      break;

    case WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED:
      statusLed.Start(std::vector<int>{20, 180, 20, 180, 20, 180, 20, 1380}, true);
      break;
    case WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP:
      statusLed.Start(std::vector<int>{200, 9800}, true);
      break;
  }
}

void setup() {

  // Initialize serial communication for debugging
  Serial.begin(115200);

  logPrintf(LOG_INFO, "=== Cat Camera Controller Starting ===");

  setupWifi(deviceName);
  setupGPIO();

  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  cameraAvailable = initCamera();
  if (!cameraAvailable) {
        logPrintf(LOG_WARNING, "Camera initialization failed!");
        logPrintf(LOG_WARNING, "System will reboot to retry...");
        delay(2000);
        rebootSystem("Camera init failed");
  }

  sensor_t* s = esp_camera_sensor_get();
  s->set_exposure_ctrl(s, 1);
  s->set_aec2(s, 0);
  //s->set_ae_level(s, 0);

  JsonCameraConfig::config.ReadNVM();
  JsonCameraConfig::config.Apply();

  statusLed.Start(std::vector<int>{20, 180}, true);

  WiFi.onEvent(WiFiStatusCallback);

  setupAwsIot();

//  InitHttpd();

  Ambient::ltr.setup();

  logPrintf(LOG_INFO, "=== Setup Complete ===");
}


void loopCallback() {
    statusLed.Tick();
    loopAwsIot();

    Ambient::ltr.loop();

    if (!IsWiFiConnected()) {

        connectWiFi();
    }
    else {
      if (cameraAction.MustAct()) {
        if (takeAndUploadPhoto("Action")) {
          cameraAction.MarkAct();
        }
      }
    }
  }

void loop() {
  loopCallback();
    //LoopHttpd(loopCallback);
}

#endif