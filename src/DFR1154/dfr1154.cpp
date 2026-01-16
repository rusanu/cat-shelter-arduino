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
#include "common.h"

#ifdef DFR1154

static DebounceTimer cameraAction;

void setup() {

  // Initialize serial communication for debugging
  Serial.begin(115200);

  logPrintf(LOG_INFO, "=== Cat Camera Controller Starting ===");

  setupWifi("DFR");
  setupGPIO();

  cameraAvailable = initCamera();
  if (!cameraAvailable) {
        logPrintf(LOG_WARNING, "Camera initialization failed!");
        logPrintf(LOG_WARNING, "System will reboot to retry...");
        delay(2000);
        rebootSystem("Camera init failed");
  }

  logPrintf(LOG_INFO, "=== Setup Complete ===");
}


void loop() {
    if (!IsWiFiConnected()) {
        connectWiFi();
    }
    else {
      bool motion = true; //readPIRSensor();
      bool canAct = cameraAction.CanAct(), mustAct = cameraAction.MustAct();
      if ((motion && canAct) || mustAct) {
        logPrintf(LOG_INFO, "ACTION: %lu %d %d %d", cameraAction.CurrentDelay(), motion, canAct, mustAct);
        if (takeAndUploadPhoto("Action")) {
          cameraAction.MarkAct();
        }
      }
    }
}

#endif