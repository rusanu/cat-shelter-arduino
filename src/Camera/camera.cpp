
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


#ifndef CAMERA
#error "This file should only be included in the CAMERA environment"
#endif

static DebounceTimer cameraAction;

void setup() {

  // Initialize serial communication for debugging
  Serial.begin(115200);

  logPrintf(LOG_INFO, "=== Cat Camera Controller Starting ===");

  setupWifi("CAMERA");

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
    if (!wifiConnected) {
        connectWiFi();
    }

    if (cameraAction.CanAct()) {
          logPrintf(LOG_INFO, "Camera action: %ld %ld", cameraAction.LastAct(), cameraAction.CurrentDelay());
    }
}