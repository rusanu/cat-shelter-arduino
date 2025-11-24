
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

void setup() {

  // Initialize serial communication for debugging
  Serial.begin(115200);

  Serial.println("\n=== Cat Camera Controller Starting ===");

  setupWifi("CAMERA");

  cameraAvailable = initCamera();
  if (!cameraAvailable) {
        Serial.println("WARNING: Camera initialization failed!");
        Serial.println("System will reboot to retry...");
        delay(2000);
        rebootSystem("Camera init failed");
  }

  Serial.println("=== Setup Complete ===\n");
}

void loop() {
  unsigned long currentMillis = millis();

 if (!wifiConnected) {
    connectWiFi();
 }
}