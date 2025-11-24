
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
}

void loop() {
}