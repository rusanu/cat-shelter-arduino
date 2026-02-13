/*
 * Cat Shelter Controller - ESP32-CAM based winter shelter automation
 *
 * Features:
 * - Temperature/humidity monitoring (DHT22)
 * - Motion-based cat presence detection (PIR HC-SR501)
 * - Smart heated blanket control
 * - Photo capture and AWS S3 upload
 * - Solar/battery power optimized
 *
 * MIT License - see LICENSE file for details
 *
 * Hardware: ESP32-CAM (AI Thinker), DHT22, PIR HC-SR501, Relay module
 */

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
#include "common.h"
#include "aws_iot.h"

const char* deviceName = "PATURA";

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n=== Cat Shelter Controller Starting ===");

  // Load boot state from NVM
  loadBootState();

  // Increment boot attempt counter
  incrementBootAttempt();

  bootStartTime = millis();

  // Initialize photo timing to allow first motion-triggered photo immediately
  // Set lastPhotoTime far enough in the past to exceed cooldown period
  lastPhotoTime = millis() - PHOTO_MOTION_COOLDOWN - 1000;
  lastHourlyPhotoTime = millis();  // Start counting from boot for scheduled photos

  // Initialize camera config check timer (starts counting from boot)
  lastCameraConfigCheck = millis();

  // Initialize camera (allow failure in safe mode)
  if (safeMode) {
    Serial.println("*** RUNNING IN SAFE MODE - Camera disabled ***");
    cameraAvailable = false;
  } else {
    cameraAvailable = initCamera();
    if (!cameraAvailable) {
      Serial.println("WARNING: Camera initialization failed!");
      Serial.println("System will reboot to retry...");
      delay(2000);
      rebootSystem("Camera init failed");
    }
  }

  // Setup GPIO pins
  setupGPIO();

  // Initialize DHT22 sensor
  dht->begin();
  Serial.println("DHT22 sensor initialized");

  setupWifi(deviceName);

  // Connect WiFi temporarily to sync time
  if (connectWiFi()) {
    syncTimeWithNTP(3);  // Try up to 3 times

    // Load camera configuration (S3 → NVM → defaults)
    loadCameraConfigAtBoot();

    // Take boot snapshot if camera is available and time is synced
    if (cameraAvailable && time(nullptr) > 100000) {
      logPrint(LOG_INFO, "Taking boot snapshot...");
      takeAndUploadPhoto("boot");
    }

    // WiFi will remain connected - continuous power available
    logPrint(LOG_INFO, "WiFi will remain connected (continuous power mode)");
  } else {
    logPrint(LOG_WARNING, "WiFi failed - cannot sync time");

    // No WiFi - load camera config from NVM or defaults
    loadCameraConfigAtBoot();
  }

  if (safeMode) {
    logPrint(LOG_WARNING, "");
    logPrint(LOG_WARNING, "!!! SAFE MODE ACTIVE !!!");
    logPrint(LOG_WARNING, "Reason: Too many failed boot attempts");
    logPrint(LOG_WARNING, "Core functions only:");
    logPrint(LOG_WARNING, "- Temperature monitoring: ENABLED");
    logPrint(LOG_WARNING, "- Blanket control: ENABLED");
    logPrint(LOG_WARNING, "- Camera/Photos: DISABLED");
    logPrint(LOG_WARNING, "");
    logPrint(LOG_INFO, "To exit safe mode:");
    logPrint(LOG_INFO, "1. Type 'reset' command to clear boot counter");
    logPrint(LOG_INFO, "2. Power cycle the device");
    logPrint(LOG_INFO, "3. If camera still fails, check power supply and connections");
    logPrint(LOG_WARNING, "");
  }

  setupAwsIot();

  Serial.println("=== Setup Complete ===\n");
}

void loop() {
  unsigned long currentMillis = millis();

  // Check if we've been running successfully for long enough
  if (bootAttempts > 0 && (currentMillis - bootStartTime) >= BOOT_SUCCESS_TIMEOUT) {
    markBootSuccess();
  }

  // Safe mode automatic recovery - attempt to exit safe mode every hour
  // Important for unattended operation over months
  if (safeMode) {
    // Initialize recovery timer on first safe mode entry
    if (lastSafeModeRecoveryAttempt == 0) {
      lastSafeModeRecoveryAttempt = currentMillis;
    }

    // Check if it's time to attempt recovery
    if ((currentMillis - lastSafeModeRecoveryAttempt) >= SAFE_MODE_RECOVERY_INTERVAL) {
      logPrint(LOG_WARNING, "Safe mode recovery attempt - resetting boot counter and rebooting");
      logPrint(LOG_INFO, "System has been stable for 1 hour in safe mode");
      logPrint(LOG_INFO, "Attempting to return to normal operation...");

      // Reset boot counter and safe mode flag
      markBootSuccess();

      // Wait a moment for logs to flush
      delay(1000);

      // Reboot to try normal mode again
      rebootSystem("Safe mode recovery attempt");
    }
  }

  // WiFi idle timeout disabled - continuous power available, keep WiFi always connected

  // Handle serial commands
  handleSerialCommands();

  // Check for cat presence
  checkPIRSensor();

  // Read temperature and humidity sensor
  readDHT22();

  // Update blanket control based on conditions
  updateBlanketControl();

  // Check if it's time to take and upload a photo
  checkPhotoSchedule();
  
  // Process AWS IoT messages
  loopAwsIot();

  // Check for camera config updates from S3 (every 60 minutes)
  if ((currentMillis - lastCameraConfigCheck) >= CAMERA_CONFIG_CHECK_INTERVAL) {
    lastCameraConfigCheck = currentMillis;
    checkCameraConfigUpdate();
  }

  // Print periodic status report
  printStatusReport();

  delay(100);
}