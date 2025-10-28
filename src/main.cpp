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
#include "esp_camera.h"
#include "esp_wifi.h"
#include "secrets.h"  // WiFi credentials (not in git)

// Default S3 folder if not defined in secrets.h (backward compatibility)
#ifndef S3_FOLDER
#define S3_FOLDER ""  // Empty string = root folder
#endif

// GPIO Pin Definitions
#define RELAY_PIN 12        // Relay control for heated blanket
#define PIR_PIN 13          // PIR motion sensor (cat detection)
#define DHT_PIN 14          // DHT22 temperature/humidity sensor
#define FLASH_LED_PIN 4     // Built-in flash LED on ESP32-CAM

// DHT22 Configuration
#define DHT_TYPE DHT22
DHT dht(DHT_PIN, DHT_TYPE);

// ESP32-CAM AI Thinker Pin Configuration
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// Timing constants
#define DHT_READ_INTERVAL 2000  // Read DHT22 every 2 seconds
#define PHOTO_HOURLY_INTERVAL 600000   // 10 minutes in milliseconds (continuous power available)
#define PHOTO_MOTION_COOLDOWN 60000    // 1 minute in milliseconds (continuous power available)
#define STATUS_REPORT_INTERVAL 60000   // Status report every 60 seconds
#define WIFI_IDLE_TIMEOUT 360000       // UNUSED - WiFi always on with continuous power
#define BLANKET_MIN_STATE_TIME 300000  // 5 minutes minimum time before blanket can change state
#define CAT_PRESENCE_TIMEOUT 3600000   // 60 minutes - PIR motion extends presence (PIR is motion, not presence)

// Temperature threshold for blanket control (in Celsius)
#define TEMP_COLD_THRESHOLD 10.0  // Turn on blanket if temp is below this and cat present
#define TEMP_MAX_REASONABLE 45.0  // Maximum reasonable outdoor temperature (detect sensor errors)
#define TEMP_MIN_REASONABLE -30.0 // Minimum reasonable outdoor temperature (detect sensor errors)

// Typical winter temperatures for Pitesti, Romania (December-February) by hour (in Celsius)
// Based on average winter nighttime lows (-5 to 5°C) and daytime highs (0 to 10°C)
const float WINTER_TEMP_TABLE[24] = {
  -2.0,  // 00:00 - coldest part of night
  -3.0,  // 01:00
  -3.5,  // 02:00
  -4.0,  // 03:00 - coldest before dawn
  -3.5,  // 04:00
  -3.0,  // 05:00
  -2.0,  // 06:00 - sunrise
  -1.0,  // 07:00
   0.0,  // 08:00
   2.0,  // 09:00
   4.0,  // 10:00 - warming up
   6.0,  // 11:00
   7.0,  // 12:00 - peak daytime
   8.0,  // 13:00
   7.0,  // 14:00
   6.0,  // 15:00 - starting to cool
   4.0,  // 16:00
   2.0,  // 17:00 - sunset
   1.0,  // 18:00
   0.0,  // 19:00
  -1.0,  // 20:00
  -1.5,  // 21:00
  -2.0,  // 22:00
  -2.0   // 23:00
};

// Boot and recovery settings
#define MAX_BOOT_ATTEMPTS 3       // Max failed boots before entering safe mode
#define BOOT_SUCCESS_TIMEOUT 300000  // 5 minutes - if running this long, boot is successful
#define SAFE_MODE_RECOVERY_INTERVAL 3600000  // 1 hour - attempt to exit safe mode and retry

// Logging levels
enum LogLevel {
  LOG_ERROR = 0,
  LOG_WARNING = 1,
  LOG_INFO = 2,
  LOG_DEBUG = 3
};

// Current log level (can be changed via serial commands)
LogLevel currentLogLevel = LOG_INFO;

// Global state variables
bool catPresent = false;
bool blanketOn = false;
bool blanketManualOverride = false;  // Track if blanket is in manual control mode
unsigned long lastBlanketChange = 0;  // Track when blanket last changed state for debouncing
unsigned long lastMotionDetected = 0;  // Track last PIR motion for presence timeout
float currentTemp = 0.0;
float currentHumidity = 0.0;
unsigned long lastDHTRead = 0;
bool dhtSensorWorking = true;  // Track DHT22 state for debounced error logging
bool wifiConnected = false;
bool wifiManualOverride = false;  // Track if WiFi is in manual control mode
unsigned long lastWiFiActivity = 0;  // Track last WiFi usage for idle timeout
unsigned long lastPhotoTime = 0;  // Will be initialized in setup to allow first motion photo
unsigned long lastHourlyPhotoTime = 0;  // Will be initialized in setup
bool lastCatPresent = false;
unsigned long lastStatusReport = 0;

// Boot and recovery state
Preferences preferences;
bool safeMode = false;
bool cameraAvailable = false;
int bootAttempts = 0;
unsigned long bootStartTime = 0;
unsigned long lastSafeModeRecoveryAttempt = 0;  // Track recovery attempts in safe mode

// Logging functions
void logPrint(LogLevel level, const char* message) {
  if (level <= currentLogLevel) {
    const char* prefix = "";
    switch (level) {
      case LOG_ERROR:   prefix = "[ERROR] "; break;
      case LOG_WARNING: prefix = "[WARN]  "; break;
      case LOG_INFO:    prefix = "[INFO]  "; break;
      case LOG_DEBUG:   prefix = "[DEBUG] "; break;
    }
    Serial.print(prefix);
    Serial.println(message);
  }
}

void logPrintf(LogLevel level, const char* format, ...) {
  if (level <= currentLogLevel) {
    const char* prefix = "";
    switch (level) {
      case LOG_ERROR:   prefix = "[ERROR] "; break;
      case LOG_WARNING: prefix = "[WARN]  "; break;
      case LOG_INFO:    prefix = "[INFO]  "; break;
      case LOG_DEBUG:   prefix = "[DEBUG] "; break;
    }
    Serial.print(prefix);

    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    Serial.println(buffer);
  }
}

// Forward declarations
void printStatusReport(bool forceImmediate = false);
String generateStatusJSON();
float getExpectedTemperature();
float getEffectiveTemperature();
void takeAndUploadPhoto(const char* reason);

// ===== AWS Signature V4 Functions =====
// Adapted from: https://github.com/Mair/esp-aws-s3-auth-header

void getSHA256AsString(const char *input, size_t input_len, char *output) {
  uint8_t shaOutput[32] = {};
  mbedtls_sha256((uint8_t *)input, input_len, shaOutput, 0);

  char temp[65] = {};
  for (int i = 0; i < 32; i++) {
    sprintf(output, "%s%02x", temp, shaOutput[i]);
    strcpy(temp, output);
  }
}

void getSignatureKey(const char *key, const char *dateStamp, const char *regionName, const char *serviceName, uint8_t *output) {
  char augKey[100];
  sprintf(augKey, "AWS4%s", key);

  memset(output, 0, 32);
  const mbedtls_md_info_t *md_info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);

  mbedtls_md_hmac(md_info, (uint8_t *)augKey, strlen(augKey), (uint8_t *)dateStamp, strlen(dateStamp), output);
  mbedtls_md_hmac(md_info, output, 32, (uint8_t *)regionName, strlen(regionName), output);
  mbedtls_md_hmac(md_info, output, 32, (uint8_t *)serviceName, strlen(serviceName), output);
  const char *aws4Request = "aws4_request";
  mbedtls_md_hmac(md_info, output, 32, (uint8_t *)aws4Request, strlen(aws4Request), output);
}

void generateAWSSignatureV4(const char *method, const char *host, const char *uri,
                           const char *region, const char *accessKey, const char *secretKey,
                           const uint8_t *payload, size_t payload_len,
                           char *outAuthHeader, char *outAmzDate, char *outPayloadHash) {
  // Get current time
  time_t now = time(nullptr);
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);

  // Format dates
  char amzDate[18];
  strftime(amzDate, sizeof(amzDate), "%Y%m%dT%H%M%SZ", &timeinfo);
  strcpy(outAmzDate, amzDate);

  char dateStamp[12];
  strftime(dateStamp, sizeof(dateStamp), "%Y%m%d", &timeinfo);

  // Calculate payload hash
  getSHA256AsString((const char *)payload, payload_len, outPayloadHash);

  // Step 1 & 2: Create canonical request and hash it
  const char *signedHeaders = "host;x-amz-content-sha256;x-amz-date";
  char canonicalHeaders[300];
  sprintf(canonicalHeaders, "host:%s\nx-amz-content-sha256:%s\nx-amz-date:%s\n",
          host, outPayloadHash, amzDate);

  const char *canonicalQueryString = "";
  char canonicalRequest[600];
  sprintf(canonicalRequest, "%s\n%s\n%s\n%s\n%s\n%s",
          method, uri, canonicalQueryString, canonicalHeaders, signedHeaders, outPayloadHash);

  char canonicalRequestHash[65];
  getSHA256AsString(canonicalRequest, strlen(canonicalRequest), canonicalRequestHash);

  // Step 3: Create string to sign
  char credentialScope[100];
  sprintf(credentialScope, "%s/%s/s3/aws4_request", dateStamp, region);

  const char *algorithm = "AWS4-HMAC-SHA256";
  char stringToSign[300];
  sprintf(stringToSign, "%s\n%s\n%s\n%s", algorithm, amzDate, credentialScope, canonicalRequestHash);

  // Step 4: Calculate signature
  uint8_t signatureKey[32];
  getSignatureKey(secretKey, dateStamp, region, "s3", signatureKey);

  uint8_t signature[32];
  const mbedtls_md_info_t *md_info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  mbedtls_md_hmac(md_info, signatureKey, 32, (uint8_t *)stringToSign, strlen(stringToSign), signature);

  // Convert signature to hex string
  char signatureStr[65] = {};
  char temp[65] = {};
  for (int i = 0; i < 32; i++) {
    sprintf(signatureStr, "%s%02x", temp, signature[i]);
    strcpy(temp, signatureStr);
  }

  // Step 5: Build authorization header
  sprintf(outAuthHeader, "%s Credential=%s/%s, SignedHeaders=%s, Signature=%s",
          algorithm, accessKey, credentialScope, signedHeaders, signatureStr);
}

// ===== End AWS Signature V4 Functions =====

void loadBootState() {
  preferences.begin("cat-shelter", false);
  bootAttempts = preferences.getInt("bootAttempts", 0);
  safeMode = preferences.getBool("safeMode", false);

  logPrintf(LOG_INFO, "Boot state: attempts=%d/%d, safeMode=%s",
            bootAttempts, MAX_BOOT_ATTEMPTS, safeMode ? "YES" : "NO");

  if (safeMode) {
    logPrint(LOG_ERROR, "SAFE MODE is active from previous boot failures");
    logPrintf(LOG_ERROR, "System failed to boot %d times consecutively", MAX_BOOT_ATTEMPTS);
  }
}

void incrementBootAttempt() {
  bootAttempts++;
  preferences.putInt("bootAttempts", bootAttempts);

  if (bootAttempts >= MAX_BOOT_ATTEMPTS) {
    logPrint(LOG_ERROR, "!!! MAX BOOT ATTEMPTS REACHED - ENTERING SAFE MODE !!!");
    logPrintf(LOG_ERROR, "Boot attempt %d/%d failed", bootAttempts, MAX_BOOT_ATTEMPTS);
    logPrint(LOG_WARNING, "Probable cause: Camera initialization failure");
    safeMode = true;
    preferences.putBool("safeMode", true);
  }
}

void markBootSuccess() {
  logPrint(LOG_INFO, "Boot successful - resetting boot counter");
  bootAttempts = 0;
  // Note: Don't clear runtime safeMode flag here - it should only change on reboot
  // Only clear NVM so that next boot will attempt normal mode
  preferences.putInt("bootAttempts", 0);
  preferences.putBool("safeMode", false);
}

void rebootSystem(const char* reason) {
  Serial.printf("\n!!! REBOOTING: %s !!!\n", reason);
  Serial.flush();
  delay(1000);
  ESP.restart();
}

bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Frame size and quality settings
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;  // 1600x1200
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;  // 800x600
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // Initialize camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("ERROR: Camera init failed with error 0x%x\n", err);
    return false;
  }

  Serial.println("Camera initialized successfully");
  return true;
}

void flashOn() {
  digitalWrite(FLASH_LED_PIN, HIGH);
}

void flashOff() {
  digitalWrite(FLASH_LED_PIN, LOW);
}

camera_fb_t* capturePhoto() {
  // Turn on flash
  flashOn();
  logPrint(LOG_DEBUG, "Flash ON");

  // Delay for camera sensors to adjust to lighting
  delay(200);

  // Capture photo
  camera_fb_t* fb = esp_camera_fb_get();

  // Turn off flash immediately after capture
  flashOff();
  logPrint(LOG_DEBUG, "Flash OFF");

  if (!fb) {
    logPrint(LOG_ERROR, "Camera capture failed");
    return nullptr;
  }

  logPrintf(LOG_INFO, "Photo captured: %d bytes", fb->len);
  return fb;
}

void releasePhoto(camera_fb_t* fb) {
  if (fb) {
    esp_camera_fb_return(fb);
  }
}

bool connectWiFi() {
  if (wifiConnected) {
    return true;
  }

  Serial.println("Starting WiFi network scan...");

  // Disconnect first to ensure clean state
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(500);

  // Configure WiFi for scanning
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.setHostname("cat-shelter");

  // Scan for available networks
  int networksFound = WiFi.scanNetworks();
  Serial.printf("Found %d networks:\n", networksFound);

  // List all networks with signal strength
  Serial.println("\n=== Available WiFi Networks ===");
  for (int i = 0; i < networksFound; i++) {
    Serial.printf("  %2d: %-32s  %3d dBm  %s\n",
                  i + 1,
                  WiFi.SSID(i).c_str(),
                  WiFi.RSSI(i),
                  WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "OPEN" : "SECURED");
  }
  Serial.println("===============================\n");

  // Find the best available known network
  const char* selectedSSID = nullptr;
  const char* selectedPassword = nullptr;
  int bestRSSI = -1000;  // Very weak signal as baseline

  Serial.println("Checking for known networks...");
  for (int i = 0; i < KNOWN_NETWORKS_COUNT; i++) {
    Serial.printf("  Looking for: %s... ", KNOWN_NETWORKS[i].ssid);

    // Check if this known network is in the scan results
    bool found = false;
    for (int j = 0; j < networksFound; j++) {
      if (WiFi.SSID(j) == String(KNOWN_NETWORKS[i].ssid)) {
        int rssi = WiFi.RSSI(j);
        Serial.printf("FOUND (signal: %d dBm)\n", rssi);
        found = true;

        // Select this network if it's the first one found or has better signal
        if (selectedSSID == nullptr || rssi > bestRSSI) {
          selectedSSID = KNOWN_NETWORKS[i].ssid;
          selectedPassword = KNOWN_NETWORKS[i].password;
          bestRSSI = rssi;
        }
        break;
      }
    }

    if (!found) {
      Serial.println("not found");
    }
  }

  // Clean up scan results
  WiFi.scanDelete();

  // Check if we found any known network
  if (selectedSSID == nullptr) {
    Serial.println("\nERROR: No known networks available!");
    Serial.println("Known networks:");
    for (int i = 0; i < KNOWN_NETWORKS_COUNT; i++) {
      Serial.printf("  - %s\n", KNOWN_NETWORKS[i].ssid);
    }
    return false;
  }

  // Connect to selected network
  Serial.printf("\nConnecting to: %s (signal: %d dBm)\n", selectedSSID, bestRSSI);

  // Start WiFi connection
  WiFi.begin(selectedSSID, selectedPassword);

  // Configure underlying ESP-IDF layer to force WPA2-PSK
  wifi_config_t wifi_config = {};
  esp_wifi_get_config(WIFI_IF_STA, &wifi_config);
  wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
  wifi_config.sta.pmf_cfg.capable = true;
  wifi_config.sta.pmf_cfg.required = false;
  esp_wifi_set_config(WIFI_IF_STA, &wifi_config);

  // Reconnect with new settings
  WiFi.reconnect();

  // Wait for connection
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    lastWiFiActivity = millis();  // Track WiFi activity
    Serial.println("\nWiFi connected successfully!");
    Serial.printf("SSID: %s\n", selectedSSID);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Signal strength: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
    return true;
  } else {
    Serial.printf("\nWiFi connection to %s failed!\n", selectedSSID);
    return false;
  }
}

void disconnectWiFi() {
  if (!wifiConnected) {
    return;
  }

  Serial.println("Disconnecting WiFi for power saving...");
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  wifiConnected = false;
}

bool syncTimeWithNTP(int maxRetries = 3) {
  logPrint(LOG_INFO, "Synchronizing time with NTP...");
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");

  // Try multiple times if needed
  for (int retry = 0; retry < maxRetries; retry++) {
    if (retry > 0) {
      logPrintf(LOG_WARNING, "NTP sync retry %d/%d", retry, maxRetries - 1);
    }

    // Wait for time to be set
    int attempts = 0;
    while (time(nullptr) < 100000 && attempts < 20) {
      delay(500);
      Serial.print(".");
      attempts++;
    }

    time_t now = time(nullptr);
    if (now > 100000) {
      struct tm timeinfo;
      gmtime_r(&now, &timeinfo);
      char buffer[64];
      strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S UTC", &timeinfo);
      logPrintf(LOG_INFO, "Time synchronized: %s", buffer);
      return true;
    }

    // Failed this attempt, wait before retry
    if (retry < maxRetries - 1) {
      delay(2000);  // Wait 2 seconds before retry
    }
  }

  logPrint(LOG_ERROR, "NTP sync failed after all retries");
  return false;
}

String getTimestamp() {
  time_t now = time(nullptr);
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  char buffer[64];
  strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", &timeinfo);
  return String(buffer);
}

bool uploadPhotoToS3(camera_fb_t* fb, const String& filename) {
  if (!fb || fb->len == 0) {
    logPrint(LOG_ERROR, "Invalid photo buffer");
    return false;
  }

  // Ensure WiFi is connected
  if (!connectWiFi()) {
    return false;
  }

  // Verify time is set (required for AWS Signature V4)
  time_t now = time(nullptr);
  if (now < 100000) {
    logPrint(LOG_WARNING, "Time not synchronized! Attempting NTP sync...");

    // Try to sync time on-demand (WiFi already connected)
    if (!syncTimeWithNTP(3)) {
      logPrint(LOG_ERROR, "Cannot generate AWS signature without time sync");
      return false;
    }
  }

  // Build S3 path with folder (if configured)
  String uri = "/";
  String logPath = "";
  if (strlen(S3_FOLDER) > 0) {
    uri += String(S3_FOLDER) + "/";
    logPath = String(S3_FOLDER) + "/";
  }
  uri += filename;
  logPath += filename;

  String host = String(S3_BUCKET) + ".s3." + AWS_REGION + ".amazonaws.com";
  String url = "https://" + host + uri;

  logPrintf(LOG_INFO, "Uploading photo to S3: %s", logPath.c_str());

  // Generate AWS Signature V4 authentication headers
  char authHeader[400];
  char amzDate[18];
  char payloadHash[65];

  generateAWSSignatureV4("PUT", host.c_str(), uri.c_str(),
                         AWS_REGION, AWS_ACCESS_KEY_ID, AWS_SECRET_ACCESS_KEY,
                         fb->buf, fb->len,
                         authHeader, amzDate, payloadHash);

  // Make authenticated request
  HTTPClient http;
  http.begin(url);

  // Set timeouts for large file uploads
  // For 78KB photos: ~10 seconds should be sufficient even on slow connections
  // Default timeout is only 5 seconds which can fail for large payloads
  http.setConnectTimeout(15000);  // 15 seconds to establish connection
  http.setTimeout(60000);        // 60 seconds for data transfer (SO_SNDTIMEO)

  http.addHeader("Host", host);
  http.addHeader("x-amz-date", amzDate);
  http.addHeader("x-amz-content-sha256", payloadHash);
  http.addHeader("Authorization", authHeader);
  http.addHeader("Content-Type", "image/jpeg");
  http.addHeader("Content-Length", String(fb->len));

  logPrint(LOG_DEBUG, "Sending PUT request with AWS Signature V4...");

  int httpResponseCode = http.PUT(fb->buf, fb->len);

  // Get response body before closing connection
  String responseBody = "";
  if (httpResponseCode > 0) {
    responseBody = http.getString();
  }

  http.end();

  // HTTP 2xx codes are success, everything else is failure
  if (httpResponseCode >= 200 && httpResponseCode < 300) {
    logPrintf(LOG_INFO, "Upload successful! HTTP %d", httpResponseCode);
    return true;
  } else if (httpResponseCode > 0) {
    // Got HTTP response but it's an error
    logPrintf(LOG_ERROR, "Upload failed! HTTP %d", httpResponseCode);

    // Log response body for debugging (truncate if too long)
    if (responseBody.length() > 0) {
      if (responseBody.length() > 200) {
        responseBody = responseBody.substring(0, 200) + "...";
      }
      logPrintf(LOG_ERROR, "Response: %s", responseBody.c_str());
    }

    // Provide helpful hints based on status code
    if (httpResponseCode == 403) {
      logPrint(LOG_ERROR, "HTTP 403 Forbidden - Check S3 bucket permissions or credentials");
    } else if (httpResponseCode == 404) {
      logPrint(LOG_ERROR, "HTTP 404 Not Found - Check S3 bucket name and region");
    } else if (httpResponseCode >= 500) {
      logPrint(LOG_ERROR, "HTTP 5xx Server Error - S3 service issue");
    }
    return false;
  } else {
    // Network/connection error
    logPrintf(LOG_ERROR, "Upload failed! Network error: %s", http.errorToString(httpResponseCode).c_str());
    return false;
  }
}

bool uploadStatusToS3(const String& filename) {
  // Ensure WiFi is connected
  if (!connectWiFi()) {
    return false;
  }

  // Verify time is set (required for AWS Signature V4)
  time_t now = time(nullptr);
  if (now < 100000) {
    logPrint(LOG_WARNING, "Time not synchronized! Attempting NTP sync...");

    // Try to sync time on-demand (WiFi already connected)
    if (!syncTimeWithNTP(3)) {
      logPrint(LOG_ERROR, "Cannot generate AWS signature without time sync");
      return false;
    }
  }

  // Generate status JSON
  String statusJSON = generateStatusJSON();
  uint8_t* payload = (uint8_t*)statusJSON.c_str();
  size_t payload_len = statusJSON.length();

  // Build S3 path with folder (if configured, same as photo)
  String uri = "/";
  String logPath = "";
  if (strlen(S3_FOLDER) > 0) {
    uri += String(S3_FOLDER) + "/";
    logPath = String(S3_FOLDER) + "/";
  }
  uri += filename;
  logPath += filename;

  String host = String(S3_BUCKET) + ".s3." + AWS_REGION + ".amazonaws.com";
  String url = "https://" + host + uri;

  logPrintf(LOG_DEBUG, "Uploading status JSON to S3: %s", logPath.c_str());

  // Generate AWS Signature V4 authentication headers
  char authHeader[400];
  char amzDate[18];
  char payloadHash[65];

  generateAWSSignatureV4("PUT", host.c_str(), uri.c_str(),
                         AWS_REGION, AWS_ACCESS_KEY_ID, AWS_SECRET_ACCESS_KEY,
                         payload, payload_len,
                         authHeader, amzDate, payloadHash);

  // Make authenticated request
  HTTPClient http;
  http.begin(url);

  // Set timeouts (same as photo upload for consistency)
  http.setConnectTimeout(5000);  // 5 seconds to establish connection
  http.setTimeout(15000);        // 15 seconds for data transfer

  http.addHeader("Host", host);
  http.addHeader("x-amz-date", amzDate);
  http.addHeader("x-amz-content-sha256", payloadHash);
  http.addHeader("Authorization", authHeader);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Content-Length", String(payload_len));

  int httpResponseCode = http.PUT(payload, payload_len);

  // Get response body before closing connection
  String responseBody = "";
  if (httpResponseCode > 0) {
    responseBody = http.getString();
  }

  http.end();

  // HTTP 2xx codes are success
  if (httpResponseCode >= 200 && httpResponseCode < 300) {
    logPrintf(LOG_DEBUG, "Status JSON uploaded successfully! HTTP %d", httpResponseCode);
    return true;
  } else if (httpResponseCode > 0) {
    logPrintf(LOG_WARNING, "Status JSON upload failed! HTTP %d", httpResponseCode);
    if (responseBody.length() > 0 && responseBody.length() <= 100) {
      logPrintf(LOG_DEBUG, "Response: %s", responseBody.c_str());
    }
    return false;
  } else {
    logPrintf(LOG_WARNING, "Status JSON upload failed! Network error: %s", http.errorToString(httpResponseCode).c_str());
    return false;
  }
}

void setupGPIO() {
  // Configure relay pin as output
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // Start with blanket off

  // Configure PIR sensor pin as input
  pinMode(PIR_PIN, INPUT);

  // Configure flash LED pin as output
  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW); // Start with flash off

  Serial.println("GPIO pins configured:");
  Serial.println("- RELAY_PIN (GPIO12): OUTPUT");
  Serial.println("- PIR_PIN (GPIO13): INPUT");
  Serial.println("- DHT_PIN (GPIO14): DHT22 sensor");
  Serial.println("- FLASH_LED_PIN (GPIO4): OUTPUT");
}

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
  dht.begin();
  Serial.println("DHT22 sensor initialized");

  // Connect WiFi temporarily to sync time
  if (connectWiFi()) {
    syncTimeWithNTP(3);  // Try up to 3 times

    // Take boot snapshot if camera is available and time is synced
    if (cameraAvailable && time(nullptr) > 100000) {
      logPrint(LOG_INFO, "Taking boot snapshot...");
      takeAndUploadPhoto("boot");
    }

    // WiFi will remain connected - continuous power available
    logPrint(LOG_INFO, "WiFi will remain connected (continuous power mode)");
  } else {
    logPrint(LOG_WARNING, "WiFi failed - cannot sync time");
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

  Serial.println("=== Setup Complete ===\n");
}

void checkPIRSensor() {
  unsigned long currentMillis = millis();

  // Read PIR sensor state
  int pirState = digitalRead(PIR_PIN);
  bool motionDetected = (pirState == HIGH);

  // PIR HC-SR501 is a MOTION detector, not a PRESENCE sensor
  // Motion detection extends the presence timer (cat might be sleeping)
  if (motionDetected) {
    // Motion detected - update timestamp and mark cat as present
    if (!catPresent) {
      logPrint(LOG_INFO, "*** CAT MOTION DETECTED! ***");
      catPresent = true;
    } else {
      logPrint(LOG_DEBUG, "Cat motion (presence extended)");
    }
    lastMotionDetected = currentMillis;
  } else {
    // No current motion - check if presence timeout expired
    if (catPresent && (currentMillis - lastMotionDetected) >= CAT_PRESENCE_TIMEOUT) {
      logPrint(LOG_INFO, "Cat presence timeout - no motion for 60 minutes");
      catPresent = false;
    }
  }
}

void controlBlanket(bool shouldBeOn) {
  if (shouldBeOn != blanketOn) {
    blanketOn = shouldBeOn;
    lastBlanketChange = millis();  // Record state change time
    digitalWrite(RELAY_PIN, blanketOn ? HIGH : LOW);

    if (blanketOn) {
      logPrint(LOG_INFO, "Blanket turned ON");
    } else {
      logPrint(LOG_INFO, "Blanket turned OFF");
    }
  }
}

void readDHT22() {
  unsigned long currentMillis = millis();

  // Check if it's time to read the sensor
  if (currentMillis - lastDHTRead >= DHT_READ_INTERVAL) {
    lastDHTRead = currentMillis;

    // Read temperature and humidity
    float temp = dht.readTemperature();
    float humidity = dht.readHumidity();

    // Check if readings are valid
    if (isnan(temp) || isnan(humidity)) {
      // Only log error if sensor state changed from working to failed
      if (dhtSensorWorking) {
        logPrint(LOG_ERROR, "DHT22 sensor failure detected!");

        // Provide detailed error information
        if (isnan(temp) && isnan(humidity)) {
          logPrint(LOG_ERROR, "Both temperature and humidity readings are NaN");
          logPrint(LOG_WARNING, "Possible causes: sensor disconnected, power issue, or sensor failure");
        } else if (isnan(temp)) {
          logPrint(LOG_ERROR, "Temperature reading is NaN (humidity OK)");
        } else {
          logPrint(LOG_ERROR, "Humidity reading is NaN (temperature OK)");
        }

        logPrint(LOG_WARNING, "Check GPIO14 connection and DHT22 sensor");
        dhtSensorWorking = false;
      }
      return;
    }

    // Sensor is working - check if it just recovered
    if (!dhtSensorWorking) {
      logPrint(LOG_INFO, "DHT22 sensor recovered and working normally");
      dhtSensorWorking = true;
    }

    // Update global state
    currentTemp = temp;
    currentHumidity = humidity;

    // Log readings at DEBUG level (happens every 2 seconds, too verbose for INFO)
    logPrintf(LOG_DEBUG, "Temperature: %.1f°C | Humidity: %.1f%%", currentTemp, currentHumidity);
  }
}

float getExpectedTemperature() {
  // Get current hour from RTC
  time_t now = time(nullptr);
  if (now < 100000) {
    // Time not synced, assume coldest hour (3 AM)
    return WINTER_TEMP_TABLE[3];
  }

  struct tm timeinfo;
  localtime_r(&now, &timeinfo);
  int hour = timeinfo.tm_hour;

  // Bounds check
  if (hour < 0 || hour >= 24) {
    hour = 0;
  }

  return WINTER_TEMP_TABLE[hour];
}

float getEffectiveTemperature() {
  // If sensor is not working, use expected temperature
  if (!dhtSensorWorking) {
    float expectedTemp = getExpectedTemperature();
    logPrintf(LOG_DEBUG, "DHT22 failed - using expected temperature: %.1f°C", expectedTemp);
    return expectedTemp;
  }

  // If sensor reading is unreasonable (aberrant), use expected temperature
  if (currentTemp > TEMP_MAX_REASONABLE || currentTemp < TEMP_MIN_REASONABLE) {
    float expectedTemp = getExpectedTemperature();
    logPrintf(LOG_WARNING, "DHT22 reading aberrant (%.1f°C) - using expected: %.1f°C",
              currentTemp, expectedTemp);
    logPrint(LOG_WARNING, "Possible cause: direct sunlight on sensor or sensor malfunction");
    return expectedTemp;
  }

  // Sensor is working and value is reasonable
  return currentTemp;
}

void updateBlanketControl() {
  // Skip automatic control if in manual override mode
  if (blanketManualOverride) {
    return;
  }

  // Blanket should be on if:
  // 1. Cat is present
  // 2. Temperature is below threshold
  // Use effective temperature (with fallback if sensor fails)
  float effectiveTemp = getEffectiveTemperature();
  bool shouldBeOn = catPresent && (effectiveTemp < TEMP_COLD_THRESHOLD);

  // Apply debouncing: only change state if minimum time has elapsed
  unsigned long timeSinceLastChange = millis() - lastBlanketChange;

  // If the desired state is different from current state
  if (shouldBeOn != blanketOn) {
    // Only change if enough time has passed since last change
    if (timeSinceLastChange >= BLANKET_MIN_STATE_TIME) {
      controlBlanket(shouldBeOn);
    } else {
      // Log at DEBUG level to avoid spam
      unsigned long remainingTime = (BLANKET_MIN_STATE_TIME - timeSinceLastChange) / 1000;
      logPrintf(LOG_DEBUG, "Blanket state change delayed (%lu seconds remaining)", remainingTime);
    }
  }
}

void takeAndUploadPhoto(const char* reason) {
  // Skip if camera not available (safe mode or camera failed)
  if (!cameraAvailable) {
    return;
  }

  logPrintf(LOG_INFO, "Taking photo (%s)...", reason);

  // Generate base filename with timestamp (without extension)
  String baseFilename = "cat_" + getTimestamp();
  String photoFilename = baseFilename + ".jpg";
  String jsonFilename = baseFilename + ".json";

  camera_fb_t* fb = capturePhoto();
  if (fb) {
    bool photoSuccess = uploadPhotoToS3(fb, photoFilename);
    releasePhoto(fb);

    if (photoSuccess) {
      logPrint(LOG_INFO, "Photo uploaded successfully!");
      lastWiFiActivity = millis();  // Update activity timestamp

      // Upload status JSON with same base filename
      bool jsonSuccess = uploadStatusToS3(jsonFilename);
      if (jsonSuccess) {
        logPrint(LOG_INFO, "Status JSON uploaded successfully!");
      } else {
        logPrint(LOG_WARNING, "Status JSON upload failed (photo was uploaded)");
      }
    } else {
      logPrint(LOG_WARNING, "Photo upload failed!");
    }

    // WiFi remains connected (continuous power mode)
  }
}

void checkPhotoSchedule() {
  // Skip photo scheduling if camera not available
  if (!cameraAvailable) {
    return;
  }

  unsigned long currentMillis = millis();

  // Check for scheduled photo (regular interval)
  if (currentMillis - lastHourlyPhotoTime >= PHOTO_HOURLY_INTERVAL) {
    lastHourlyPhotoTime = currentMillis;
    lastPhotoTime = currentMillis;
    takeAndUploadPhoto("scheduled");
    return;
  }

  // Check for motion-triggered photo
  // Take photo when cat arrives, but not more than once every 5 minutes
  bool catJustArrived = catPresent && !lastCatPresent;
  bool cooldownExpired = (currentMillis - lastPhotoTime) >= PHOTO_MOTION_COOLDOWN;

  if (catJustArrived && cooldownExpired) {
    lastPhotoTime = currentMillis;
    takeAndUploadPhoto("motion detected");
  }

  // Update previous cat presence state
  lastCatPresent = catPresent;
}

void handleSerialCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toLowerCase();

    if (command == "help" || command == "?") {
      Serial.println("\n=== Available Commands ===");
      Serial.println("help or ?     - Show this help");
      Serial.println("status        - Print current status");
      Serial.println("snapshot      - Take and upload photo now");
      Serial.println("blanket on    - Turn blanket ON (manual mode)");
      Serial.println("blanket off   - Turn blanket OFF (manual mode)");
      Serial.println("blanket auto  - Return to automatic blanket control");
      Serial.println("wifi on       - Turn WiFi ON (manual mode)");
      Serial.println("wifi off      - Turn WiFi OFF (manual mode)");
      Serial.println("wifi auto     - Return to automatic WiFi control");
      Serial.println("wifi strength - Monitor WiFi signal strength (press any key to stop)");
      Serial.println("loglevel <n>  - Set log level (0=ERROR, 1=WARN, 2=INFO, 3=DEBUG)");
      Serial.println("reboot        - Reboot system");
      Serial.println("safemode      - Enter safe mode");
      Serial.println("reset         - Reset boot counter");
      Serial.println("========================\n");
    }
    else if (command == "status") {
      printStatusReport(true);  // Force immediate output, bypass 60s interval
    }
    else if (command == "snapshot") {
      if (cameraAvailable) {
        logPrint(LOG_INFO, "Manual snapshot triggered");
        takeAndUploadPhoto("manual");
      } else {
        logPrint(LOG_ERROR, "Camera not available (safe mode or init failed)");
      }
    }
    else if (command == "blanket on") {
      blanketManualOverride = true;
      controlBlanket(true);
      logPrint(LOG_INFO, "Blanket MANUALLY turned ON (automatic control disabled)");
    }
    else if (command == "blanket off") {
      blanketManualOverride = true;
      controlBlanket(false);
      logPrint(LOG_INFO, "Blanket MANUALLY turned OFF (automatic control disabled)");
    }
    else if (command == "blanket auto") {
      blanketManualOverride = false;
      logPrint(LOG_INFO, "Blanket returned to AUTOMATIC control");
    }
    else if (command == "wifi on") {
      wifiManualOverride = true;
      if (connectWiFi()) {
        logPrint(LOG_INFO, "WiFi MANUALLY turned ON (automatic control disabled)");
      } else {
        logPrint(LOG_ERROR, "WiFi connection failed");
      }
    }
    else if (command == "wifi off") {
      wifiManualOverride = true;
      disconnectWiFi();
      logPrint(LOG_INFO, "WiFi MANUALLY turned OFF (automatic control disabled)");
    }
    else if (command == "wifi auto") {
      wifiManualOverride = false;
      logPrint(LOG_INFO, "WiFi returned to AUTOMATIC control (idle timeout enabled)");
    }
    else if (command == "wifi strength") {
      // Check if WiFi is connected
      if (!wifiConnected) {
        Serial.println("WiFi not connected. Attempting to connect...");
        if (!connectWiFi()) {
          logPrint(LOG_ERROR, "Cannot monitor WiFi strength - connection failed");
          return;
        }
      }

      Serial.println("\n=== WiFi Signal Strength Monitor ===");
      Serial.printf("Connected to: %s\n", WiFi.SSID().c_str());
      Serial.println("Press any key to stop monitoring...\n");

      unsigned long lastPrint = 0;
      while (true) {
        unsigned long currentMillis = millis();

        // Check for key press to exit
        if (Serial.available() > 0) {
          // Clear the serial buffer
          while (Serial.available() > 0) {
            Serial.read();
          }
          Serial.println("\nMonitoring stopped.");
          break;
        }

        // Print signal strength at most once per second
        if (currentMillis - lastPrint >= 1000) {
          lastPrint = currentMillis;

          if (WiFi.status() == WL_CONNECTED) {
            int rssi = WiFi.RSSI();
            const char* quality;

            // Interpret signal strength
            if (rssi >= -50) {
              quality = "Excellent";
            } else if (rssi >= -60) {
              quality = "Good";
            } else if (rssi >= -70) {
              quality = "Fair";
            } else if (rssi >= -80) {
              quality = "Weak";
            } else {
              quality = "Very Weak";
            }

            Serial.printf("Signal: %4d dBm  [%s]\n", rssi, quality);
          } else {
            Serial.println("WiFi disconnected!");
            break;
          }
        }

        delay(100);  // Small delay to avoid busy-waiting
      }
    }
    else if (command.startsWith("loglevel ")) {
      int level = command.substring(9).toInt();
      if (level >= 0 && level <= 3) {
        currentLogLevel = (LogLevel)level;
        const char* levelNames[] = {"ERROR", "WARNING", "INFO", "DEBUG"};
        logPrintf(LOG_INFO, "Log level set to: %s", levelNames[level]);
      } else {
        logPrint(LOG_ERROR, "Invalid log level. Use 0-3 (ERROR, WARN, INFO, DEBUG)");
      }
    }
    else if (command == "reboot") {
      rebootSystem("Manual reboot command");
    }
    else if (command == "safemode") {
      logPrint(LOG_WARNING, "Entering safe mode manually");
      safeMode = true;
      cameraAvailable = false;
      preferences.putBool("safeMode", true);
      logPrint(LOG_INFO, "Safe mode activated. Reboot to apply.");
    }
    else if (command == "reset") {
      markBootSuccess();
      logPrint(LOG_INFO, "Boot counter reset to 0");
    }
    else if (command.length() > 0) {
      logPrintf(LOG_ERROR, "Unknown command: %s (type 'help' for commands)", command.c_str());
    }
  }
}

String generateStatusJSON() {
  unsigned long currentMillis = millis();
  time_t now = time(nullptr);
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  char timestamp[64];
  strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);

  // Generate JSON manually (no complex libraries needed)
  String json = "{\n";
  json += "  \"timestamp\": \"" + String(timestamp) + "\",\n";
  json += "  \"uptime_seconds\": " + String(currentMillis / 1000) + ",\n";
  json += "  \"mode\": \"" + String(safeMode ? "SAFE_MODE" : "NORMAL") + "\",\n";
  json += "  \"boot_attempts\": " + String(bootAttempts) + ",\n";
  json += "  \"max_boot_attempts\": " + String(MAX_BOOT_ATTEMPTS) + ",\n";
  json += "  \"temperature_celsius\": " + String(currentTemp, 1) + ",\n";
  json += "  \"humidity_percent\": " + String(currentHumidity, 1) + ",\n";
  json += "  \"dht22_sensor_working\": " + String(dhtSensorWorking ? "true" : "false") + ",\n";

  // Add effective temperature and expected temperature
  float effectiveTemp = getEffectiveTemperature();
  float expectedTemp = getExpectedTemperature();
  json += "  \"effective_temperature_celsius\": " + String(effectiveTemp, 1) + ",\n";
  json += "  \"expected_temperature_celsius\": " + String(expectedTemp, 1) + ",\n";
  json += "  \"using_fallback_temperature\": " + String((effectiveTemp != currentTemp) ? "true" : "false") + ",\n";

  json += "  \"cat_present\": " + String(catPresent ? "true" : "false") + ",\n";

  // Cat presence timing info (PIR motion-based with timeout)
  if (lastMotionDetected > 0) {
    unsigned long timeSinceMotion = currentMillis - lastMotionDetected;
    json += "  \"seconds_since_last_motion\": " + String(timeSinceMotion / 1000) + ",\n";
    if (catPresent) {
      unsigned long timeUntilTimeout = CAT_PRESENCE_TIMEOUT - timeSinceMotion;
      json += "  \"presence_timeout_seconds\": " + String(timeUntilTimeout / 1000) + ",\n";
    }
  }

  json += "  \"blanket_on\": " + String(blanketOn ? "true" : "false") + ",\n";
  json += "  \"blanket_manual_override\": " + String(blanketManualOverride ? "true" : "false") + ",\n";
  json += "  \"camera_available\": " + String(cameraAvailable ? "true" : "false") + ",\n";
  json += "  \"wifi_connected\": " + String(wifiConnected ? "true" : "false") + ",\n";
  json += "  \"wifi_manual_override\": " + String(wifiManualOverride ? "true" : "false") + ",\n";

  // Memory status (for leak detection)
  json += "  \"heap_free_bytes\": " + String(ESP.getFreeHeap()) + ",\n";
  json += "  \"heap_size_bytes\": " + String(ESP.getHeapSize()) + ",\n";
  json += "  \"heap_min_free_bytes\": " + String(ESP.getMinFreeHeap()) + ",\n";
  json += "  \"psram_free_bytes\": " + String(ESP.getFreePsram()) + ",\n";
  json += "  \"psram_size_bytes\": " + String(ESP.getPsramSize());

  if (cameraAvailable) {
    unsigned long nextScheduledPhoto = PHOTO_HOURLY_INTERVAL - (currentMillis - lastHourlyPhotoTime);
    unsigned long timeSinceLastPhoto = currentMillis - lastPhotoTime;
    json += ",\n";
    json += "  \"next_scheduled_photo_minutes\": " + String(nextScheduledPhoto / 60000) + ",\n";
    json += "  \"time_since_last_photo_minutes\": " + String(timeSinceLastPhoto / 60000);
  }

  json += "\n}";
  return json;
}

void printStatusReport(bool forceImmediate) {
  unsigned long currentMillis = millis();

  // Check timing unless forced (e.g., from manual command)
  if (forceImmediate || (currentMillis - lastStatusReport >= STATUS_REPORT_INTERVAL)) {
    lastStatusReport = currentMillis;

    String statusJSON = generateStatusJSON();

    Serial.println("\n===== STATUS REPORT =====");
    Serial.printf("Mode: %s\n", safeMode ? "SAFE MODE" : "NORMAL");
    Serial.printf("Uptime: %lu seconds\n", currentMillis / 1000);
    Serial.printf("Boot attempts: %d/%d\n", bootAttempts, MAX_BOOT_ATTEMPTS);
    Serial.printf("Temperature: %.1f°C\n", currentTemp);
    Serial.printf("Humidity: %.1f%%\n", currentHumidity);
    Serial.printf("DHT22 Sensor: %s\n", dhtSensorWorking ? "WORKING" : "FAILED");

    // Cat presence with timeout info
    if (catPresent) {
      unsigned long timeSinceMotion = currentMillis - lastMotionDetected;
      unsigned long timeUntilTimeout = CAT_PRESENCE_TIMEOUT - timeSinceMotion;
      Serial.printf("Cat Present: YES (motion %lu min ago, timeout in %lu min)\n",
                    timeSinceMotion / 60000, timeUntilTimeout / 60000);
    } else {
      if (lastMotionDetected > 0) {
        unsigned long timeSinceMotion = currentMillis - lastMotionDetected;
        Serial.printf("Cat Present: NO (last motion %lu min ago)\n", timeSinceMotion / 60000);
      } else {
        Serial.printf("Cat Present: NO (no motion detected since boot)\n");
      }
    }

    if (blanketManualOverride) {
      Serial.printf("Blanket: %s (MANUAL OVERRIDE)\n", blanketOn ? "ON" : "OFF");
    } else {
      Serial.printf("Blanket: %s (automatic)\n", blanketOn ? "ON" : "OFF");
    }
    Serial.printf("Camera: %s\n", cameraAvailable ? "AVAILABLE" : "DISABLED");
    if (wifiManualOverride) {
      Serial.printf("WiFi: %s (MANUAL OVERRIDE)\n", wifiConnected ? "CONNECTED" : "DISCONNECTED");
    } else {
      Serial.printf("WiFi: %s (automatic)\n", wifiConnected ? "CONNECTED" : "DISCONNECTED");
    }

    // Memory status (for leak detection)
    Serial.println("--- Memory Status ---");
    Serial.printf("Heap Free: %u bytes (%.1f KB)\n", ESP.getFreeHeap(), ESP.getFreeHeap() / 1024.0);
    Serial.printf("Heap Size: %u bytes (%.1f KB)\n", ESP.getHeapSize(), ESP.getHeapSize() / 1024.0);
    Serial.printf("Heap Min Free: %u bytes (%.1f KB)\n", ESP.getMinFreeHeap(), ESP.getMinFreeHeap() / 1024.0);
    Serial.printf("PSRAM Free: %u bytes (%.1f KB)\n", ESP.getFreePsram(), ESP.getFreePsram() / 1024.0);
    Serial.printf("PSRAM Size: %u bytes (%.1f KB)\n", ESP.getPsramSize(), ESP.getPsramSize() / 1024.0);
    Serial.println("---------------------");

    if (cameraAvailable) {
      unsigned long nextScheduledPhoto = PHOTO_HOURLY_INTERVAL - (currentMillis - lastHourlyPhotoTime);
      Serial.printf("Next scheduled photo in: %lu minutes\n", nextScheduledPhoto / 60000);

      unsigned long timeSinceLastPhoto = currentMillis - lastPhotoTime;
      Serial.printf("Time since last photo: %lu minutes\n", timeSinceLastPhoto / 60000);
    }

    Serial.println("========================\n");
  }
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

  // Print periodic status report
  printStatusReport();

  delay(100);
}