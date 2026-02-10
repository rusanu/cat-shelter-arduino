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
#include "secrets.h"  // WiFi credentials (not in git)

#include "common.h"

// Default S3 folder if not defined in secrets.h (backward compatibility)
#ifndef S3_FOLDER
#define S3_FOLDER ""  // Empty string = root folder
#endif

// GPIO Pin Definitions
#if !defined(RELAY_PIN)\
  || !defined(PIR_PIN)\
  || !defined(DHT_PIN)\
  || !defined(FLASH_LED_PIN)
#error "Initialize config pins in platform.io build_flags"
#endif  

//#define RELAY_PIN 12        // Relay control for heated blanket
//#define PIR_PIN 13          // PIR motion sensor (cat detection)
//#define DHT_PIN 14          // DHT22 temperature/humidity sensor
//#define FLASH_LED_PIN 4     // Built-in flash LED on ESP32-CAM

// DHT22 Configuration
#define DHT_TYPE DHT22
DHT* dht = nullptr;

#ifdef DFR1154
// DFR1154 OV3660 IR camera
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      5
#define SIOD_GPIO_NUM      8
#define SIOC_GPIO_NUM      9
#define Y9_GPIO_NUM        4
#define Y8_GPIO_NUM        6
#define Y7_GPIO_NUM        7
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       17
#define Y4_GPIO_NUM       21
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM       16
#define VSYNC_GPIO_NUM     1
#define HREF_GPIO_NUM      2
#define PCLK_GPIO_NUM     15

#else

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
#endif


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


// Current log level (can be changed via serial commands)
LogLevel currentLogLevel = LOG_INFO;

float getChipTemperature()
{
#ifdef CONFIG_IDF_TARGET_ESP32
  // ESP32 classic chip has the temprature_sens_read() function
  uint8_t temp_raw = temprature_sens_read();
  float temp_celsius = (temp_raw - 32) / 1.8;
  return temp_celsius;
#else
  // ESP32-S3 and newer chips don't have temprature_sens_read()
  // Temperature monitoring is not critical, return placeholder
  return 0.0f;
#endif
}

// Read current camera configuration from sensor (hardware truth)
CameraConfig readCurrentCameraConfig() {
  CameraConfig config;
  sensor_t* s = esp_camera_sensor_get();

  if (!s) {
    logPrint(LOG_ERROR, "Failed to get camera sensor for reading config");
    // Return empty config if sensor unavailable
    memset(&config, 0, sizeof(config));
    return config;
  }

  // Read actual sensor values
  config.brightness = s->status.brightness;
  config.contrast = s->status.contrast;
  config.saturation = s->status.saturation;
  config.special_effect = s->status.special_effect;
  config.whitebal = s->status.awb;
  config.awb_gain = s->status.awb_gain;
  config.wb_mode = s->status.wb_mode;
  config.exposure_ctrl = s->status.aec;
  config.aec2 = s->status.aec2;
  config.ae_level = s->status.ae_level;
  config.aec_value = s->status.aec_value;
  config.gain_ctrl = s->status.agc;
  config.agc_gain = s->status.agc_gain;
  config.gainceiling = s->status.gainceiling;
  config.bpc = s->status.bpc;
  config.wpc = s->status.wpc;
  config.raw_gma = s->status.raw_gma;
  config.lenc = s->status.lenc;
  config.hmirror = s->status.hmirror;
  config.vflip = s->status.vflip;
  config.dcw = s->status.dcw;
  config.colorbar = s->status.colorbar;

  return config;
}

// Get default camera configuration (read from sensor's initial state)
CameraConfig getDefaultCameraConfig() {
  // Default means "use sensor's own defaults" - just read current values
  return readCurrentCameraConfig();
}

// Validate camera configuration values are in valid ranges
bool validateCameraConfig(const CameraConfig& config, String& errorMsg) {
  if (config.brightness < -2 || config.brightness > 2) {
    errorMsg = "brightness must be -2 to 2";
    return false;
  }
  if (config.contrast < -2 || config.contrast > 2) {
    errorMsg = "contrast must be -2 to 2";
    return false;
  }
  if (config.saturation < -2 || config.saturation > 2) {
    errorMsg = "saturation must be -2 to 2";
    return false;
  }
  if (config.special_effect > 6) {
    errorMsg = "special_effect must be 0 to 6";
    return false;
  }
  if (config.wb_mode > 4) {
    errorMsg = "wb_mode must be 0 to 4";
    return false;
  }
  if (config.ae_level < -2 || config.ae_level > 2) {
    errorMsg = "ae_level must be -2 to 2";
    return false;
  }
  if (config.aec_value > 1200) {
    errorMsg = "aec_value must be 0 to 1200";
    return false;
  }
  if (config.agc_gain > 30) {
    errorMsg = "agc_gain must be 0 to 30";
    return false;
  }
  if (config.gainceiling > 6) {
    errorMsg = "gainceiling must be 0 to 6";
    return false;
  }
  return true;
}

// Apply camera configuration to the camera sensor
bool applyCameraConfig(const CameraConfig& config) {
  sensor_t* s = esp_camera_sensor_get();
  if (!s) {
    logPrint(LOG_ERROR, "Failed to get camera sensor");
    return false;
  }

  s->set_brightness(s, config.brightness);
  s->set_contrast(s, config.contrast);
  s->set_saturation(s, config.saturation);
  s->set_special_effect(s, config.special_effect);
  s->set_whitebal(s, config.whitebal ? 1 : 0);
  s->set_awb_gain(s, config.awb_gain ? 1 : 0);
  s->set_wb_mode(s, config.wb_mode);
  s->set_exposure_ctrl(s, config.exposure_ctrl ? 1 : 0);
  s->set_aec2(s, config.aec2 ? 1 : 0);
  s->set_ae_level(s, config.ae_level);
  s->set_aec_value(s, config.aec_value);
  s->set_gain_ctrl(s, config.gain_ctrl ? 1 : 0);
  s->set_agc_gain(s, config.agc_gain);
  s->set_gainceiling(s, (gainceiling_t)config.gainceiling);
  s->set_bpc(s, config.bpc ? 1 : 0);
  s->set_wpc(s, config.wpc ? 1 : 0);
  s->set_raw_gma(s, config.raw_gma ? 1 : 0);
  s->set_lenc(s, config.lenc ? 1 : 0);
  s->set_hmirror(s, config.hmirror ? 1 : 0);
  s->set_vflip(s, config.vflip ? 1 : 0);
  s->set_dcw(s, config.dcw ? 1 : 0);
  s->set_colorbar(s, config.colorbar ? 1 : 0);

  logPrint(LOG_INFO, "Camera configuration applied successfully");
  return true;
}

// Serialize camera config to JSON string
String configToJSON(const CameraConfig& config) {
  JsonDocument doc;

  doc["brightness"] = config.brightness;
  doc["contrast"] = config.contrast;
  doc["saturation"] = config.saturation;
  doc["special_effect"] = config.special_effect;
  doc["whitebal"] = config.whitebal;
  doc["awb_gain"] = config.awb_gain;
  doc["wb_mode"] = config.wb_mode;
  doc["exposure_ctrl"] = config.exposure_ctrl;
  doc["aec2"] = config.aec2;
  doc["ae_level"] = config.ae_level;
  doc["aec_value"] = config.aec_value;
  doc["gain_ctrl"] = config.gain_ctrl;
  doc["agc_gain"] = config.agc_gain;
  doc["gainceiling"] = config.gainceiling;
  doc["bpc"] = config.bpc;
  doc["wpc"] = config.wpc;
  doc["raw_gma"] = config.raw_gma;
  doc["lenc"] = config.lenc;
  doc["hmirror"] = config.hmirror;
  doc["vflip"] = config.vflip;
  doc["dcw"] = config.dcw;
  doc["colorbar"] = config.colorbar;

  String output;
  serializeJson(doc, output);
  return output;
}

// Deserialize JSON string to camera config
bool configFromJSON(const String& jsonStr, CameraConfig& config, String& errorMsg) {
  JsonDocument doc;

  DeserializationError error = deserializeJson(doc, jsonStr);
  if (error) {
    errorMsg = "JSON parse error: ";
    errorMsg += error.c_str();
    return false;
  }

  // Parse all fields with type checking
  if (!doc["brightness"].is<int>()) {
    errorMsg = "Missing or invalid 'brightness' field";
    return false;
  }
  config.brightness = doc["brightness"];

  if (!doc["contrast"].is<int>()) {
    errorMsg = "Missing or invalid 'contrast' field";
    return false;
  }
  config.contrast = doc["contrast"];

  if (!doc["saturation"].is<int>()) {
    errorMsg = "Missing or invalid 'saturation' field";
    return false;
  }
  config.saturation = doc["saturation"];

  if (!doc["special_effect"].is<int>()) {
    errorMsg = "Missing or invalid 'special_effect' field";
    return false;
  }
  config.special_effect = doc["special_effect"];

  if (!doc["whitebal"].is<bool>()) {
    errorMsg = "Missing or invalid 'whitebal' field";
    return false;
  }
  config.whitebal = doc["whitebal"];

  if (!doc["awb_gain"].is<bool>()) {
    errorMsg = "Missing or invalid 'awb_gain' field";
    return false;
  }
  config.awb_gain = doc["awb_gain"];

  if (!doc["wb_mode"].is<int>()) {
    errorMsg = "Missing or invalid 'wb_mode' field";
    return false;
  }
  config.wb_mode = doc["wb_mode"];

  if (!doc["exposure_ctrl"].is<bool>()) {
    errorMsg = "Missing or invalid 'exposure_ctrl' field";
    return false;
  }
  config.exposure_ctrl = doc["exposure_ctrl"];

  if (!doc["aec2"].is<bool>()) {
    errorMsg = "Missing or invalid 'aec2' field";
    return false;
  }
  config.aec2 = doc["aec2"];

  if (!doc["ae_level"].is<int>()) {
    errorMsg = "Missing or invalid 'ae_level' field";
    return false;
  }
  config.ae_level = doc["ae_level"];

  if (!doc["aec_value"].is<int>()) {
    errorMsg = "Missing or invalid 'aec_value' field";
    return false;
  }
  config.aec_value = doc["aec_value"];

  if (!doc["gain_ctrl"].is<bool>()) {
    errorMsg = "Missing or invalid 'gain_ctrl' field";
    return false;
  }
  config.gain_ctrl = doc["gain_ctrl"];

  if (!doc["agc_gain"].is<int>()) {
    errorMsg = "Missing or invalid 'agc_gain' field";
    return false;
  }
  config.agc_gain = doc["agc_gain"];

  if (!doc["gainceiling"].is<int>()) {
    errorMsg = "Missing or invalid 'gainceiling' field";
    return false;
  }
  config.gainceiling = doc["gainceiling"];

  if (!doc["bpc"].is<bool>()) {
    errorMsg = "Missing or invalid 'bpc' field";
    return false;
  }
  config.bpc = doc["bpc"];

  if (!doc["wpc"].is<bool>()) {
    errorMsg = "Missing or invalid 'wpc' field";
    return false;
  }
  config.wpc = doc["wpc"];

  if (!doc["raw_gma"].is<bool>()) {
    errorMsg = "Missing or invalid 'raw_gma' field";
    return false;
  }
  config.raw_gma = doc["raw_gma"];

  if (!doc["lenc"].is<bool>()) {
    errorMsg = "Missing or invalid 'lenc' field";
    return false;
  }
  config.lenc = doc["lenc"];

  if (!doc["hmirror"].is<bool>()) {
    errorMsg = "Missing or invalid 'hmirror' field";
    return false;
  }
  config.hmirror = doc["hmirror"];

  if (!doc["vflip"].is<bool>()) {
    errorMsg = "Missing or invalid 'vflip' field";
    return false;
  }
  config.vflip = doc["vflip"];

  if (!doc["dcw"].is<bool>()) {
    errorMsg = "Missing or invalid 'dcw' field";
    return false;
  }
  config.dcw = doc["dcw"];

  if (!doc["colorbar"].is<bool>()) {
    errorMsg = "Missing or invalid 'colorbar' field";
    return false;
  }
  config.colorbar = doc["colorbar"];

  // Validate the parsed config
  if (!validateCameraConfig(config, errorMsg)) {
    return false;
  }

  return true;
}

// Save camera config to NVM (non-volatile memory) with ETag
bool saveConfigToNVM(const CameraConfig& config, const String& etag) {
  Preferences prefs;
  prefs.begin("cam-config", false);

  // Serialize config to JSON
  String jsonStr = configToJSON(config);

  // Save JSON string and ETag
  bool success = true;
  success &= prefs.putString("config", jsonStr);
  success &= prefs.putString("etag", etag);

  prefs.end();

  if (success) {
    logPrintf(LOG_INFO, "Camera config saved to NVM (etag: %s)", etag.c_str());
  } else {
    logPrint(LOG_ERROR, "Failed to save camera config to NVM");
  }

  return success;
}

// Load camera config from NVM with ETag
bool loadConfigFromNVM(CameraConfig& config, String& etag, String& errorMsg) {
  Preferences prefs;
  prefs.begin("cam-config", true);  // Read-only mode

  String jsonStr = prefs.getString("config", "");
  etag = prefs.getString("etag", "");

  prefs.end();

  if (jsonStr.isEmpty()) {
    errorMsg = "No camera config found in NVM";
    return false;
  }

  // Deserialize JSON
  if (!configFromJSON(jsonStr, config, errorMsg)) {
    logPrintf(LOG_ERROR, "Failed to parse NVM config: %s", errorMsg.c_str());
    return false;
  }

  logPrintf(LOG_INFO, "Camera config loaded from NVM (etag: %s)", etag.c_str());
  return true;
}

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

// Camera configuration state
CameraConfig currentCameraConfig;
String cameraConfigSource = "default";  // "default", "nvm", or "s3"
String cameraConfigETag = "";  // ETag of the config currently in use
unsigned long lastCameraConfigCheck = 0;  // Track last time we checked S3 for config updates

// Logging functions
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

// Download file from S3 with AWS Signature V4 authentication
// Returns true on success, false on failure (including 404)
// On success: content contains file data, etag contains ETag header
bool downloadFromS3(const String& filename, String& content, String& etag, String& errorMsg) {
  // Ensure WiFi is connected
  if (!connectWiFi()) {
    errorMsg = "WiFi not connected";
    return false;
  }

  // Verify time is set (required for AWS Signature V4)
  time_t now = time(nullptr);
  if (now < 100000) {
    errorMsg = "Time not synchronized";
    return false;
  }

  // Build S3 path with folder (if configured)
  String uri = "/";
  if (strlen(S3_FOLDER) > 0) {
    uri += String(S3_FOLDER) + "/";
  }
  uri += filename;

  String host = String(S3_BUCKET) + ".s3." + AWS_REGION + ".amazonaws.com";
  String url = "https://" + host + uri;

  logPrintf(LOG_DEBUG, "Downloading from S3: %s", uri.c_str());

  // Generate AWS Signature V4 authentication headers for GET
  char authHeader[400];
  char amzDate[18];
  char payloadHash[65];

  // For GET, payload is empty
  const uint8_t emptyPayload[] = "";
  generateAWSSignatureV4("GET", host.c_str(), uri.c_str(),
                         AWS_REGION, AWS_ACCESS_KEY_ID, AWS_SECRET_ACCESS_KEY,
                         emptyPayload, 0,
                         authHeader, amzDate, payloadHash);

  // Make authenticated GET request
  HTTPClient http;
  http.begin(url);

  // Set timeouts
  http.setConnectTimeout(10000);  // 10 seconds to establish connection
  http.setTimeout(30000);         // 30 seconds for data transfer

  http.addHeader("Host", host);
  http.addHeader("x-amz-date", amzDate);
  http.addHeader("x-amz-content-sha256", payloadHash);
  http.addHeader("Authorization", authHeader);

  logPrint(LOG_DEBUG, "Sending GET request with AWS Signature V4...");

  int httpResponseCode = http.GET();

  if (httpResponseCode == 200) {
    // Success - get content and ETag
    content = http.getString();
    etag = http.header("ETag");

    // Remove quotes from ETag if present
    if (etag.startsWith("\"") && etag.endsWith("\"")) {
      etag = etag.substring(1, etag.length() - 1);
    }

    http.end();
    logPrintf(LOG_INFO, "Downloaded from S3: %s (%d bytes, ETag: %s)",
              filename.c_str(), content.length(), etag.c_str());
    return true;
  } else if (httpResponseCode == 404) {
    // File not found - not an error, just doesn't exist
    http.end();
    errorMsg = "File not found (404)";
    logPrintf(LOG_DEBUG, "S3 file not found: %s", filename.c_str());
    return false;
  } else if (httpResponseCode > 0) {
    // Got HTTP response but it's an error
    String responseBody = http.getString();
    http.end();

    errorMsg = "HTTP ";
    errorMsg += String(httpResponseCode);
    if (responseBody.length() > 0 && responseBody.length() <= 200) {
      errorMsg += ": ";
      errorMsg += responseBody;
    }

    logPrintf(LOG_ERROR, "S3 download failed: HTTP %d for %s", httpResponseCode, filename.c_str());
    return false;
  } else {
    // Network/connection error
    errorMsg = "Network error: ";
    errorMsg += http.errorToString(httpResponseCode);
    http.end();

    logPrintf(LOG_ERROR, "S3 download network error: %s", http.errorToString(httpResponseCode).c_str());
    return false;
  }
}

// Upload JSON string to S3
bool uploadJSONToS3(const String& jsonContent, const String& filename) {
  // Ensure WiFi is connected
  if (!connectWiFi()) {
    return false;
  }

  // Verify time is set (required for AWS Signature V4)
  time_t now = time(nullptr);
  if (now < 100000) {
    logPrint(LOG_ERROR, "Time not synchronized for S3 upload");
    return false;
  }

  // Build S3 path with folder (if configured)
  String uri = "/";
  if (strlen(S3_FOLDER) > 0) {
    uri += String(S3_FOLDER) + "/";
  }
  uri += filename;

  String host = String(S3_BUCKET) + ".s3." + AWS_REGION + ".amazonaws.com";
  String url = "https://" + host + uri;

  logPrintf(LOG_DEBUG, "Uploading JSON to S3: %s", filename.c_str());

  // Generate AWS Signature V4 authentication headers
  char authHeader[400];
  char amzDate[18];
  char payloadHash[65];

  generateAWSSignatureV4("PUT", host.c_str(), uri.c_str(),
                         AWS_REGION, AWS_ACCESS_KEY_ID, AWS_SECRET_ACCESS_KEY,
                         (const uint8_t*)jsonContent.c_str(), jsonContent.length(),
                         authHeader, amzDate, payloadHash);

  // Make authenticated request
  HTTPClient http;
  http.begin(url);

  http.setConnectTimeout(10000);
  http.setTimeout(30000);

  http.addHeader("Host", host);
  http.addHeader("x-amz-date", amzDate);
  http.addHeader("x-amz-content-sha256", payloadHash);
  http.addHeader("Authorization", authHeader);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Content-Length", String(jsonContent.length()));

  int httpResponseCode = http.PUT((uint8_t*)jsonContent.c_str(), jsonContent.length());

  http.end();

  if (httpResponseCode >= 200 && httpResponseCode < 300) {
    logPrintf(LOG_DEBUG, "JSON uploaded to S3: %s", filename.c_str());
    return true;
  } else {
    logPrintf(LOG_WARNING, "JSON upload failed: HTTP %d for %s", httpResponseCode, filename.c_str());
    return false;
  }
}

// Load camera configuration at boot with cascade: S3 → NVM → defaults
void loadCameraConfigAtBoot() {
  logPrint(LOG_INFO, "Loading camera configuration...");

  CameraConfig config;
  String etag;
  String errorMsg;
  bool configLoaded = false;

  // Step 1: Try to load from S3 (camera.json)
  String s3Content;
  String s3ETag;
  if (downloadFromS3("camera.json", s3Content, s3ETag, errorMsg)) {
    // S3 download successful - check if it's newer than NVM
    String nvmETag;
    CameraConfig nvmConfig;
    String nvmError;

    bool nvmExists = loadConfigFromNVM(nvmConfig, nvmETag, nvmError);

    if (!nvmExists || s3ETag != nvmETag) {
      // S3 is new or different from NVM - try to parse it
      if (configFromJSON(s3Content, config, errorMsg)) {
        logPrintf(LOG_INFO, "Loaded camera config from S3 (ETag: %s)", s3ETag.c_str());
        cameraConfigSource = "s3";
        cameraConfigETag = s3ETag;
        configLoaded = true;

        // Save to NVM for future use
        saveConfigToNVM(config, s3ETag);
      } else {
        logPrintf(LOG_ERROR, "S3 config parse error: %s", errorMsg.c_str());
        // Fall through to try NVM
      }
    } else {
      logPrintf(LOG_INFO, "S3 config unchanged (ETag: %s), using NVM", s3ETag.c_str());
      config = nvmConfig;
      cameraConfigSource = "s3-cached";
      cameraConfigETag = nvmETag;
      configLoaded = true;
    }
  } else {
    // Check if it's 404 (intentional deletion = reset) vs error (temporary problem)
    if (errorMsg == "File not found (404)") {
      logPrint(LOG_INFO, "camera.json not found in S3 - clearing NVM cache and using defaults");
      // Clear NVM cache
      Preferences prefs;
      prefs.begin("cam-config", false);
      prefs.clear();
      prefs.end();
      // Skip NVM, go straight to defaults (configLoaded stays false)
    } else {
      logPrintf(LOG_DEBUG, "S3 config not available: %s", errorMsg.c_str());
      // Temporary error - try NVM in Step 2
    }
  }

  // Step 2: If S3 failed (but not 404), try NVM
  if (!configLoaded) {
    if (loadConfigFromNVM(config, etag, errorMsg)) {
      logPrintf(LOG_INFO, "Loaded camera config from NVM (ETag: %s)", etag.c_str());
      cameraConfigSource = "s3-cached";
      cameraConfigETag = etag;
      configLoaded = true;
    } else {
      logPrintf(LOG_DEBUG, "NVM config not available: %s", errorMsg.c_str());
    }
  }

  // Step 3: If both failed, use defaults
  if (!configLoaded) {
    config = getDefaultCameraConfig();
    logPrint(LOG_INFO, "Using default camera configuration");
    cameraConfigSource = "default";
    cameraConfigETag = "";
  }

  // Apply the configuration (unless using defaults, which means don't modify sensor)
  if (cameraAvailable && configLoaded) {
    if (!applyCameraConfig(config)) {
      logPrint(LOG_ERROR, "Failed to apply camera configuration");
    }
  }

  // Read back actual sensor values (hardware truth) and upload as camera.use.json
  if (cameraAvailable) {
    CameraConfig actualConfig = readCurrentCameraConfig();
    currentCameraConfig = actualConfig;

    String useJSON = configToJSON(actualConfig);
    uploadJSONToS3(useJSON, "camera.use.json");
  } else {
    // Camera not available - just store what we have
    currentCameraConfig = config;
  }
}

// Check for camera config updates from S3 (called periodically)
void checkCameraConfigUpdate() {
  // Only check if camera is available and WiFi connected
  if (!cameraAvailable || !IsWiFiConnected()) {
    return;
  }

  logPrint(LOG_DEBUG, "Checking S3 for camera config updates...");

  String s3Content;
  String s3ETag;
  String errorMsg;

  if (downloadFromS3("camera.json", s3Content, s3ETag, errorMsg)) {
    // Check if ETag changed
    if (s3ETag != cameraConfigETag) {
      logPrintf(LOG_INFO, "New camera config detected (ETag: %s -> %s)",
                cameraConfigETag.c_str(), s3ETag.c_str());

      // Try to parse new config
      CameraConfig newConfig;
      if (configFromJSON(s3Content, newConfig, errorMsg)) {
        // Apply new configuration
        if (applyCameraConfig(newConfig)) {
          logPrint(LOG_INFO, "New camera configuration applied successfully");

          // Update state
          cameraConfigSource = "s3";
          cameraConfigETag = s3ETag;

          // Save to NVM
          saveConfigToNVM(newConfig, s3ETag);

          // Read back actual hardware values and upload
          CameraConfig actualConfig = readCurrentCameraConfig();
          currentCameraConfig = actualConfig;

          String useJSON = configToJSON(actualConfig);
          uploadJSONToS3(useJSON, "camera.use.json");
        } else {
          logPrint(LOG_ERROR, "Failed to apply new camera configuration");
        }
      } else {
        logPrintf(LOG_ERROR, "Invalid camera config from S3: %s", errorMsg.c_str());
      }
    } else {
      logPrint(LOG_DEBUG, "Camera config unchanged");
    }
  } else {
    // Check if it's 404 (intentional deletion = reset) vs error (temporary problem)
    if (errorMsg == "File not found (404)") {
      logPrint(LOG_INFO, "camera.json deleted from S3 - clearing NVM cache and resetting to defaults");

      // Clear NVM cache
      Preferences prefs;
      prefs.begin("cam-config", false);
      prefs.clear();
      prefs.end();

      // Read current sensor defaults (don't modify sensor)
      CameraConfig defaultConfig = readCurrentCameraConfig();
      currentCameraConfig = defaultConfig;

      // Update state
      cameraConfigSource = "default";
      cameraConfigETag = "";

      // Upload actual sensor state
      String useJSON = configToJSON(defaultConfig);
      uploadJSONToS3(useJSON, "camera.use.json");

      logPrint(LOG_INFO, "Camera reset to defaults complete");
    } else {
      logPrintf(LOG_DEBUG, "Camera config check failed: %s", errorMsg.c_str());
      // Temporary error - keep current config
    }
  }
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

    logPrintf(LOG_INFO, "PSRAM found: %d (%d)", ESP.getPsramSize(), ESP.getFreePsram());

  } else {
    config.frame_size = FRAMESIZE_SVGA;  // 800x600
    config.jpeg_quality = 12;
    config.fb_count = 1;

        logPrintf(LOG_INFO, "No PSRAM");
  }

  // Initialize camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("ERROR: Camera init failed with error 0x%x\n", err);
    return false;
  }

  // Apply physical mounting correction - camera is mounted upside down
  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_vflip(s, 1);  // Flip vertically for upside-down mounting
    s->set_hmirror(s, 1); // Flip horizontally for on-site mounting position
    Serial.println("Camera initialized successfully (vflip applied for mounting)");
  } else {
    Serial.println("Camera initialized successfully (warning: could not apply vflip)");
  }

  // Prime frame buffers to clear garbage/uninitialized data
  // With fb_count=2 (PSRAM mode), both buffers need to be cleared once at boot
  if (psramFound()) {
    Serial.println("Priming 2-frame buffers (clearing garbage data)...");

    // Discard first garbage frame
    camera_fb_t* fb1 = esp_camera_fb_get();
    if (fb1) {
      esp_camera_fb_return(fb1);
    }

    // Discard second garbage frame
    camera_fb_t* fb2 = esp_camera_fb_get();
    if (fb2) {
      esp_camera_fb_return(fb2);
    }

    Serial.println("Frame buffers primed");
  }

  return true;
}

void flashOn() {
  digitalWrite(FLASH_LED_PIN, HIGH);
}

void flashOff() {
  digitalWrite(FLASH_LED_PIN, LOW);
}

camera_fb_t* capturePhoto() {
   camera_fb_t* fb = nullptr;
   
  // force out the stale internal capture(s)
  for(int i=0;i<5;++i) {
    fb = esp_camera_fb_get();
    if (fb) {
      esp_camera_fb_return(fb);
      fb = nullptr;
    }
  }
  
  // Turn on flash
  flashOn();
  logPrint(LOG_DEBUG, "Flash ON");

  // Delay for camera sensors to adjust to lighting
  delay(200);

  // Capture photo
  // Note: Frame buffers are primed once at boot in initCamera()
  fb = esp_camera_fb_get();

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

bool uploadStatusToS3(const String& filename, const ImageQualityMetrics& stats) {
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
  String statusJSON = generateStatusJSON(stats);
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
  if (RELAY_PIN) {
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW); // Start with blanket off
  }

  // Configure PIR sensor pin as input
  if (PIR_PIN) {
    pinMode(PIR_PIN, INPUT);
  }

  // Configure flash LED pin as output
  if (FLASH_LED_PIN) {
    pinMode(FLASH_LED_PIN, OUTPUT);
    digitalWrite(FLASH_LED_PIN, LOW); // Start with flash off
  }

  if (DHT_PIN) {
    dht = new DHT(DHT_PIN, DHT22);
  }

  logPrintf(LOG_INFO, "GPIO pins configured:");
  logPrintf(LOG_INFO, "- RELAY_PIN (%i): OUTPUT", RELAY_PIN);
  logPrintf(LOG_INFO, "- PIR_PIN (%i): INPUT", PIR_PIN);
  logPrintf(LOG_INFO, "- DHT_PIN (%i, %p): DHT22 sensor", DHT_PIN, dht);
  logPrintf(LOG_INFO, "- FLASH_LED_PIN (%i): OUTPUT", FLASH_LED_PIN);
}

bool readPIRSensor() {
    int pirState = digitalRead(PIR_PIN);
    return pirState == HIGH;
}

void checkPIRSensor() {
  unsigned long currentMillis = millis();

  // Read PIR sensor state
  bool motionDetected = readPIRSensor();

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

  if (dht == nullptr) {
    return;
  }

  unsigned long currentMillis = millis();

  // Check if it's time to read the sensor
  if (currentMillis - lastDHTRead >= DHT_READ_INTERVAL) {
    lastDHTRead = currentMillis;

    // Read temperature and humidity
    float temp = dht->readTemperature();
    float humidity = dht->readHumidity();

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

bool uploadFbTimeS3(camera_fb_t* fb, struct tm& timeinfo)
{
  if (fb == nullptr) {
    return false;
  }

  char buffer[64];
  strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", &timeinfo);
  String strTime(buffer);

  String baseFilename = "cat_" + strTime;
  String photoFilename = baseFilename + ".jpg";
  String jsonFilename = baseFilename + ".json";

  bool photoSuccess = uploadPhotoToS3(fb, photoFilename);
  bool jsonSuccess = false;
  if (photoSuccess) {
    ImageAnalyzer analizer;
    auto stats = analizer.analyze(fb);    
    jsonSuccess = uploadStatusToS3(jsonFilename, stats);  
  }

  logPrintf(LOG_INFO, "Photo upload: %s [%d, %d]", baseFilename.c_str(), photoSuccess, jsonSuccess);

  return photoSuccess;
}

bool takeAndUploadPhoto(const char* reason) {
  // Skip if camera not available (safe mode or camera failed)
  if (!cameraAvailable || !IsWiFiConnected()) {
    return false;
  }

  logPrintf(LOG_INFO, "Taking photo (%s)...", reason);

  // Generate base filename with timestamp (without extension)
  String baseFilename = "cat_" + getTimestamp();
  String photoFilename = baseFilename + ".jpg";
  String jsonFilename = baseFilename + ".json";

  bool photoSuccess = false;

  camera_fb_t* fb = capturePhoto();
  if (fb) {
    ImageAnalyzer analizer;
    auto stats = analizer.analyze(fb);

    photoSuccess = uploadPhotoToS3(fb, photoFilename);
    releasePhoto(fb);

    if (photoSuccess) {
      logPrint(LOG_INFO, "Photo uploaded successfully!");
      lastWiFiActivity = millis();  // Update activity timestamp

      // Upload status JSON with same base filename
      bool jsonSuccess = uploadStatusToS3(jsonFilename, stats);
      if (jsonSuccess) {
        logPrint(LOG_INFO, "Status JSON uploaded successfully!");
      } else {
        logPrint(LOG_WARNING, "Status JSON upload failed (photo was uploaded)");
      }
    } else {
      logPrint(LOG_WARNING, "Photo upload failed!");
    }
  }
  return photoSuccess;
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
      if (!WiFi.isConnected()) {
        Serial.println("WiFi not connected.");
        return;
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

String generateStatusJSON(const ImageQualityMetrics& stats) {
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
  json += "  \"camera_config_source\": \"" + cameraConfigSource + "\",\n";
  if (cameraConfigETag.length() > 0) {
    json += "  \"camera_config_etag\": \"" + cameraConfigETag + "\",\n";
  }

  json+= "  \"image_quality_metrics\": {\n"
      "    \"brightness\": " + String(stats.brightness) +",\n"
      "    \"contrast\": " + String(stats.contrast) +",\n"
      "    \"isBright\": " + String(stats.isBright) +",\n"
      "    \"isDark\": " + String(stats.isDark) +",\n"
      "    \"noiseLevel\": " + String(stats.noiseLevel) +",\n"
      "    \"overexposure\": " + String(stats.overexposure) +",\n"
      "    \"qualityScore\": " + String(stats.qualityScore) +",\n"
      "    \"sharpness\": " + String(stats.sharpness) +",\n"
      "    \"underexposure\": " + String(stats.underexposure) +"\n"
    "  },\n";

  // Add current camera sensor values (hardware truth at time of photo)
  if (cameraAvailable) {
    CameraConfig actualConfig = readCurrentCameraConfig();
    json += "  \"camera_config\": {\n";
    json += "    \"brightness\": " + String(actualConfig.brightness) + ",\n";
    json += "    \"contrast\": " + String(actualConfig.contrast) + ",\n";
    json += "    \"saturation\": " + String(actualConfig.saturation) + ",\n";
    json += "    \"special_effect\": " + String(actualConfig.special_effect) + ",\n";
    json += "    \"whitebal\": " + String(actualConfig.whitebal ? "true" : "false") + ",\n";
    json += "    \"awb_gain\": " + String(actualConfig.awb_gain ? "true" : "false") + ",\n";
    json += "    \"wb_mode\": " + String(actualConfig.wb_mode) + ",\n";
    json += "    \"exposure_ctrl\": " + String(actualConfig.exposure_ctrl ? "true" : "false") + ",\n";
    json += "    \"aec2\": " + String(actualConfig.aec2 ? "true" : "false") + ",\n";
    json += "    \"ae_level\": " + String(actualConfig.ae_level) + ",\n";
    json += "    \"aec_value\": " + String(actualConfig.aec_value) + ",\n";
    json += "    \"gain_ctrl\": " + String(actualConfig.gain_ctrl ? "true" : "false") + ",\n";
    json += "    \"agc_gain\": " + String(actualConfig.agc_gain) + ",\n";
    json += "    \"gainceiling\": " + String(actualConfig.gainceiling) + ",\n";
    json += "    \"bpc\": " + String(actualConfig.bpc ? "true" : "false") + ",\n";
    json += "    \"wpc\": " + String(actualConfig.wpc ? "true" : "false") + ",\n";
    json += "    \"raw_gma\": " + String(actualConfig.raw_gma ? "true" : "false") + ",\n";
    json += "    \"lenc\": " + String(actualConfig.lenc ? "true" : "false") + ",\n";
    json += "    \"hmirror\": " + String(actualConfig.hmirror ? "true" : "false") + ",\n";
    json += "    \"vflip\": " + String(actualConfig.vflip ? "true" : "false") + ",\n";
    json += "    \"dcw\": " + String(actualConfig.dcw ? "true" : "false") + ",\n";
    json += "    \"colorbar\": " + String(actualConfig.colorbar ? "true" : "false") + "\n";
    json += "  },\n";
  }

  json += "  \"wifi_connected\": " + String(WiFi.isConnected() ? "true" : "false") + ",\n";
  json += "  \"wifi_manual_override\": " + String(wifiManualOverride ? "true" : "false") + ",\n";

  // Memory status (for leak detection)
  json += "  \"heap_free_bytes\": " + String(ESP.getFreeHeap()) + ",\n";
  json += "  \"heap_size_bytes\": " + String(ESP.getHeapSize()) + ",\n";
  json += "  \"heap_min_free_bytes\": " + String(ESP.getMinFreeHeap()) + ",\n";
  json += "  \"psram_free_bytes\": " + String(ESP.getFreePsram()) + ",\n";
  json += "  \"psram_size_bytes\": " + String(ESP.getPsramSize()) +",\n";
  json += "  \"chip_temperature\": " + String(getChipTemperature());

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
    if (cameraAvailable) {
      Serial.printf("Camera Config: %s", cameraConfigSource.c_str());
      if (cameraConfigETag.length() > 0) {
        Serial.printf(" (ETag: %s)", cameraConfigETag.c_str());
      }
      Serial.println();
    }
    if (wifiManualOverride) {
      Serial.printf("WiFi: %s (MANUAL OVERRIDE)\n", WiFi.isConnected() ? "CONNECTED" : "DISCONNECTED");
    } else {
      Serial.printf("WiFi: %s (automatic)\n", WiFi.isConnected() ? "CONNECTED" : "DISCONNECTED");
    }

    // Memory status (for leak detection)
    Serial.println("--- Memory Status ---");
    Serial.printf("Heap Free: %u bytes (%.1f KB)\n", ESP.getFreeHeap(), ESP.getFreeHeap() / 1024.0);
    Serial.printf("Heap Size: %u bytes (%.1f KB)\n", ESP.getHeapSize(), ESP.getHeapSize() / 1024.0);
    Serial.printf("Heap Min Free: %u bytes (%.1f KB)\n", ESP.getMinFreeHeap(), ESP.getMinFreeHeap() / 1024.0);
    Serial.printf("PSRAM Free: %u bytes (%.1f KB)\n", ESP.getFreePsram(), ESP.getFreePsram() / 1024.0);
    Serial.printf("PSRAM Size: %u bytes (%.1f KB)\n", ESP.getPsramSize(), ESP.getPsramSize() / 1024.0);
    Serial.printf("Chip Temperature: %.2f celsius\n", getChipTemperature());
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
