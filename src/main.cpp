#include <Arduino.h>
#include <DHT.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <mbedtls/md.h>
#include <Preferences.h>
#include "esp_camera.h"
#include "esp_wifi.h"
#include "secrets.h"  // WiFi credentials (not in git)

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
#define PHOTO_HOURLY_INTERVAL 3600000  // 60 minutes in milliseconds
#define PHOTO_MOTION_COOLDOWN 300000   // 5 minutes in milliseconds
#define STATUS_REPORT_INTERVAL 60000   // Status report every 60 seconds

// Temperature threshold for blanket control (in Celsius)
#define TEMP_COLD_THRESHOLD 10.0  // Turn on blanket if temp is below this and cat present

// Boot and recovery settings
#define MAX_BOOT_ATTEMPTS 3       // Max failed boots before entering safe mode
#define BOOT_SUCCESS_TIMEOUT 300000  // 5 minutes - if running this long, boot is successful

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
float currentTemp = 0.0;
float currentHumidity = 0.0;
unsigned long lastDHTRead = 0;
bool wifiConnected = false;
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
void printStatusReport();

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
  safeMode = false;
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

  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);

  // Disconnect first to ensure clean state
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(500);

  // Configure WiFi
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.setHostname("cat-shelter");

  // Start WiFi connection
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

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
    Serial.println("\nWiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Signal strength: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
    return true;
  } else {
    Serial.println("\nWiFi connection failed!");
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

String getTimestamp() {
  time_t now = time(nullptr);
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  char buffer[64];
  strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", &timeinfo);
  return String(buffer);
}

bool uploadPhotoToS3(camera_fb_t* fb) {
  if (!fb || fb->len == 0) {
    Serial.println("ERROR: Invalid photo buffer");
    return false;
  }

  // Ensure WiFi is connected
  if (!connectWiFi()) {
    return false;
  }

  // Generate filename with timestamp
  String filename = "cat_" + getTimestamp() + ".jpg";
  String url = String("https://") + S3_BUCKET + ".s3." + AWS_REGION + ".amazonaws.com/" + filename;

  Serial.print("Uploading photo to S3: ");
  Serial.println(filename);

  HTTPClient http;
  http.begin(url);
  http.addHeader("Content-Type", "image/jpeg");
  http.addHeader("Content-Length", String(fb->len));

  // Note: For production, you should implement AWS Signature V4
  // This is a simplified version that requires S3 bucket to accept unsigned uploads
  // or use pre-signed URLs generated by a backend service

  int httpResponseCode = http.PUT(fb->buf, fb->len);

  if (httpResponseCode > 0) {
    Serial.printf("Upload successful! HTTP Response code: %d\n", httpResponseCode);
    http.end();
    return true;
  } else {
    Serial.printf("Upload failed! Error: %s\n", http.errorToString(httpResponseCode).c_str());
    http.end();
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
  lastHourlyPhotoTime = millis();  // Start counting from boot for hourly photos

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
    Serial.println("Synchronizing time with NTP...");
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");

    // Wait for time to be set
    int attempts = 0;
    while (time(nullptr) < 100000 && attempts < 10) {
      delay(500);
      Serial.print(".");
      attempts++;
    }
    Serial.println("\nTime synchronized");

    // Disconnect WiFi to save power
    disconnectWiFi();
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
  // Read PIR sensor state
  int pirState = digitalRead(PIR_PIN);
  bool motionDetected = (pirState == HIGH);

  // Update cat presence state if it changed
  if (motionDetected != catPresent) {
    catPresent = motionDetected;

    if (catPresent) {
      logPrint(LOG_INFO, "*** CAT DETECTED! ***");
    } else {
      logPrint(LOG_INFO, "Cat left");
    }
  }
}

void controlBlanket(bool shouldBeOn) {
  if (shouldBeOn != blanketOn) {
    blanketOn = shouldBeOn;
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
      logPrint(LOG_ERROR, "Failed to read from DHT22 sensor!");
      return;
    }

    // Update global state
    currentTemp = temp;
    currentHumidity = humidity;

    // Log readings at DEBUG level (happens every 2 seconds, too verbose for INFO)
    logPrintf(LOG_DEBUG, "Temperature: %.1f°C | Humidity: %.1f%%", currentTemp, currentHumidity);
  }
}

void updateBlanketControl() {
  // Blanket should be on if:
  // 1. Cat is present
  // 2. Temperature is below threshold
  bool shouldBeOn = catPresent && (currentTemp < TEMP_COLD_THRESHOLD);
  controlBlanket(shouldBeOn);
}

void takeAndUploadPhoto(const char* reason) {
  // Skip if camera not available (safe mode or camera failed)
  if (!cameraAvailable) {
    return;
  }

  logPrintf(LOG_INFO, "Taking photo (%s)...", reason);

  camera_fb_t* fb = capturePhoto();
  if (fb) {
    bool success = uploadPhotoToS3(fb);
    releasePhoto(fb);

    if (success) {
      logPrint(LOG_INFO, "Photo uploaded successfully!");
    } else {
      logPrint(LOG_WARNING, "Photo upload failed!");
    }

    // Disconnect WiFi after upload to save power
    disconnectWiFi();
  }
}

void checkPhotoSchedule() {
  // Skip photo scheduling if camera not available
  if (!cameraAvailable) {
    return;
  }

  unsigned long currentMillis = millis();

  // Check for hourly photo
  if (currentMillis - lastHourlyPhotoTime >= PHOTO_HOURLY_INTERVAL) {
    lastHourlyPhotoTime = currentMillis;
    lastPhotoTime = currentMillis;
    takeAndUploadPhoto("hourly");
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
      Serial.println("loglevel <n>  - Set log level (0=ERROR, 1=WARN, 2=INFO, 3=DEBUG)");
      Serial.println("reboot        - Reboot system");
      Serial.println("safemode      - Enter safe mode");
      Serial.println("reset         - Reset boot counter");
      Serial.println("========================\n");
    }
    else if (command == "status") {
      printStatusReport();
    }
    else if (command == "snapshot") {
      if (cameraAvailable) {
        logPrint(LOG_INFO, "Manual snapshot triggered");
        takeAndUploadPhoto("manual");
      } else {
        logPrint(LOG_ERROR, "Camera not available (safe mode or init failed)");
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

void printStatusReport() {
  unsigned long currentMillis = millis();

  if (currentMillis - lastStatusReport >= STATUS_REPORT_INTERVAL) {
    lastStatusReport = currentMillis;

    Serial.println("\n===== STATUS REPORT =====");
    Serial.printf("Mode: %s\n", safeMode ? "SAFE MODE" : "NORMAL");
    Serial.printf("Uptime: %lu seconds\n", currentMillis / 1000);
    Serial.printf("Boot attempts: %d/%d\n", bootAttempts, MAX_BOOT_ATTEMPTS);
    Serial.printf("Temperature: %.1f°C\n", currentTemp);
    Serial.printf("Humidity: %.1f%%\n", currentHumidity);
    Serial.printf("Cat Present: %s\n", catPresent ? "YES" : "NO");
    Serial.printf("Blanket: %s\n", blanketOn ? "ON" : "OFF");
    Serial.printf("Camera: %s\n", cameraAvailable ? "AVAILABLE" : "DISABLED");
    Serial.printf("WiFi: %s\n", wifiConnected ? "CONNECTED" : "DISCONNECTED");

    if (cameraAvailable) {
      unsigned long nextHourlyPhoto = PHOTO_HOURLY_INTERVAL - (currentMillis - lastHourlyPhotoTime);
      Serial.printf("Next hourly photo in: %lu minutes\n", nextHourlyPhoto / 60000);

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