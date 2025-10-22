#include <Arduino.h>
#include <DHT.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <mbedtls/md.h>
#include "esp_camera.h"
#include "esp_wifi.h"
#include "secrets.h"  // WiFi credentials (not in git)

// GPIO Pin Definitions
#define RELAY_PIN 12        // Relay control for heated blanket
#define PIR_PIN 13          // PIR motion sensor (cat detection)
#define DHT_PIN 14          // DHT22 temperature/humidity sensor

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

// Global state variables
bool catPresent = false;
bool blanketOn = false;
float currentTemp = 0.0;
float currentHumidity = 0.0;
unsigned long lastDHTRead = 0;
bool wifiConnected = false;
unsigned long lastPhotoTime = 0;
unsigned long lastHourlyPhotoTime = 0;
bool lastCatPresent = false;
unsigned long lastStatusReport = 0;

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

camera_fb_t* capturePhoto() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("ERROR: Camera capture failed");
    return nullptr;
  }

  Serial.printf("Photo captured: %d bytes\n", fb->len);
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

  Serial.println("GPIO pins configured:");
  Serial.println("- RELAY_PIN (GPIO12): OUTPUT");
  Serial.println("- PIR_PIN (GPIO13): INPUT");
  Serial.println("- DHT_PIN (GPIO14): DHT22 sensor");
}

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n=== Cat Shelter Controller Starting ===");

  // Initialize camera first
  if (!initCamera()) {
    Serial.println("FATAL: Camera initialization failed!");
    while (1) delay(1000);  // Halt if camera fails
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
      Serial.println("*** CAT DETECTED! ***");
    } else {
      Serial.println("--- Cat left ---");
    }
  }
}

void controlBlanket(bool shouldBeOn) {
  if (shouldBeOn != blanketOn) {
    blanketOn = shouldBeOn;
    digitalWrite(RELAY_PIN, blanketOn ? HIGH : LOW);

    if (blanketOn) {
      Serial.println(">>> BLANKET TURNED ON <<<");
    } else {
      Serial.println("<<< BLANKET TURNED OFF >>>");
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
      Serial.println("ERROR: Failed to read from DHT22 sensor!");
      return;
    }

    // Update global state
    currentTemp = temp;
    currentHumidity = humidity;

    // Log readings
    Serial.print("Temperature: ");
    Serial.print(currentTemp);
    Serial.print("°C | Humidity: ");
    Serial.print(currentHumidity);
    Serial.println("%");
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
  Serial.printf("Taking photo (%s)...\n", reason);

  camera_fb_t* fb = capturePhoto();
  if (fb) {
    bool success = uploadPhotoToS3(fb);
    releasePhoto(fb);

    if (success) {
      Serial.println("Photo uploaded successfully!");
    } else {
      Serial.println("Photo upload failed!");
    }

    // Disconnect WiFi after upload to save power
    disconnectWiFi();
  }
}

void checkPhotoSchedule() {
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

void printStatusReport() {
  unsigned long currentMillis = millis();

  if (currentMillis - lastStatusReport >= STATUS_REPORT_INTERVAL) {
    lastStatusReport = currentMillis;

    Serial.println("\n===== STATUS REPORT =====");
    Serial.printf("Uptime: %lu seconds\n", currentMillis / 1000);
    Serial.printf("Temperature: %.1f°C\n", currentTemp);
    Serial.printf("Humidity: %.1f%%\n", currentHumidity);
    Serial.printf("Cat Present: %s\n", catPresent ? "YES" : "NO");
    Serial.printf("Blanket: %s\n", blanketOn ? "ON" : "OFF");
    Serial.printf("WiFi: %s\n", wifiConnected ? "CONNECTED" : "DISCONNECTED");

    unsigned long nextHourlyPhoto = PHOTO_HOURLY_INTERVAL - (currentMillis - lastHourlyPhotoTime);
    Serial.printf("Next hourly photo in: %lu minutes\n", nextHourlyPhoto / 60000);

    unsigned long timeSinceLastPhoto = currentMillis - lastPhotoTime;
    Serial.printf("Time since last photo: %lu minutes\n", timeSinceLastPhoto / 60000);

    Serial.println("========================\n");
  }
}

void loop() {
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