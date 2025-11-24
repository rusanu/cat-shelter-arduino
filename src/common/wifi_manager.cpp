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
#include <esp_sntp.h>
#include "image_analyzer.h"
#include "secrets.h"  // WiFi credentials (not in git)

#include "common.h"

typedef enum {
    Disconnected,
    Connecting,
    Connected
} EWiFiState;

static bool wifiSetupInitialized = false;

static EWiFiState _wifiState = EWiFiState::Disconnected;

bool wifiConnected = false;
bool wifiManualOverride = false;  // Track if WiFi is in manual control mode
unsigned long lastWiFiActivity = 0;  // Track last WiFi usage for idle timeout
unsigned long lastDisconnectTime = 0;
unsigned long lastConnectTime = 0;
unsigned long lastSntpSync = 0;
bool hasSNTPTime = false;

void timeSyncCallback(struct timeval *tv) {
    time_t now = time(nullptr);
    if (now > 100000) {
      struct tm timeinfo;
      gmtime_r(&now, &timeinfo);
      char buffer[64];
      strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S UTC", &timeinfo);
      logPrintf(LOG_INFO, "SNTP Time synchronized: %s", buffer);
      lastSntpSync = millis();
      hasSNTPTime = true;
    } else {
      logPrintf(LOG_INFO, "SNTP Time callback but not synchronized");
    }
}

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.println("WiFi disconnected.");
    Serial.print("Reason: ");
    Serial.println(info.wifi_sta_disconnected.reason);
    
    _wifiState = EWiFiState::Disconnected;
    wifiConnected = false;
    lastDisconnectTime = millis();
}

void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.println("WiFi connected");
    Serial.print("AuthMode: ");
    Serial.println(info.wifi_sta_connected.authmode);
    Serial.print("Signal strength: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
}

void WiFiStationGotIP(WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.println("WiFi got IP");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    
    _wifiState = EWiFiState::Connected;
    wifiConnected = true;
    lastConnectTime = millis();
}

void setupWifi(const char* hostname) {
    if (!wifiSetupInitialized) {
        WiFi.onEvent(WiFiStationDisconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
        WiFi.onEvent(WiFiStationConnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
        WiFi.onEvent(WiFiStationGotIP, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);

        WiFi.mode(WIFI_STA);

        WiFi.setTxPower(WIFI_POWER_19_5dBm);
        WiFi.setSleep(false);
        WiFi.setAutoReconnect(false);
        sntp_set_time_sync_notification_cb(timeSyncCallback);
        hasSNTPTime = false;

        if (hostname) {
            uint64_t chipId = ESP.getEfuseMac();
            char chiphost[32+1];
            snprintf(chiphost, 32, "ESP32CAM-%s-%04X%08X", hostname, chipId, (uint32_t)chipId);
            Serial.print("hostname:");
            Serial.println(chiphost);
            WiFi.setHostname(chiphost);
        }
        wifiSetupInitialized = true;
    }
}

bool connectWiFi() {

  if (wifiConnected) {
    return true;
  }

  if (_wifiState == EWiFiState::Connecting) {
    return false;
  }

  _wifiState = EWiFiState::Connecting;

  Serial.println("Starting WiFi network scan...");


  // Scan for available networks
  int networksFound = WiFi.scanNetworks();
  Serial.printf("Found %d networks:\n", networksFound);
  if (networksFound <= 0) {
      // no need to call scanDelete()
      return false;
  }

    // List all networks with signal strength
    Serial.println("\n=== Available WiFi Networks ===");
    for (int i = 0; i < networksFound; i++) {
        Serial.printf("  %2d: %-32s  %3d dBm  %d\n",
                    i + 1,
                    WiFi.SSID(i).c_str(),
                    WiFi.RSSI(i),
                    WiFi.encryptionType(i));
    }
    Serial.println("===============================\n");

    // Find the best available known network
    const char* selectedSSID = nullptr;
    const char* selectedPassword = nullptr;
    int bestRSSI = -1000;  // Very weak signal as baseline

    wifi_auth_mode_t selectedAuthMode = WIFI_AUTH_OPEN;

    Serial.println("Checking for known networks...");
    for (int i = 0; i < KNOWN_NETWORKS_COUNT; i++) {
        Serial.printf("  Looking for: %s... ", KNOWN_NETWORKS[i].ssid);

        // Check if this known network is in the scan results
        bool found = false;
        for (int j = 0; j < networksFound; j++) {
        if (WiFi.SSID(j) == String(KNOWN_NETWORKS[i].ssid)) {
            int rssi = WiFi.RSSI(j);
            wifi_auth_mode_t authMode = WiFi.encryptionType(j);
            Serial.printf("FOUND (signal: %d dBm, authMode: %d)\n", rssi, authMode);
            found = true;

            // Select this network if it's the first one found or has better signal
            if (selectedSSID == nullptr || rssi > bestRSSI) {
            selectedSSID = KNOWN_NETWORKS[i].ssid;
            selectedPassword = KNOWN_NETWORKS[i].password;
            selectedAuthMode = authMode;
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
    _wifiState = EWiFiState::Disconnected;
    lastDisconnectTime = millis();
    return false;
  }

  // Connect to selected network
  Serial.printf("\nConnecting to: %s (signal: %d dBm, AuthMode: %d)\n", selectedSSID, bestRSSI, selectedAuthMode);

  wifi_config_t wifi_config = {};
  esp_wifi_get_config(WIFI_IF_STA, &wifi_config);
  wifi_config.sta.threshold.authmode = selectedAuthMode;
  wifi_config.sta.pmf_cfg.capable = true;
  wifi_config.sta.pmf_cfg.required = false;
  esp_wifi_set_config(WIFI_IF_STA, &wifi_config);  

  // Start WiFi connection
  WiFi.begin(selectedSSID, selectedPassword);

  return false;
}

void disconnectWiFi() {
  if (!wifiConnected) {
    return;
  }

  Serial.println("Disconnecting WiFi for power saving...");
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  wifiConnected = false;
  _wifiState = EWiFiState::Disconnected;
}
