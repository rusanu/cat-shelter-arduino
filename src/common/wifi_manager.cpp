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

static BackOffRetry connectRetry(WIFI_RETRY_CONNECT);

static EWiFiState _wifiState = EWiFiState::Disconnected;

volatile bool wifiConnected = false;
bool wifiManualOverride = false;  // Track if WiFi is in manual control mode
unsigned long lastWiFiActivity = 0;  // Track last WiFi usage for idle timeout
unsigned long lastDisconnectTime = 0;
unsigned long lastConnectTime = 0;
unsigned long lastConnectAttempt = 0;
unsigned long lastSntpSync = 0;
volatile bool hasSNTPTime = false;

bool IsWiFiConnected() {
    return wifiConnected && hasSNTPTime;
}

bool syncTimeWithNTP(int _) {
    return hasSNTPTime;
}

void timeSyncCallback(struct timeval *tv) {
    time_t now = time(nullptr);
    if (now > 100000) {
      struct tm timeinfo;
      gmtime_r(&now, &timeinfo);
      char buffer[64];
      strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S UTC", &timeinfo);
      lastSntpSync = millis();
      hasSNTPTime = true;
      logPrintf(LOG_INFO, "SNTP Time synchronized: %s %d", buffer, (int)hasSNTPTime);
    } else {
      logPrintf(LOG_INFO, "SNTP Time callback but not synchronized");
    }
}

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
    logPrintf(LOG_INFO, "WiFi disconnected. Reason: %d", info.wifi_sta_disconnected.reason);
    
    _wifiState = EWiFiState::Disconnected;
    wifiConnected = false;
    lastDisconnectTime = millis();
}

void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info) {
    logPrintf(LOG_INFO, "WiFi connected. AuthMode:%d Signal strength:%d dBm", info.wifi_sta_connected.authmode, WiFi.RSSI());
}

void WiFiStationGotIP(WiFiEvent_t event, WiFiEventInfo_t info) {
    logPrintf(LOG_INFO, "WiFi got IP. Address:%s", WiFi.localIP().toString().c_str());

    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    
    _wifiState = EWiFiState::Connected;
    wifiConnected = true;
    lastConnectTime = millis();
    connectRetry.Reset();
}

void setupWifi(const char* hostname) {
    if (!wifiSetupInitialized) {
        WiFi.onEvent(WiFiStationDisconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
        WiFi.onEvent(WiFiStationConnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
        WiFi.onEvent(WiFiStationGotIP, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);

        WiFi.mode(WIFI_STA);

        //WiFi.setTxPower(WIFI_POWER_19_5dBm);
        WiFi.setSleep(false);
        WiFi.setAutoReconnect(false);
        sntp_set_time_sync_notification_cb(timeSyncCallback);
        hasSNTPTime = false;

        if (hostname) {
            uint64_t chipId = ESP.getEfuseMac();
            char chiphost[32+1];
            snprintf(chiphost, 32, "ESP32CAM-%s-%08X", hostname, (uint32_t)chipId);
            logPrintf(LOG_INFO, "Hostname: %s", chiphost);
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

    if (!connectRetry.CanRetry()) {
        return false;
    }

    _wifiState = EWiFiState::Connecting;
    
    logPrintf(LOG_INFO, "Starting WiFi connect [Retry %ld %ld %ld]...", connectRetry.AllowedCount(), connectRetry.DelayedCount(), connectRetry.ResetCount());

    // Scan for available networks
    int networksFound = WiFi.scanNetworks();
    if (networksFound <= 0) {
        logPrintf(LOG_WARNING, "No  networks found: %d", networksFound);
        // no need to call scanDelete()
        return false;
    }

    logPrintf(LOG_INFO, "Found %d networks:", networksFound);

    // List all networks with signal strength
    for (int i = 0; i < networksFound; i++) {
        logPrintf(LOG_INFO, "  %2d: %-32s  %3d dBm  AuthMode:%d",
                    i + 1,
                    WiFi.SSID(i).c_str(),
                    WiFi.RSSI(i),
                    WiFi.encryptionType(i));
    }

    // Find the best available known network
    const char* selectedSSID = nullptr;
    const char* selectedPassword = nullptr;
    int bestRSSI = -1000;  // Very weak signal as baseline

    wifi_auth_mode_t selectedAuthMode = WIFI_AUTH_OPEN;

    for (int i = 0; i < KNOWN_NETWORKS_COUNT; i++) {

        // Check if this known network is in the scan results
        bool found = false;
        for (int j = 0; j < networksFound; j++) {
            if (WiFi.SSID(j) == String(KNOWN_NETWORKS[i].ssid)) {
                int rssi = WiFi.RSSI(j);
                wifi_auth_mode_t authMode = WiFi.encryptionType(j);
                logPrintf(LOG_INFO, "FOUND %s (signal: %d dBm, authMode: %d)", WiFi.SSID(j).c_str(), rssi, authMode);
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
    }
  
    // Clean up scan results
    WiFi.scanDelete();

    // Check if we found any known network
    if (selectedSSID == nullptr) {
        logPrintf(LOG_ERROR, "No known networks available");
        _wifiState = EWiFiState::Disconnected;
        lastDisconnectTime = millis();
        return false;
    }

    // Connect to selected network
    logPrintf(LOG_INFO, "Connecting to: %s (signal: %d dBm, AuthMode: %d)", selectedSSID, bestRSSI, selectedAuthMode);

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

  logPrintf(LOG_INFO, "Disconnecting WiFi");
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  wifiConnected = false;
  _wifiState = EWiFiState::Disconnected;
}
