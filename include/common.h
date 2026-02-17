#ifndef COMMON_H
#define COMMON_H

#include <Arduino.h>
#include <DHT.h>
#include <Preferences.h>
#include "esp_camera.h"
#include "image_analyzer.h"

// Forward declarations and external declarations
extern "C" {
  uint8_t temprature_sens_read();
}

// Timing constants
#define DHT_READ_INTERVAL 2000  // Read DHT22 every 2 seconds
#define PHOTO_HOURLY_INTERVAL 600000   // 10 minutes in milliseconds (continuous power available)
#define PHOTO_MOTION_COOLDOWN 60000    // 1 minute in milliseconds (continuous power available)
#define STATUS_REPORT_INTERVAL 60000   // Status report every 60 seconds
#define WIFI_IDLE_TIMEOUT 360000       // UNUSED - WiFi always on with continuous power
#define BLANKET_MIN_STATE_TIME 300000  // 5 minutes minimum time before blanket can change state
#define CAT_PRESENCE_TIMEOUT 3600000   // 60 minutes - PIR motion extends presence (PIR is motion, not presence)
#define WIFI_RETRY_CONNECT 30000 // 30s between WiFi connect attempts

// Temperature threshold for blanket control (in Celsius)
#define TEMP_COLD_THRESHOLD 13.0  // Turn on blanket if temp is below this and cat present
#define TEMP_MAX_REASONABLE 45.0  // Maximum reasonable outdoor temperature (detect sensor errors)
#define TEMP_MIN_REASONABLE -30.0 // Minimum reasonable outdoor temperature (detect sensor errors)

// Boot and recovery settings
#define MAX_BOOT_ATTEMPTS 3       // Max failed boots before entering safe mode
#define BOOT_SUCCESS_TIMEOUT 300000  // 5 minutes - if running this long, boot is successful
#define SAFE_MODE_RECOVERY_INTERVAL 3600000  // 1 hour - attempt to exit safe mode and retry

// Camera configuration check interval
#define CAMERA_CONFIG_CHECK_INTERVAL 3600000  // 60 minutes in milliseconds

extern const char* deviceName; 
extern const char* s3Folder;

/// @brief A class for debouncing timer with min and max guard (for motion detected trigger)
class DebounceTimer {
  private:
    unsigned long _lastAct = 0;
    unsigned long _currentDelay = 0;
    unsigned long _minAct = 10*1000; // 10s time between actions when motion is detected
    unsigned long _maxDelay = 5*60*1000; // 5m max time between actions for contigous motion
    unsigned long _cooldown = 10*60*1000; // 10m contigous motion cooldown when no motion
    unsigned long _maxAct = 30*60*1000; // 30m max time between actions when no motion
  public:

    inline unsigned long LastAct() {
      return _lastAct;
    }

    inline unsigned long CurrentDelay() {
      return _currentDelay;
    }

    inline bool CanAct() const {
      unsigned long now = millis();
      if (_lastAct + _currentDelay > now) {
        return false;
      };
      return true;
    }

    inline bool MustAct() const {
      unsigned long now = millis();
      if (_lastAct > 0 && _lastAct + _maxAct > now) {
        return false;
      }      
      return true;
    }

    inline void MarkAct() {
      unsigned long now = millis();
      // Reset delay if cooldown has passed
      if (_lastAct + _cooldown < now) {
        _currentDelay = _minAct;
      }
      else {
        _currentDelay = max(_minAct, min(_currentDelay * 2, _maxDelay));
      }      
      _lastAct = now;
    }
};

/// @brief A class for retry with back-off
class BackOffRetry {
  private:
    unsigned long _lastRetry = 0;
    unsigned long _currentDelay = 0;
    unsigned long _minDelay = 1000;
    unsigned long _maxDelay = 30000;

    unsigned long _allowedCount = 0;
    unsigned long _delayedCount = 0;
    unsigned long _resetCount = 0;
  public:
    inline BackOffRetry(unsigned long maxDelay) {
      _maxDelay = maxDelay;
    }

    inline BackOffRetry(unsigned long maxDelay, unsigned long minDelay) {
      _maxDelay = maxDelay;
      _minDelay = minDelay;
    }

    inline unsigned long AllowedCount() {
      return _allowedCount;
    }

    inline unsigned long DelayedCount() {
      return _delayedCount;
    }

    inline unsigned long ResetCount() {
      return _resetCount;
    }

    inline void Reset() {
      _currentDelay = 0;
      _lastRetry = 0;
      _allowedCount = _delayedCount = 0;
      ++_resetCount;
    }

    inline bool CanRetry() {
      unsigned long now = millis();
      if (_lastRetry + _currentDelay > now) {
        ++_delayedCount;
        return false;
      }
      _lastRetry = now;
      _currentDelay = min(_maxDelay, max(_currentDelay * 2, _minDelay));
      ++_allowedCount;
      return true;
    }
};

// Logging levels
enum LogLevel {
  LOG_ERROR = 0,
  LOG_WARNING = 1,
  LOG_INFO = 2,
  LOG_DEBUG = 3
};

// Logging macro - logPrint implemented as macro using logPrintf
#define logPrint(level, msg) logPrintf(level, "%s", msg)

// External global variables
extern LogLevel currentLogLevel;
extern DHT* dht;
extern bool catPresent;
extern bool blanketOn;
extern bool blanketManualOverride;
extern unsigned long lastBlanketChange;
extern unsigned long lastMotionDetected;
extern float currentTemp;
extern float currentHumidity;
extern bool dhtSensorWorking;
extern bool wifiManualOverride;
extern unsigned long lastWiFiActivity;
extern unsigned long lastPhotoTime;
extern unsigned long lastHourlyPhotoTime;
extern bool lastCatPresent;
extern unsigned long lastStatusReport;
extern Preferences preferences;
extern bool safeMode;
extern bool cameraAvailable;
extern int bootAttempts;
extern unsigned long bootStartTime;
extern unsigned long lastSafeModeRecoveryAttempt;

// Function declarations

// Temperature functions
float getChipTemperature();

// Logging functions
void logPrintf(LogLevel level, const char* format, ...);

// AWS Signature V4 functions
void getSHA256AsString(const char *input, size_t input_len, char *output);
void getSignatureKey(const char *key, const char *dateStamp, const char *regionName, const char *serviceName, uint8_t *output);
void generateAWSSignatureV4(const char *method, const char *host, const char *uri,
                           const char *region, const char *accessKey, const char *secretKey,
                           const uint8_t *payload, size_t payload_len,
                           char *outAuthHeader, char *outAmzDate, char *outPayloadHash);
bool downloadFromS3(const String& filename, String& content, String& etag, String& errorMsg);
bool uploadJSONToS3(const String& jsonContent, const String& filename);

// Boot and recovery functions
void loadBootState();
void incrementBootAttempt();
void markBootSuccess();
void rebootSystem(const char* reason);

// Camera functions
bool initCamera();
void flashOn();
void flashOff();
camera_fb_t* capturePhoto();
void releasePhoto(camera_fb_t* fb);

// WiFi functions
void setupWifi(const char* hostname);
bool connectWiFi();
void disconnectWiFi();
bool syncTimeWithNTP(int maxRetries = 3);
bool IsWiFiConnected();

// S3 upload functions
String getTimestamp();
bool uploadPhotoToS3(camera_fb_t* fb, const String& filename, const String& folderName);
bool uploadStatusToS3(const String& filename, const ImageQualityMetrics& stats);

// GPIO and sensor functions
void setupGPIO();
bool readPIRSensor();
void checkPIRSensor();
void controlBlanket(bool shouldBeOn);
void readDHT22();

// Temperature calculation functions
float getExpectedTemperature();
float getEffectiveTemperature();

// Control functions
void updateBlanketControl();
bool takeAndUploadPhoto(const char* reason);
void checkPhotoSchedule();

// Serial command functions
void handleSerialCommands();

// Status reporting functions
String generateStatusJSON(const ImageQualityMetrics& stats);
void printStatusReport(bool forceImmediate = false);

#endif // COMMON_H