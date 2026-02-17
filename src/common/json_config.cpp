#include <Arduino.h>
#include <ArduinoJson.h>
#include "common.h"
#include "json_config.h"

JsonCameraConfig JsonCameraConfig::config;

#define NVM_PREFS_SECTION "json-config"
#define NVM_STATUS_KEY "status"

void JsonCameraConfig::SaveNVM() {
  JsonDocument doc;

  #define SAVE_FIELD(T, FT, name)               \
    if (_##name.isSet) {                        \
        doc[#name] = _##name.value;             \
    }
  SENSOR_FIELDS(SAVE_FIELD);
  #undef SAVE_FIELD

  String jsonStr;
  serializeJson(doc, jsonStr);

  Preferences prefs;
  prefs.begin(NVM_PREFS_SECTION, false);
  prefs.putString(NVM_STATUS_KEY, jsonStr);
  prefs.end();
}

void JsonCameraConfig::ReadNVM() {
  Preferences prefs;
  prefs.begin(NVM_PREFS_SECTION, true);
  String jsonStr = prefs.getString(NVM_STATUS_KEY, "");
  prefs.end();

  if (jsonStr.length() > 0) {
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, jsonStr);
    if (err) {
        logPrintf(LOG_ERROR, "NVM JSON parse error: %s: %s", err.c_str(), jsonStr.c_str());
        return;
    }
    logPrintf(LOG_INFO, "Camera config from NVM: %s", jsonStr.c_str());
    this->Read(doc);
  }
}

void JsonCameraConfig::ClearNVM() {
    Preferences prefs;
    prefs.begin(NVM_PREFS_SECTION, false);
    prefs.remove(NVM_STATUS_KEY);
    prefs.end();
}

void JsonCameraConfig::Read(const JsonDocument& json) {
    #define VALUE_READ(T, FT, name)                 \
    {                                               \
        auto key = json[#name];                     \
        if (key.is<T>()) {                          \
            FT value = static_cast<FT>(key.as<T>());\
            if (!checkConfigValue(                  \
                #name,                              \
                static_cast<int>(value))) {         \
                logPrintf(LOG_WARNING,              \
                    "Config value outside range: "  \
                    #name " %i",                    \
                    static_cast<int>(value));       \
            }                                       \
            else  {                                 \
                _##name.value = value;              \
                _##name.isSet = true;               \
                logPrintf(LOG_INFO,                 \
                    "Camera configured: "           \
                    #name ": %s",                   \
                    String(_##name.value).c_str()); \
            }                                       \
        }                                           \
    }

    SENSOR_FIELDS(VALUE_READ)
    #undef VALUE_READ
};


void JsonCameraConfig::Apply() {
    sensor_t* s = esp_camera_sensor_get();
    #define APPLY_VALUE(T, FT, name)                                    \
    if (_##name.isSet) {                                                \
        auto v = s->set_##name(s, static_cast<FT>(_##name.value));      \
        logPrintf(LOG_INFO, "Applied config: " #name ": %s [%i]",       \
        String(_##name.value).c_str(),                                  \
        static_cast<int>(v));                                           \
    }

    SENSOR_FIELDS(APPLY_VALUE)
    #undef APPLY_VALUE
};

JsonDocument JsonCameraConfig::BuildStatus() const {
    
    JsonDocument status;

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        #define BUILD_FIELD(T, FT, name) status[#name] = _##name.isSet ? _##name.value : static_cast<T>(s->status.name);
        STATUS_FIELDS(BUILD_FIELD)
        #undef BUILD_FIELD

        #define BUILD_FIELD(T, FT, name) status[#name] = static_cast<T>(_##name.value);
        NOSTATUS_FIELDS(BUILD_FIELD)
        #undef BUILD_FIELD
    }

    return status;
}

JsonDocument JsonCameraConfig::BuildInfo() const {
    
    JsonDocument info;

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        camera_sensor_info_t *i = esp_camera_sensor_get_info(&(s->id));
        if (i) {
            info["model"] = i->model;
            info["name"]= i->name;
            info["max_size"] = i->max_size;
            info["support_jpeg"] = i->support_jpeg;
            info["pid"] = i->pid;
            info["sccb_addr"] = i->sccb_addr;
        }
    }

    return info;
}

JsonDocument JsonCameraConfig::buildConfigurationDocument() const {
    JsonDocument doc;
    doc["device"] = deviceName;
    doc["timestamp"] = getTimestamp();
    doc["status"] = this->BuildStatus();
    doc["info"] =  this->BuildInfo();

    return doc;
}

void JsonCameraConfig::readCameraConfiguration(const JsonDocument& doc) {
    if (doc["clear"].is<bool>() && doc["clear"].as<bool>())
    {
        #define CLEAR_FIELD(T, FT, name) _##name.isSet = false;
        SENSOR_FIELDS(CLEAR_FIELD)
        #undef CLEAR_FIELD

        sensor_t* s = esp_camera_sensor_get();
        s->reset(s);

        ClearNVM();

        logPrintf(LOG_INFO, "Camera config: all field reset");
    }
    if (doc["status"].is<JsonObjectConst>()) {
        Read(doc["status"]);
        Apply();
        SaveNVM();
    }
    else
    {
        logPrintf(LOG_WARNING, "missing key \"status\": %s", doc.as<String>().c_str());
    }
}


bool JsonCameraConfig::checkConfigValue(String field, int value) {
    #define FIELD_VALUES(name, min, max)     \
    if (field == #name) {                    \
        return min <= value && value <= max; \
    }

    FIELD_VALUES(brightness, -2, 2);
    FIELD_VALUES(contrast, -2, 2);
    FIELD_VALUES(saturation, -2, 2);
    FIELD_VALUES(ae_level, -2, 2);
    FIELD_VALUES(special_effect, 0, 6);
    FIELD_VALUES(wb_mode, 0, 4);
    FIELD_VALUES(aec_value, 0, 1200);
    FIELD_VALUES(agc_gain, 0, 30);
    FIELD_VALUES(gainceiling, 0, 6);

    return true;
    #undef FIELD_VALUES
}