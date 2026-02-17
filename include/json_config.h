#pragma once

#define STATUS_FIELDS(X)            \
    X(int, int, vflip)              \
    X(int, int, hmirror)            \
    X(int, int, quality)            \
    X(int, int, brightness)         \
    X(int, int, contrast)           \
    X(int, int, saturation)         \
    X(int, int, sharpness)          \
    X(int, int, denoise)            \
    X(int, int, aec_value)          \
    X(int, int, ae_level)           \
    X(int, int, aec2)               \
    X(int, int, agc_gain)           \
    X(int, gainceiling_t, gainceiling)\
    X(int, framesize_t, framesize)

#define NOSTATUS_FIELDS(X)          \
    X(bool, int, exposure_ctrl)     \
    X(bool, int, gain_ctrl)     \

#define SENSOR_FIELDS(X)            \
    STATUS_FIELDS(X)                \
    NOSTATUS_FIELDS(X)


template <typename T>
struct SensorValue {
    T value;
    bool isSet = false;
};


class JsonCameraConfig {
private:
    #define SENSOR_FIELD(T, FT, name) SensorValue<FT> _##name;
    SENSOR_FIELDS(SENSOR_FIELD)
    #undef SENSOR_FIELD

public:
    static JsonCameraConfig config;
    static bool checkConfigValue(String field, int value);

    void Apply();
    void SaveNVM();
    void ReadNVM();
    void ClearNVM();

    void Read(const JsonDocument& node);
    JsonDocument BuildStatus() const;
    JsonDocument BuildInfo() const;
    JsonDocument buildConfigurationDocument() const;
    void readCameraConfiguration(const JsonDocument& doc);
};
