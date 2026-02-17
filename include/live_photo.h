#pragma once

class LivePhoto {
    bool _isStreaming = false;
    unsigned long _streamStartedMs = 0;
    unsigned long _lastPhotoMs = 0;

    pixformat_t _pixFormat = PIXFORMAT_JPEG;
    

public:
    void Start();
    void Stop();
    void Loop();
    void sendCameraConfiguration(const String& topic);
    void readCameraConfiguration(const JsonDocument& doc);
};
