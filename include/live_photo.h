#pragma once

class LivePhoto {
    bool _isStreaming = false;
    unsigned long _streamStartedMs = 0;
    unsigned long _lastPhotoMs = 0;
public:
    void Start();
    void Stop();
    void Loop();
};
