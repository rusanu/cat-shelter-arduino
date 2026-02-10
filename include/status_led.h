#pragma once

#include <map>
#include <vector>
#include <esp32-hal-timer.h>

class StatusLed
{
private:    
    std::vector<int> _pattern;
    unsigned long _nextMillis;
    int _idx = 0;
    bool _state = false;
    bool _running = false;
    int _pin = 0;

    void ChangeState(bool state);
    void SetIndex(int idx);

public:
    StatusLed(int pin): _pin(pin) {};
    void Start(const std::vector<int>& pattern, bool initialState);
    void Stop();
    void Tick();
};

void MonitorWifiStatus(StatusLed& status);