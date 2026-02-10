
#include <status_led.h>


void StatusLed::Start(const std::vector<int>& pattern, bool initialState)
{
    _pattern = pattern;
    _running = true;
    ChangeState(initialState);
    SetIndex(0);
}

void StatusLed::Stop() {
    _running = false;
    ChangeState(false);
}

void StatusLed::Tick() {
    if (_running) {
        auto current = millis();
        if (current > _nextMillis) {
            ChangeState(!_state);
            SetIndex(++_idx);
        }
    }
}

void StatusLed::SetIndex(int idx) {
    _idx = idx % _pattern.size();
    _nextMillis = millis() + _pattern[_idx];
}

void StatusLed::ChangeState(bool state) {
    _state = state;
    if (_pin) {
        digitalWrite(_pin, state ? HIGH : LOW);
    }
}