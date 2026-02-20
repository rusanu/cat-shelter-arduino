#pragma once

class Ambient {
public:
    enum ECondition {
        Unknown,
        Night,
        Dark,
        Light,
        Bright
    };

private:
    bool _init = false;
    double _lux = 0;
    ECondition _condition = ECondition::Unknown;

public:

    static Ambient ltr;

    void setup();
    void loop();
    inline ECondition getCondition() const { return _condition; }
    inline double getLux() const { return _lux; }
    JsonDocument describe() const;
};