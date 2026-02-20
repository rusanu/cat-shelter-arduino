#include <Arduino.h>
#include <ArduinoJson.h>
#ifdef HAS_LTR308    
#include  <DFRobot_LTR308.h>
#endif
#include "common.h"
#include "ambient.h"


#ifdef HAS_LTR308    
static DFRobot_LTR308 ltr308;
#endif

#define MEASURE_FREQUENCY_MS 60000

static unsigned long lastMeasureMs = 0;

Ambient Ambient::ltr;


void Ambient::setup() {
#ifdef HAS_LTR308    
    _init = _init || ltr308.begin();
    logPrintf(LOG_INFO, "LTR308: init %i", _init);
#endif    
}

JsonDocument Ambient::describe() const {
    JsonDocument doc;
    doc["init"] = _init;
    doc["lux"] = _lux;
    doc["condition"] = _condition;
    return doc;
}

void Ambient::loop() {
#ifdef HAS_LTR308    
    _init = _init || ltr308.begin();
    auto now = millis();

    if (_init &&
        ((lastMeasureMs == 0) || (now - lastMeasureMs > MEASURE_FREQUENCY_MS))) {
        lastMeasureMs = now;
        auto data = ltr308.getData();
        auto lux = ltr308.getLux(data);
        auto gain = ltr308.getGain();
        auto status = ltr308.getStatus();
        if (lux > 0) { // 0 means no data acquired
            _lux = lux;
        }
        if (lux > 0 && lux < 1) {
            _condition = Night;
        }
        else if (lux >= 1 && lux < 5) {
            _condition = Dark;
        }
        else if (lux >= 5 && lux < 10) {
            _condition = Light;
        }
        else if (lux >= 10) {
            _condition = Bright;
        }
        logPrintf(LOG_INFO, "LTR308: %" PRIu32 " %f %" PRIu8 " %i", 
            data,
            lux,
            gain,
            _condition);
    }
#endif    
}