#include "common.h"
#include "offline_reboot.h"

void OfflineReboot::Reset() {
    _lastOnline = millis();
}

bool OfflineReboot::Check(bool reboot) {
    unsigned long now = millis();
    if (now - _lastOnline > _maxOffline) {
        if (reboot) {
            rebootSystem("Max offline exceeded");
        }
        return false;
    }
    return true;
}