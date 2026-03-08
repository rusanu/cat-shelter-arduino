#pragma once


class OfflineReboot {
    unsigned long _lastOnline = 0;
    unsigned long _maxOffline;

    public:
        inline OfflineReboot(unsigned long maxOffline): _maxOffline(maxOffline) {}

        void Reset();
        bool Check(bool reboot);
};