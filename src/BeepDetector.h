#ifndef BEEPDETECTOR_H
#define BEEPDETECTOR_H

#include <Arduino.h>

class BeepDetector {
public:
    BeepDetector(int multiWindowConfidence, unsigned long detectionDelayMs);

    /**
     * @param beepInThisWindow  True if beep detected in the current FFT window.
     * @param now               Current time (millis()).
     * @return True if a new beep event is confirmed.
     */
    bool update(bool beepInThisWindow, unsigned long now);

private:
    int _multiWindowConfidence;
    unsigned long _detectionDelayMs;

    int _confirmedWindows;
    unsigned long _lastDetectionTime;
};

#endif