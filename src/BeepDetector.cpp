#include "BeepDetector.h"

BeepDetector::BeepDetector(int multiWindowConfidence, unsigned long detectionDelayMs)
    : _multiWindowConfidence(multiWindowConfidence),
      _detectionDelayMs(detectionDelayMs),
      _confirmedWindows(0),
      _lastDetectionTime(0)
{
}

bool BeepDetector::update(bool beepInThisWindow, unsigned long now) {
    if (!beepInThisWindow) {
        _confirmedWindows = 0;
        return false;
    }
    // beepInThisWindow == true
    _confirmedWindows++;

    bool multiWindowConfidenceMet = (_confirmedWindows >= _multiWindowConfidence);
    bool beepDelayElapsed = ((now - _lastDetectionTime) >= _detectionDelayMs);

    if (multiWindowConfidenceMet && beepDelayElapsed) {
        _confirmedWindows = 0;         
        _lastDetectionTime = now;      
        return true;                   
    }
    return false;
}