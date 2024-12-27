#include "BeepHistory.h"

BeepHistory::BeepHistory(int thresholdCount, unsigned long timeWindowMs)
  : _thresholdCount(thresholdCount),
    _timeWindowMs(timeWindowMs) {
}

void BeepHistory::addBeep(unsigned long timestamp) {
    // Add new timestamp
    _timestamps.push_back(timestamp);

    // Remove any timestamps that are older than timeWindowMs
    while (!_timestamps.empty() && (timestamp - _timestamps.front()) > _timeWindowMs) {
        _timestamps.pop_front();
    }
}

bool BeepHistory::hasReachedThreshold() const {
    return (_timestamps.size() >= (size_t)_thresholdCount);
}

void BeepHistory::clear() {
    _timestamps.clear();
}