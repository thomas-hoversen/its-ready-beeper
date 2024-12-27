#ifndef BEEPHISTORY_H
#define BEEPHISTORY_H

#include <Arduino.h>
#include <deque>

/**
 * @brief A small helper class to track timestamps of beep detections.
 * 
 * Usage:
 *   - Call addBeep(...) each time a beep is confirmed.
 *   - hasReachedThreshold() checks if enough beeps have occurred
 *     within the configured time window.
 */
class BeepHistory {
public:
    /**
     * @param thresholdCount Minimum number of beeps required.
     * @param timeWindowMs   Time window in milliseconds to monitor.
     */
    BeepHistory(int thresholdCount, unsigned long timeWindowMs);

    /**
     * @brief Record a beep's timestamp (in ms).
     * @param timestamp current time (e.g. millis()).
     */
    void addBeep(unsigned long timestamp);

    /**
     * @brief Check if we've reached thresholdCount beeps in the last timeWindowMs.
     */
    bool hasReachedThreshold() const;

    /**
     * @brief Clear stored beep timestamps.
     */
    void clear();

private:
    int _thresholdCount;
    unsigned long _timeWindowMs;
    std::deque<unsigned long> _timestamps; // keep track of beep timestamps
};

#endif // BEEPHISTORY_H