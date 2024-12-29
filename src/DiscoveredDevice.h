// DiscoveredDevice.h
#ifndef DISCOVERED_DEVICE_H
#define DISCOVERED_DEVICE_H

#include <Arduino.h>  // or <string>, whichever you use for 'String'

struct DiscoveredDevice {
    String name;
    String address;  // "XX:XX:XX:XX:XX:XX"
};

#endif