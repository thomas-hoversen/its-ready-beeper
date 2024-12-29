#ifndef CAPTIVE_PORTAL_H
#define CAPTIVE_PORTAL_H

#include <WiFi.h>
#include <DNSServer.h>
#include <WebServer.h>

/**
 * @brief Minimal captive portal that:
 *  - Hosts an AP
 *  - DNS -> redirect to ourselves
 *  - Web pages that let user start scanning, list discovered devices, connect to selected speaker
 */
class CaptivePortal {
public:
    CaptivePortal();
    void begin();
    void handleClients();

private:
    void handleRoot();
    void handleScan();
    void handleDevices();
    void handleConnect();

    static const byte DNS_PORT = 53;
    DNSServer dnsServer; 
    WebServer webServer; 
};

#endif // CAPTIVE_PORTAL_H