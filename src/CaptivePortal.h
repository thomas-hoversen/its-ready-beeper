#ifndef CAPTIVE_PORTAL_H
#define CAPTIVE_PORTAL_H

#include <WiFi.h>
#include <DNSServer.h>
#include <WebServer.h>

/**
 * @brief Minimal example of a Captive Portal that always displays "Hello".
 */
class CaptivePortal {
public:
    CaptivePortal();
    void begin();
    void handleClients();

private:
    void handleRoot();

    // Typically, port 53 for DNS, port 80 for HTTP
    static const byte DNS_PORT = 53;
    
    DNSServer dnsServer; 
    WebServer webServer; 
};

#endif // CAPTIVE_PORTAL_H