#include "CaptivePortal.h"

CaptivePortal::CaptivePortal() 
: webServer(80) // Initialize WebServer on port 80
{
}

/**
 * @brief Initializes Wi-Fi in AP mode + DNS + basic web server
 */
void CaptivePortal::begin() {
    // Start the AP
    WiFi.mode(WIFI_AP);
    WiFi.softAP("MyCaptivePortal", "12345678", 1);
    IPAddress apIP = WiFi.softAPIP();
    Serial.print("[CaptivePortal] AP IP address: ");
    Serial.println(apIP);

    // Start DNS server, catch all domains, redirect to our AP IP
    dnsServer.start(DNS_PORT, "*", apIP);

    // Any unknown path => show "Hello"
    webServer.onNotFound([this]() {
        handleRoot();
    });

    // Start web server
    webServer.begin();
    Serial.println("[CaptivePortal] Web server started");
}

/**
 * @brief Periodically handle DNS + HTTP requests
 */
void CaptivePortal::handleClients() {
    dnsServer.processNextRequest();
    webServer.handleClient();
}

/**
 * @brief Called for all requests; returns "Hello"
 */
void CaptivePortal::handleRoot() {
    // Respond with a simple Hello page
    String helloPage = "<!DOCTYPE html><html><head><title>Captive Portal</title></head>"
                       "<body><h1>Hello</h1></body></html>";
    webServer.send(200, "text/html", helloPage);
}