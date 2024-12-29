#include "CaptivePortal.h"
#include <vector>
#include "esp_gap_bt_api.h"
#include "DiscoveredDevice.h"

// We only declare these as extern so the compiler knows they exist.
// Their actual definitions are in main.cpp.
extern std::vector<DiscoveredDevice> discoveredDevices;
extern void startScan();
extern void stopScan();
extern bool scanning;
extern bool connectToDeviceByMac(const String& macStr);

CaptivePortal::CaptivePortal() 
: webServer(80) // Initialize WebServer on port 80
{
    Serial.println("[CaptivePortal] Constructor called.");
}

void CaptivePortal::begin() {
    Serial.println("[CaptivePortal] begin() starting...");

    // Start the AP
    WiFi.mode(WIFI_AP);
    WiFi.softAP("MyCaptivePortal", "12345678", 1, 0, 4);
    IPAddress apIP = WiFi.softAPIP();
    Serial.print("[CaptivePortal] AP IP address: ");
    Serial.println(apIP);

    // Start DNS server, catch all domains, redirect to our AP IP
    dnsServer.start(DNS_PORT, "*", apIP);

    // Define routes
    webServer.on("/", HTTP_GET, [this]() { handleRoot(); });
    webServer.on("/scan", HTTP_GET, [this]() { handleScan(); });
    webServer.on("/devices", HTTP_GET, [this]() { handleDevices(); });
    webServer.on("/connect", HTTP_GET, [this]() { handleConnect(); });

    // If any other path => redirect to root
    webServer.onNotFound([this]() {
        Serial.println("[CaptivePortal] NotFound => redirecting to root.");
        webServer.sendHeader("Location", "/", true);
        webServer.send(302, "text/plain", "");
    });

    // Start web server
    webServer.begin();
    Serial.println("[CaptivePortal] Web server started");
}

void CaptivePortal::handleClients() {
    dnsServer.processNextRequest();
    webServer.handleClient();
}

void CaptivePortal::handleRoot() {
    Serial.println("[CaptivePortal] handleRoot called.");
    String page = "<!DOCTYPE html><html><head><title>Captive Portal</title></head>"
                  "<body><h1>Welcome to Captive Portal</h1>"
                  "<p><a href='/scan'>Scan for Speakers</a></p>"
                  "<p><a href='/devices'>View Discovered Devices</a></p>"
                  "</body></html>";
    webServer.send(200, "text/html", page);
}

void CaptivePortal::handleScan() {
    Serial.print("[CaptivePortal] handleScan: scanning was ");
    Serial.println(scanning ? "true" : "false");

    // Start or stop scanning
    if (!scanning) {
        startScan();
        webServer.send(200, "text/html", "<h1>Scanning started. <a href='/devices'>Check Devices</a></h1>");
    } else {
        stopScan();
        webServer.send(200, "text/html", "<h1>Stopped scanning. <a href='/devices'>Check Devices</a></h1>");
    }
}

void CaptivePortal::handleDevices() {
    Serial.println("[CaptivePortal] handleDevices called.");
    String page = "<!DOCTYPE html><html><head><title>Devices</title></head><body>";
    page += "<h1>Discovered Devices</h1>";

    if (scanning) {
        page += "<p>Currently scanning...</p>";
    }

    if (discoveredDevices.empty()) {
        page += "<p>No devices found yet.</p>";
    } else {
        page += "<ul>";
        for (auto &dev : discoveredDevices) {
            page += "<li>";
            if (dev.name.length() > 0) {
                page += dev.name + " ";
            } else {
                page += "(unknown) ";
            }
            page += "(" + dev.address + ") ";
            page += "<a href='/connect?mac=" + dev.address + "'>Connect</a>";
            page += "</li>";
        }
        page += "</ul>";
    }
    page += "<p><a href='/scan'>Start/Stop Scan</a> | <a href='/'>Back</a></p>";
    page += "</body></html>";

    webServer.send(200, "text/html", page);
}

void CaptivePortal::handleConnect() {
    Serial.println("[CaptivePortal] handleConnect called.");
    if (!webServer.hasArg("mac")) {
        Serial.println("[CaptivePortal] No 'mac' argument => error.");
        webServer.send(400, "text/html", "<h1>Error: missing 'mac' param</h1>");
        return;
    }
    String mac = webServer.arg("mac");
    Serial.printf("[CaptivePortal] Attempting connect to %s\n", mac.c_str());

    bool ok = connectToDeviceByMac(mac);
    if (ok) {
        webServer.send(200, "text/html", "<h1>Connecting to " + mac + "...</h1><p><a href='/'>Back</a></p>");
    } else {
        webServer.send(200, "text/html", "<h1>Connection failed to " + mac + "</h1><p><a href='/devices'>Back</a></p>");
    }
}