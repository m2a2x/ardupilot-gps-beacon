#include "udp_module.h"
#include <algorithm>
#include "log_proxy.h"  // For logging

// WiFi AP configuration
const char* UDP_AP_SSID = "ESP32-MAVLink";
const char* UDP_AP_PASS = "mavlink123";
const uint16_t UDP_PORT = 14550;

// Network configuration
const IPAddress UDP_LOCAL_IP(192, 168, 4, 1);
const IPAddress UDP_SUBNET_MASK(255, 255, 255, 0);

// Global UDP module instance
UDPModule udpModule;

UDPModule::UDPModule() : enabled(false), initialized(false) {
}

bool UDPModule::begin() {
    if (initialized) {
        return true;
    }
    
    // Initialize WiFi in AP mode
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(UDP_LOCAL_IP, UDP_LOCAL_IP, UDP_SUBNET_MASK);
    
    initialized = true;
    LogProxy::log("UDP module initialized");
    return true;
}

bool UDPModule::enable() {
    if (!initialized) {
        if (!begin()) {
            return false;
        }
    }
    
    if (enabled) {
        LogProxy::log("UDP module already enabled");
        return true; // Already enabled
    }
    
    LogProxy::log("Starting WiFi Access Point...");
    // Start WiFi Access Point
    if (!WiFi.softAP(UDP_AP_SSID, UDP_AP_PASS)) {
        LogProxy::log("Error: Failed to start Access Point!");
        return false;
    }
    
    LogProxy::log("WiFi AP started successfully");
    delay(1000); // Give WiFi time to start
    
    LogProxy::log("Starting UDP server on port " + String(UDP_PORT));
    
    // Stop any existing UDP connection first
    udp.stop();
    delay(100);
    
    // Initialize UDP server with retry logic
    int retryCount = 0;
    bool udpStarted = false;
    
    while (retryCount < 3 && !udpStarted) {
        udpStarted = udp.begin(UDP_PORT);
        if (!udpStarted) {
            LogProxy::log("UDP start attempt " + String(retryCount + 1) + " failed, retrying...");
            delay(500);
            retryCount++;
        }
    }
    
    if (!udpStarted) {
        LogProxy::log("Error: Failed to start UDP server after 3 attempts!");
        WiFi.softAPdisconnect(true);
        return false;
    }
    
    enabled = true;
    LogProxy::log("UDP module enabled - AP: " + String(UDP_AP_SSID) + ", Port: " + String(UDP_PORT));
    LogProxy::log("ESP32 IP: " + WiFi.softAPIP().toString());
    LogProxy::log("UDP server is ready to receive connections");
    
    return true;
}

void UDPModule::disable() {
    if (!enabled) {
        return; // Already disabled
    }
    
    // Stop UDP server
    udp.stop();
    
    // Disconnect WiFi AP
    WiFi.softAPdisconnect(true);
    
    // Clear client list
    clients.clear();
    
    enabled = false;
    LogProxy::log("UDP module disabled");
}

bool UDPModule::isEnabled() const {
    return enabled;
}

bool UDPModule::isConnected() const {
    return enabled && WiFi.softAPgetStationNum() > 0;
}

IPAddress UDPModule::getLocalIP() const {
    return WiFi.softAPIP();
}

bool UDPModule::sendPacket(const uint8_t* data, size_t len, IPAddress ip) {
    if (!enabled) {
        return false;
    }
    
    // Try to send the packet with error handling
    udp.beginPacket(ip, UDP_PORT);
    size_t written = udp.write(data, len);
    bool ok = udp.endPacket();
    
    if (!ok || written != len) {
        LogProxy::log("UDP send failed to " + ip.toString() + " (" + String(len) + " bytes, written: " + String(written) + ")");
        return false;
    }
    
    return true;
}

int UDPModule::broadcastPacket(const uint8_t* data, size_t len) {
    if (!enabled) {
        return 0;
    }
    
    if (clients.empty()) {
        return 0;
    }
    
    int sentCount = 0;
    int failedCount = 0;
    
    for (const auto& client : clients) {
        if (sendPacket(data, len, client.ip)) {
            sentCount++;
        } else {
            failedCount++;
        }
    }
    
    // If we have too many failures, try to reset the UDP connection
    if (failedCount > 0 && sentCount == 0 && clients.size() > 0) {
        static unsigned long lastResetTime = 0;
        unsigned long currentTime = millis();
        
        // Only reset once every 10 seconds to avoid constant resets
        if (currentTime - lastResetTime > 10000) {
            LogProxy::log("Multiple send failures detected, resetting UDP connection...");
            udp.stop();
            delay(100);
            if (udp.begin(UDP_PORT)) {
                LogProxy::log("UDP connection reset successfully");
            } else {
                LogProxy::log("Failed to reset UDP connection");
            }
            lastResetTime = currentTime;
        }
    }
    
    return sentCount;
}

int UDPModule::receivePacket(uint8_t* data, size_t maxLen, IPAddress* senderIP) {
    if (!enabled) {
        return 0;
    }
    
    int packetSize = udp.parsePacket();
    if (packetSize <= 0) {
        return 0;
    }
    
    IPAddress remote = udp.remoteIP();
    if (senderIP) {
        *senderIP = remote;
    }
    
    // Add or update client
    addOrUpdateClient(remote);
    
    // Read packet data
    size_t bytesRead = 0;
    while (packetSize-- && bytesRead < maxLen) {
        data[bytesRead++] = udp.read();
    }
    
    return bytesRead;
}

void UDPModule::addOrUpdateClient(IPAddress ip) {
    for (auto& client : clients) {
        if (client.ip == ip) {
            client.lastSeen = millis();
            return;
        }
    }
    
    clients.push_back({ip, millis()});
    LogProxy::log("New GCS client connected: " + ip.toString() + " (Total: " + String(clients.size()) + ")");
}

void UDPModule::pruneClients() {
    unsigned long now = millis();
    clients.erase(
        std::remove_if(clients.begin(), clients.end(),
            [now](const UDPClient& client) {
                return now - client.lastSeen > 30000; // 30 seconds timeout
            }),
        clients.end()
    );
}

size_t UDPModule::getClientCount() const {
    return clients.size();
}

const std::vector<UDPClient>& UDPModule::getClients() const {
    return clients;
}

int8_t UDPModule::getRSSI() const {
    if (!enabled) {
        return 0;
    }
    
    // For AP mode, we can't get RSSI directly, but we can return a default value
    // or calculate based on connected stations
    int stationCount = WiFi.softAPgetStationNum();
    if (stationCount > 0) {
        return -50; // Good signal when stations are connected
    } else {
        return -100; // No signal when no stations connected
    }
}

WiFiUDP& UDPModule::getUDP() {
    return udp;
}

String UDPModule::getStats() {
    String stats = "UDP Stats: ";
    stats += "Enabled=" + String(enabled ? "YES" : "NO");
    stats += ", Clients=" + String(clients.size());
    stats += ", WiFi=" + String(WiFi.softAPgetStationNum());
    stats += ", IP=" + WiFi.softAPIP().toString();
    return stats;
} 