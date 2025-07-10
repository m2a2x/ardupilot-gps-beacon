#include "udp_module.h"
#include <algorithm>

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
    Serial.println("UDP module initialized");
    return true;
}

bool UDPModule::enable() {
    if (!initialized) {
        if (!begin()) {
            return false;
        }
    }
    
    if (enabled) {
        return true; // Already enabled
    }
    
    // Start WiFi Access Point
    if (!WiFi.softAP(UDP_AP_SSID, UDP_AP_PASS)) {
        Serial.println("Error: Failed to start Access Point!");
        return false;
    }
    
    delay(1000); // Give WiFi time to start
    
    // Initialize UDP server
    if (!udp.begin(UDP_PORT)) {
        Serial.println("Error: Failed to start UDP server!");
        WiFi.softAPdisconnect(true);
        return false;
    }
    
    enabled = true;
    Serial.printf("UDP module enabled - AP: %s, Port: %d\n", UDP_AP_SSID, UDP_PORT);
    Serial.printf("ESP32 IP: %s\n", WiFi.softAPIP().toString().c_str());
    
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
    Serial.println("UDP module disabled");
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
    
    udp.beginPacket(ip, UDP_PORT);
    udp.write(data, len);
    bool ok = udp.endPacket();
    
    if (!ok) {
        Serial.printf("[ERROR] UDP send failed to %s (%d bytes)\n", ip.toString().c_str(), len);
    }
    
    return ok;
}

int UDPModule::broadcastPacket(const uint8_t* data, size_t len) {
    if (!enabled || clients.empty()) {
        return 0;
    }
    
    int sentCount = 0;
    for (const auto& client : clients) {
        if (sendPacket(data, len, client.ip)) {
            sentCount++;
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
    Serial.printf("New GCS client: %s\n", ip.toString().c_str());
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