#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <IPAddress.h>
#include <vector>

/**
 * UDP Module for MAVLink Bridge
 * 
 * This module handles all WiFi and UDP communication functionality.
 * It provides a clean interface for enabling/disabling WiFi and sending UDP packets.
 */

// WiFi AP configuration
extern const char* UDP_AP_SSID;
extern const char* UDP_AP_PASS;
extern const uint16_t UDP_PORT;

// Network configuration
extern const IPAddress UDP_LOCAL_IP;
extern const IPAddress UDP_SUBNET_MASK;

/**
 * Structure representing a Ground Control Station client
 */
struct UDPClient {
    IPAddress ip;
    unsigned long lastSeen;
};

class UDPModule {
private:
    WiFiUDP udp;
    std::vector<UDPClient> clients;
    bool enabled;
    bool initialized;
    
public:
    /**
     * Constructor
     */
    UDPModule();
    
    /**
     * Initialize the UDP module
     * @return true if initialization successful, false otherwise
     */
    bool begin();
    
    /**
     * Enable WiFi and UDP functionality
     * @return true if enabled successfully, false otherwise
     */
    bool enable();
    
    /**
     * Disable WiFi and UDP functionality
     */
    void disable();
    
    /**
     * Check if UDP module is enabled
     * @return true if enabled, false otherwise
     */
    bool isEnabled() const;
    
    /**
     * Check if WiFi is connected
     * @return true if WiFi is active, false otherwise
     */
    bool isConnected() const;
    
    /**
     * Get the local IP address
     * @return IPAddress of the ESP32
     */
    IPAddress getLocalIP() const;
    
    /**
     * Send UDP packet to specified IP address
     * @param data Pointer to data buffer
     * @param len Length of data
     * @param ip Target IP address
     * @return true if packet was sent successfully
     */
    bool sendPacket(const uint8_t* data, size_t len, IPAddress ip);
    
    /**
     * Send UDP packet to all known clients
     * @param data Pointer to data buffer
     * @param len Length of data
     * @return Number of clients the packet was sent to
     */
    int broadcastPacket(const uint8_t* data, size_t len);
    
    /**
     * Check for incoming UDP packets
     * @param data Buffer to store received data
     * @param maxLen Maximum length of data to read
     * @param senderIP Pointer to store sender's IP address
     * @return Number of bytes received, 0 if no packet available
     */
    int receivePacket(uint8_t* data, size_t maxLen, IPAddress* senderIP = nullptr);
    
    /**
     * Add a new client or update existing client's last seen timestamp
     * @param ip IP address of the client
     */
    void addOrUpdateClient(IPAddress ip);
    
    /**
     * Remove clients that haven't been seen for more than 30 seconds
     */
    void pruneClients();
    
    /**
     * Get the number of connected clients
     * @return Number of active clients
     */
    size_t getClientCount() const;
    
    /**
     * Get client list
     * @return Reference to the clients vector
     */
    const std::vector<UDPClient>& getClients() const;
    
    /**
     * Get WiFi RSSI (signal strength)
     * @return RSSI value in dBm
     */
    int8_t getRSSI() const;
    
    /**
     * Get the UDP instance (for advanced usage)
     * @return Reference to the WiFiUDP instance
     */
    WiFiUDP& getUDP();
    
    /**
     * Get UDP statistics for debugging
     * @return String with UDP statistics
     */
    String getStats();
    
    /**
     * Send a test packet to verify UDP functionality
     * @param testIP Target IP address for test packet
     * @return true if test packet was sent successfully
     */
    bool sendTestPacket(IPAddress testIP);
    
    /**
     * Print current UDP status for debugging
     */
    void printStatus();
};

// Global UDP module instance
extern UDPModule udpModule; 