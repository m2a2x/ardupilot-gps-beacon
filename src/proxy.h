#pragma once
#include <Arduino.h>
#include <mavlink/v2.0/common/mavlink.h>

/**
 * Simple Proxy for MAVLink Communication
 * 
 * This proxy acts as an intermediary between the application and the radio (mavSerial).
 * It provides a clean interface for reading and writing MAVLink messages,
 * making it easy to add future improvements like logging, filtering, or protocol conversion.
 * 
 * USAGE EXAMPLES:
 * 
 * // Reading messages from radio
 * mavlink_message_t msg;
 * if (proxy.readMessage(&msg)) {
 *     // Process the received message
 *     if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
 *         // Handle heartbeat
 *     }
 * }
 * 
 * // Writing messages to radio
 * mavlink_message_t msg;
 * mavlink_msg_heartbeat_pack(system_id, component_id, &msg, ...);
 * proxy.writeMessage(&msg);
 * 
 * // Check if data is available
 * if (proxy.available()) {
 *     // Data ready to read
 * }
 */
class MavlinkProxy {
private:
    HardwareSerial& radio;  // Reference to the radio serial connection
    
public:
    /**
     * Constructor
     * @param radioSerial Reference to the HardwareSerial instance for radio communication
     */
    MavlinkProxy(HardwareSerial& radioSerial);
    
    /**
     * Read a MAVLink message from the radio
     * @param msg Pointer to store the received MAVLink message
     * @return true if a valid message was received, false otherwise
     */
    bool readMessage(mavlink_message_t* msg);
    
    /**
     * Write a MAVLink message to the radio
     * @param msg Pointer to the MAVLink message to send
     * @return true if message was sent successfully, false otherwise
     */
    bool writeMessage(const mavlink_message_t* msg);
    
    /**
     * Write raw data to the radio
     * @param data Pointer to the data to send
     * @param len Length of the data
     * @return true if data was sent successfully, false otherwise
     */
    bool writeRaw(const uint8_t* data, size_t len);
    
    /**
     * Check if data is available to read from the radio
     * @return true if data is available, false otherwise
     */
    bool available();
    
    /**
     * Flush the radio buffer
     */
    void flush();
};

// Global proxy instance
extern MavlinkProxy proxy; 