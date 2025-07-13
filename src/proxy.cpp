#include "proxy.h"
#include "conf.h"

// Global proxy instance
MavlinkProxy proxy(mavSerial);

/**
 * Constructor
 */
MavlinkProxy::MavlinkProxy(HardwareSerial& radioSerial) : radio(radioSerial) {
    // Constructor - radio reference is initialized in the initialization list
}

/**
 * Read a MAVLink message from the radio
 */
bool MavlinkProxy::readMessage(mavlink_message_t* msg) {
    if (!msg) {
        return false;
    }
    
    // Check if data is available
    if (!available()) {
        return false;
    }
    
    // Read and parse MAVLink message
    mavlink_status_t status;
    uint8_t byte;
    
    while (radio.available()) {
        byte = radio.read();
        
        if (mavlink_parse_char(MAVLINK_COMM_0, byte, msg, &status)) {
            // Valid message received
            return true;
        }
    }
    
    return false;
}

/**
 * Write a MAVLink message to the radio
 */
bool MavlinkProxy::writeMessage(const mavlink_message_t* msg) {
    if (!msg) {
        return false;
    }
    
    
    // Use a fixed-size buffer instead of variable-length array to prevent stack overflow
    // MAVLINK_MAX_PACKET_LEN is typically 280 bytes, which is safe for stack allocation
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    
    
    // Calculate message length and serialize message to buffer
    uint16_t len = mavlink_msg_to_send_buffer(buffer, msg);
    
    
    // Write to radio
    size_t written = radio.write(buffer, len);
    return written == len;
}

/**
 * Write raw data to the radio
 */
bool MavlinkProxy::writeRaw(const uint8_t* data, size_t len) {
    if (!data || len == 0) {
        return false;
    }
    
    // Write raw data directly to radio
    size_t written = radio.write(data, len);
    return written == len;
}

/**
 * Check if data is available to read from the radio
 */
bool MavlinkProxy::available() {
    return radio.available() > 0;
}

/**
 * Flush the radio buffer
 */
void MavlinkProxy::flush() {
    radio.flush();
} 