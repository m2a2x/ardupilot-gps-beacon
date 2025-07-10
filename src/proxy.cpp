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
    
    // Calculate message length
    uint16_t len = mavlink_msg_to_send_buffer(NULL, msg);
    uint8_t buffer[len];
    
    // Serialize message to buffer
    mavlink_msg_to_send_buffer(buffer, msg);
    
    // Write to radio
    size_t written = radio.write(buffer, len);
    
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