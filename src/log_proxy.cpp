#include "log_proxy.h"
#include "mavlink_cmds.h"

void LogProxy::log(const String& message) {
    // Serial.println(message);
    // Send status message to ground station console
    char status_msg[150];
    snprintf(status_msg, sizeof(status_msg), "%s", message.c_str());
    send_status_text(status_msg, MAV_SEVERITY_INFO);
}

void LogProxy::log(const char* message) {
    // Serial.println(message);

    // Send status message to ground station console
    char status_msg[150];
    snprintf(status_msg, sizeof(status_msg), "%s", message);
    send_status_text(status_msg, MAV_SEVERITY_INFO);
}