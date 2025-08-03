#include "tasks.h"
#include "log_proxy.h"
#include "conf.h"
#include "mavlink_cmds.h"
#include "button.h"
#include "gps.h"

// GPS functionality is now accessed through proxy functions in gps.h

// Task handles
TaskHandle_t gpsTaskHandle = NULL;
TaskHandle_t mavlinkTaskHandle = NULL;
TaskHandle_t displayTaskHandle = NULL;
TaskHandle_t buttonTaskHandle = NULL;
TaskHandle_t wifiTaskHandle = NULL;
TaskHandle_t missionTaskHandle = NULL;

// Semaphores
SemaphoreHandle_t displayMutex = NULL;

// External variables
extern StatusDisplay oled;

// Global drone status object
DroneStatus droneStatus;

// Transmission activity tracking
unsigned long lastTxTime = 0;
unsigned long lastRxTime = 0;

// External variables
extern Mission* currentMission;
extern UDPModule udpModule;
extern bool gps_enabled;
extern uint32_t followMeUpdates;

// GPS Task
void gpsTask(void *pvParameters) {
    const TickType_t xDelay = pdMS_TO_TICKS(GPS_TASK_DELAY);
    
    while (1) {
        if (gps_enabled) {
            // Update GPS data through proxy
            updateGPS();
        }
        vTaskDelay(xDelay);
    }
}

// MAVLink Task
void mavlinkTask(void *pvParameters) {
    const TickType_t xDelay = pdMS_TO_TICKS(MAVLINK_TASK_DELAY);
    const TickType_t heartbeatDelay = pdMS_TO_TICKS(1000); // 1 second for heartbeat
    mavlink_message_t mavlink_msg;
    mavlink_status_t status;
    TickType_t lastHeartbeat = 0;
    
    while (1) {
        TickType_t currentTime = xTaskGetTickCount();
        
        // Send heartbeat every second to maintain connection with flight controller
        if (currentTime - lastHeartbeat >= heartbeatDelay) {
            send_heartbeat();
            lastHeartbeat = currentTime;
        }
        
        // Handle incoming UDP packets only if UDP module is enabled
        if (udpModule.isEnabled()) {
            uint8_t packetData[MAVLINK_MAX_PACKET_LEN];
            IPAddress senderIP;
            int bytesReceived = udpModule.receivePacket(packetData, sizeof(packetData), &senderIP);
            
            if (bytesReceived > 0) {
                // Send raw UDP data directly to radio via radio
                if (radio.writeRaw(packetData, bytesReceived)) {
                    lastRxTime = millis(); // Track receive activity
                }
            }
        }
        
        // Handle incoming data from radio
        mavlink_message_t incoming_msg;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        int len = 0;
        
        while (radio.available()) {
            if (radio.readMessage(&incoming_msg)) {
                
                // Parse all MAVLink messages through DroneStatus object
                droneStatus.parseMessage(incoming_msg);
                
                // Handle special messages that need additional processing
                if (incoming_msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
                    mavlink_command_ack_t command_ack;
                    mavlink_msg_command_ack_decode(&incoming_msg, &command_ack);
                    if (currentMission != nullptr) {
                        // Send command acknowledgment to current mission
                        // Each mission can decide whether to use it or not
                        currentMission->onCommandAck(command_ack.command, command_ack.result);
                    }
                }
                
                // Track when we receive data from the drone (for display purposes)
                lastTxTime = millis();
                
                // Convert message to buffer for UDP transmission only if UDP module is enabled
                len = mavlink_msg_to_send_buffer(buf, &incoming_msg);
                if (len > 0 && udpModule.isEnabled()) {
                    // Rate limit UDP transmission to prevent overwhelming the WiFi stack
                    static unsigned long lastUdpTxTime = 0;
                    unsigned long currentTime = millis();
                    
                    // Limit to max 50 packets per second (0ms between packets)
                    // Send to all connected UDP clients
                    udpModule.broadcastPacket(buf, len);
                    lastUdpTxTime = currentTime;
                }
            }
        }
        
        vTaskDelay(xDelay);
    }
}

// Display Task
void displayTask(void *pvParameters) {
    const TickType_t xDelay = pdMS_TO_TICKS(DISPLAY_TASK_DELAY);
    
    while (1) {
        std::vector<String> lines;
        
        // Read GPS data through proxy functions
        bool gpsDataValid = gpsHasFix();
        
        if (isInMenu()) {
            std::vector<int> highlightLines;
            getMenuDisplayWithHighlight(lines, highlightLines);
            
            // Update display with highlighting if there are lines to highlight
            if (xSemaphoreTake(displayMutex, portMAX_DELAY) == pdTRUE) {
                if (!highlightLines.empty()) {
                    oled.updateWithHighlight(lines, highlightLines);
                } else {
                    oled.update(lines);
                }
                xSemaphoreGive(displayMutex);
            }
        } else {
            // Show transmission activity with + or -
            unsigned long currentTime = millis();
            String txStatus = (currentTime - lastTxTime < ACTIVITY_TIMEOUT) ? "+" : "-";
            
            // Build status display lines
            lines.push_back("=== MAVLink Bridge ===");
            lines.push_back("GPS: " + String(gpsDataValid ? "OK" : "NO") + " " + txStatus);
            
            if (gpsDataValid) {
                lines.push_back("Sats: " + String(getSatelliteCount()));
            }
            
            // Check radio connectivity
            bool radioConnected = droneStatus.isConnected();
            lines.push_back("RSSI" + String(radioConnected ? "(OK) " : "(LOST) ") + String(droneStatus.radio_rssi));
            
            // Update display
            if (xSemaphoreTake(displayMutex, portMAX_DELAY) == pdTRUE) {
                oled.update(lines);
                xSemaphoreGive(displayMutex);
            }
        }
        
        vTaskDelay(xDelay);
    }
}

// Button Task
void buttonTask(void *pvParameters) {
    const TickType_t xDelay = pdMS_TO_TICKS(BUTTON_TASK_DELAY);
    
    while (1) {
        handleButton();
        handleRTLButton();
        vTaskDelay(xDelay);
    }
}

// WiFi Task
void wifiTask(void *pvParameters) {
    const TickType_t xDelay = pdMS_TO_TICKS(WIFI_TASK_DELAY);
    
    while (1) {
        // WiFi management is now handled by UDP module
        vTaskDelay(xDelay);
    }
}

// Mission Task
void missionTask(void *pvParameters) {
    const TickType_t xDelay = pdMS_TO_TICKS(MISSION_TASK_DELAY);
    
    while (1) {
        if (currentMission != nullptr) {
            currentMission->update();
        }
        vTaskDelay(xDelay);
    }
}

// Task initialization
void initTasks() {
    // Create semaphores
    displayMutex = xSemaphoreCreateMutex();
    
    // Create tasks
    xTaskCreatePinnedToCore(
        gpsTask,
        "GPS Task",
        4096,
        NULL,
        GPS_TASK_PRIORITY,
        &gpsTaskHandle,
        0
    );
    
    xTaskCreatePinnedToCore(
        mavlinkTask,
        "MAVLink Task",
        4096,
        NULL,
        MAVLINK_TASK_PRIORITY,
        &mavlinkTaskHandle,
        1
    );
    
    xTaskCreatePinnedToCore(
        displayTask,
        "Display Task",
        2048,
        NULL,
        DISPLAY_TASK_PRIORITY,
        &displayTaskHandle,
        0
    );
    
    xTaskCreatePinnedToCore(
        buttonTask,
        "Button Task",
        2048,
        NULL,
        BUTTON_TASK_PRIORITY,
        &buttonTaskHandle,
        0
    );
    
    xTaskCreatePinnedToCore(
        wifiTask,
        "WiFi Task",
        2048,
        NULL,
        WIFI_TASK_PRIORITY,
        &wifiTaskHandle,
        0
    );
    
    xTaskCreatePinnedToCore(
        missionTask,
        "Mission Task",
        4096,
        NULL,
        MISSION_TASK_PRIORITY,
        &missionTaskHandle,
        1
    );

}
