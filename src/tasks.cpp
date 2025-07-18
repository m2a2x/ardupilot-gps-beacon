#include "tasks.h"
#include "gps.h"
#include "display.h"
#include "button.h"
#include "conf.h"
#include "menu/menu.h"
#include "mavlink_cmds.h"
#include "udp_module.h"  // For UDP module client management
#include "battery.h"  // For battery functions
#include "utils.h"    // For mission helpers
#include "udp_module.h"  // For UDP module
#include "radio.h"
#include "menu/flight_modes.h"  // For flight mode functions
#include "log_proxy.h"  // For logging

#define MISSION_TASK_STACK_SIZE 2048
#define MISSION_TASK_PRIORITY   1

// External declarations
extern HardwareSerial gpsSerial;
extern TinyGPSPlus gps;

// Follow me status
extern uint32_t followMeUpdates;
extern bool followMeEnabled;

// Radio signal strength
int8_t radio_rssi = 0;

// Task handles
TaskHandle_t gpsTaskHandle = NULL;
TaskHandle_t mavlinkTaskHandle = NULL;
TaskHandle_t displayTaskHandle = NULL;
TaskHandle_t buttonTaskHandle = NULL;
TaskHandle_t wifiTaskHandle = NULL;

// Queue handles
QueueHandle_t mavlinkQueue = NULL;
QueueHandle_t displayQueue = NULL;
QueueHandle_t gpsQueue = NULL;

// Mutex handles
SemaphoreHandle_t gpsMutex = NULL;
SemaphoreHandle_t displayMutex = NULL;

// External variables
extern StatusDisplay oled;

// Transmission activity tracking
static unsigned long lastRxTime = 0;
static unsigned long lastTxTime = 0;
static const unsigned long ACTIVITY_TIMEOUT = 1000; // 1 second timeout for activity indication

// GPS Task
void gpsTask(void *pvParameters) {
    const TickType_t xDelay = pdMS_TO_TICKS(200); // 200ms delay
    
    while (1) {
        if (gps_enabled) {
            if (updateGPS()) {
                GPSData gpsData;
                gpsData.latitude = getLatitude();
                gpsData.longitude = getLongitude();
                gpsData.altitude = getAltitude();
                gpsData.satellites = getSatelliteCount();
                gpsData.hasFix = gpsHasFix();
                gpsData.isStale = isGPSStale();
                
                if (xQueueSend(gpsQueue, &gpsData, 0) != pdPASS) {
                    LogProxy::log("Failed to send GPS data to queue");
                }

                // Update GPS data
                while (gpsSerial.available()) {
                    gps.encode(gpsSerial.read());
                }
            }
        }
        vTaskDelay(xDelay);
    }
}

// MAVLink Task
void mavlinkTask(void *pvParameters) {
    const TickType_t xDelay = pdMS_TO_TICKS(1); // 1ms delay
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
                
                // Handle radio status message
                if (incoming_msg.msgid == MAVLINK_MSG_ID_RADIO_STATUS) {
                    mavlink_radio_status_t radio_status;
                    mavlink_msg_radio_status_decode(&incoming_msg, &radio_status);
                    radio_rssi = radio_status.rssi;
                }
                
                // Handle command acknowledgment for missions
                if (incoming_msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
                    mavlink_command_ack_t command_ack;
                    mavlink_msg_command_ack_decode(&incoming_msg, &command_ack);
                    handleMissionCommandAck(command_ack.command, command_ack.result);
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
    const TickType_t xDelay = pdMS_TO_TICKS(200); // 200ms delay
    GPSData gpsData;
    bool gpsDataValid = false;
    
    while (1) {
        std::vector<String> lines;
        
        // Get GPS data from queue
        if (xQueueReceive(gpsQueue, &gpsData, 0) == pdPASS) {
            gpsDataValid = true;
        }
        
        if (isInMenu()) {
            std::vector<int> highlightLines;
            getMenuDisplayWithHighlight(lines, highlightLines);
            
            // Update display with highlighting if there are lines to highlight
            if (xSemaphoreTake(displayMutex, portMAX_DELAY) == pdTRUE) {
                if (!highlightLines.empty()) {
                    if (!oled.updateWithHighlight(lines, highlightLines)) {
                        LogProxy::log("Error: Failed to update display with highlight");
                    }
                } else {
                    if (!oled.update(lines)) {
                        LogProxy::log("Error: Failed to update display");
                    }
                }
                xSemaphoreGive(displayMutex);
            }
        } else {
            // Show transmission activity with + or -
            unsigned long currentTime = millis();
            String txStatus = (currentTime - lastTxTime < ACTIVITY_TIMEOUT) ? "+" : "-";
            String rxStatus = (currentTime - lastRxTime < ACTIVITY_TIMEOUT) ? "+" : "-";
            lines.push_back("Tx: " + txStatus + " Rx: " + rxStatus);
            
            // Radio signal strength
            // int8_t rssi_dbm = (int8_t)(radio_rssi * 1.9f - 127);
            lines.push_back("Radio: " + String(radio_rssi));
            
            // GPS information only if enabled
            if (gps_enabled && gpsDataValid) {
                if (gpsData.hasFix) {
                    lines.push_back("Sats: " + String(gpsData.satellites));
                } else {
                    if (gpsData.isStale) {
                        lines.push_back("GPS: No Signal");
                    } else {
                        lines.push_back("GPS: No Fix");
                    }
                    lines.push_back("Sats: " + String(gpsData.satellites));
                }
            } else {
                lines.push_back("GPS: OFF");
            }

            // Follow me status if enabled
            if (followMeEnabled) {
                lines.push_back("Follow Me: " + String(followMeUpdates));
            }
            
            // UDP client information if enabled
            if (udpModule.isEnabled()) {
                lines.push_back("Clients: " + String(udpModule.getClientCount()));
            }
            
            // Update display with mutex protection
            if (xSemaphoreTake(displayMutex, portMAX_DELAY) == pdTRUE) {
                if (!oled.update(lines)) {
                    LogProxy::log("Error: Failed to update display");
                }
                xSemaphoreGive(displayMutex);
            }
        }
        
        vTaskDelay(xDelay);
    }
}

// Button Task
void buttonTask(void *pvParameters) {
    const TickType_t xDelay = pdMS_TO_TICKS(5); // 5ms delay for more responsive button detection
    
    while (1) {
        handleButton();
        vTaskDelay(xDelay);
    }
}

// WiFi Task
void wifiTask(void *pvParameters) {
    const TickType_t xDelay = pdMS_TO_TICKS(1000); // 1s delay
    bool last_udp_enabled = udpModule.isEnabled();
    static unsigned long lastHealthCheck = 0;
    
    while (1) {
        // Check if UDP status changed
        bool current_udp_enabled = udpModule.isEnabled();
        if (current_udp_enabled != last_udp_enabled) {
            last_udp_enabled = current_udp_enabled;
        }
        
        // Prune old clients periodically
        if (current_udp_enabled) {
            size_t beforeCount = udpModule.getClientCount();
            udpModule.pruneClients();
            size_t afterCount = udpModule.getClientCount();
            if (beforeCount != afterCount) {
                Serial.printf("[WiFi] Pruned clients: %d -> %d\n", (int)beforeCount, (int)afterCount);
            }
            
            // Periodic UDP health check every 30 seconds
            unsigned long currentTime = millis();
            if (currentTime - lastHealthCheck > 30000) {
                // The health check is now handled in broadcastPacket when failures are detected
                lastHealthCheck = currentTime;
            }
        }
        
        vTaskDelay(xDelay);
    }
}

// Mission Task
void missionTask(void *pvParameters) {
    const TickType_t xDelay = pdMS_TO_TICKS(100); // 100ms delay
    while (1) {
        // Add safety check to prevent crashes
        if (currentMission != nullptr) {
            updateCurrentMission();
        }
        vTaskDelay(xDelay);
    }
}

// Initialize all tasks and queues
void initTasks() {
    // Create queues
    mavlinkQueue = xQueueCreate(MAVLINK_QUEUE_SIZE, sizeof(MavlinkMessage));
    displayQueue = xQueueCreate(DISPLAY_QUEUE_SIZE, sizeof(DisplayMessage));
    gpsQueue = xQueueCreate(GPS_QUEUE_SIZE, sizeof(GPSData));
    
    if (mavlinkQueue == NULL || displayQueue == NULL || gpsQueue == NULL) {
        LogProxy::log("Error: Failed to create queues");
        return;
    }
    
    // Create mutexes
    gpsMutex = xSemaphoreCreateMutex();
    displayMutex = xSemaphoreCreateMutex();
    
    if (gpsMutex == NULL || displayMutex == NULL) {
        LogProxy::log("Error: Failed to create mutexes");
        return;
    }
    
    // Create tasks
    BaseType_t xReturned;
    
    xReturned = xTaskCreatePinnedToCore(gpsTask, "GPS", GPS_TASK_STACK_SIZE, NULL, GPS_TASK_PRIORITY, &gpsTaskHandle, 0);
    if (xReturned != pdPASS) {
        LogProxy::log("Error: Failed to create GPS task");
        return;
    }
    
    xReturned = xTaskCreatePinnedToCore(mavlinkTask, "MAVLink", MAVLINK_TASK_STACK_SIZE, NULL, MAVLINK_TASK_PRIORITY, &mavlinkTaskHandle, 1);
    if (xReturned != pdPASS) {
        LogProxy::log("Error: Failed to create MAVLink task");
        return;
    }
    
    xReturned = xTaskCreatePinnedToCore(displayTask, "Display", DISPLAY_TASK_STACK_SIZE, NULL, DISPLAY_TASK_PRIORITY, &displayTaskHandle, 0);
    if (xReturned != pdPASS) {
        LogProxy::log("Error: Failed to create Display task");
        return;
    }
    
    xReturned = xTaskCreatePinnedToCore(buttonTask, "Button", BUTTON_TASK_STACK_SIZE, NULL, BUTTON_TASK_PRIORITY, &buttonTaskHandle, 0);
    if (xReturned != pdPASS) {
        LogProxy::log("Error: Failed to create Button task");
        return;
    }
    
    xReturned = xTaskCreatePinnedToCore(wifiTask, "WiFi", WIFI_TASK_STACK_SIZE, NULL, WIFI_TASK_PRIORITY, &wifiTaskHandle, 1);
    if (xReturned != pdPASS) {
        LogProxy::log("Error: Failed to create WiFi task");
        return;
    }
    
    xReturned = xTaskCreatePinnedToCore(missionTask, "Mission", MISSION_TASK_STACK_SIZE, NULL, MISSION_TASK_PRIORITY, NULL, 0);
    if (xReturned != pdPASS) {
        LogProxy::log("Error: Failed to create Mission task");
        return;
    }
    
    LogProxy::log("All tasks created successfully");
} 