#include "tasks.h"
#include "gps.h"
#include "display.h"
#include "button.h"
#include "conf.h"
#include "menu/menu.h"
#include "mavlink_cmds.h"
#include "clients.h"  // For clients and addOrUpdateClient
#include "func.h"     // For sendUDP
#include "battery.h"  // For battery functions
#include "utils.h"    // For addMavlinkMessage and new helpers
#include <WiFi.h>
#include <WiFiUdp.h>

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
SemaphoreHandle_t wifiMutex = NULL;
SemaphoreHandle_t gpsMutex = NULL;
SemaphoreHandle_t displayMutex = NULL;

// External variables
extern StatusDisplay oled;
extern unsigned long rxBytes, txBytes;
extern std::vector<GCSClient> clients;  // From clients.h

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
                    Serial.println("Failed to send GPS data to queue");
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
    const TickType_t xDelay = pdMS_TO_TICKS(10); // 10ms delay
    const TickType_t heartbeatDelay = pdMS_TO_TICKS(1000); // 1 second for heartbeat
    MavlinkMessage msg;
    mavlink_message_t mavlink_msg;
    mavlink_status_t status;
    TickType_t lastHeartbeat = 0;
    
    while (1) {
        TickType_t currentTime = xTaskGetTickCount();
        
        // Send heartbeat every second
        if (currentTime - lastHeartbeat >= heartbeatDelay) {
            // send_heartbeat();
            lastHeartbeat = currentTime;
        }
        
        // Handle incoming UDP packets
        int packetSize = udp.parsePacket();
        if (packetSize > 0) {
            IPAddress remote = udp.remoteIP();
            addOrUpdateClient(remote);
            
            msg.length = 0;
            while (packetSize-- && msg.length < sizeof(msg.data)) {
                msg.data[msg.length++] = udp.read();
            }
            msg.sourceIP = remote;
            
            if (xQueueSend(mavlinkQueue, &msg, 0) != pdPASS) {
                Serial.println("Failed to send MAVLink message to queue");
            }
        }
        
        // Process messages from queue and send to drone
        if (xQueueReceive(mavlinkQueue, &msg, 0) == pdPASS) {
            // Send the message to the drone via UART
            mavSerial.write(msg.data, msg.length);
            rxBytes += msg.length;
        }
        
        // Handle incoming UART data
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        int len = 0;
        while (mavSerial.available() && len < sizeof(buf)) {
            uint8_t byte = mavSerial.read();
            buf[len++] = byte;
            
            // Parse MAVLink message
            if (mavlink_parse_char(MAVLINK_COMM_0, byte, &mavlink_msg, &status)) {
                // Add message to history for menu display with parsed fields
                addMavlinkMessage(mavlink_msg);
                
                // Handle radio status message
                if (mavlink_msg.msgid == MAVLINK_MSG_ID_RADIO_STATUS) {
                    mavlink_radio_status_t radio_status;
                    mavlink_msg_radio_status_decode(&mavlink_msg, &radio_status);
                    radio_rssi = radio_status.rssi;
                }
            }
        }
        if (len > 0) {
            for (auto client : clients) {
                sendUDP(buf, len, client.ip);
            }
            txBytes += len;
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
                        Serial.println("Error: Failed to update display with highlight");
                    }
                } else {
                    if (!oled.update(lines)) {
                        Serial.println("Error: Failed to update display");
                    }
                }
                xSemaphoreGive(displayMutex);
            }
        } else {
            if (currentMission) {
                lines.push_back(String("Active Mission: ") + currentMission->getName());
            }
            // --- FLIGHT MODE from MAVLink ---
            // String flightMode;
            // if (getLatestFlightMode(flightMode)) {
            //     lines.push_back("Mode: " + flightMode);
            // }
            // --- BATTERY from MAVLink ---
            float vbat = 0.0f, ibat = 0.0f;
            int bat_rem = 0;
            if (getLatestBatteryInfo(vbat, ibat, bat_rem)) {
                lines.push_back("Battery: " + String(vbat, 2) + "V " + String(ibat, 2) + "A " + String(bat_rem) + "%");
            }
            lines.push_back("Drone/Tx: " + String(txBytes));
            lines.push_back("Device/Rx: " + String(rxBytes));
            
            // Radio signal strength
            int8_t rssi_dbm = (int8_t)(radio_rssi * 1.9f - 127);
            lines.push_back("Radio: " + String(rssi_dbm) + " dBm");
            
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
            
            // Update display with mutex protection
            if (xSemaphoreTake(displayMutex, portMAX_DELAY) == pdTRUE) {
                if (!oled.update(lines)) {
                    Serial.println("Error: Failed to update display");
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
    bool last_wifi_enabled = wifi_enabled;
    
    while (1) {
        if (wifi_enabled != last_wifi_enabled) {
            if (xSemaphoreTake(wifiMutex, portMAX_DELAY) == pdTRUE) {
                if (wifi_enabled) {
                    WiFi.softAP(ap_ssid, ap_pass);
                    Serial.println("WiFi AP enabled");
                } else {
                    WiFi.softAPdisconnect(true);
                    Serial.println("WiFi AP disabled");
                }
                xSemaphoreGive(wifiMutex);
            }
            last_wifi_enabled = wifi_enabled;
        }
        vTaskDelay(xDelay);
    }
}

// Mission Task
void missionTask(void *pvParameters) {
    const TickType_t xDelay = pdMS_TO_TICKS(100); // 100ms delay
    while (1) {
        updateCurrentMission();
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
        Serial.println("Error: Failed to create queues");
        return;
    }
    
    // Create mutexes
    wifiMutex = xSemaphoreCreateMutex();
    gpsMutex = xSemaphoreCreateMutex();
    displayMutex = xSemaphoreCreateMutex();
    
    if (wifiMutex == NULL || gpsMutex == NULL || displayMutex == NULL) {
        Serial.println("Error: Failed to create mutexes");
        return;
    }
    
    // Create tasks
    BaseType_t xReturned;
    
    xReturned = xTaskCreatePinnedToCore(gpsTask, "GPS", GPS_TASK_STACK_SIZE, NULL, GPS_TASK_PRIORITY, &gpsTaskHandle, 0);
    if (xReturned != pdPASS) {
        Serial.println("Error: Failed to create GPS task");
        return;
    }
    
    xReturned = xTaskCreatePinnedToCore(mavlinkTask, "MAVLink", MAVLINK_TASK_STACK_SIZE, NULL, MAVLINK_TASK_PRIORITY, &mavlinkTaskHandle, 1);
    if (xReturned != pdPASS) {
        Serial.println("Error: Failed to create MAVLink task");
        return;
    }
    
    xReturned = xTaskCreatePinnedToCore(displayTask, "Display", DISPLAY_TASK_STACK_SIZE, NULL, DISPLAY_TASK_PRIORITY, &displayTaskHandle, 0);
    if (xReturned != pdPASS) {
        Serial.println("Error: Failed to create Display task");
        return;
    }
    
    xReturned = xTaskCreatePinnedToCore(buttonTask, "Button", BUTTON_TASK_STACK_SIZE, NULL, BUTTON_TASK_PRIORITY, &buttonTaskHandle, 0);
    if (xReturned != pdPASS) {
        Serial.println("Error: Failed to create Button task");
        return;
    }
    
    xReturned = xTaskCreatePinnedToCore(wifiTask, "WiFi", WIFI_TASK_STACK_SIZE, NULL, WIFI_TASK_PRIORITY, &wifiTaskHandle, 1);
    if (xReturned != pdPASS) {
        Serial.println("Error: Failed to create WiFi task");
        return;
    }
    
    xReturned = xTaskCreatePinnedToCore(missionTask, "Mission", MISSION_TASK_STACK_SIZE, NULL, MISSION_TASK_PRIORITY, NULL, 0);
    if (xReturned != pdPASS) {
        Serial.println("Error: Failed to create Mission task");
        return;
    }
    
    Serial.println("All tasks created successfully");
} 