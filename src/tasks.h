#pragma once
#include "mavlink_cmds.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <vector>
#include <IPAddress.h>
#include <WString.h>
#include <string>
#include <algorithm>
#include <memory>

// Task priorities
#define GPS_TASK_PRIORITY 3
#define MAVLINK_TASK_PRIORITY 4
#define DISPLAY_TASK_PRIORITY 2
#define BUTTON_TASK_PRIORITY 3
#define WIFI_TASK_PRIORITY 2

// Task stack sizes
#define GPS_TASK_STACK_SIZE 4096
#define MAVLINK_TASK_STACK_SIZE 4096
#define DISPLAY_TASK_STACK_SIZE 2048
#define BUTTON_TASK_STACK_SIZE 2048
#define WIFI_TASK_STACK_SIZE 2048

// Queue sizes
#define MAVLINK_QUEUE_SIZE 10
#define DISPLAY_QUEUE_SIZE 5
#define GPS_QUEUE_SIZE 20

// Task handles
extern TaskHandle_t gpsTaskHandle;
extern TaskHandle_t mavlinkTaskHandle;
extern TaskHandle_t displayTaskHandle;
extern TaskHandle_t buttonTaskHandle;
extern TaskHandle_t wifiTaskHandle;

// Queue handles
extern QueueHandle_t mavlinkQueue;
extern QueueHandle_t displayQueue;
extern QueueHandle_t gpsQueue;

// Mutex handles
extern SemaphoreHandle_t gpsMutex;
extern SemaphoreHandle_t displayMutex;

// Task function declarations
void gpsTask(void *pvParameters);
void mavlinkTask(void *pvParameters);
void displayTask(void *pvParameters);
void buttonTask(void *pvParameters);
void wifiTask(void *pvParameters);

// Initialize all tasks and queues
void initTasks();

// Queue message structures
struct MavlinkMessage {
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    size_t length;
    IPAddress sourceIP;
};

struct DisplayMessage {
    std::vector<String> lines;
};

struct GPSData {
    double latitude;
    double longitude;
    float altitude;
    int satellites;
    bool hasFix;
    bool isStale;
};

// Radio signal strength
extern int8_t radio_rssi; 