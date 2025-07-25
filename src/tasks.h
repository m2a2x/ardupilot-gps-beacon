#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <vector>
#include <string>
#include "display.h"
#include "menu/menu.h"
#include "gps.h"
#include "radio.h"
#include "udp_module.h"
#include "mission/mission.h"
#include "drone_status.h"

// Task priorities
#define GPS_TASK_PRIORITY 2
#define MAVLINK_TASK_PRIORITY 3
#define DISPLAY_TASK_PRIORITY 1
#define BUTTON_TASK_PRIORITY 1
#define WIFI_TASK_PRIORITY 1
#define MISSION_TASK_PRIORITY 2

// Task delays
#define GPS_TASK_DELAY 200
#define MAVLINK_TASK_DELAY 1
#define DISPLAY_TASK_DELAY 200
#define BUTTON_TASK_DELAY 50
#define WIFI_TASK_DELAY 1000
#define MISSION_TASK_DELAY 100

// Activity timeout for display
#define ACTIVITY_TIMEOUT 3000  // 3 seconds

// Task handles
extern TaskHandle_t gpsTaskHandle;
extern TaskHandle_t mavlinkTaskHandle;
extern TaskHandle_t displayTaskHandle;
extern TaskHandle_t buttonTaskHandle;
extern TaskHandle_t wifiTaskHandle;
extern TaskHandle_t missionTaskHandle;

// Queues and semaphores
extern SemaphoreHandle_t displayMutex;

// External variables
extern StatusDisplay oled;

// Global drone status object
extern DroneStatus droneStatus;

// Transmission activity tracking
extern unsigned long lastTxTime;
extern unsigned long lastRxTime;

// Task function declarations
void gpsTask(void *pvParameters);
void mavlinkTask(void *pvParameters);
void displayTask(void *pvParameters);
void buttonTask(void *pvParameters);
void wifiTask(void *pvParameters);
void missionTask(void *pvParameters);

// Task initialization
void initTasks(); 