#pragma once

#include <Arduino.h>

/**
 * Simple Log Proxy Class
 * 
 * Provides a centralized logging interface to replace direct Serial.println calls.
 * This allows for easy modification of logging behavior across the entire application.
 */
class LogProxy {
public:
    /**
     * Log a message with automatic newline
     * @param message The message to log
     */
    static void log(const String& message);
    
    /**
     * Log a message with automatic newline (const char* version)
     * @param message The message to log
     */
    static void log(const char* message);
}; 