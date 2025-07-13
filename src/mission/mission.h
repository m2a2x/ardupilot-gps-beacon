#pragma once
#include <Arduino.h>
#include <string>

class Mission {
public:
    virtual ~Mission() {}
    virtual void start() = 0;
    virtual void update() = 0;
    virtual void stop() = 0;
    virtual const char* getName() const = 0;
    virtual const char* getType() const = 0;  // Get mission type for type checking
    
    // Update counter methods
    unsigned long getUpdateCount() const { return updateCount; }
    void resetUpdateCount() { updateCount = 0; }
    
protected:
    unsigned long updateCount = 0;  // Counter for successful updates
}; 