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
}; 