#pragma once
#include <Arduino.h>
#include <string>
#include <vector>
#include "../menu/menu_types.h"  // Include MenuOption enum definition

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
    
    // Get current state for display (default implementation returns "Unknown")
    virtual const char* getCurrentStateName() const { return "Unknown"; }
    
    // Menu control methods - each mission handles its own menu
    virtual std::vector<MenuOption> getMenuOptions() const = 0;
    virtual void handleMenuAction(MenuOption option) = 0;
    virtual void getMenuDisplay(std::vector<String>& lines, MenuOption selectedOption) const = 0;
    virtual bool isMenuActive() const = 0;
    
protected:
    unsigned long updateCount = 0;  // Counter for successful updates
};

/**
 * Base mission class that provides common menu functionality
 * Most missions can inherit from this to get standard start/stop/back menu
 */
class BaseMission : public Mission {
public:
    // Common menu options for most missions
    std::vector<MenuOption> getMenuOptions() const override;
    
    // Common menu action handling
    void handleMenuAction(MenuOption option) override;
    
    // Common menu display
    void getMenuDisplay(std::vector<String>& lines, MenuOption selectedOption) const override;
    
    // Menu is active if mission is running
    bool isMenuActive() const override { return isRunning; }
    
protected:
    bool isRunning = false;
    
    // Override these in derived classes for mission-specific actions
    virtual void onStart() = 0;
    virtual void onStop() = 0;
    virtual void onRTL() = 0;
}; 