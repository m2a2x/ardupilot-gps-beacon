#include "mission.h"
#include "menu/menu.h"
#include "menu/menu_types.h"  // For MenuOption enum
#include "log_proxy.h"

std::vector<MenuOption> BaseMission::getMenuOptions() const {
    return {START_MODE, STOP_MODE, RTL_MODE, BACK_TO_MODE};
}

void BaseMission::handleMenuAction(MenuOption option) {
    switch (option) {
        case START_MODE:
            if (!isRunning) {
                onStart();
                start(); // Call the original start method
                isRunning = true;
                LogProxy::log(String(getName()) + " started successfully");
            }
            break;
            
        case STOP_MODE:
            if (isRunning) {
                onStop();
                stop(); // Call the original stop method
                isRunning = false;
                LogProxy::log(String(getName()) + " stopped successfully");
            }
            break;
            
        case RTL_MODE:
            if (isRunning) {
                onRTL();
                isRunning = false;
                LogProxy::log(String(getName()) + " RTL initiated successfully");
            }
            break;
            
        case BACK_TO_MODE:
            // This will be handled by the main menu system
            break;
            
        default:
            break;
    }
}

void BaseMission::getMenuDisplay(std::vector<String>& lines, MenuOption selectedOption) const {
    lines.push_back("== " + String(getName()) + " ==");
    
    if (isRunning) {
        lines.push_back("Updates: " + String(getUpdateCount()));
        lines.push_back("State: " + String(getCurrentStateName()));
    }
    
    lines.push_back("");
    lines.push_back((selectedOption == START_MODE ? "> " : "  ") + String("Start"));
    lines.push_back((selectedOption == STOP_MODE ? "> " : "  ") + String("Stop"));
    lines.push_back((selectedOption == RTL_MODE ? "> " : "  ") + String("RTL"));
    lines.push_back((selectedOption == BACK_TO_MODE ? "> " : "  ") + String("Back"));
} 