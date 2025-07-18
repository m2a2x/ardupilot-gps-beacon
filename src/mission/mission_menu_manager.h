#pragma once
#include <Arduino.h>
#include <vector>
#include "mission.h"
#include "../menu/menu_types.h"

/**
 * Mission Menu Manager
 * Handles mission-specific menu operations and delegates to individual missions
 */
class MissionMenuManager {
public:
    static MissionMenuManager& getInstance();
    
    // Get menu options for a specific mission type
    std::vector<MenuOption> getMissionMenuOptions(const Mission* mission) const;
    
    // Handle menu action for a specific mission
    void handleMissionMenuAction(Mission* mission, MenuOption option);
    
    // Get menu display for a specific mission
    void getMissionMenuDisplay(const Mission* mission, std::vector<String>& lines, MenuOption selectedOption);
    
    // Check if a mission's menu is active
    bool isMissionMenuActive(const Mission* mission) const;
    
    // Get the first menu option for a mission
    MenuOption getFirstMenuOption(const Mission* mission) const;
    
    // Get the next menu option for a mission
    MenuOption getNextMenuOption(const Mission* mission, MenuOption currentOption) const;
    
    // Get the previous menu option for a mission
    MenuOption getPreviousMenuOption(const Mission* mission, MenuOption currentOption) const;
    
private:
    MissionMenuManager() = default;
    ~MissionMenuManager() = default;
    MissionMenuManager(const MissionMenuManager&) = delete;
    MissionMenuManager& operator=(const MissionMenuManager&) = delete;
}; 