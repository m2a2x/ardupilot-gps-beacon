#include "mission_menu_manager.h"
#include "menu/menu.h"
#include "menu/menu_types.h"  // For MenuOption enum
#include "log_proxy.h"

MissionMenuManager& MissionMenuManager::getInstance() {
    static MissionMenuManager instance;
    return instance;
}

std::vector<MenuOption> MissionMenuManager::getMissionMenuOptions(const Mission* mission) const {
    if (mission) {
        return mission->getMenuOptions();
    }
    return {};
}

void MissionMenuManager::handleMissionMenuAction(Mission* mission, MenuOption option) {
    if (mission) {
        mission->handleMenuAction(option);
    }
}

void MissionMenuManager::getMissionMenuDisplay(const Mission* mission, std::vector<String>& lines, MenuOption selectedOption) {
    if (mission) {
        mission->getMenuDisplay(lines, selectedOption);
    }
}

bool MissionMenuManager::isMissionMenuActive(const Mission* mission) const {
    if (mission) {
        return mission->isMenuActive();
    }
    return false;
}

MenuOption MissionMenuManager::getFirstMenuOption(const Mission* mission) const {
    auto options = getMissionMenuOptions(mission);
    if (!options.empty()) {
        return options[0];
    }
    return START_MODE; // Default fallback
}

MenuOption MissionMenuManager::getNextMenuOption(const Mission* mission, MenuOption currentOption) const {
    auto options = getMissionMenuOptions(mission);
    if (options.empty()) {
        return currentOption;
    }
    
    // Find current option in array
    for (size_t i = 0; i < options.size(); i++) {
        if (options[i] == currentOption) {
            // Return next option (wrap around)
            return options[(i + 1) % options.size()];
        }
    }
    
    // If not found, return first option
    return options[0];
}

MenuOption MissionMenuManager::getPreviousMenuOption(const Mission* mission, MenuOption currentOption) const {
    auto options = getMissionMenuOptions(mission);
    if (options.empty()) {
        return currentOption;
    }
    
    // Find current option in array
    for (size_t i = 0; i < options.size(); i++) {
        if (options[i] == currentOption) {
            // Return previous option (wrap around)
            return options[(i - 1 + options.size()) % options.size()];
        }
    }
    
    // If not found, return first option
    return options[0];
} 