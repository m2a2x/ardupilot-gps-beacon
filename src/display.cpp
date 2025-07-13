#include "display.h"
#include "conf.h"  // For SDA_PIN and SCL_PIN
#include "log_proxy.h"  // For logging

StatusDisplay::StatusDisplay() : display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1), initialized(false) {}

bool StatusDisplay::begin() {
  Wire.begin(SDA_PIN, SCL_PIN);  // SDA = GPIO23, SCL = GPIO22
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    LogProxy::log("Error: SSD1306 display not found");
    return false;
  }
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Display Ready");
  display.display();
  
  initialized = true;
  return true;
}

bool StatusDisplay::update(const std::vector<String>& lines) {
  if (!initialized) {
    LogProxy::log("Error: Display not initialized");
    return false;
  }

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  int y = 0;
  for (const String& line : lines) {
    if (y + CHAR_HEIGHT > SCREEN_HEIGHT) {
      LogProxy::log("Warning: Display overflow, truncating content");
      break;
    }
    display.setCursor(0, y);
    display.println(line);
    y += CHAR_HEIGHT;
  }

  display.display();
  return true;
}

bool StatusDisplay::update(const char* lines[], size_t count) {
  if (!initialized) {
    LogProxy::log("Error: Display not initialized");
    return false;
  }

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  int y = 0;
  for (size_t i = 0; i < count; ++i) {
    if (y + CHAR_HEIGHT > SCREEN_HEIGHT) {
      LogProxy::log("Warning: Display overflow, truncating content");
      break;
    }
    display.setCursor(0, y);
    display.println(lines[i]);
    y += CHAR_HEIGHT;
  }

  display.display();
  return true;
}

bool StatusDisplay::updateWithHighlight(const std::vector<String>& lines, const std::vector<int>& highlightLines) {
  if (!initialized) {
    LogProxy::log("Error: Display not initialized");
    return false;
  }

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);

  int y = 0;
  for (size_t i = 0; i < lines.size(); ++i) {
    if (y + CHAR_HEIGHT > SCREEN_HEIGHT) {
      LogProxy::log("Warning: Display overflow, truncating content");
      break;
    }
    
    // Check if this line should be highlighted
    bool shouldHighlight = false;
    for (int highlightLine : highlightLines) {
      if (highlightLine == static_cast<int>(i)) {
        shouldHighlight = true;
        break;
      }
    }
    
    // Set text color based on highlight status
    if (shouldHighlight) {
      display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Inverted colors for highlight
    } else {
      display.setTextColor(SSD1306_WHITE, SSD1306_BLACK); // Normal colors
    }
    
    display.setCursor(0, y);
    display.println(lines[i]);
    y += CHAR_HEIGHT;
  }

  display.display();
  return true;
}

bool StatusDisplay::isInitialized() const {
  return initialized;
} 