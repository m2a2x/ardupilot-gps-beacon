#pragma once
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <vector>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define CHAR_HEIGHT 8  // Pixels per line at text size 1
#define MAX_DISPLAY_LINES (SCREEN_HEIGHT / CHAR_HEIGHT)

class StatusDisplay {
public:
  StatusDisplay();
  
  /**
   * Initialize the display
   * @return true if initialization was successful
   */
  bool begin();

  /**
   * Update display with vector of strings
   * @param lines Vector of strings to display
   * @return true if update was successful
   */
  bool update(const std::vector<String>& lines);

  /**
   * Update display with C-style array of strings
   * @param lines Array of strings to display
   * @param count Number of strings in array
   * @return true if update was successful
   */
  bool update(const char* lines[], size_t count);

  /**
   * Update display with vector of strings and highlight specific lines
   * @param lines Vector of strings to display
   * @param highlightLines Vector of line indices to highlight (0-based)
   * @return true if update was successful
   */
  bool updateWithHighlight(const std::vector<String>& lines, const std::vector<int>& highlightLines);

  /**
   * Check if display is initialized
   * @return true if display is ready to use
   */
  bool isInitialized() const;

private:
  Adafruit_SSD1306 display;
  bool initialized;
};
