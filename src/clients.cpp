#include "clients.h"
#include <algorithm>

// Define the global clients vector
std::vector<GCSClient> clients;

void addOrUpdateClient(IPAddress ip) {
  for (auto &c : clients) {
    if (c.ip == ip) {
      c.lastSeen = millis();
      return;
    }
  }
  clients.push_back({ip, millis()});
  Serial.print("New GCS detected: ");
  Serial.println(ip);
}

void pruneClients() {
  unsigned long now = millis();
  clients.erase(
    std::remove_if(clients.begin(), clients.end(),
      [now](GCSClient &c) {
        return now - c.lastSeen > 30000; // 30 seconds timeout
      }),
    clients.end()
  );
} 