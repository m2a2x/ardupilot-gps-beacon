#pragma once
#include <Arduino.h>
#include <vector>
#include <IPAddress.h>

/**
 * Structure representing a Ground Control Station client
 */
struct GCSClient {
  IPAddress ip;
  unsigned long lastSeen;
};

// Global client list
extern std::vector<GCSClient> clients;

/**
 * Add a new client or update existing client's last seen timestamp
 * @param ip IP address of the client
 */
void addOrUpdateClient(IPAddress ip);

/**
 * Remove clients that haven't been seen for more than 30 seconds
 */
void pruneClients();
