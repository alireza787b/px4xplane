#include "ConnectionManager.h"
#if IBM
#include <winsock2.h>
#endif
#if LIN || APL
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#endif
#include "XPLMUtilities.h"
#include <cstring>
#include <string>

static bool connected = false; // To maintain the connection state
static std::string status = "Disconnected"; // Default status

void ConnectionManager::connect() {
    if (connected) return; // Already connected

    // Initialize your connection here
    // Example: Creating socket, connecting to a server, etc.

    // If connection is successful:
    connected = true;
    status = "Connected"; // Update status on successful connection
    XPLMDebugString("px4xplane: Connected to SITL\n");
}

void ConnectionManager::disconnect() {
    if (!connected) return; // Not connected

    // Clean up your connection here
    // Example: Closing socket, etc.

    connected = false;
    status = "Disconnected"; // Update status on successful disconnection
    XPLMDebugString("px4xplane: Disconnected from SITL\n");
}

void ConnectionManager::sendData() {
    if (!connected) return; // Not connected

    // Implement sending data here
}

void ConnectionManager::receiveData() {
    if (!connected) return; // Not connected

    // Implement receiving data here
}

bool ConnectionManager::isConnected() {
    return connected;
}
const std::string& ConnectionManager::getStatus() {
    return status;
}