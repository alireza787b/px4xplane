#include "MAVLinkManager.h"

MAVLinkManager::MAVLinkManager(ConnectionManager& connManager, DataRefManager& dataRefManager)
    : m_connManager(connManager), m_dataRefManager(dataRefManager) {}

void MAVLinkManager::initialize() {
    // Initialize MAVLink related stuff if needed
}

void MAVLinkManager::createAndSendMAVLinkMessages() {
    if (m_connManager.isConnected()) {
        sendHILSensor();
    }
}

void MAVLinkManager::sendHILSensor() {
    // Get needed data from DataRefManager
    // Create HIL_SENSOR message
    // Send message via ConnectionManager
}

void MAVLinkManager::receiveMAVLinkMessages() {
    // Receive and handle incoming MAVLink messages
}
