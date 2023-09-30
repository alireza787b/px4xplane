#include "ConnectionManager.h"
#include "DataRefManager.h"

class MAVLinkManager {
public:
    MAVLinkManager(ConnectionManager& connManager, DataRefManager& dataRefManager);
    void initialize();
    void createAndSendMAVLinkMessages();

private:
    ConnectionManager& m_connManager;
    DataRefManager& m_dataRefManager;
    void sendHILSensor();
    void receiveMAVLinkMessages();
};
