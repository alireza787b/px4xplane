#pragma once
#include <string>
class ConnectionManager {
public:
    static void connect();
    static void disconnect();
    static void sendData();
    static void receiveData();
    static bool isConnected();
    static const std::string& getStatus();

};
