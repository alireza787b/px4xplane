#pragma once
#include <string>
class ConnectionManager {
public:
    static void setupServerSocket();
    static void acceptConnection();
    static void disconnect();
    static void closeSocket(int& sock);
    static void sendData(const uint8_t* buffer, int len);
    static void receiveData();
    static bool isConnected();
    static const std::string& getStatus();
    static void setLastMessage(const std::string& message); // Function to set the last message
    static const std::string& getLastMessage(); // Function to get the last message
    static void cleanupWinSock();
    static bool initializeWinSock();
    static void receiveHILActuatorControls();

private:
    static int sockfd; // Socket file descriptor
    static int newsockfd; // Declaration of newsockfd
    static std::string status;
    static int sitlPort;
    static std::string lastMessage; // Variable to keep the last message
};
