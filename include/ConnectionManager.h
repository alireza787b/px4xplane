#pragma once
#include <string>
#include <map>

class ConnectionManager {
public:
    static void setupServerSocket();
    static void tryAcceptConnection();  // RENAMED from acceptConnection() - now non-blocking
    static bool isWaitingForConnection();  // NEW: Check if socket ready but not connected
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
    static std::map<int, int> motorMappings;
    static std::map<int, int> loadMotorMappings(const std::string& filename);

private:
    static int sockfd; // Socket file descriptor
    static int newsockfd; // Declaration of newsockfd
    static std::string status;
    static int sitlPort;
    static std::string lastMessage; // Variable to keep the last message

};