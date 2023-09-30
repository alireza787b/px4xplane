#pragma once
#include <string>
class ConnectionManager {
public:
    static void connect(const std::string& sitlIp);
    static void disconnect();
    static void closeSocket(int& sock);
    static void sendData();
    static void receiveData();
    static bool isConnected();
    static const std::string& getStatus();
    static void setLastMessage(const std::string& message); // Function to set the last message
    static const std::string& getLastMessage(); // Function to get the last message

private:
    static int sockfd; // Socket file descriptor
    static std::string status;
    static int sitlPort;
    static std::string lastMessage; // Variable to keep the last message

};
