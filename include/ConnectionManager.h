#pragma once
#include <cstdint>
#include <string>
#include <map>
#include "ByteSendQueue.h"

#if defined(_WIN32)
using SocketHandle = std::uintptr_t;
static constexpr SocketHandle INVALID_SOCKET_HANDLE =
    ~static_cast<SocketHandle>(0);
#else
using SocketHandle = int;
static constexpr SocketHandle INVALID_SOCKET_HANDLE = -1;
#endif

class ConnectionManager {
public:
    static void setupServerSocket();
    static void tryAcceptConnection();  // RENAMED from acceptConnection() - now non-blocking
    static bool isWaitingForConnection();  // NEW: Check if socket ready but not connected
    static bool isConnectionActiveOrWaiting();
    static void disconnect();
    static uint64_t sendData(const uint8_t* buffer, int len);
    static bool flushSendQueue();
    static bool isSendComplete(uint64_t completionToken);
    static size_t getPendingSendBytes();
    static void receiveData(uint32_t initialWaitUsec = 0);
    static bool isConnected();
    static const std::string& getStatus();
    static const std::string& getPeerEndpoint();
    static void setLastMessage(const std::string& message); // Function to set the last message
    static const std::string& getLastMessage(); // Function to get the last message
    static void cleanupWinSock();
    static bool initializeWinSock();
    static std::map<int, int> motorMappings;
    static std::map<int, int> loadMotorMappings(const std::string& filename);

private:
    static void closeSocket(SocketHandle& sock);
    static SocketHandle sockfd;
    static SocketHandle newsockfd;
    static std::string status;
    static int sitlPort;
    static std::string lastMessage; // Variable to keep the last message
    static std::string peerEndpoint;
    static ByteSendQueue pendingSendQueue;

};
