#include "ConnectionManager.h"
#if IBM
#include <winsock2.h>
#include <ws2tcpip.h> // For inet_pton
#pragma comment(lib, "Ws2_32.lib")
#endif
#if LIN || APL
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#endif
#include "XPLMUtilities.h"
#include <cstring>
#include <string>

static bool connected = false;
int ConnectionManager::sockfd = -1;
int ConnectionManager::sitlPort = 4560;
std::string ConnectionManager::status = "Disconnected";
std::string ConnectionManager::lastMessage = "";


void ConnectionManager::connect(const std::string& sitlIp) {
    if (connected) return;

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        XPLMDebugString("px4xplane: Error opening socket\n");
        return;
    }

    sockaddr_in serv_addr{};
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(sitlPort);
    serv_addr.sin_addr.s_addr = inet_addr(sitlIp.c_str());

    if (bind(sockfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        XPLMDebugString("px4xplane: Error on binding the socket\n");
        closeSocket(sockfd);
        return;
    }

    if (listen(sockfd, 5) < 0) {
        XPLMDebugString("px4xplane: Error on listening\n");
        closeSocket(sockfd);
        return;
    }

    XPLMDebugString("px4xplane: Waiting for SITL connection...\n");

    sockaddr_in client_addr{};
    socklen_t client_len = sizeof(client_addr);
    int newsockfd = accept(sockfd, (struct sockaddr*)&client_addr, &client_len);
    if (newsockfd < 0) {
        XPLMDebugString("px4xplane: Acceptance failed\n");
        closeSocket(sockfd);
        return;
    }

    XPLMDebugString("px4xplane: Connected to SITL\n");
    closeSocket(sockfd); // Consider if you want to close the listening socket now or keep it for more connections.

    connected = true;
    status = "Connected";
    setLastMessage("Connected to SITL successfully.");
    }

void ConnectionManager::disconnect() {
    if (!connected) return;
    closeSocket(sockfd);
    sockfd = -1;
    connected = false;
    status = "Disconnected";
    setLastMessage("Disconnected from SITL.");
    XPLMDebugString("px4xplane: Disconnected from SITL\n");
}

void ConnectionManager::closeSocket(int& sock) {
#if IBM
    closesocket(sock);
#else
    close(sock);
#endif
}
void ConnectionManager::sendData() {
    if (!connected) return;
    // Implement sending data here
}

void ConnectionManager::receiveData() {
    if (!connected) return;
    // Implement receiving data here
}

bool ConnectionManager::isConnected() {
    return connected;
}

const std::string& ConnectionManager::getStatus() {
    return status;
}

void ConnectionManager::setLastMessage(const std::string& message) {
    lastMessage = message;
}

const std::string& ConnectionManager::getLastMessage() {
    return lastMessage;
}