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
#include <errno.h> 

static bool connected = false;
int ConnectionManager::sockfd = -1;
int ConnectionManager::newsockfd = -1;

int ConnectionManager::sitlPort = 4560;
std::string ConnectionManager::status = "Disconnected";
std::string ConnectionManager::lastMessage = "";

#if IBM
bool ConnectionManager::initializeWinSock() {
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        XPLMDebugString("px4xplane: Could not initialize Winsock.\n");
        return false;
    }
    return true;
}
#endif


void ConnectionManager::setupServerSocket() {
    XPLMDebugString("px4xplane: Setting up server socket...\n");

    if (sockfd != -1 || connected) {
        XPLMDebugString("px4xplane: Server socket already set up or connected, aborting new setup attempt.\n");
        return;
    }

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        XPLMDebugString("px4xplane: Error opening socket.\n");
        return;
    }

    sockaddr_in serv_addr{};
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(sitlPort);

    if (bind(sockfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        XPLMDebugString("px4xplane: Error on binding.\n");
        closeSocket(sockfd);
        return;
    }

    if (listen(sockfd, 5) < 0) {
        XPLMDebugString("px4xplane: Error on listen.\n");
        closeSocket(sockfd);
        return;
    }
    XPLMDebugString("px4xplane: Server socket set up, waiting for connection...\n");
    acceptConnection();

}


void ConnectionManager::acceptConnection() {

    if (connected) {
        XPLMDebugString("px4xplane: Already connected, aborting new accept attempt.\n");
        return;
    }

    XPLMDebugString("px4xplane: Waiting for SITL connection...\n");

    sockaddr_in cli_addr{};
    socklen_t clilen = sizeof(cli_addr);

     newsockfd = accept(sockfd, (struct sockaddr*)&cli_addr, &clilen);
    if (newsockfd < 0) {
        XPLMDebugString("px4xplane: Error on accept.\n");
        return;
    }

    XPLMDebugString("px4xplane: Connected to SITL successfully.\n");
    connected = true;
    status = "Connected";
    setLastMessage("Connected to SITL successfully.");

    // You might want to spawn a thread to handle communication over newsockfd or set it to non-blocking mode.
}



void ConnectionManager::disconnect() {
    if (!connected) return;
    closeSocket(sockfd);
    sockfd = -1;
    closeSocket(newsockfd); // Close the newsockfd
    newsockfd = -1;
    connected = false;
    status = "Disconnected";
    setLastMessage("Disconnected from SITL.");
    XPLMDebugString("px4xplane: Disconnected from SITL\n");
}

void ConnectionManager::closeSocket(int& sockfd) {
    if (sockfd != INVALID_SOCKET) {
#if IBM
        closesocket(sockfd);
#elif LIN || APL
        close(sockfd);
#endif
        sockfd = -1; // set to -1 to indicate that the socket is no longer valid
    }



}
void ConnectionManager::sendData(const uint8_t* buffer, int len) {
    if (!connected) return;

    // Log the MAVLink packet in an interpretable format
    std::string logMessage = "Sending MAVLink packet: ";
    /*for (int i = 0; i < len; i++) {
        logMessage += std::to_string(buffer[i]) + " ";
    }*/
    XPLMDebugString(logMessage.c_str());

    int totalBytesSent = 0;
    while (totalBytesSent < len) {
        int bytesSent = send(newsockfd, reinterpret_cast<const char*>(buffer) + totalBytesSent, len - totalBytesSent, 0);

        if (bytesSent < 0) {
            char buf[256];
#if IBM
            snprintf(buf, sizeof(buf), "px4xplane: Error sending data: %d\n", WSAGetLastError());
#elif LIN || APL
            snprintf(buf, sizeof(buf), "px4xplane: Error sending data: %s\n", strerror(errno));
#endif
            XPLMDebugString(buf);
            // Optionally, handle error, e.g., clean up and/or reconnect
            return;
        }
        else if (bytesSent == 0) {
            // The peer has closed the connection.
            XPLMDebugString("px4xplane: Peer has closed the connection\n");

            // Clean up and/or try to reconnect.
            return;
        }

        totalBytesSent += bytesSent;
    }
}


void ConnectionManager::receiveData() {
    if (!connected) return;

    char buffer[256];
    memset(buffer, 0, sizeof(buffer));

    int bytesReceived = recv(newsockfd, buffer, sizeof(buffer) - 1, 0);
    if (bytesReceived < 0) {
        XPLMDebugString("px4xplane: Error receiving data\n");
    }
    else if (bytesReceived > 0) {
        buffer[bytesReceived] = '\0'; // Null-terminate the received data
        setLastMessage(buffer); // Store the received message
        XPLMDebugString(buffer); // Write the received message to the X-Plane log
    }
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
#if IBM
void ConnectionManager::cleanupWinSock() {
    WSACleanup();
}
#endif
