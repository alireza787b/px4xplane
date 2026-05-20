#include "ConnectionManager.h"
#include "MAVLinkManager.h"
#if IBM
#include <winsock2.h>
#include <ws2tcpip.h> // For inet_pton
#pragma comment(lib, "Ws2_32.lib")
#endif
#if LIN || APL
#include <unistd.h>
#include <fcntl.h>      // For fcntl(), F_GETFL, F_SETFL, O_NONBLOCK
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

#endif
#include "XPLMUtilities.h"
#include <cstring>
#include <cstdio>
#include <string>
#include <errno.h> 


#include <fstream>
#include <sstream>
#include "DataRefManager.h"
#include <ConfigManager.h>

#if LIN || APL
#define INVALID_SOCKET -1
#endif


static bool connected = false;
std::map<int, int> ConnectionManager::motorMappings;
int ConnectionManager::sockfd = -1;
int ConnectionManager::newsockfd = -1;

int ConnectionManager::sitlPort = 4560;
std::string ConnectionManager::status = "Disconnected";
std::string ConnectionManager::lastMessage = "";
std::string ConnectionManager::peerEndpoint = "";

namespace {

std::string socketErrorString() {
    char buf[160];
#if IBM
    snprintf(buf, sizeof(buf), "Windows socket error %d", WSAGetLastError());
#else
    snprintf(buf, sizeof(buf), "%s", strerror(errno));
#endif
    return std::string(buf);
}

bool socketWouldBlock() {
#if IBM
    const int err = WSAGetLastError();
    return err == WSAEWOULDBLOCK;
#else
    return errno == EWOULDBLOCK || errno == EAGAIN;
#endif
}

bool socketInterrupted() {
#if IBM
    return false;
#else
    return errno == EINTR;
#endif
}

int sendNoSignalFlags() {
#ifdef MSG_NOSIGNAL
    return MSG_NOSIGNAL;
#else
    return 0;
#endif
}

bool setSocketNonBlocking(int socketFd, const char* label) {
#if IBM
    u_long mode = 1;  // 1 = non-blocking, 0 = blocking
    if (ioctlsocket(socketFd, FIONBIO, &mode) != 0) {
        std::string msg = std::string("px4xplane: Error setting ") + label +
            " socket to non-blocking mode: " + socketErrorString() + "\n";
        XPLMDebugString(msg.c_str());
        return false;
    }
#elif LIN || APL
    int flags = fcntl(socketFd, F_GETFL, 0);
    if (flags < 0 || fcntl(socketFd, F_SETFL, flags | O_NONBLOCK) < 0) {
        std::string msg = std::string("px4xplane: Error setting ") + label +
            " socket to non-blocking mode: " + socketErrorString() + "\n";
        XPLMDebugString(msg.c_str());
        return false;
    }
#endif
    return true;
}

bool setTcpNoDelay(int socketFd) {
    int enabled = 1;
    if (setsockopt(socketFd, IPPROTO_TCP, TCP_NODELAY,
                   reinterpret_cast<const char*>(&enabled), sizeof(enabled)) < 0) {
        std::string msg = "px4xplane: Warning - could not set TCP_NODELAY: " +
            socketErrorString() + "\n";
        XPLMDebugString(msg.c_str());
        return false;
    }
    return true;
}

void setNoSigPipeIfSupported(int socketFd) {
#ifdef SO_NOSIGPIPE
    int enabled = 1;
    setsockopt(socketFd, SOL_SOCKET, SO_NOSIGPIPE,
               reinterpret_cast<const char*>(&enabled), sizeof(enabled));
#else
    (void)socketFd;
#endif
}

std::string endpointString(const sockaddr_in& addr) {
    char host[INET_ADDRSTRLEN] = {0};
    const char* converted = inet_ntop(AF_INET, &addr.sin_addr, host, sizeof(host));
    if (!converted) {
        snprintf(host, sizeof(host), "unknown");
    }

    char endpoint[128];
    snprintf(endpoint, sizeof(endpoint), "%s:%u", host, ntohs(addr.sin_port));
    return std::string(endpoint);
}

} // namespace

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
        status = "Socket Error";
        setLastMessage("Failed to create socket. System error.");
        XPLMSpeakString("Socket creation failed");
        return;
    }

    // CRITICAL FIX (January 2025): Allow immediate port reuse after disconnect
    // Without this, port 4560 enters TIME_WAIT state and remains unavailable for 30-60s
    // This caused "nothing happens" bug when user tried to reconnect quickly
    int reuse = 1;
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR,
                   (const char*)&reuse, sizeof(reuse)) < 0) {
        XPLMDebugString("px4xplane: Warning - could not set SO_REUSEADDR\n");
        // Continue anyway - not critical enough to abort
    }

    sockaddr_in serv_addr{};
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(sitlPort);

    if (bind(sockfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        XPLMDebugString("px4xplane: Error on binding port 4560.\n");

        // UX FIX: Notify user of error instead of silent failure
        status = "Bind Error";
        setLastMessage("Failed to bind port 4560. Port may be in use by another program.");
        XPLMSpeakString("Port bind failed");

        closeSocket(sockfd);
        sockfd = -1;
        return;
    }

    if (listen(sockfd, 5) < 0) {
        XPLMDebugString("px4xplane: Error on listen.\n");
        closeSocket(sockfd);
        return;
    }

    // CRITICAL FIX (January 2025): Make socket non-blocking
    // BEFORE: accept() was blocking → X-Plane froze until PX4 connected
    // AFTER: Non-blocking socket → poll in flight loop → no freezing
    if (!setSocketNonBlocking(sockfd, "listening")) {
        closeSocket(sockfd);
        return;
    }

    // UX FIX (January 2025): Update status for user visibility
    status = "Waiting for PX4 SITL on TCP 4560...";
    setLastMessage("Server socket ready on port 4560. Start PX4 SITL to connect.");

    XPLMDebugString("px4xplane: Server socket ready on port 4560, waiting for PX4 SITL to connect...\n");
    XPLMSpeakString("Waiting for PX4 connection");  // Audio feedback

    // NOTE: Don't call acceptConnection() here anymore - poll in flight loop instead

}


/**
 * @brief Non-blocking poll for incoming PX4 connection.
 *
 * CRITICAL FIX (January 2025): Non-blocking connection accept
 *
 * BEFORE: acceptConnection() used blocking accept() → X-Plane froze
 * AFTER: tryAcceptConnection() polls non-blocking socket → no freeze
 *
 * This function is called every flight loop frame when waiting for connection.
 * Returns immediately if no connection available (EWOULDBLOCK/EAGAIN).
 * Only accepts and initializes when PX4 actually connects.
 */
void ConnectionManager::tryAcceptConnection() {

    if (connected || sockfd == -1) {
        return;  // Already connected or no socket
    }

    sockaddr_in cli_addr{};
    socklen_t clilen = sizeof(cli_addr);

    int acceptedSocket = accept(sockfd, (struct sockaddr*)&cli_addr, &clilen);

    if (acceptedSocket < 0) {
        // Check if it's "no connection yet" (not an error) or real error
        if (socketWouldBlock()) {
            // No connection pending, not an error - just return and try next frame
            return;
        }

        std::string msg = "px4xplane: Error on accept: " + socketErrorString() + "\n";
        XPLMDebugString(msg.c_str());
        status = "Accept Error";
        setLastMessage("Error accepting PX4 SITL TCP connection.");
        closeSocket(sockfd);
        extern void updateMenuItems();  // Defined in px4xplane.cpp
        updateMenuItems();
        return;
    }

    if (!setSocketNonBlocking(acceptedSocket, "PX4 client")) {
        closeSocket(acceptedSocket);
        status = "Socket Error";
        setLastMessage("Accepted PX4 SITL connection, but failed to configure socket.");
        closeSocket(sockfd);
        extern void updateMenuItems();  // Defined in px4xplane.cpp
        updateMenuItems();
        return;
    }

    setTcpNoDelay(acceptedSocket);
    setNoSigPipeIfSupported(acceptedSocket);

    newsockfd = acceptedSocket;
    peerEndpoint = endpointString(cli_addr);

    // The connected TCP stream is now established. Close the listening socket
    // so port ownership is unambiguous until the user disconnects/reconnects.
    closeSocket(sockfd);

    // Successfully connected!
    std::string connectedMsg = "px4xplane: PX4 SITL connected successfully from " + peerEndpoint + "\n";
    XPLMDebugString(connectedMsg.c_str());
    connected = true;

    // UX FIX (January 2025): Update status and notify user
    status = "Connected";
    setLastMessage("PX4 SITL connected from " + peerEndpoint);
    XPLMSpeakString("PX4 connected");  // Audio feedback

    // CRITICAL: Update menu to show "Disconnect from SITL"
    extern void updateMenuItems();  // Defined in px4xplane.cpp
    updateMenuItems();

    DataRefManager::enableOverride();

     DataRefManager::initializeMagneticField();
     XPLMDebugString("px4xplane: Init Magnetic Done.\n");

     // Initialize magnetic field with current aircraft position
     GeodeticPosition initialPosition = {
         DataRefManager::getFloat("sim/flightmodel/position/latitude"),
         DataRefManager::getFloat("sim/flightmodel/position/longitude"),
         DataRefManager::getFloat("sim/flightmodel/position/elevation")
     };

     DataRefManager::updateEarthMagneticFieldNED(initialPosition);
     DataRefManager::lastPosition = initialPosition;
     XPLMDebugString("px4xplane: Magnetic field initialized at current position.\n");

     // Load motor mappings from config.ini
     ConfigManager::loadConfiguration();
     XPLMDebugString("px4xplane: Motor mappings loaded from config.ini.\n");

     // Debug: Log loaded configuration
     std::string debugMsg = "Config Name: " + ConfigManager::getAirframeDisplayName(ConfigManager::getConfigName()) +
         " (" + ConfigManager::getConfigName() + ")\n";
     XPLMDebugString(debugMsg.c_str());

}


//not working now ... cannot read ini file...
std::map<int, int> ConnectionManager::loadMotorMappings(const std::string& filename) {
    std::map<int, int> motorMappings;
    std::ifstream file(filename);

    if (!file) {
        XPLMDebugString("Error: Unable to open file ");
        XPLMDebugString(filename.c_str());
        XPLMDebugString("\n");
        return motorMappings;
    }

    std::string line;
    while (std::getline(file, line)) {
        // Ignore comments and empty lines
        if (line.empty() || line[0] == '#') continue;

        std::istringstream iss(line);
        std::string key, equals, value;

        if (!(iss >> key >> equals >> value) || equals != "=") {
            XPLMDebugString("Warning: Ignoring malformed line: ");
            XPLMDebugString(line.c_str());
            XPLMDebugString("\n");
            continue;
        }

        // Check if the key is a PX4 motor
        if (key.substr(0, 4) == "PX4_") {
            int px4Motor = std::stoi(key.substr(4));
            int xplaneMotor = std::stoi(value);
            motorMappings[px4Motor] = xplaneMotor;
            XPLMDebugString("Loaded mapping: PX4 motor ");
            XPLMDebugString(std::to_string(px4Motor).c_str());
            XPLMDebugString(" -> X-Plane motor ");
            XPLMDebugString(std::to_string(xplaneMotor).c_str());
            XPLMDebugString("\n");
        }
    }

    if (motorMappings.empty()) {
        XPLMDebugString("Warning: No motor mappings loaded from ");
        XPLMDebugString(filename.c_str());
        XPLMDebugString("\n");
    }

    return motorMappings;
}




void ConnectionManager::disconnect() {
    const bool wasConnected = connected;
    const bool hadSocket = (sockfd != -1 || newsockfd != -1);
    if (!wasConnected && !hadSocket) {
        return;
    }

    // CRITICAL FIX (January 2025): Reset state BEFORE closing sockets
    // Order matters:
    //   1. Zero actuator values (prevents ghost commands)
    //   2. Clear MAVLink command history
    //   3. Disable override flags
    //   4. Close sockets

    if (wasConnected) {
        DataRefManager::resetActuatorValues();  // Zero all throttle/control surface datarefs
        MAVLinkManager::reset();                 // Clear actuator command history
        DataRefManager::disableOverride();       // Disable override flags
    }

    closeSocket(sockfd);
    sockfd = -1;
    closeSocket(newsockfd); // Close the newsockfd
    newsockfd = -1;
    peerEndpoint.clear();

    connected = false;
    status = "Disconnected";
    setLastMessage(wasConnected ? "Disconnected from PX4 SITL." : "PX4 connection wait cancelled.");
    XPLMDebugString(wasConnected ? "px4xplane: Disconnected from SITL\n" : "px4xplane: PX4 connection wait cancelled\n");
    XPLMDebugString("px4xplane: Socket closed\n");

    // UX FIX (January 2025): Update menu and notify user
    if (wasConnected) {
        XPLMSpeakString("Disconnected");  // Audio feedback
    }
    extern void updateMenuItems();  // Defined in px4xplane.cpp
    updateMenuItems();  // Change menu back to "Connect to SITL"
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
    //std::string logMessage = "Sending MAVLink packet: ";
    /*for (int i = 0; i < len; i++) {
        logMessage += std::to_string(buffer[i]) + " ";
    }*/
    //XPLMDebugString(logMessage.c_str());

    int totalBytesSent = 0;
    while (totalBytesSent < len) {
        int bytesSent = send(newsockfd, reinterpret_cast<const char*>(buffer) + totalBytesSent,
                             len - totalBytesSent, sendNoSignalFlags());

        if (bytesSent < 0) {
            if (socketWouldBlock()) {
                if (ConfigManager::debug_verbose_logging) {
                    XPLMDebugString("px4xplane: TCP send would block; dropping current MAVLink packet\n");
                }
                return;
            }

            std::string msg = "px4xplane: Error sending data to PX4: " + socketErrorString() + "\n";
            XPLMDebugString(msg.c_str());
            setLastMessage("PX4 socket send failed; disconnected.");
            disconnect();
            return;
        }
        else if (bytesSent == 0) {
            // The peer has closed the connection.
            XPLMDebugString("px4xplane: PX4 socket closed during send\n");
            setLastMessage("PX4 closed the socket; disconnected.");
            disconnect();
            return;
        }

        totalBytesSent += bytesSent;
    }
}


void ConnectionManager::receiveData() {
    if (!connected) return;

    constexpr int MAX_RECV_PASSES_PER_FRAME = 16;
    constexpr int RECV_BUFFER_SIZE = 512;
    uint8_t buffer[RECV_BUFFER_SIZE];

    int passes = 0;
    for (; passes < MAX_RECV_PASSES_PER_FRAME; ++passes) {
        // Set up the read set and timeout for select
        fd_set readSet;
        FD_ZERO(&readSet);
        FD_SET(newsockfd, &readSet);
        struct timeval timeout;
        timeout.tv_sec = 0; // Zero seconds
        timeout.tv_usec = 0; // Zero microseconds

        // Use select to check if there is data available to read
        int result = select(newsockfd + 1, &readSet, NULL, NULL, &timeout);
        if (result < 0) {
            if (socketInterrupted()) {
                continue;
            }
            std::string msg = "px4xplane: Error in select: " + socketErrorString() + "\n";
            XPLMDebugString(msg.c_str());
            setLastMessage("PX4 socket select failed; disconnected.");
            disconnect();
            return;
        }
        if (result == 0 || !FD_ISSET(newsockfd, &readSet)) {
            break;
        }

        int bytesReceived = recv(newsockfd, reinterpret_cast<char*>(buffer), sizeof(buffer), 0);
        if (bytesReceived < 0) {
            if (socketWouldBlock()) {
                break;
            }
            if (socketInterrupted()) {
                continue;
            }
            std::string msg = "px4xplane: Error receiving data from PX4: " + socketErrorString() + "\n";
            XPLMDebugString(msg.c_str());
            setLastMessage("PX4 socket receive failed; disconnected.");
            disconnect();
            return;
        }
        else if (bytesReceived == 0) {
            XPLMDebugString("px4xplane: PX4 closed the MAVLink socket\n");
            disconnect();
            return;
        }
        else if (bytesReceived > 0) {
            setLastMessage("Receiving from PX4!"); // Store the received message
            //XPLMDebugString("px4xplane: Data received: ");
            //XPLMDebugString(reinterpret_cast<char*>(buffer)); // Write the received message to the X-Plane log

            // Call MAVLinkManager::receiveHILActuatorControls() function here
            MAVLinkManager::receiveHILActuatorControls(buffer, bytesReceived);
        }
    }

    if (passes == MAX_RECV_PASSES_PER_FRAME && ConfigManager::debug_verbose_logging) {
        XPLMDebugString("px4xplane: MAVLink receive budget exhausted; remaining data will be processed next frame\n");
    }
}




bool ConnectionManager::isConnected() {
    return connected;
}

/**
 * @brief Check if socket is listening but not yet connected.
 *
 * Returns true when server socket is set up and waiting for PX4 to connect.
 * Used by flight loop to know when to poll for incoming connections.
 *
 * @return true if waiting for connection, false otherwise
 */
bool ConnectionManager::isWaitingForConnection() {
    return (sockfd != -1 && !connected);
}

bool ConnectionManager::isConnectionActiveOrWaiting() {
    return connected || isWaitingForConnection();
}

const std::string& ConnectionManager::getStatus() {
    return status;
}

const std::string& ConnectionManager::getPeerEndpoint() {
    return peerEndpoint;
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
