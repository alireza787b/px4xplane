import socket

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('0.0.0.0', 4560))  # Listen on port 4560
server_socket.listen(1)

print("Server is listening on port 4560...")

client_socket, addr = server_socket.accept()
print(f"Connection accepted from {addr}")

# To keep the connection alive, you may want to read or send data here
# For example, to receive data from the client:
data = client_socket.recv(1024)
print(f"Received data: {data.decode()}")

# Don't forget to close the sockets when done
client_socket.close()
server_socket.close()
