import socket

host = '0.0.0.0'
port = 4560

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((host, port))
s.listen(1)

print(f"Server is listening on {host}:{port}")

try:
    conn, addr = s.accept()
    print(f"Connection from {addr}")

    conn.sendall(b'Welcome to the test server!\n')

    data = conn.recv(1024)
    print(f"Received: {data.hex()}")  # Print received data in hexadecimal format

finally:
    conn.close()
    print("Connection closed")
