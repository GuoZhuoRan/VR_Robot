import socket

def start_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('localhost', 8001))
    server_socket.listen(1)
    print("Server 2 is running on port 8001...")
    while True:
        client_socket, addr = server_socket.accept()
        print(f"Connection from {addr}")
        client_socket.send(b"Hello from server 2!")
        client_socket.close()

if __name__ == "__main__":
    # start_server()
    print('2222222222222222222222222222')
