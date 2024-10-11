import socket
import struct

def main():

    # 服务器地址和端口
    # server_host = '127.0.0.1'
    # server_port = 7003

    # 定义服务器地址和端口
    server_host = '192.168.112.95'
    server_port = 7003

    # 创建UDP套接字
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # 绑定服务器地址和端口
    server_socket.bind((server_host, server_port))

    print("服务器已启动，等待接收消息...")

    try:
        while True:
            # 接收数据
            packed_data, client_address = server_socket.recvfrom(5000)

            # 解析接收到的二进制数据
            try:
                # 解析为单精度浮点数列表
                floats = struct.unpack(f'>{len(packed_data) // 4}f', packed_data)
                print(f"从客户端 {client_address} 收到消息: {floats}")
            except struct.error:
                print("无法解析接收到的数据")

    except KeyboardInterrupt:
        print("服务器已关闭。")
        server_socket.close()

if __name__ == "__main__":
    main()
