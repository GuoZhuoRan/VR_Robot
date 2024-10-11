import socket
import keyboard

# UDP设置
udp_ip = "127.0.0.1"  # 要发送数据的IP地址
udp_port = 5005       # 端口号
message = "111111"    # 默认消息内容

# 创建一个UDP套接字
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def toggle_message():
    global message
    if message == "111111":
        message = "222222"
    else:
        message = "111111"

# 设置键盘监听
keyboard.add_hotkey('h', toggle_message)

try:
    print("UDP发送器正在运行。按'h'键切换消息内容。")
    while True:
        # 发送消息
        print(message)
        sock.sendto(message.encode(), (udp_ip, udp_port))
except KeyboardInterrupt:
    print("程序已退出。")
finally:
    sock.close()
