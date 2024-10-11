import numpy as np
import socket
import struct


######################################################
# 把从软件中接收到的原始数据进行进一步提取
def split_and_convert_to_float(data):
    # 将接收到的数据按空格分割成字符串列表
    data_list = data.split()
    # 将除最后一个字符串之外的字符串转换为浮点数
    float_list = [float(item) if index != len(data_list) - 1 else item for index, item in enumerate(data_list)]

    # 这里提取了第103到114的数（右拇指指根，右拇指指中），127到132（右 食 指 指 根），151到156（右 中 指 指 根），175到180（右 无 名 指 指 根） ，199到204（右 小 指 指 根）
    # 241到252（左 拇 指 指 根，左 拇 指 指 中），265到270（左 食 指 指 根），289到294（左 中 指 指 根），313到318（左 无 名 指 指 根），337到342（左 小 指 指 根）
    # 原始数据的位姿是位移xyz欧拉角zyx
    # return (float_list[102:114] + float_list[126:132] + float_list[150:156]+ float_list[174:180]+ float_list[198:204]
    #         + float_list[240:252]+ float_list[264:270]+ float_list[288:294]+ float_list[312:318]+ float_list[336:342])
    # 实际上只需要下面手部这些数据：大拇指指根x轴旋量，大拇指指中z轴旋量，食 指 指 根z轴旋量， 中 指 指 根z轴旋量，无 名 指 指 根z轴旋量， 小 指 指 根z轴旋量
    return float_list[245:246] + float_list[249:250] + float_list[267:268] + float_list[291:292] + float_list[315:316] + float_list[339:340] + float_list[107:108] + float_list[111:112] + float_list[129:130] + float_list[153:154] + float_list[177:178] + float_list[201:202]


######################################################


def udp_client():
    # 定义服务器地址和端口
    # server_host = '192.168.112.94'
    # server_port = 5005

    # 服务器地址和端口
    server_host = '192.168.112.95'
    server_port = 7003

    # 创建UDP套接字
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # 绑定客户端地址
    client_host = '192.168.112.95'
    client_port = 7002

    # 客户端地址和端口
    # client_host = '127.0.0.1'
    # client_port = 7002

    client_socket.bind((client_host, client_port))

    print("等待接收消息...")

    try:
        while True:
            # 接收数据
            data, server_address = client_socket.recvfrom(3000)

            # 打印接收到的消息
            print("原始数据", data.decode())
            # 去掉接收到的原始数据的前8个无用字符
            trimmed_data = data.decode()[8:]
            print("截取的有效位姿数据", trimmed_data)

            # 输出split_and_convert_to_float函数的结果在一个数组
            result_array = split_and_convert_to_float(trimmed_data)
            # print("选取要使用的关节位姿", result_array)

            # 如果result_array[0]和result_array[6]是负数，则分别自加180
            if result_array[0] < 0:
                result_array[0] += 180
            if result_array[0] > 140:
                result_array[0] = 0

            if result_array[6] < 0:
                result_array[6] += 180
            if result_array[6] > 140:
                result_array[6] = 0

            # 其他10个数，先自加-180，若结果大于140则置0
            for i in range(1, 6):
                result_array[i] -= 180
                if result_array[i] > 140:
                    result_array[i] = 0
                result_array[i] = -result_array[i]
                if result_array[i] > 140:
                    result_array[i] = 0
            result_array[1] *= 1.4

            result_array[7] -= 180
            if result_array[7] > 140:
                result_array[7] = 0
            result_array[7] += 180
            if result_array[7] > 140:
                result_array[7] = 0
            result_array[7] *= 1.4

            for i in range(8, 12):
                result_array[i] -= 180
                if result_array[i] > 140:
                    result_array[i] = 0
                result_array[i] += 180
                if result_array[i] > 140:
                    result_array[i] = 0

            for i in range(0, 12):
                result_array[i] = (result_array[i]/180)*3.14
                if result_array[i] > 1.57:
                    result_array[i] = 1.57
                if result_array[i] < 0:
                    result_array[i] = 0

            print(result_array)

            # 转发消息到服务器
            packed_data = b''.join([struct.pack('>f', num) for num in result_array])
            client_socket.sendto(packed_data, (server_host, server_port))
            print("消息已转发至服务器:", server_host, server_port)

    except KeyboardInterrupt:
        print("客户端已关闭。")
        client_socket.close()


if __name__ == "__main__":
    udp_client()
