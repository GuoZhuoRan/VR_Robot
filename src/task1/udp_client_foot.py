import numpy as np
import socket
import struct
import serial
import time

# 串口配置
port = 'COM3'  # 串口名称，例如COM3、COM4等
baudrate = 9600  # 波特率
timeout = 1  # 超时设置

# 创建串口连接
ser = serial.Serial(port, baudrate, timeout=timeout)
######################################################
# 把从软件中接收到的原始数据进行进一步提取
def split_and_convert_to_float(data):
    # 将去掉前8个字符的数据按空格分割成字符串列表
    data_list = data.split()
    # 将除最后一个字符串之外的字符串转换为浮点数
    float_list = [float(item) if index != len(data_list) - 1 else item for index, item in enumerate(data_list)]

    # 这里提取了第1到42个数，分别是 臀部1   右大腿2  右小腿 3  右脚4    左大腿5   左小腿6   左脚7的原始位姿
    # 原始数据的位姿是位移xyz欧拉角zyx
    return float_list[0:42]


######################################################


######################################################
# 位姿转齐次变换矩阵
def generate_homogeneous_matrices(data):
    matrices = []
    extracted_numbers = split_and_convert_to_float(data)

    for i in range(0, len(extracted_numbers), 6):
        translation = extracted_numbers[i:i + 3]
        rotation = extracted_numbers[i + 3:i + 6]

        # 将角度转换为弧度
        rotation = np.radians(rotation)

        # 构造zyx欧拉角的旋转矩阵
        r_x = np.array([[1, 0, 0],
                        [0, np.cos(rotation[2]), -np.sin(rotation[2])],
                        [0, np.sin(rotation[2]), np.cos(rotation[2])]])

        r_y = np.array([[np.cos(rotation[1]), 0, np.sin(rotation[1])],
                        [0, 1, 0],
                        [-np.sin(rotation[1]), 0, np.cos(rotation[1])]])

        r_z = np.array([[np.cos(rotation[0]), -np.sin(rotation[0]), 0],
                        [np.sin(rotation[0]), np.cos(rotation[0]), 0],
                        [0, 0, 1]])

        # 合并旋转矩阵，按照 zyx旋转顺序
        r = np.dot(r_z, np.dot(r_y, r_x))

        # 构造平移矩阵
        t = np.eye(4)
        t[:3, 3] = translation

        # 构造齐次变换矩阵
        homogeneous_matrix = np.eye(4)
        homogeneous_matrix[:3, :3] = r
        homogeneous_matrix[:3, 3] = translation

        matrices.append(homogeneous_matrix)

    return matrices


######################################################


######################################################
# 齐次矩阵转位姿
def convert_to_translation_and_euler(matrix):
    # 从矩阵中提取平移向量
    translation = matrix[:3, 3]

    # 提取旋转部分
    rotation_matrix = matrix[:3, :3]

    # 将旋转部分转换为欧拉角
    sy = np.sqrt(rotation_matrix[0, 0] * rotation_matrix[0, 0] + rotation_matrix[1, 0] * rotation_matrix[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
        y = np.arctan2(-rotation_matrix[2, 0], sy)
        z = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    else:
        x = np.arctan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
        y = np.arctan2(-rotation_matrix[2, 0], sy)
        z = 0

    euler_angles = np.array([z, y, x])

    # 将弧度转换为角度
    # 角度顺序为zyx
    euler_angles_degrees = np.degrees(euler_angles)

    return translation, euler_angles_degrees


######################################################


######################################################
# 内旋转外旋（移动坐标系转固定坐标系）
def matrix3d_to_euler_angles_zyx(m3dr):
    beta_y = np.arctan2(m3dr[0, 2], np.sqrt(m3dr[0, 0] * m3dr[0, 0] + m3dr[0, 1] * m3dr[0, 1]))
    alpha_z = np.arctan2(-m3dr[0, 1] / np.cos(beta_y), m3dr[0, 0] / np.cos(beta_y))
    gamma_x = np.arctan2(-m3dr[1, 2] / np.cos(beta_y), m3dr[2, 2] / np.cos(beta_y))

    if np.abs(beta_y - np.pi / 2) < 10e-4:
        gamma_x = 0
        alpha_z = np.arctan2(m3dr[1, 0], m3dr[1, 1])

    if np.abs(beta_y + np.pi / 2) < 10e-4:
        gamma_x = 0
        alpha_z = np.arctan2(m3dr[1, 0], m3dr[1, 1])

    gamma_x = (gamma_x + np.pi) % (2 * np.pi) - np.pi
    beta_y = (beta_y + np.pi) % (2 * np.pi) - np.pi
    alpha_z = (alpha_z + np.pi) % (2 * np.pi) - np.pi

    return np.array([alpha_z, beta_y, gamma_x])


######################################################

def main():
    # 客户端地址和端口
    client_host = '192.168.112.95'
    client_port = 7002

    # 服务器地址和端口
    server_host = '192.168.112.158'
    server_port = 7003
    # lastyouwem = 0.0
    # 创建UDP套接字
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # 绑定客户端地址和端口
    client_socket.bind((client_host, client_port))

    print("等待接收消息...")

    try:
        count = 0
        matrix_count = 0  # 新增矩阵计数器
        a = 0
        b = 0
        left_foot_1 = [0, 0, 0]
        last_you = 0.0
        youwenmmmm = 0.0
        while True:
            # 接收数据
            data, server_address = client_socket.recvfrom(5000)
            # print("原始数据", data.decode())
            # 去掉接收到的原始数据的前8个无用字符
            trimmed_data = data.decode()[8:]
            # print("截取的有效位姿数据", trimmed_data)

            # 输出split_and_convert_to_float函数的结果在一个数组
            result_array = split_and_convert_to_float(trimmed_data)
            # print("选取要使用的关节位姿", result_array)

            # 将截取的有效位姿数据转换为齐次变换矩阵
            matrices = generate_homogeneous_matrices(trimmed_data)

            # 打印转换后的7个关节的齐次变换矩阵
            # （因为这里我在上面选取了7个关节数据，所以每打印7个齐次矩阵重新对矩阵编号排序）
            for matrix in matrices:
                count += 1
                matrix_count += 1  # 每收到一个矩阵，计数器加一

                # 打印单个矩阵
                # matrix_count是矩阵序号，从1开始
                # print(f"矩阵{matrix_count}:")
                # print(matrix)

                # 如果是第7个矩阵，即我选取的7个关节位姿已全部转换完，计算并打印下面结果
                if count % 7 == 0:

                    ###################################################################
                    # 下面是左脚相对臀部的软件中的坐标系的旋转矩阵
                    # 虚拟坐标系就是软件中默认的坐标系
                    matrix2_1 = np.dot(np.dot(np.dot(matrices[0], matrices[4]), matrices[5]), matrices[6])
                    # print("左脚", matrix2_1)
                    translation2_1, euler_angles2_1 = convert_to_translation_and_euler(matrix2_1)
                    # print(f"左脚虚拟坐标系位姿 - 位移: {translation2_1}, 欧拉角: {euler_angles2_1}")
                    euler_angles2_waixuan_hudu = matrix3d_to_euler_angles_zyx(matrix2_1)
                    # 弧度转角度
                    euler_angles2_waixuan_jiaodu = np.degrees(euler_angles2_waixuan_hudu)
                    # print("左脚虚拟外旋欧拉角", euler_angles2_waixuan_jiaodu)

####################################################################
                    # 检查串口是否打开
                    # lastyouwem = 0.0
                    # youwenmmmm = 0.0
                    if ser.isOpen():
                        # print(f"串口{port}已打开")

                        try:
                            # while True:
                            hex_data_list = []  # 初始化一个空列表用于存储十六进制数据
                            while len(hex_data_list) < 4 and ser.in_waiting > 0:
                                data = ser.read(size=1)  # 读取一个字节
                                hex_data_list.append(hex(data[0]))  # 将字节转换为十六进制并添加到列表中
                            if len(hex_data_list) == 4:  # 如果列表中有四个字节
                                # 检查第一位是否为0xaa
                                if hex_data_list[0] == '0xaa':
                                    # 计算校验和（只考虑最低8位）
                                    checksum = sum(int(hex_data_list[i], 16) for i in range(3)) % 0x100
                                    # 检查第四位是否为校验和（只考虑最低8位）
                                    if checksum == int(hex_data_list[3], 16):
                                        # print("true")  # 校验通过，打印true
                                        # print(hex_data_list)
                                        high_byte = int(hex_data_list[1], 16)
                                        low_byte = int(hex_data_list[2], 16)
                                        decimal_value = (high_byte << 8) | low_byte
                                        # print(decimal_value / 292)  # 打印合并后的十进制数
                                        youwenmmmm = decimal_value / 292
                                    else:
                                        print("false")
                                        print(hex_data_list)
                                        youwenmmmm = last_you
                                # print(hex_data_list)
                                ser.flushInput()  # 清空串口接收缓冲区

                        except KeyboardInterrupt:
                            print("中断串口读取")

                    # finally:
                    #     ser.close()  # 关闭串口
                    #     print(f"串口{port}已关闭")
                    # lastyouwem = youwenmmmm
                    # print(youwenmmmm)
                    #######################################################################################
                    # 右脚(油门踏板)x轴角度k1,启动角度k0=30
                    # k0 = 30
                    # k1 = euler_angles1_waixuan_jiaodu[2]
                    # # k是把踏板深度映射成0到1,k=1就是右脚(油门踏板)x轴角度为0即踩到底
                    # k = ((k0 - k1) + 30) / 30
                    # 左脚控制需要的数据
                    left_foot_2 = [translation2_1[2], translation2_1[0], euler_angles2_waixuan_jiaodu[1]]
                    # print(youwenmmmm)
                    if youwenmmmm > 0.01:
                        a += 1
                        b +=1
                    else:
                        a = 0
                        b +=1
                        # print("11111111111111")
                    if a == 1:
                        left_foot_1 = [translation2_1[2], translation2_1[0], euler_angles2_waixuan_jiaodu[1]]
                    if youwenmmmm > 0.01:
                        vx = youwenmmmm*(left_foot_2[1] - left_foot_1[1])/200
                        vy = youwenmmmm*(left_foot_2[0] - left_foot_1[0])/200
                        wz = youwenmmmm*(left_foot_2[2] - left_foot_1[2])/200
                        # print(left_foot_2[2])
                        # print(left_foot_1[2])
                    else:
                        vx = vy = wz = 0
                    if a > 5000:
                        a = 500
                    if b > 4999:
                        b = 500
                    print("a:",a)
                    # 将右左脚发送到服务器
                    message = [
                        vy, wz
                    ]
                    # print(lastyouwem)
                    print(message)
                    if b%5==0:
                        packed_data = b''.join([struct.pack('>f', num) for num in message])
                        sent = client_socket.sendto(packed_data, (server_host, server_port))
                    print("消息已发送至服务器。")

                # 每行输出7个矩阵后，重置计数器
                if count % 7 == 0:
                    count = 0

                # 每输出一个矩阵后，重置矩阵计数器
                if matrix_count % 7 == 0:
                    matrix_count = 0
            last_you = youwenmmmm
            # time.sleep(0.075)
    except KeyboardInterrupt:
        # print("客户端已关闭。")
        client_socket.close()


if __name__ == "__main__":
    main()
