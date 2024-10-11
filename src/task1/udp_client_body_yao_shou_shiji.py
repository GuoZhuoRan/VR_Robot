import numpy as np
import socket
import struct
######################################################
# 提取手指的原始数据
def split_and_convert_to_float_hand(data):
    # 将接收到的数据按空格分割成字符串列表
    data_list = data.split()
    # 将除最后一个字符串之外的字符串转换为浮点数
    float_list = [float(item) if index != len(data_list) - 1 else item for index, item in enumerate(data_list)]

    # 这里提取了第103到114的数（右拇指指根，右拇指指中），127到132（右 食 指 指 根），151到156（右 中 指 指 根），175到180（右 无 名 指 指 根） ，199到204（右 小 指 指 根）
    # 241到252（左 拇 指 指 根，左 拇 指 指 中），265到270（左 食 指 指 根），289到294（左 中 指 指 根），313到318（左 无 名 指 指 根），337到342（左 小 指 指 根）
    # 原始数据的位姿是位移xyz欧拉角zyx
    # 实际上只需要下面手部这些数据：大拇指指根x轴旋量，大拇指指中z轴旋量，食 指 指 根z轴旋量， 中 指 指 根z轴旋量，无 名 指 指 根z轴旋量， 小 指 指 根z轴旋量
    return (float_list[245:246] + float_list[249:250] + float_list[267:268] + float_list[291:292] + float_list[315:316]
            + float_list[339:340] + float_list[107:108] + float_list[111:112] + float_list[129:130] + float_list[153:154]
            + float_list[177:178] + float_list[201:202])
######################################################


# 把从软件中接收到的原始数据进行进一步提取
def split_and_convert_to_float(data):
    # 将去掉前8个字符的数据按空格分割成字符串列表
    data_list = data.split()
    # 将除最后一个字符串之外的字符串转换为浮点数
    float_list = [float(item) if index != len(data_list) - 1 else item for index, item in enumerate(data_list)]

    # 这里提取了第55到102和216到240这72个数，分别是脊椎上，颈部下，颈部上，头，右肩，右大臂，右前臂，右手，左肩，左大臂，左前臂，左手的原始位姿
    # float_list[42:54]是脊椎下和中，float_list[0:6]是屁股
    #第三行是手指
    # 原始数据的位姿是位移xyz欧拉角zyx
    return (float_list[54:60] + float_list[60:102] + float_list[216:240]
            +float_list[0:6] + float_list[42:54])

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
    # # 客户端地址和端口
    client_host = '192.168.112.95'
    client_port = 7002

    # 客户端地址和端口
    # client_host = '127.0.0.1'
    # client_port = 7002

    # 服务器1地址和端口
    server_host = '192.168.112.72'
    server_port = 3378

    # 服务器1地址和端口
    # server_host = '127.0.0.1'
    # server_port = 7003

    # 服务器2地址和端口
    server2_host = '192.168.112.72'
    server2_port = 3379

    # 创建UDP套接字
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # 绑定客户端地址和端口
    client_socket.bind((client_host, client_port))

    print("等待接收消息...")


    try:
        count = 0
        matrix_count = 0  # 新增矩阵计数器
        while True:
            # 接收数据
            data, server_address = client_socket.recvfrom(5000)
            # print("原始数据", data.decode())
            # 去掉接收到的原始数据的前8个无用字符
            trimmed_data = data.decode()[8:]
            # print("截取的有效位姿数据", trimmed_data)

            # 输出split_and_convert_to_float函数的结果在一个数组
            result_array1 = split_and_convert_to_float(trimmed_data)
            # print("选取要使用的关节位姿", result_array1)

            # 将截取的有效位姿数据转换为齐次变换矩阵
            matrices = generate_homogeneous_matrices(trimmed_data)

            # 打印转换后的15个关节的齐次变换矩阵
            # （因为这里我在上面选取了15个关节数据，所以每打印15个齐次矩阵重新对矩阵编号排序）
            for matrix in matrices:
                count += 1
                matrix_count += 1  # 每收到一个矩阵，计数器加一

                # 打印单个矩阵
                # matrix_count是矩阵序号，从1开始
                # print(f"矩阵{matrix_count}:")
                # print(matrix)

                # 如果是第15个矩阵，即我选取的15个关节位姿已全部转换完，计算并打印下面结果（）
                if count % 15 == 0:

####################################################################

                    # 头部相对于脊椎上部的变换矩阵，
                    matrix1_1 = np.dot(matrices[1], np.dot(matrices[2], matrices[3]))
                    # print("头部虚拟坐标齐次变换矩阵", matrix1_1)
                    translation1_1, euler_angles1_1 = convert_to_translation_and_euler(matrix1_1)
                    # print(f"右手虚拟坐标系位姿 - 位移: {translation1_1}, 欧拉角: {euler_angles1_1}")

                    # 下面是头部相对脊椎上的实机中的坐标系的旋转矩阵


                    # 定义矩阵 E，即实机脊椎上坐标系相对于虚拟脊椎上坐标系的旋转矩阵
                    E = np.array([[0, 0, 1, 0],
                                  [1, 0, 0, 0],
                                  [0, 1, 0, 0],
                                  [0, 0, 0, 1]])

                    # 定义矩阵 F，即头部实机坐标系相对于头部虚拟坐标系的变换矩阵
                    F = np.array([[0, 0, 1, 0],
                                  [1, 0, 0, 0],
                                  [0, 1, 0, 0],
                                  [0, 0, 0, 1]])
                    matrix1_2 = np.dot(np.linalg.inv(E), np.dot(matrices[1], np.dot(matrices[2], np.dot(matrices[3], F))))
                    # 求齐次变换矩阵的欧拉角外旋欧拉角（默认内旋）
                    euler_angles1_waixuan_hudu = matrix3d_to_euler_angles_zyx(matrix1_2)
                    # 弧度转角度
                    euler_angles1_waixuan_jiaodu = np.degrees(euler_angles1_waixuan_hudu)
                    # print("头部外旋欧拉角", euler_angles1_waixuan_jiaodu)
                    translation1_2, euler_angles1_neixuan_jiaodu = convert_to_translation_and_euler(matrix1_2)
                    # print("头内旋欧拉角", euler_angles1_neixuan_jiaodu)
                    # print("头实机坐标齐次变换矩阵", matrix1_2)
                    # print(f"头实机坐标系外旋位姿 - 外旋欧拉角: {euler_angles1_waixuan_jiaodu}")
###################################################################



                    # 脊椎上相对于世界坐标系的变换矩阵，
                    #matrix0_1 = np.dot(matrices[12],np.dot(matrices[13], np.dot(matrices[14], matrices[0])))
                    matrix0_1 = np.dot(matrices[13], np.dot(matrices[14], matrices[0]))
                    # print("脊椎上虚拟坐标齐次变换矩阵", matrix0_1)
                    translation0_1, euler_angles0_1 = convert_to_translation_and_euler(matrix0_1)
                    # print(f"脊椎上虚拟坐标系位姿 - 欧拉角: {euler_angles0_1}")



                    # 下面是脊椎上相对世界坐标系的实机中的坐标系的旋转矩阵
                    # 定义矩阵 G，即实机脊椎上坐标系相对于虚拟脊椎上坐标系的旋转矩阵
                    G = np.array([[0, 0, 1, 0],
                                  [1, 0, 0, 0],
                                  [0, 1, 0, 0],
                                  [0, 0, 0, 1]])

                    # 定义矩阵 H，即世界坐标系实机坐标系相对于世界坐标系虚拟坐标系的变换矩阵
                    H = np.array([[0, 0, 1, 0],
                                  [1, 0, 0, 0],
                                  [0, 1, 0, 0],
                                  [0, 0, 0, 1]])
                    matrix0_2 = np.dot(np.linalg.inv(H), np.dot(matrices[13], np.dot(matrices[14], np.dot(matrices[0], G))))
                    # 求齐次变换矩阵的欧拉角外旋欧拉角（默认内旋）
                    euler_angles0_waixuan_hudu = matrix3d_to_euler_angles_zyx(matrix0_2)
                    # 弧度转角度
                    euler_angles0_waixuan_jiaodu = np.degrees(euler_angles0_waixuan_hudu)
                    # print("脊椎上实机外旋欧拉角", euler_angles0_waixuan_jiaodu)
                    translation0_2, euler_angles0_neixuan_jiaodu = convert_to_translation_and_euler(matrix0_2)
                    # print("脊椎上实机内旋欧拉角", euler_angles0_neixuan_jiaodu)
                    # print("脊椎上实机坐标齐次变换矩阵", matrix0_2)
                    # print(f"脊椎上实机坐标系外旋位姿 - 外旋欧拉角: {euler_angles0_waixuan_jiaodu}")

###################################################################



                    # 下面是右手相对脊椎上的软件中的坐标系的旋转矩阵
                    # 虚拟坐标系就是软件中默认的坐标系
                    matrix2_1 = np.dot(
                        np.dot(np.dot(np.dot(np.linalg.inv(matrices[0]), matrices[4]), matrices[5]), matrices[6]),
                        matrices[7])
                    # print("右手虚拟坐标齐次变换矩阵", matrix2_1)
                    translation2_1, euler_angles2_1 = convert_to_translation_and_euler(matrix2_1)
                    # print(f"右手虚拟坐标系位姿 - 位移: {translation2_1}, 欧拉角: {euler_angles2_1}")

                    # 下面是右手相对脊椎上的实机中的坐标系的旋转矩阵
                    # 定义矩阵 C，即实机脊椎上坐标系相对于虚拟脊椎上坐标系的旋转矩阵
                    C = np.array([[0, 0, 1, 0],
                                  [1, 0, 0, 0],
                                  [0, 1, 0, 0],
                                  [0, 0, 0, 1]])

                    # 定义矩阵 D，即右手实机坐标系相对于右手虚拟坐标系的变换矩阵
                    D = np.array([[-1, 0, 0, 0],
                                  [0, 0, -1, 0],
                                  [0, -1, 0, 0],
                                  [0, 0, 0, 1]])
                    # 求出实机坐标系下的右手的变化矩阵
                    matrix2_2 = np.dot(np.dot(np.linalg.inv(np.dot(matrices[0], C)), matrices[4]),
                                       np.dot(np.dot(matrices[5], matrices[6]), np.dot(matrices[7], D)))
                    # 求齐次变换矩阵的欧拉角外旋欧拉角（默认内旋）
                    euler_angles2_waixuan_hudu = matrix3d_to_euler_angles_zyx(matrix2_2)
                    # 弧度转角度
                    euler_angles2_waixuan_jiaodu = np.degrees(euler_angles2_waixuan_hudu)
                    # print("右手实机坐标齐次变换矩阵", matrix2_2)
                    # print("右手外旋欧拉角", euler_angles2_waixuan_jiaodu)
                    translation2_2, euler_angles2_neixuan_jiaodu = convert_to_translation_and_euler(matrix2_2)
                    # print("右手内旋欧拉角", euler_angles2_neixuan_jiaodu)
                    # print(f"右手实机坐标系外旋位姿 - 位移: {translation2_2}, 外旋欧拉角: {euler_angles2_waixuan_hudu}")

###################################################################


                    # 下面是左手相对脊椎上的软件中的坐标系的旋转矩阵
                    matrix3_1 = np.dot(
                        np.dot(np.dot(np.dot(np.linalg.inv(matrices[0]), matrices[8]), matrices[9]), matrices[10]),
                        matrices[11])
                    # print("左手虚拟坐标齐次变换矩阵", matrix3_1)
                    translation3_1, euler_angles3_1 = convert_to_translation_and_euler(matrix3_1)
                    # print(f"虚拟坐标系左手位姿 - 位移: {translation3_1}, 欧拉角: {euler_angles3_1}")
                    # 下面是左手相对脊椎上的实际机器人的坐标系的旋转矩阵
                    # 定义矩阵 A,即实机脊椎上坐标系相对于虚拟脊椎上坐标系的旋转矩阵
                    A = np.array([[0, 0, 1, 0],
                                  [1, 0, 0, 0],
                                  [0, 1, 0, 0],
                                  [0, 0, 0, 1]])

                    # 定义矩阵 B，即左手实机坐标系相对于左手虚拟坐标系的变换矩阵
                    B = np.array([[1, 0, 0, 0],
                                  [0, 0, 1, 0],
                                  [0, -1, 0, 0],
                                  [0, 0, 0, 1]])
                    matrix3_2 = np.dot(np.dot(np.linalg.inv(np.dot(matrices[0], A)), matrices[8]),
                                       np.dot(np.dot(matrices[9], matrices[10]), np.dot(matrices[11], B)))
                    # 内旋转外旋
                    euler_angles3_waixuan_hudu = matrix3d_to_euler_angles_zyx(matrix3_2)
                    # 弧度转角度
                    euler_angles3_waixuan_jiaodu = np.degrees(euler_angles3_waixuan_hudu)
                    # print("左手实机坐标齐次变换矩阵", matrix3_2)
                    # print("左手外旋欧拉角", euler_angles3_waixuan_jiaodu)
                    translation3_2, euler_angles3_neixuan_jiaodu = convert_to_translation_and_euler(matrix3_2)
                    # print("左手内旋欧拉角", euler_angles3_neixuan_jiaodu)
                    # print(f"左手实机坐标系左手外旋位姿 - 位移: {translation3_2}, 外旋欧拉角: {euler_angles3_waixuan_hudu}")


                    # # 将左手、右手、头、脊椎上部位姿的‘顺序’发送到服务器
                    # message = [
                    #     euler_angles3_waixuan_hudu[0], euler_angles3_waixuan_hudu[1], euler_angles3_waixuan_hudu[2],
                    #     translation3_2[0] * 10, translation3_2[1] * 10, translation3_2[2] * 10, 0.553,
                    #     euler_angles2_waixuan_hudu[0], euler_angles2_waixuan_hudu[1] * -1, euler_angles2_waixuan_hudu[2] * -1,
                    #     translation2_2[0] * 10, translation2_2[1] * 10, translation2_2[2] * -10,
                    #     0.553,  euler_angles1_waixuan_hudu[0], euler_angles1_waixuan_hudu[2], euler_angles0_waixuan_hudu[0], euler_angles0_waixuan_hudu[2],
                    #     result_array2[0], result_array2[1], result_array2[2], result_array2[3], result_array2[4], result_array2[5], result_array2[6], result_array2[7], result_array2[8], result_array2[9], result_array2[10], result_array2[11]
                    # ]
                    # print(message)
                    #
                    # packed_data = b''.join([struct.pack('>f', num) for num in message])
                    #
                    # # client_socket.sendto(str(message).encode(), (server_host, server_port))
                    # client_socket.sendto(packed_data, (server_host, server_port))
                    # print("身体消息已发送至服务器。")

#########################################################################################

                    # 输出split_and_convert_to_float函数的结果在一个数组
                    result_array2 = split_and_convert_to_float_hand(trimmed_data)
                    # print("选取要使用的关节位姿", result_array2)

                    # 如果result_array2[0]和result_array2[6]是负数，则分别自加180
                    if result_array2[0] < 0:
                        result_array2[0] += 180
                    if result_array2[0] > 140:
                        result_array2[0] = 0

                    if result_array2[6] < 0:
                        result_array2[6] += 180
                    if result_array2[6] > 140:
                        result_array2[6] = 0

                    # 其他10个数，先自加-180，若结果大于140则置0
                    for i in range(1, 6):
                        result_array2[i] -= 180
                        if result_array2[i] > 140:
                            result_array2[i] = 0
                        result_array2[i] = -result_array2[i]
                        if result_array2[i] > 140:
                            result_array2[i] = 0
                    result_array2[1] *= 1.4

                    result_array2[7] -= 180
                    if result_array2[7] > 140:
                        result_array2[7] = 0
                    result_array2[7] += 180
                    if result_array2[7] > 140:
                        result_array2[7] = 0
                    result_array2[7] *= 1.4

                    for i in range(8, 12):
                        result_array2[i] -= 180
                        if result_array2[i] > 140:
                            result_array2[i] = 0
                        result_array2[i] += 180
                        if result_array2[i] > 140:
                            result_array2[i] = 0

                    for i in range(0, 12):
                        result_array2[i] = (result_array2[i] / 180) * 3.14
                        if result_array2[i] > 1.57:
                            result_array2[i] = 1.57
                        if result_array2[i] < 0:
                            result_array2[i] = 0

#########################################################################################

                    message = [
                        translation3_2[0] * 10+75, translation3_2[1] * 10-50, translation3_2[2] * 10+50,
                        euler_angles3_waixuan_jiaodu[2]-30, euler_angles3_waixuan_jiaodu[1], euler_angles3_waixuan_jiaodu[0], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        translation2_2[0] * 10+75, translation2_2[1] * 10-50 , translation2_2[2] * 10-50,
                        euler_angles2_waixuan_jiaodu[2]+30, euler_angles2_waixuan_jiaodu[1],euler_angles2_waixuan_jiaodu[0], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        euler_angles1_waixuan_jiaodu[2],euler_angles1_waixuan_jiaodu[0]+15,
                        euler_angles0_waixuan_jiaodu[2], euler_angles0_waixuan_jiaodu[0],
                    ]
                    # 发送身体数据
                    print(message)
                    packed_data_body = b''.join([struct.pack('>f', num) for num in message])
                    client_socket.sendto(packed_data_body, (server_host, server_port))

                    # 发送手指数据
                    packed_data_hand = b''.join([struct.pack('>f', num) for num in result_array2])
                    client_socket.sendto(packed_data_hand, (server2_host, server2_port))
                    # print("全部消息已发送至服务器。")

                # 每行输出15个矩阵后，重置计数器
                if count % 15 == 0:
                    count = 0

                # 每输出一个矩阵后，重置矩阵计数器
                if matrix_count % 15 == 0:
                    matrix_count = 0

    except KeyboardInterrupt:
        print("客户端已关闭。")
        client_socket.close()


if __name__ == "__main__":
    main()
