import socket
import struct
from bixingjiao import *
import numpy as np
######################################################
def juzhen_to_shuzu(homogeneous_matrix):
    rotation_matrix = homogeneous_matrix[:3, :3]
    rotation_array = rotation_matrix.flatten()
    return(rotation_array)



def split_and_convert_to_float(data):

    # 将接收到的数据按空格分割成字符串列表
    data_list = data.split()
    # 将除最后一个字符串之外的字符串转换为浮点数
    float_list = [float(item) if index != len(data_list) - 1 else item for index, item in enumerate(data_list)]

   # 脊椎下脊椎中43-54 右肩右大臂右小臂右手79-102 左肩左大臂左小臂左手217-240 屁股左右大腿左右小腿左右脚1-42
    return (float_list[42:54]+float_list[78:102]+float_list[216:240]+float_list[0:42])

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


# 齐次矩阵转位姿
def convert_to_translation_and_euler(matrix):
    # 从矩阵中提取平移向量
    # translation = np.zero[3]
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
    client_host = '192.168.112.94'
    client_port = 7003

    # 客户端地址和端口
    # client_host = '127.0.0.1'
    # client_port = 7002

    # 服务器1地址和端口
    server_host = '192.168.112.72'
    server_port = 8010

    # 服务器1地址和端口
    # server_host = '127.0.0.1'
    # server_port = 7003

    # # 服务器2地址和端口
    # server2_host = '192.168.112.95'
    # server2_port = 7004

    # 创建UDP套接字
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # 绑定客户端地址和端口
    client_socket.bind((client_host, client_port))

    # print("等待接收消息...")


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
                if count % 17 == 0:

###################################################################

                    A = np.array([[0, 1, 0, 0],
                                  [0, 0, 1, 0],
                                  [1, 0, 0, 0],
                                  [0, 0, 0, 1]])
                    B = np.array([[-1, 0, 0, 0],
                                  [0, 1, 0, 0],
                                  [0, 0, -1, 0],
                                  [0, 0, 0, 1]])
                    # B矩阵是把手心朝上

                    # 虚拟坐标系
                    # matrix_rhand =  np.dot(np.dot(np.dot(np.dot(np.dot(matrices[0], matrices[1]), matrices[2]), matrices[3]),matrices[4]),matrices[5])

                    # 实机坐标系 手相对于臀
                    # matrix_rhand =  np.dot(np.dot(np.dot(np.dot(np.dot(np.dot(np.dot(np.dot(np.linalg.inv(A),matrices[0]), matrices[1]), matrices[2]), matrices[3]),matrices[4]),matrices[5]),A),B)


                    C = np.array([[0, 0, -1, 0],
                                  [1, 0, 0, 0],
                                  [0, -1, 0, 0],
                                  [0, 0, 0, 1]])


                    D = np.array([[-1, 0, 0, 0],
                                  [0, 1, 0, 0],
                                  [0, 0, -1, 0],
                                  [0, 0, 0, 1]])

                    # 实机坐标系 手相对于肩膀
                    matrix_rhand = np.dot(np.dot(np.dot(np.linalg.inv(C),matrices[3]),matrices[4]),matrices[5])
                    matrix_rhand[2,3] = matrix_rhand[2,3]-15
                    # matrix_rhand[1,3] = matrix_rhand[1,3]+10

                    matrix_rhand_1 = np.dot(np.dot(matrices[3], matrices[4]), matrices[5])

                    #肘关节相对于肩膀
                    matrix_relbow = np.dot(np.dot(np.linalg.inv(C),matrices[3]),matrices[4])
                    matrix_relbow[2,3] = matrix_relbow[2,3]-15
                    # matrix_relbow[1,3] = matrix_relbow[1,3]+10
                    #shuzu_rhand = juzhen_to_shuzu(matrix_rhand)
                    # print("右手MUJOCO齐次变换矩阵", matrix_rhand)
                    translation_rhand, euler_rhand = convert_to_translation_and_euler(matrix_rhand)
                    translation_relbow, euler_relbow = convert_to_translation_and_euler(matrix_relbow)
                    # print(f"右手MUJOCO位姿 - 位移: {translation_rhand}, 欧拉角: {euler_rhand}")
                    euler_rhand_waixuan = matrix3d_to_euler_angles_zyx(matrix_rhand)

                    # print(translation_relbow)



###################################################################

                    A = np.array([[0, 1, 0, 0],
                                  [0, 0, 1, 0],
                                  [1, 0, 0, 0],
                                  [0, 0, 0, 1]])
                    B = np.array([[-1, 0, 0, 0],
                                  [0, 1, 0, 0],
                                  [0, 0, -1, 0],
                                  [0, 0, 0, 1]])

                    # 虚拟坐标系
                    # matrix_lhand = np.dot(np.dot(
                    #     np.dot(np.dot(np.dot(np.dot(np.dot(np.dot(np.linalg.inv(A), matrices[0]), matrices[1]), matrices[6]), matrices[7]), matrices[8]),matrices[9]), A), B)
                    # shuzu_lhand = juzhen_to_shuzu(matrix_lhand)

                    # 实机坐标系 手相对于臀
                    # matrix_lhand = np.dot(np.dot(np.dot(np.dot(np.dot(matrices[0], matrices[1]), matrices[6]), matrices[7]),matrices[8]),matrices[9])


                    E = np.array([[0, 0, 1, 0],
                                  [1, 0, 0, 0],
                                  [0, 1, 0, 0],
                                  [0, 0, 0, 1]])

                    F = np.array([[-1, 0, 0, 0],
                                  [0, -1, 0, 0],
                                  [0, 0, 1, 0],
                                  [0, 0, 0, 1]])
                    # 实机坐标系 手相对于肩膀
                    matrix_lhand = np.dot(np.dot(np.dot(np.dot(np.linalg.inv(E), matrices[7]), matrices[8]), matrices[9]), F)
                    matrix_lhand[2,3] = matrix_lhand[2,3]-15
                    # matrix_lhand[1,3] = matrix_lhand[1,3]-10
                    # matrix_lhand_1 =  np.dot(np.dot(matrices[7], matrices[8]), matrices[9])

                    # print(matrix_lhand_1)
                    # 肘关节相对于肩膀
                    matrix_lelbow = np.dot(np.dot(np.dot(np.linalg.inv(E), matrices[7]), matrices[8]), F)
                    matrix_lelbow[2,3] = matrix_lelbow[2,3]-15
                    # matrix_lelbow[1,3] = matrix_lelbow[1,3]-10
                    # print("左手MUJOCO齐次变换矩阵", matrix_lhand)
                    translation_lhand, euler_lhand = convert_to_translation_and_euler(matrix_lhand)
                    # print(translation_lhand)
                    translation_lelbow, euler_lelbow = convert_to_translation_and_euler(matrix_lelbow)
                    # print(translation_lelbow)
                    # print(f"左手MUJOCO位姿 - 位移: {translation_lhand}, 欧拉角: {euler_lhand}")
                    euler_lhand_waixuan = matrix3d_to_euler_angles_zyx(matrix_lhand)
                    # print(euler_lhand_waixuan)

###################################################################

                    A = np.array([[0, 1, 0, 0],
                                  [0, 0, 1, 0],
                                  [1, 0, 0, 0],
                                  [0, 0, 0, 1]])

                    matrix_rfoot = np.dot(np.dot(np.dot(np.dot(np.linalg.inv(A), matrices[11]), matrices[12]), matrices[13]), A)
                    # shuzu_rfoot = juzhen_to_shuzu(matrix_rfoot)
                    # print("右脚MUJOCO齐次变换矩阵", matrix_rfoot)
                    translation_rfoot, euler_rfoot = convert_to_translation_and_euler(matrix_rfoot)
                    # print
                    # print(f"右脚MUJOCO位姿 - 位移: {translation_rfoot}, 欧拉角: {euler_rfoot}")
                    euler_rfoot_waixuan = matrix3d_to_euler_angles_zyx( matrix_rfoot)
#########################################################################################

                    A = np.array([[0, 1, 0, 0],
                                  [0, 0, 1, 0],
                                  [1, 0, 0, 0],
                                  [0, 0, 0, 1]])

                    matrix_lfoot = np.dot(np.dot(np.dot(np.dot(np.linalg.inv(A), matrices[14]), matrices[15]), matrices[16]), A)
                    # shuzu_lfoot = juzhen_to_shuzu(matrix_lfoot)
                    # print("左脚MUJOCO齐次变换矩阵", matrix_lfoot)
                    translation_lfoot, euler_lfoot = convert_to_translation_and_euler(matrix_lfoot)
                    # print(f"左脚MUJOCO位姿 - 位移: {translation_lfoot}, 欧拉角: {euler_lfoot}")
                    euler_lfoot_waixuan = matrix3d_to_euler_angles_zyx(matrix_lfoot)

#########################################################################################



##########################################################################################

                    elbow_left = np.array([translation_lelbow[0]*10, translation_lelbow[1]*10, translation_lelbow[2]*10])
                    hand_left = np.array([translation_lhand[0]*10, translation_lhand[1]*10, translation_lhand[2]*10])


                    elbow_right = np.array([translation_relbow[0] * 10, translation_relbow[1] * 10, translation_relbow[2] * 10])
                    hand_right = np.array([translation_rhand[0] * 10, translation_rhand[1] * 10, translation_rhand[2] * 10])
                    bingxingjiao_l= angle_bixing_left(np.array([0, 0, 0]),elbow_left, hand_left)
                    bingxingjiao_r =angle_bixing_right(np.array([0, 0, 0]), elbow_right, hand_right)



                    message =np.concatenate((
                                             euler_lhand_waixuan, hand_left,np.array([bingxingjiao_l]),
                                             euler_rhand_waixuan, hand_right,-np.array([bingxingjiao_r])
                                            ))
                    print(message[:])
                    # print("     ")


                    packed_data = b''.join([struct.pack('<f', num) for num in message])
                    # client_socket.sendto(str(message).encode(), (server_host, server_port))
                    client_socket.sendto(packed_data, (server_host, server_port))
                    # print("全部消息已发送至服务器。")

                # 每行输出17个矩阵后，重置计数器
                if count % 17 == 0:
                    count = 0

                # 每输出一个矩阵后，重置矩阵计数器
                if matrix_count % 17 == 0:
                    matrix_count = 0

    except KeyboardInterrupt:
        # print("客户端已关闭。")
        client_socket.close()


if __name__ == "__main__":
    main()
