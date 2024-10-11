#!/usr/bin/env python
'''
accept vuer vr support
adapt devices: quest3 and vision pro
drive QinLong robot
'''
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler, euler_from_matrix, quaternion_from_matrix
import struct
from HelpFuction import xyz_quaternion_to_homogeneous, rpy2rotation_matrix, rotation_matrix_to_rpy, \
    find_axis_angle, calc_dist

import receiver
import datetime
import numpy as np
import socket
import struct
import cv2

left_lim = []
right_lim = []


def lim_angle(angle):
    if angle < 0:
        angle = 0
    if angle > 90:
        angle = 90

    return angle


def create_tf_xyz_rpy(x, y, z, roll, pitch, yaw, parent, child):
    tf_msg = TransformStamped()

    # Set the frame IDs
    tf_msg.header.frame_id = parent  # Parent frame
    tf_msg.child_frame_id = child  # Child frame

    # Set the initial position(x, y, z)
    tf_msg.transform.translation.x = x
    tf_msg.transform.translation.y = y
    tf_msg.transform.translation.z = z

    # Set the initial orientation (roll, pitch, yaw)

    quaternion = quaternion_from_euler(roll, pitch, yaw)
    tf_msg.transform.rotation.x = quaternion[0]
    tf_msg.transform.rotation.y = quaternion[1]
    tf_msg.transform.rotation.z = quaternion[2]
    tf_msg.transform.rotation.w = quaternion[3]

    tf_msg.header.stamp = rospy.Time.now()

    return tf_msg


def create_tf_xyz_quat(x, y, z, qw, qx, qy, qz, parent, child):
    tf_msg = TransformStamped()

    # Set the frame IDs
    tf_msg.header.frame_id = parent  # Parent frame
    tf_msg.child_frame_id = child  # Child frame

    # Set the initial position (x, y, z)
    tf_msg.transform.translation.x = x
    tf_msg.transform.translation.y = y
    tf_msg.transform.translation.z = z

    # Set the initial orientation (roll, pitch, yaw)

    tf_msg.transform.rotation.x = qx
    tf_msg.transform.rotation.y = qy
    tf_msg.transform.rotation.z = qz
    tf_msg.transform.rotation.w = qw

    tf_msg.header.stamp = rospy.Time.now()

    return tf_msg


def create_tf_xyz_qxyzw(x, y, z, qx, qy, qz, qw, parent, child):
    tf_msg = TransformStamped()

    # Set the frame IDs
    tf_msg.header.frame_id = parent  # Parent frame
    tf_msg.child_frame_id = child  # Child frame

    # Set the initial position (x, y, z)
    tf_msg.transform.translation.x = x
    tf_msg.transform.translation.y = y
    tf_msg.transform.translation.z = z

    # Set the initial orientation (roll, pitch, yaw)

    tf_msg.transform.rotation.x = qx
    tf_msg.transform.rotation.y = qy
    tf_msg.transform.rotation.z = qz
    tf_msg.transform.rotation.w = qw

    tf_msg.header.stamp = rospy.Time.now()

    return tf_msg


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

def get_hand_tf(tf_base_robot_to_he_robot, tf_base_robot_to_l_ha_robot, tf_base_robot_to_r_ha_robot):
    def get_tf_by_rpy(r, p, y):
        tf = np.eye(4)
        # rt_l_hand_quest_2_l_hand_robot[:3,:3]=rpy2rotation_matrix(np.deg2rad(180),0,np.deg2rad(0))
        tf[:3, :3] = rpy2rotation_matrix(r, p, y)
        return tf

    # left
    tf_l_s_robot_to_l_s_sun = get_tf_by_rpy(np.deg2rad(-90), 0, np.deg2rad(90))

    tf_l_ha_robot_to_l_ha_sun = get_tf_by_rpy(np.deg2rad(180), np.deg2rad(-90), 0)

    tf_he_robot_to_l_s_robot = np.array([[1.0000000, 0.0000000, 0.0000000, -0.2],
                                         [0.0000000, 1.0000000, 0.0000000, -0.2],
                                         [0.0000000, 0.0000000, 1.0000000, 0],
                                         [0, 0, 0, 1]])

    # right
    tf_r_s_robot_to_r_s_sun = get_tf_by_rpy(np.deg2rad(90), 0, np.deg2rad(90))

    tf_r_ha_robot_to_r_ha_sun = get_tf_by_rpy(np.deg2rad(0), np.deg2rad(-90), 0)

    tf_he_robot_to_r_s_robot = np.array([[1.0000000, -0.0000000, 0.0000000, 0.2],
                                         [0.0000000, 1.0000000, -0.0000000, -0.2],
                                         [0.0000000, 0.0000000, 1.0000000, 0],
                                         [0, 0, 0, 1]])

    tf_l_s_sun_to_l_s_robot = np.linalg.inv(tf_l_s_robot_to_l_s_sun)
    tf_l_s_robot_to_he_robot = np.linalg.inv(tf_he_robot_to_l_s_robot)

    tf_r_s_sun_to_r_s_robot = np.linalg.inv(tf_r_s_robot_to_r_s_sun)
    tf_r_s_robot_to_he_robot = np.linalg.inv(tf_he_robot_to_r_s_robot)

    tf_he_robot_to_base_robot = np.linalg.inv(tf_base_robot_to_he_robot)

    tf_l_s_sun_to_l_ha_sun = tf_l_s_sun_to_l_s_robot @ tf_l_s_robot_to_he_robot @ tf_he_robot_to_base_robot @ tf_base_robot_to_l_ha_robot @ tf_l_ha_robot_to_l_ha_sun
    tf_r_s_sun_to_r_ha_sun = tf_r_s_sun_to_r_s_robot @ tf_r_s_robot_to_he_robot @ tf_he_robot_to_base_robot @ tf_base_robot_to_r_ha_robot @ tf_r_ha_robot_to_r_ha_sun

    zyx_left = matrix3d_to_euler_angles_zyx(tf_l_s_sun_to_l_ha_sun)
    zyx_right = matrix3d_to_euler_angles_zyx(tf_r_s_sun_to_r_ha_sun)

    return tf_l_s_sun_to_l_ha_sun, tf_r_s_sun_to_r_ha_sun, zyx_left, zyx_right


class UdpIkSender:
    def __init__(self):
        self.client_host = "127.0.0.1"
        self.client_port = 5005
        BUFFER_SIZE = 1024

        # 创建UDP套接字
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # 绑定客户端地址和端口
        # client_socket.bind((client_host, client_port))

    def send(self, message):
        packed_data = b''.join([struct.pack('<f', num) for num in message])  # simulation >
        self.client_socket.sendto(packed_data, (self.client_host, self.client_port))
        # #print('send')


class TFPublisher:
    def __init__(self):
        rospy.init_node('tf_publisher')
        # print("asdfagehbr")
        # Create a TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Create a TransformStamped message

        # read tf

        # self.tf_msg = create_tf_xyz_rpy(1,2,3,0,0,0,'map','camera_link')

        # Publish the TF message at a rate of 10 Hz
        self.tf_publish_rate = rospy.Rate(10)

        UDP_IP = "127.0.0.1"  # IP address to listen on
        UDP_PORT = 5015  # Port to listen on
        self.BUFFER_SIZE = 2888  # Buffer size for incoming messages

        # Create UDP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((UDP_IP, UDP_PORT))

        # print("UDP receiver started")

        self.udp_ik_sender = UdpIkSender()

        self.left_theta_rad = 0.3
        self.right_theta_rad = 0.3
        self.start_time = datetime.datetime.now()

        self.data = {}
        self.data_len_f = 0
        while not rospy.is_shutdown():
            #     # print("1111111")
            data_bytes, addr = sock.recvfrom(self.BUFFER_SIZE)  ####receive
            print("2222222")
            print(data_bytes)
            if not data_bytes:
                # print('no data')
                break

            l = len(data_bytes)
            #     # print("1234455", l)
            # for test
            # float_array = np.load("/home/jyw/PycharmProjects/TeleVision/teleop/saved_data.npy")
            # self.process(float_array)

            if l == 222 * 4:
                # Unpack the received data into a float array
                # print(f'databytes is {data_bytes}')
                float_array = struct.unpack('222f', data_bytes)
                # print('init!')
                self.process(float_array)

    def process(self, float_array):
        # print('begin processing!')

        head_data = np.array(float_array[:16])
        left_data = np.array(float_array[16:32])
        right_data = np.array(float_array[32:48])

        head_data = head_data.reshape((4, 4))
        left_data = left_data.reshape((4, 4))
        right_data = right_data.reshape((4, 4))

        # head_data[:3,:3]=np.eye(3)

        # left right positions

        head_data_t = head_data[:3, -1]
        left_data_t = left_data[:3, -1]
        right_data_t = right_data[:3, -1]

        head_quaternion = quaternion_from_matrix(head_data)
        l_ha_quaternion = quaternion_from_matrix(left_data)
        r_ha_quaternion = quaternion_from_matrix(right_data)

        # head_data_rpy = euler_from_matrix(head_data[:3, :3])
        # left_data_rpy = euler_from_matrix(left_data[:3, :3])
        # right_data_rpy = euler_from_matrix(right_data[:3, :3])

        head_pose_new = head_data[:, :]
        # head_position_save=group_to_head[:3,3]

        # head_r_zyx = matrix3d_to_euler_angles_zyx(head_data[:3, :3])
        # head_r_rpy_new = [0, 0, head_r_zyx[0]]
        # head_r_new = rpy2rotation_matrix(*head_r_rpy_new)
        # head_pose_new[:3, :3] = head_r_new

        left_data_robot, right_data_robot, zyx_left_robot, zyx_right_robot = get_hand_tf(head_pose_new, left_data,
                                                                                         right_data)

        left_wrist_t = left_data_robot[:3, -1] * 1000
        right_wrist_t = right_data_robot[:3, -1] * 1000

        message = [
            #
            *zyx_left_robot,
            # -1.5708, 1.5708, 0,
            *left_wrist_t, 1.0,
            # -500.0, 300, 100.0, 0.5233,
            1.0,
            # right
            # 0.0, 1.5708, 0.0,
            # 100.0, -200, 500.0, 0.0,
            #  0.0True
            *zyx_right_robot,
            *right_wrist_t, -1.0,
            0.0,
            *float_array[198:]
        ]

        self.udp_ik_sender.send(message)
        np.set_printoptions(formatter={'float': '{:.2f}'.format})
        float_array=np.array(float_array)
        print(float_array[198:198+12])

        left_wrist_t = left_data_robot[:3, -1] * 1000
        right_wrist_t = right_data_robot[:3, -1] * 1000
        # #print("left_theta_rad",self.left_theta_rad)
        # #print(left_data,left_wrist_t)

        # tf_msg = create_tf_xyz_quat(float_array[1]/1000,float_array[2]/1000,float_array[3]/1000,float_array[4],float_array[5],float_array[6],float_array[7],'map',link)
        # tf_msg = create_tf_xyz_rpy(head_data_t[0], head_data_t[1], head_data_t[2], head_data_rpy[0], head_data_rpy[1],
        #                            head_data_rpy[2], 'map', 'head')
        tf_msg = create_tf_xyz_qxyzw(head_data_t[0], head_data_t[1], head_data_t[2], *head_quaternion, 'map', 'head')

        # Publish the TF message
        self.tf_broadcaster.sendTransform(tf_msg)

        # tf_msg = create_tf_xyz_quat(float_array[1]/1000,float_array[2]/1000,float_array[3]/1000,float_array[4],float_array[5],float_array[6],float_array[7],'map',link)
        tf_msg = create_tf_xyz_qxyzw(left_data_t[0], left_data_t[1], left_data_t[2], *l_ha_quaternion, 'map', 'left')

        # Publish the TF message
        self.tf_broadcaster.sendTransform(tf_msg)

        # tf_msg = create_tf_xyz_quat(float_array[1]/1000,float_array[2]/1000,float_array[3]/1000,float_array[4],float_array[5],float_array[6],float_array[7],'map',link)
        tf_msg = create_tf_xyz_qxyzw(right_data_t[0], right_data_t[1], right_data_t[2], *r_ha_quaternion, 'map',
                                     'right')

        # Publish the TF message
        self.tf_broadcaster.sendTransform(tf_msg)

        rospy.sleep(0.00)


if __name__ == '__main__':
    print("UDP start!")
    rec = TFPublisher()
