#!/usr/bin/env python

# import rospy
# import tf2_ros
# from geometry_msgs.msg import TransformStamped
from transformations import quaternion_from_euler, euler_from_matrix
import struct
from HelpFuction import matrix3d_to_euler_angles_zyx, xyz_quaternion_to_homogeneous, rpy2rotation_matrix, rotation_matrix_to_rpy, \
    find_axis_angle, calc_dist

import quest3_hand

import receiver
import datetime
import numpy as np
import socket
import struct
import cv2

import threading

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation
from pynput import keyboard


'''
Guozi:modify should pose
'''

head_to_left_S = np.array([[1, 0, 0, -0.2],
                            [0, 1, 0, -0.15],
                            [0, 0, 1, -0.25],
                            [0, 0, 0, 1]])

head_to_right_S = np.array([[1, 0, 0, 0.13],
                            [0, 1, 0, -0.15],
                            [0, 0, 1, -0.25],
                            [0, 0, 0, 1]])


server_ip="127.0.0.1"
# server_ip="192.168.113.244"

server_port=5015

'''
type 1 mujoco
type 2 robot
'''
robot_type="mujoco"


robot_ip="127.0.0.1"
robot_port=5005

left_hand=np.zeros((12))
right_hand=np.zeros((12))


recv_head_pose = np.eye(4)
save_head_pose = np.eye(4)
left_lim = np.zeros((4,4))
right_lim = np.zeros((4,4))
# left:up_lim_min = [-300:300,100:600,-150:500] 
# right:up_lim_min=[-300:300,-600:-100,-150:500]

running = False

vr_device_type=''

def calibrate():
    global save_head_pose
    if vr_device_type=='vision_pro':

        save_head_pose=recv_head_pose[:,:]
        print(f'save_head_pose is {save_head_pose}')
    elif vr_device_type =='quest3':
        quest3_hand.calibrate_shoudler()
    else:
        print("calibrate error: can not determine device!")
    
    print(f'calibrate is success')


def on_press(key):
    global save_head_pose
    try:
        if key.char == 's':
            save_head_pose=recv_head_pose[:,:]
            print(f'svae_head_pose is {save_head_pose}')
            print('Key pressed: {0}'.format(key.char))
            
    except AttributeError:
        print('Special key pressed: {0}'.format(key))

def on_release(key):
    print('Key released: {0}'.format(key))
    if key == keyboard.Key.esc:
        # Stop listener
        return False
    

# Collect events until released
def listening():
    print('=================start listening==================')
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()
        # listener.start()
    #     ...
    # listener=keyboard.Listener(on_press=on_press, on_release=on_release)
# listener_thread = threading.Thread(target=listening)
# listener_thread.start()





def save_head_array(float_array):

    while running:

        if isinstance(float_array, tuple):
               float_array = np.array(float_array)
    
        np.save('/home/guozhuoran/catkin_ws/head_re_matrix.npy',float_array.reshape(4,4))

def lim_angle(angle):
        
        if angle <0:
            angle=0
        if angle>90:
            angle=90

        return angle


def cross_raw(a, b):
    return np.cross(a, b)


def cross(a, b):
    c = cross_raw(a, b)
    c = c / np.linalg.norm(c)
    return c


def calc_arm_angle(T_base_to_w, T_base_to_e, T_base_to_s):
    # calc arm angle
    p_left_wrist = T_base_to_w[:3, -1]
    p_left_elbow = T_base_to_e[:3, -1]
    p_left_shoulder = T_base_to_s[:3, -1]
    p_left_shoulder_g = np.array([-1000, p_left_wrist[1], p_left_wrist[2]])

    v_left_se = p_left_elbow - p_left_shoulder
    v_left_sw = p_left_wrist - p_left_shoulder
    norm_surface_sew = cross(v_left_se, v_left_sw)

    v_left_sg = p_left_shoulder_g - p_left_shoulder
    v_left_sw = p_left_wrist - p_left_shoulder
    norm_surface_sgw = cross(v_left_sw, v_left_sg)

    _, theta_rad = find_axis_angle(norm_surface_sgw, norm_surface_sew)

    # #print(norm_surface_sgw, norm_surface_sew)

    # #print(np.rad2deg(theta_rad))
    return theta_rad




class UdpIkSender:
    def __init__(self):
        self.client_host = robot_ip
        self.client_port = robot_port
        BUFFER_SIZE = 1024

        # 创建UDP套接字
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # 绑定客户端地址和端口
        # client_socket.bind((client_host, client_port))

    def send(self, message):
        #guozi: modification
        # packed_data = b''

        if robot_type =='mujoco':
            # print("send mujoco")
            packed_data = b''.join([struct.pack('>f', num) for num in message])#simulation >
            self.client_socket.sendto(packed_data, (self.client_host, self.client_port))
        elif robot_type == 'robot':
            # print("send robot")
            packed_data = b''.join([struct.pack('<f', num) for num in message])
            self.client_socket.sendto(packed_data, (self.client_host, self.client_port))

        # self.client_socket.sendto(packed_data, (self.client_host, self.client_port))
        # #print('send')





def get_hand_tf(group_to_head,group_to_left_hand,group_to_right_hand):

    # head_to_left_S = np.array([[1, 0, 0, -0.2],
                            # [0, 1, 0, -0.15],
                            # [0, 0, 1, -0.25],
                            # [0, 0, 0, 1]])

    # head_to_right_S = np.array([[1, 0, 0, 0.13],
                            # [0, 1, 0, -0.15],
                            # [0, 0, 1, -0.25],
                            # [0, 0, 0, 1]])

    

    should_to_sun_left_should = np.array([[0.0000000,  0.0000000, -1.0000000, 0],
                            [0.0000000,  1.0000000,  0.0000000, 0],
                            [1.0000000,  0.0000000,  0.0000000, 0],
                            [0, 0, 0, 1]])


    should_to_sun_right_should = np.array([[0.0000000, -0.0000000,  1.0000000, 0],
                            [0.0000000, -1.0000000, -0.0000000, 0],
                            [1.0000000,  0.0000000, -0.0000000, 0],
                            [0, 0, 0, 1]])


    hand_to_sun_left_hand = np.array([
        [-1.0000000,  0.0000000,  0.0000000, 0],
        [0.0000000,  1.0000000,  0.0000000, 0],
        [0.0000000,  0.0000000, -1.0000000, 0],
        [0, 0, 0, 1]])


    hand_to_sun_right_hand = np.array([
        [1.0000000,  0.0000000,  0.0000000, 0],
        [0.0000000,  1.0000000,  0.0000000, 0],
        [0.0000000,  0.0000000, 1.0000000, 0],
        [0, 0, 0, 1]])

    should_to_hand_left = np.linalg.inv(should_to_sun_left_should)@np.linalg.inv(head_to_left_S)@np.linalg.inv(group_to_head)@group_to_left_hand@hand_to_sun_left_hand


    should_to_hand_right = np.linalg.inv(should_to_sun_right_should)@np.linalg.inv(head_to_right_S)@np.linalg.inv(group_to_head)@group_to_right_hand@hand_to_sun_right_hand

    print(head_to_left_S)

    zyx_left=matrix3d_to_euler_angles_zyx(should_to_hand_left)
    zyx_right=matrix3d_to_euler_angles_zyx(should_to_hand_right)


    return should_to_hand_left,should_to_hand_right,zyx_left,zyx_right


def calc_arm_angle2(p_right_wrist, p_right_elbow, p_right_shoulder):
    # calc arm angle

    p_right_shoulder_g = np.array([p_right_shoulder[0], 1000, p_right_shoulder[2]])

    v_right_se = p_right_elbow - p_right_shoulder
    v_right_sw = p_right_wrist - p_right_shoulder
    norm_surface_sew = cross(v_right_se, v_right_sw)

    v_right_sg = p_right_shoulder_g - p_right_shoulder
    v_right_sw = p_right_wrist - p_right_shoulder
    norm_surface_sgw = cross(v_right_sw, v_right_sg)

    _, theta_rad = find_axis_angle(norm_surface_sgw, norm_surface_sew)

    # #print(norm_surface_sgw, norm_surface_sew)

    # #print(np.rad2deg(theta_rad))
    return theta_rad
    

class TFPublisher:
    def __init__(self):
        global recv_head_pose,vr_device_type
        # listening()

        # print("asdfagehbr")
        # Create a TF broadcaster

        # Create a TransformStamped message
        
        # read tf


        # self.tf_msg = create_tf_xyz_rpy(1,2,3,0,0,0,'map','camera_link')

        # Publish the TF message at a rate of 10 Hz

        self.last_right_pose=([0,0,0],[0,0,0])
        self.last_left_pose=([0,0,0],[0,0,0])


        # UDP_IP = "127.0.0.1"  # IP address to listen on
        # UDP_PORT = 5015  # Port to listen on
        self.BUFFER_SIZE = 8888  # Buffer size for incoming messages

        # Create UDP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((server_ip, server_port))

        #print("UDP receiver started")

        self.udp_ik_sender = UdpIkSender()

        self.left_theta_rad=0.3
        self.right_theta_rad=0.3
        self.start_time=datetime.datetime.now()


        self.data={}
        self.data_len_f=0

        #Guozi:set boundaries

        self.x_min = -0.426
        self.x_max =0.464
        self.len_x =self.x_max - self.x_min
        self.y_min = 0.152
        self.y_max = 0.438 
        self.len_y = self.y_max - self.y_min    
        self.z_min = 1.00
        self.z_max = 1.80
        self.len_z = self.z_max-self.z_min
        

        # left:[-300:300,100:600,-150:500]
        # right:[-300:300,-600:-100,-150:500]

        while True:
            # print("1111111")
            data_bytes, addr = sock.recvfrom(self.BUFFER_SIZE)####receive
            #print("2222222")
            #print(data_bytes)
            if not data_bytes:
                #print('no data')
                break

            l = len(data_bytes)
            #print("1234455",l)
            if l==16*4*53:
                vr_device_type='vision_pro'
                # Unpack the received data into a float array
                # print(f'databytes is {l}')
                float_array = struct.unpack('848f', data_bytes)

                self.bounding_box = {
                    "length": np.array([self.len_x,self.len_y,self.len_z]),
                    "rotation":np.array(float_array[:16]).reshape(4,4)[:3,:3]
                }


                recv_head_pose=np.array(float_array[:16]).reshape(4,4)[:,:]


            
                # print('----------------registration-----------')
                # self.register(float_array)                
                valid_left ,valid_right = self.is_point_in_rotated_bounding_box(np.array(float_array[16:32]).reshape(4,4),
                                                            np.array(float_array[32:48]).reshape(4,4))
                
                valid_left=True
                valid_right=True
                self.process_vision_pro_data(valid_left ,valid_right,float_array)
            elif l == 4 * 49 * 7:
                vr_device_type='quest3'
                xyzqwqxqyqz = quest3_hand.handle_raw_data(data_bytes)
                self.process_quest3_data(xyzqwqxqyqz)

            elif l == 4*4:
                float_array = struct.unpack(f'{4}f', data_bytes)
                self.data={'n':int(float_array[0]),'body':[]}
                self.data_len_f=int(float_array[1])
                # if data['n']==0:
                #     process_function(data)

            elif l == 4 * (self.data_len_f):
                # self.data_len_f = 1 + 32 * 8

                # Unpack the received data into a float array
                float_array = struct.unpack(f'{self.data_len_f}f', data_bytes)

                self.data['body'].append(float_array)
                # #print("Received float array:", float_array[:10])
                if float_array[0]==self.data['n']-1:
                    self.process_function(self.data)

          
    def process_function(self,data):
        if data['n'] > 0:
            body_data_with_index = data['body'][0]
            body_data = body_data_with_index[1:]
            body = np.array([body_data])
            body = body.reshape(32, 8)
            # move mid_hip to zero
            body[:, 1:4] = body[:, 1:4] - body[0, 1:4]

            s = body[5]
            e = body[6]
            w = body[7]
            h = body[8]
            if s[0] > 1 and e[0] > 1 and w[0] > 1:
                end_time = datetime.datetime.now()
                time_interval = end_time - self.start_time
                interval_seconds = time_interval.total_seconds()
                # #print(interval_seconds)
                if interval_seconds > 0.1 or True:
                    self.calc_robot_input(time_interval, body[:, 1:])
                    self.start_time = end_time

    

    def calc_robot_input(self, ts, body):
        # camera in front of people, people face font
        # x left, z back , y down

        s_r = body[12]
        e_r = body[13]
        w_r = body[14]

        # calc arm angle
        theta_rad_r = calc_arm_angle2(w_r[:3], e_r[:3], s_r[:3])
        theta_rad_r = np.pi - theta_rad_r

        s_l = body[5]
        e_l = body[6]
        w_l = body[7]

        theta_rad_l = calc_arm_angle2(w_l[:3], e_l[:3], s_l[:3])
        theta_rad_l = np.pi - theta_rad_l

        self.left_theta_rad = theta_rad_l
        self.right_theta_rad = theta_rad_r

        return 0
    
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

    
    def register(self,float_array):
        global running

        #print('------start saving head array-------')

        # Initialize a named window
        cv2.namedWindow("Test Window")

        while True:
            # Display an empty frame (you can replace this with your actual frame)
            frame = cv2.imread("/home/guozhuoran/catkin_ws/example.jpg")  # Replace with your image path or capture from camera
            cv2.imshow("Test Window", frame)

            # Wait for a key press
            key = cv2.waitKey(1) & 0xFF

            # Start the function when 's' is pressed
            if key == ord('s'):
                if not running:
                    running = True
                    threading.Thread(target=save_head_array,args=(np.array(float_array[:16]),)).start()
                    #print("Started  registering.")

            # Stop the function when 'q' is pressed
            elif key == ord('q'):
                if running:
                    running = False
                    #print("Stopped registering.")

            # Break the loop when 'esc' is pressed
            elif key == 27:  # Esc key
                break

        # Release resources
        cv2.destroyAllWindows()

    from scipy.spatial.transform import Rotation

    def is_point_in_rotated_bounding_box(self,left_hand,right_hand):
        """
        判断一个点是否在旋转后的3D边界框内。

        :param point: 点的坐标，格式为 [x, y, z]
        :param bounding_box: 旋转后的边界框，定义为一个字典，包含旋转矩阵和边界框的尺寸
                             {'rotation_matrix': rotation_matrix, 'length': [length_x, length_y, length_z]}
        :return: 如果点在旋转后的边界框内，返回True；否则返回False
        """
        # 获取旋转矩阵和边界框尺寸
        rotation_matrix = self.bounding_box['rotation']
        # length = self.bounding_box['length']

        # 点的坐标转换到旋转后的边界框的本地坐标系
        point_local_left = rotation_matrix.T @ np.array(left_hand[:3,3]).T
        point_local_right = rotation_matrix.T @ np.array(right_hand[:3,3]).T

        # 判断点是否在边界框的本地坐标系内
        in_x_l = self.x_min <= point_local_left[0] <= self.x_max
        in_y_l = self.y_min <= point_local_left[1] #<= self.y_max
        
        in_z_l = self.z_min-1.65+save_head_pose[2,3] <= point_local_left[2] <= self.z_max-1.65+save_head_pose[2,3]
        
        in_x_r = self.x_min <= point_local_right[0] <= self.x_max
        in_y_r = self.y_min <= point_local_right[1] #<= self.y_max
        in_z_r = self.z_min-1.65+save_head_pose[2,3] <= point_local_right[2] <= self.z_max-1.65+save_head_pose[2,3]


        return in_x_l and in_y_l and in_z_l , in_x_r and in_y_r and in_z_r

    def process_vision_pro_data(self,valid_left ,valid_right,float_array):
        global left_hand,right_hand
        #print('--------------begin  visionpro processing!-------------')
        head_data=np.array(float_array[:16])

        # head_data = np.load('/home/guozhuoran/catkin_ws/head_re_matrix.npy')
        left_data=np.array(float_array[16:32])
        right_data=np.array(float_array[32:48])
        left_fin_data_1=np.array(float_array[64:80])
        left_fin_data_2=np.array(float_array[80:96])
        left_fin_data_3=np.array(float_array[48+6*16:48+7*16])
        left_fin_data_4=np.array(float_array[48+11*16:48+12*16])
        left_fin_data_5=np.array(float_array[48+16*16:48+17*16])
        left_fin_data_6=np.array(float_array[48+21*16:48+22*16])

        right_fin_data_1=np.array(float_array[48+26*16:48+27*16])
        right_fin_data_2=np.array(float_array[48+27*16:48+28*16])
        right_fin_data_3=np.array(float_array[(48+25*16)+6*16:(48+25*16)+7*16])
        right_fin_data_4=np.array(float_array[(48+25*16)+11*16:(48+25*16)+12*16])
        right_fin_data_5=np.array(float_array[(48+25*16)+16*16:(48+25*16)+17*16])
        right_fin_data_6=np.array(float_array[(48+25*16)+21*16:(48+25*16)+22*16])
   

        left_fin_data_1=left_fin_data_1.reshape((4,4))
        left_fin_data_2=left_fin_data_2.reshape((4,4))
        left_fin_data_3=left_fin_data_3.reshape((4,4))
        left_fin_data_4=left_fin_data_4.reshape((4,4))
        left_fin_data_5=left_fin_data_5.reshape((4,4))
        left_fin_data_6=left_fin_data_6.reshape((4,4))

        right_fin_data_1=right_fin_data_1.reshape((4,4))
        right_fin_data_2=right_fin_data_2.reshape((4,4))
        right_fin_data_3=right_fin_data_3.reshape((4,4))
        right_fin_data_4=right_fin_data_4.reshape((4,4))
        right_fin_data_5=right_fin_data_5.reshape((4,4))
        right_fin_data_6=right_fin_data_6.reshape((4,4))
        

        left_fin_data_1_xyz = matrix3d_to_euler_angles_zyx(left_fin_data_1)
        left_fin_data_2_xyz = matrix3d_to_euler_angles_zyx(left_fin_data_2)
        left_fin_data_3_xyz = matrix3d_to_euler_angles_zyx(left_fin_data_3)
        left_fin_data_4_xyz = matrix3d_to_euler_angles_zyx(left_fin_data_4)
        left_fin_data_5_xyz = matrix3d_to_euler_angles_zyx(left_fin_data_5)
        left_fin_data_6_xyz = matrix3d_to_euler_angles_zyx(left_fin_data_6)

        right_fin_data_1_xyz = matrix3d_to_euler_angles_zyx(right_fin_data_1)
        right_fin_data_2_xyz = matrix3d_to_euler_angles_zyx(right_fin_data_2)
        right_fin_data_3_xyz = matrix3d_to_euler_angles_zyx(right_fin_data_3)
        right_fin_data_4_xyz = matrix3d_to_euler_angles_zyx(right_fin_data_4)
        right_fin_data_5_xyz = matrix3d_to_euler_angles_zyx(right_fin_data_5)
        right_fin_data_6_xyz = matrix3d_to_euler_angles_zyx(right_fin_data_6)


        left_finger_1 = (left_fin_data_1_xyz[0]+0.65)/0.5*90
        left_finger_1 = lim_angle(left_finger_1)

        left_finger_2 = (left_fin_data_2_xyz[0]+0.6)/1.6*90
        left_finger_2 = lim_angle(left_finger_2)

        left_finger_3 = (left_fin_data_3_xyz[0]+0.2)/1.2*90
        left_finger_3 = lim_angle(left_finger_3)

        left_finger_4 = (left_fin_data_4_xyz[0]+0.2)/1.2*90
        left_finger_4 = lim_angle(left_finger_4)

        left_finger_5 = (left_fin_data_5_xyz[0]+0.2)/1.2*90
        left_finger_5 = lim_angle(left_finger_5)

        left_finger_6 = (left_fin_data_6_xyz[0]+0.2)/1.2*90
        left_finger_6 = lim_angle(left_finger_6)

        right_finger_1 = (right_fin_data_1_xyz[0]+0.65)/0.5*90
        right_finger_1 = lim_angle(right_finger_1)

        right_finger_2 = (right_fin_data_2_xyz[0]+0.6)/1.6*90
        right_finger_2 = lim_angle(right_finger_2)

        right_finger_3 = (right_fin_data_3_xyz[0]+0.2)/1.2*90
        right_finger_3 = lim_angle(right_finger_3)

        right_finger_4 = (right_fin_data_4_xyz[0]+0.2)/1.2*90
        right_finger_4 = lim_angle(right_finger_4)

        right_finger_5 = (right_fin_data_5_xyz[0]+0.2)/1.2*90
        right_finger_5 = lim_angle(right_finger_5)

        right_finger_6 = (right_fin_data_6_xyz[0]+0.2)/1.2*90
        right_finger_6 = lim_angle(right_finger_6)


        # #print(left_finger_1,left_finger_2,left_finger_3,left_finger_4,left_finger_5,left_finger_6)
        # #print(right_finger_1,right_finger_2,right_finger_3,right_finger_4,right_finger_5,right_finger_6)

        head_data=head_data.reshape((4,4))
        # head_data[:3,:3]=save_head_pose[:3,:3]
        head_data = save_head_pose[:,:]
        # left_data=left_data.reshape((4,4))
        # right_data=right_data.reshape((4,4))

        #Guozi: using global variables
        left_lim = left_data.reshape((4,4))
        right_lim= right_data.reshape((4,4))

        # #print(f'left_lim is {left_lim},right_lim is {right_lim}')


        #left right positions

        head_data_t=head_data[:3,-1]
        # left_data_t=left_data[:3,-1]
        left_data_t=left_lim[:3,-1]

        #Guozi:set limitations
        # if left_lim[:3,-1][0]>=self.left_x_max:
        #     left_lim[:3,-1][0]=self.left_x_max
        # if left_lim[:3,-1][0]<=self.left_x_min:
        #     left_lim[:3,-1][0]=self.left_x_min
        # if left_lim[:3,-1][1]>=self.left_y_max:
        #     left_lim[:3,-1][1]=self.left_y_max
        # if left_lim[:3,-1][1]<=self.left_y_min:
        #     left_lim[:3,-1][1]=self.left_y_min


        # right_data_t=right_data[:3,-1]
        right_data_t=right_lim[:3,-1]

        #Guozi:set limitations
        # if right_lim[:3,-1][0]>=self.right_x_max:
        #     right_lim[:3,-1][0]=self.right_x_max
        # if right_lim[:3,-1][0]<=self.right_x_min:
        #     right_lim[:3,-1][0]=self.right_x_min
        # if right_lim[:3,-1][1]>=self.right_y_max:
        #     right_lim[:3,-1][1]=self.right_y_max
        # if right_lim[:3,-1][1]<=self.right_y_min:
        #     right_lim[:3,-1][1]=self.right_y_min

        

        head_data_rpy = euler_from_matrix(head_data[:3,:3])
        left_data_rpy = euler_from_matrix(left_lim[:3,:3])
        right_data_rpy = euler_from_matrix(right_lim[:3,:3])


        left_data_robot,right_data_robot,zyx_left_robot,zyx_right_robot =get_hand_tf(head_data,left_lim,right_lim)
        # #print(left_data_robot)

        left_wrist_t=left_data_robot[:3,-1]*1000

        right_wrist_t=right_data_robot[:3,-1]*1000

        if valid_left:
            self.last_left_pose=(left_wrist_t,zyx_left_robot)
        else:
            left_wrist_t,zyx_left_robot=self.last_left_pose

        if valid_right:
            self.last_right_pose=(right_wrist_t,zyx_right_robot)
        else:
            right_wrist_t,zyx_right_robot=self.last_right_pose


        
        # ##print("left_theta_rad",self.left_theta_rad)
        # ##print(left_data,left_wrist_t)
        self.send_message(zyx_left_robot,
            left_wrist_t,
            zyx_right_robot,
            right_wrist_t,
            left_finger_1,left_finger_2,left_finger_3,left_finger_4,left_finger_5,left_finger_6,
            right_finger_1,right_finger_2,right_finger_3,right_finger_4,right_finger_5,right_finger_6)


    
        # tf_msg = create_tf_xyz_quat(float_array[1]/1000,float_array[2]/1000,float_array[3]/1000,float_array[4],float_array[5],float_array[6],float_array[7],'map',link)
        

        # Publish the TF message

        # tf_msg = create_tf_xyz_quat(float_array[1]/1000,float_array[2]/1000,float_array[3]/1000,float_array[4],float_array[5],float_array[6],float_array[7],'map',link)


        # Publish the TF message

        # tf_msg = create_tf_xyz_quat(float_array[1]/1000,float_array[2]/1000,float_array[3]/1000,float_array[4],float_array[5],float_array[6],float_array[7],'map',link)


        # Publish the TF message


        #print("---visionpro process is done------------------")

    def send_message(self,zyx_left_robot,
            left_wrist_t,
            zyx_right_robot,
            right_wrist_t,
            left_finger_1,left_finger_2,left_finger_3,left_finger_4,left_finger_5,left_finger_6,
            right_finger_1,right_finger_2,right_finger_3,right_finger_4,right_finger_5,right_finger_6):
        

        global left_hand,right_hand

        if robot_type=='mujoco':
             message = [
            # 
            *zyx_left_robot,
            # -1.5708, 1.5708, 0,
            *left_wrist_t,1.0,
            # -500.0, 300, 100.0, 0.5233,
           1.0,
            # right
            # 0.0, 1.5708, 0.0,
            # 100.0, -200, 500.0, 0.0,
            #  0.0True
            *zyx_right_robot,
            *right_wrist_t,-1.0,
                0.0,
            ]
        elif robot_type=='robot':
            message = [
            #
            *zyx_left_robot,
            *left_wrist_t,1.0,
            *zyx_right_robot,
            *right_wrist_t,-1.0,
            left_finger_1,left_finger_2,left_finger_3,left_finger_4,left_finger_5,left_finger_6,
            right_finger_1,right_finger_2,right_finger_3,right_finger_4,right_finger_5,right_finger_6,
            left_finger_3/90*85,right_finger_3/90*85
            ]


        self.udp_ik_sender.send(message)

        left_hand=[*zyx_left_robot,*left_wrist_t,left_finger_1,left_finger_2,left_finger_3,left_finger_4,left_finger_5,left_finger_6]
        right_hand=[*zyx_right_robot,*right_wrist_t,right_finger_1,right_finger_2,right_finger_3,right_finger_4,right_finger_5,right_finger_6]
        


    def process_quest3_data(self,xyzqwqxqyqz):
        
        
        [
            zyx_left_robot,
            left_wrist_t,
            zyx_right_robot,
            right_wrist_t,
            left_finger_1,left_finger_2,left_finger_3,left_finger_4,left_finger_5,left_finger_6,
            right_finger_1,right_finger_2,right_finger_3,right_finger_4,right_finger_5,right_finger_6
            ]=quest3_hand.process(xyzqwqxqyqz)
        
        self.send_message(zyx_left_robot,
            left_wrist_t,
            zyx_right_robot,
            right_wrist_t,
            left_finger_1,left_finger_2,left_finger_3,left_finger_4,left_finger_5,left_finger_6,
            right_finger_1,right_finger_2,right_finger_3,right_finger_4,right_finger_5,right_finger_6)
  





def main():
    rec = TFPublisher()

if __name__ == '__main__':
    
        #print("UDP start!")
        rec = TFPublisher()
        ##print("123456")
        rec.publish_tf()


        
