from transformations import quaternion_from_euler, euler_from_matrix,quaternion_matrix,quaternion_from_matrix
import struct
from HelpFuction import matrix3d_to_euler_angles_zyx, xyz_quaternion_to_homogeneous, rpy2rotation_matrix, rotation_matrix_to_rpy, \
    find_axis_angle, calc_dist


import struct
import numpy as np


head_pose_new=np.eye(4)
head_pose_save=np.eye(4)


def lim_angle(angle):

    if angle <0:
        angle=0
    if angle>90:
        angle=90

    return int(angle)

def calibrate_shoudler():
     global head_pose_save
     head_pose_save=head_pose_new[:,:]

def get_hand_tf_quest_fixed_head(group_to_head_new,group_to_left_hand,group_to_right_hand):

    global head_pose_new

    group_to_head=np.eye(4)

    # group_to_head[0,3]=0 #left right offset
    # group_to_head[1,3]=0 # front back offset

    head_pose_new=group_to_head_new[:,:]
    # head_position_save=group_to_head[:3,3]

    head_r_zyx=matrix3d_to_euler_angles_zyx(group_to_head_new[:3,:3])
    head_r_rpy_new=[0,0,head_r_zyx[0]]
    head_r_new=rpy2rotation_matrix(*head_r_rpy_new)
    head_pose_new[:3,:3]=head_r_new

    group_to_head=head_pose_save[:,:] # if using floor level
    
    # group_to_head[2,3]=0 # if using eye level

    
    head_to_left_S = np.array([[1, 0, 0, -0.2],
                            [0, 1, 0, -0.2],
                            [0, 0, 1, -0.2],
                            [0, 0, 0, 1]])

    head_to_right_S = np.array([[1, 0, 0, 0.2],
                            [0, 1, 0, -0.2],
                            [0, 0, 1, -0.2],
                            [0, 0, 0, 1]])

    should_to_sun_left_should = np.array([[0.0000000,  0.0000000, -1.0000000, 0],
                            [0.0000000,  1.0000000,  0.0000000, 0],
                            [1.0000000,  0.0000000,  0.0000000, 0],
                            [0, 0, 0, 1]])


    should_to_sun_right_should = np.array([[0.0000000, -0.0000000,  1.0000000, 0],
                            [0.0000000, -1.0000000, -0.0000000, 0],
                            [1.0000000,  0.0000000, -0.0000000, 0],
                            [0, 0, 0, 1]])


    should_to_hand_left = np.linalg.inv(should_to_sun_left_should)@np.linalg.inv(head_to_left_S)@np.linalg.inv(group_to_head)@group_to_left_hand


    should_to_hand_right = np.linalg.inv(should_to_sun_right_should)@np.linalg.inv(head_to_right_S)@np.linalg.inv(group_to_head)@group_to_right_hand

    zyx_left=matrix3d_to_euler_angles_zyx(should_to_hand_left)
    zyx_right=matrix3d_to_euler_angles_zyx(should_to_hand_right)


    return should_to_hand_left,should_to_hand_right,zyx_left,zyx_right



def handle_raw_data(data_bytes):
     '''
     if l == 4 * 49 * 7:
                    ...
                    fun(data_bytes)
     '''
     float_array = struct.unpack(f'{49 * 7}f', data_bytes)
     xyzqwqxqyqz = np.array(float_array).reshape((49, 7))
     return xyzqwqxqyqz

def process(xyzqwqxqyqz):
        

        # if save_once:
        #     np.save('/home/jyw/posedata.npy',xyzqwqxqyqz)
        #     save_once=False

        xyzqwqxqyqz[:,3]*=-1

        xyzqwqxqyqz=xyzqwqxqyqz[:,[0,2,1,3,4,6,5]]

        xyz=xyzqwqxqyqz[:,:3]
        # qwqxqyqz=xyzqwqxqyqz[:,3:]
        qxqyqzqw=xyzqwqxqyqz[:,[4,5,6,3]]

        rt_list=[]
        for i in range(49):
            rt_base_quest_2_part_quest = quaternion_matrix(qxqyqzqw[i,:])
            rt_base_quest_2_part_quest[:3,-1]=xyz[i,:]
            rt_list.append(rt_base_quest_2_part_quest)
        
        # print("append finish")
        
        # get finger
        # for quest3 start from 1
        # thumb0 3
        # thumb2 4
        # index 7
        # mid 10
        # ring 13
        # little 17
        rt_base_2_left_hand=rt_list[0]

        rt_base_2_left_fin_data_1=(rt_list[2]) # thumb knuckle
        rt_base_2_left_fin_data_2=(rt_list[3]) # thumb 2
        rt_base_2_left_fin_data_3=(rt_list[6]) # index
        rt_base_2_left_fin_data_4=(rt_list[9]) # mid
        rt_base_2_left_fin_data_5=(rt_list[12]) # ring
        rt_base_2_left_fin_data_6=(rt_list[16]) # little

        rt_left_hand_2_base=np.linalg.inv(rt_base_2_left_hand)

        left_fin_data_1=rt_left_hand_2_base @ rt_base_2_left_fin_data_1
        left_fin_data_2=rt_left_hand_2_base @ rt_base_2_left_fin_data_2
        left_fin_data_3=rt_left_hand_2_base @ rt_base_2_left_fin_data_3
        left_fin_data_4=rt_left_hand_2_base @ rt_base_2_left_fin_data_4
        left_fin_data_5=rt_left_hand_2_base @ rt_base_2_left_fin_data_5
        left_fin_data_6=rt_left_hand_2_base @ rt_base_2_left_fin_data_6

        left_fin_data_1_xyz = matrix3d_to_euler_angles_zyx(left_fin_data_1)
        left_fin_data_2_xyz = matrix3d_to_euler_angles_zyx(left_fin_data_2)
        left_fin_data_3_xyz = matrix3d_to_euler_angles_zyx(left_fin_data_3)
        left_fin_data_4_xyz = matrix3d_to_euler_angles_zyx(left_fin_data_4)
        left_fin_data_5_xyz = matrix3d_to_euler_angles_zyx(left_fin_data_5)
        left_fin_data_6_xyz = matrix3d_to_euler_angles_zyx(left_fin_data_6)

        left_finger_1 = (left_fin_data_1_xyz[1])/np.pi*180 # axiz 2 and  0 open -1 close
        left_finger_1 = lim_angle(left_finger_1)
        # -0.7 -- -0.2
        left_finger_2 = (left_fin_data_2_xyz[1] +0.7 )/(0.5)/np.pi*180 *2 # ?
        # print(left_finger_2)
        left_finger_2 = lim_angle(left_finger_2)

        left_finger_1=left_finger_2

        left_finger_3 = (left_fin_data_3_xyz[1])/np.pi*180# axiz 1 and  0 open 1 close
        left_finger_3 = lim_angle(left_finger_3)

        left_finger_4 = (left_fin_data_4_xyz[1])/np.pi*180# axiz 1 and  0 open 1 close
        left_finger_4 = lim_angle(left_finger_4)

        left_finger_5 = (left_fin_data_5_xyz[1])/np.pi*180# axiz 1 and  0 open 1 close
        left_finger_5 = lim_angle(left_finger_5)

        left_finger_6 = (left_fin_data_6_xyz[1])/np.pi*180# axiz 1 and  0 open 1 close
        left_finger_6 = lim_angle(left_finger_6)
        # print(left_fin_data_3_xyz,left_fin_data_4_xyz,left_fin_data_5_xyz,left_fin_data_6_xyz)
        # print(left_fin_data_2_xyz)
        # print(left_finger_1,left_finger_2,left_finger_3,left_finger_4,left_finger_5,left_finger_6)

        rt_base_2_right_hand=rt_list[24+0]

        rt_base_2_right_fin_data_1=(rt_list[24+2]) # thumb knuckle
        rt_base_2_right_fin_data_2=(rt_list[24+3]) # thumb 2
        rt_base_2_right_fin_data_3=(rt_list[24+6]) # index
        rt_base_2_right_fin_data_4=(rt_list[24+9]) # mid
        rt_base_2_right_fin_data_5=(rt_list[24+12]) # ring
        rt_base_2_right_fin_data_6=(rt_list[24+16]) # little

        rt_right_hand_2_base=np.linalg.inv(rt_base_2_right_hand)

        right_fin_data_1=rt_right_hand_2_base @ rt_base_2_right_fin_data_1
        right_fin_data_2=rt_right_hand_2_base @ rt_base_2_right_fin_data_2
        right_fin_data_3=rt_right_hand_2_base @ rt_base_2_right_fin_data_3
        right_fin_data_4=rt_right_hand_2_base @ rt_base_2_right_fin_data_4
        right_fin_data_5=rt_right_hand_2_base @ rt_base_2_right_fin_data_5
        right_fin_data_6=rt_right_hand_2_base @ rt_base_2_right_fin_data_6

        right_fin_data_1_xyz = matrix3d_to_euler_angles_zyx(right_fin_data_1)
        right_fin_data_2_xyz = matrix3d_to_euler_angles_zyx(right_fin_data_2)
        right_fin_data_3_xyz = matrix3d_to_euler_angles_zyx(right_fin_data_3)
        right_fin_data_4_xyz = matrix3d_to_euler_angles_zyx(right_fin_data_4)
        right_fin_data_5_xyz = matrix3d_to_euler_angles_zyx(right_fin_data_5)
        right_fin_data_6_xyz = matrix3d_to_euler_angles_zyx(right_fin_data_6)

        right_finger_1 = (right_fin_data_1_xyz[2]+0.65)/0.5*90 # axiz 2 and  0 open -1 close
        right_finger_1 = lim_angle(right_finger_1)

        
        right_finger_2 = (right_fin_data_2_xyz[1] +0.7 )/(0.5)/np.pi*180*2 # ?
        right_finger_2 = lim_angle(right_finger_2)

        right_finger_1=right_finger_2

        # print(right_fin_data_2_xyz)
        # print(right_finger_2)

        right_finger_3 = (right_fin_data_3_xyz[1])/np.pi*180# axiz 1 and  0 open 1 close
        right_finger_3 = lim_angle(right_finger_3)

        right_finger_4 = (right_fin_data_4_xyz[1])/np.pi*180# axiz 1 and  0 open 1 close
        right_finger_4 = lim_angle(right_finger_4)

        right_finger_5 = (right_fin_data_5_xyz[1])/np.pi*180# axiz 1 and  0 open 1 close
        right_finger_5 = lim_angle(right_finger_5)

        right_finger_6 = (right_fin_data_6_xyz[1])/np.pi*180# axiz 1 and  0 open 1 close
        right_finger_6 = lim_angle(right_finger_6)

        # print("right hand",right_fin_data_3_xyz,right_fin_data_4_xyz,right_fin_data_5_xyz,right_fin_data_6_xyz)
        # print(right_finger_1,right_finger_2,right_finger_3,right_finger_4,right_finger_5,right_finger_6)



        # tf_msg = create_tf_xyz_quat(*xyzqwqxqyqz[0, :], 'map', 'left')
        # self.tf_broadcaster.sendTransform(tf_msg)
        # tf_msg = create_tf_xyz_quat(*xyzqwqxqyqz[24, :], 'map', 'right')
        # self.tf_broadcaster.sendTransform(tf_msg)
        # tf_msg = create_tf_xyz_quat(*xyzqwqxqyqz[48, :], 'map', 'head')
        # self.tf_broadcaster.sendTransform(tf_msg)

        xyz=xyzqwqxqyqz[:,:3]

        qxqyqzqw=xyzqwqxqyqz[:,[4,5,6,3]]

        rt_l_hand_quest_2_l_hand_robot=np.eye(4)
        # rt_l_hand_quest_2_l_hand_robot[:3,:3]=rpy2rotation_matrix(np.deg2rad(180),0,np.deg2rad(0))
        rt_l_hand_quest_2_l_hand_robot[:3,:3]=rpy2rotation_matrix(np.deg2rad(90),0,np.deg2rad(0))

        rt_r_hand_quest_2_r_hand_robot=np.eye(4)
        # rt_r_hand_quest_2_r_hand_robot[:3,:3]=rpy2rotation_matrix(np.deg2rad(90),0,np.deg2rad(180))
        # rt_r_hand_quest_2_r_hand_robot[:3,:3]=rpy2rotation_matrix(np.deg2rad(0),np.deg2rad(180),0)
        rt_r_hand_quest_2_r_hand_robot[:3,:3]=rpy2rotation_matrix(np.deg2rad(-90),np.deg2rad(180),0)

        # # left hand
        rt_base_quest_2_l_hand_quest = quaternion_matrix(qxqyqzqw[0,:])
        rt_base_quest_2_l_hand_quest[:3,-1]=xyz[0,:]



        rt_base_quest_2_l_hand_robot =  rt_base_quest_2_l_hand_quest @ rt_l_hand_quest_2_l_hand_robot
        qxyzw_l_hand_robot = quaternion_from_matrix(rt_base_quest_2_l_hand_robot)
        xyz_l_hand_robot=rt_base_quest_2_l_hand_robot[:3,-1]

        # # right hand
        rt_base_quest_2_r_hand_quest = quaternion_matrix(qxqyqzqw[24,:])
        rt_base_quest_2_r_hand_quest[:3,-1]=xyz[24,:]

        rt_base_quest_2_r_hand_robot =  rt_base_quest_2_r_hand_quest @ rt_r_hand_quest_2_r_hand_robot
        qxyzw_r_hand_robot = quaternion_from_matrix(rt_base_quest_2_r_hand_robot)
        xyz_r_hand_robot=rt_base_quest_2_r_hand_robot[:3,-1]

        # head
        rt_base_quest_2_head_quest = quaternion_matrix(qxqyqzqw[48,:])
        rt_base_quest_2_head_quest[:3,-1]=xyz[48,:]
        
        # finish

        head_data=rt_base_quest_2_head_quest
        left_data=rt_base_quest_2_l_hand_robot
        right_data=rt_base_quest_2_r_hand_robot


        left_data_robot,right_data_robot,zyx_left_robot,zyx_right_robot =get_hand_tf_quest_fixed_head(
            head_data,left_data,right_data)
        left_wrist_t=left_data_robot[:3,-1]*1000
        right_wrist_t=right_data_robot[:3,-1]*1000
        # print("left_theta_rad",self.left_theta_rad)
        # print(left_data,left_wrist_t)

        return [zyx_left_robot,
                left_wrist_t,
                zyx_right_robot,
                right_wrist_t,
                left_finger_1,left_finger_2,left_finger_3,left_finger_4,left_finger_5,left_finger_6,
                right_finger_1,right_finger_2,right_finger_3,right_finger_4,right_finger_5,right_finger_6,]