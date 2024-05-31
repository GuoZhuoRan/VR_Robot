# -*- coding: utf-8 -*-

import os
import cv2
import xlrd2
from math import *
import numpy as np

# p[321.501 242.543]  f[606.25 605.65]
class Calibration:
    def __init__(self):
        #内参矩阵 K
        self.K = np.array([[431.42334, 0.00000000e+00,308.17798],
                           [0.00000000e+00, 431.01035, 240.87384],
                           [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]], dtype=np.float64)
        #畸变系数 D
        self.distortion = np.array([[-0.05460, 0.05655, 0.00090, 0.00092, -0.01838 ]])
        #棋盘格尺寸和角点个数 横向是x target_cell_size尺寸是mm
        self.target_x_number = 13
        self.target_y_number = 9
        self.target_cell_size = 18

    def angle2rotation(self, x, y, z):
        Rx = np.array([[1, 0, 0], [0, cos(x), -sin(x)], [0, sin(x), cos(x)]])
        Ry = np.array([[cos(y), 0, sin(y)], [0, 1, 0], [-sin(y), 0, cos(y)]])
        Rz = np.array([[cos(z), -sin(z), 0], [sin(z), cos(z), 0], [0, 0, 1]])
        R = Rz @ Ry @ Rx
        return R
    
    def matrix_to_translation_quaternion(self,matrix):
    # 提取旋转矩阵和平移向量
        R = matrix[:3, :3]
        T = matrix[:3, 3]

    # 计算四元数
        qw = np.sqrt(1 + R[0,0] + R[1,1] + R[2,2]) / 2
        qx = (R[2,1] - R[1,2]) / (4 * qw)
        qy = (R[0,2] - R[2,0]) / (4 * qw)
        qz = (R[1,0] - R[0,1]) / (4 * qw)

        return T, qx, qy, qz, qw

    def gripper2base(self, x, y, z, tx, ty, tz):
        thetaX = x #/ 180 * pi
        thetaY = y #/ 180 * pi
        thetaZ = z #/ 180 * pi
        R_gripper2base = self.angle2rotation(thetaX, thetaY, thetaZ)
        T_gripper2base = np.array([[tx], [ty], [tz]])
        Matrix_gripper2base = np.column_stack([R_gripper2base, T_gripper2base])
        Matrix_gripper2base = np.row_stack((Matrix_gripper2base, np.array([0, 0, 0, 1])))
        R_gripper2base = Matrix_gripper2base[:3, :3]
        T_gripper2base = Matrix_gripper2base[:3, 3].reshape((3, 1))
        return R_gripper2base, T_gripper2base

    def target2camera(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (self.target_x_number, self.target_y_number), None)
        corner_points = np.zeros((2, corners.shape[0]), dtype=np.float64)
        for i in range(corners.shape[0]):
            corner_points[:, i] = corners[i, 0, :]
        object_points = np.zeros((3, self.target_x_number * self.target_y_number), dtype=np.float64)
        count = 0
        for i in range(self.target_y_number):
            for j in range(self.target_x_number):
                object_points[:2, count] = np.array(
                    [(self.target_x_number - j - 1) * self.target_cell_size,
                     (self.target_y_number - i - 1) * self.target_cell_size])
                count += 1
        retval, rvec, tvec = cv2.solvePnP(object_points.T, corner_points.T, self.K, distCoeffs=self.distortion)
        Matrix_target2camera = np.column_stack(((cv2.Rodrigues(rvec))[0], tvec))
        Matrix_target2camera = np.row_stack((Matrix_target2camera, np.array([0, 0, 0, 1])))
        R_target2camera = Matrix_target2camera[:3, :3]
        T_target2camera = Matrix_target2camera[:3, 3].reshape((3, 1))
        return R_target2camera, T_target2camera

    def process(self, img_path, pose_path):
        image_list = []
        for root, dirs, files in os.walk(img_path):
            if files:
                for file in files:
                    image_name = os.path.join(root, file)
                    image_list.append(image_name)
        R_target2camera_list = []
        T_target2camera_list = []
        for img_path in image_list:
            img = cv2.imread(img_path)
            R_target2camera, T_target2camera = self.target2camera(img)
            R_target2camera_list.append(R_target2camera)
            T_target2camera_list.append(T_target2camera)
        R_gripper2base_list = []
        T_gripper2base_list = []
        data = xlrd2.open_workbook(pose_path)
        table = data.sheets()[0]
        for row in range(table.nrows):
            x = table.cell_value(row, 3)
            y = table.cell_value(row, 4)
            z = table.cell_value(row, 5)
            tx = table.cell_value(row, 0)
            ty = table.cell_value(row, 1)
            tz = table.cell_value(row, 2)
            R_gripper2base, T_gripper2base = self.gripper2base(x, y, z, tx, ty, tz)
            R_gripper2base_list.append(R_gripper2base)
            T_gripper2base_list.append(T_gripper2base)
        R_camera2base, T_camera2base = cv2.calibrateHandEye(R_gripper2base_list, T_gripper2base_list,
                                                            R_target2camera_list, T_target2camera_list)
        return R_camera2base, T_camera2base, R_gripper2base_list, T_gripper2base_list, R_target2camera_list, T_target2camera_list

    def check_result(self, R_cb, T_cb, R_gb, T_gb, R_tc, T_tc):
        for i in range(len(R_gb)):
            RT_gripper2base = np.column_stack((R_gb[i], T_gb[i]))
            RT_gripper2base = np.row_stack((RT_gripper2base, np.array([0, 0, 0, 1])))
            # print(RT_gripper2base)
            #手眼矩阵部分
            RT_camera_to_gripper = np.column_stack((R_cb, T_cb))
            RT_camera_to_gripper = np.row_stack((RT_camera_to_gripper, np.array([0, 0, 0, 1])))
            print("手眼矩阵为",RT_camera_to_gripper)#这个就是手眼矩阵
            T_, qx, qy, qz, qw=self.matrix_to_translation_quaternion(RT_camera_to_gripper)
            print("位移和四元数为:",T_)
            print(qx,qy,qz,qw)
            
            RT_target_to_camera = np.column_stack((R_tc[i], T_tc[i]))
            RT_target_to_camera = np.row_stack((RT_target_to_camera, np.array([0, 0, 0, 1])))
            # print(RT_target_to_camera)
            RT_target_to_base = RT_gripper2base @ RT_camera_to_gripper @ RT_target_to_camera
            
            print("第{}次验证结果为:".format(i))
            print(RT_target_to_base)
            print('')


if __name__ == "__main__":
#存放的图片和excel文件地址
    image_path = "/home/jyw/Downloads/data_handeye/img"
    pose_path = "/home/jyw/Downloads/data_handeye/pose.xlsx"
    calibrator = Calibration()
    R_cb, T_cb, R_gb, T_gb, R_tc, T_tc = calibrator.process(image_path, pose_path)
    calibrator.check_result(R_cb, T_cb, R_gb, T_gb, R_tc, T_tc)
