import numpy as np
from math import *
import cv2


def angle2rotation(x, y, z):
    Rx = np.array([[1, 0, 0], [0, cos(x), -sin(x)], [0, sin(x), cos(x)]])
    Ry = np.array([[cos(y), 0, sin(y)], [0, 1, 0], [-sin(y), 0, cos(y)]])
    Rz = np.array([[cos(z), -sin(z), 0], [sin(z), cos(z), 0], [0, 0, 1]])
    R = Rz @ Ry @ Rx
    return R


def gen_RT(r, p, y, tx, ty, tz):
    thetaX = r  #/ 180 * pi
    thetaY = p  #/ 180 * pi
    thetaZ = y  #/ 180 * pi
    R_gripper2base = cv2.Rodrigues(np.array([[r], [p], [y]]))[0]  # angle2rotation(thetaX, thetaY, thetaZ)
    T_gripper2base = np.array([[tx], [ty], [tz]])
    Matrix_gripper2base = np.column_stack([R_gripper2base, T_gripper2base])
    Matrix_gripper2base = np.row_stack((Matrix_gripper2base, np.array([0, 0, 0, 1])))
    R_gripper2base = Matrix_gripper2base[:3, :3]
    T_gripper2base = Matrix_gripper2base[:3, 3].reshape((3, 1))
    RT_gripper2base = np.column_stack((R_gripper2base, T_gripper2base))
    RT_gripper2base = np.row_stack((RT_gripper2base, np.array([0, 0, 0, 1])))
    return RT_gripper2base


def find_axis_angle(v1, v2):
    '''
    rotate a to b

    :param v1:
    :param v2:
    :return:
    '''
    v1_normalized = v1 / np.linalg.norm(v1)
    v2_normalized = v2 / np.linalg.norm(v2)
    rotation_axis = np.cross(v1_normalized, v2_normalized)
    rotation_axis_normalized = rotation_axis / np.linalg.norm(rotation_axis)
    rotation_cos_angle_unsafe = np.dot(v1_normalized, v2_normalized)
    rotation_cos_angle = np.clip(rotation_cos_angle_unsafe, -1, 1)
    rotation_angle_rad = np.arccos(rotation_cos_angle)
    return rotation_axis_normalized, rotation_angle_rad


def find_align_rt(v, v_reference):
    v_normalized = v / np.linalg.norm(v)
    rotation_axis = np.cross(v_normalized, v_reference)
    rotation_axis_normalized = rotation_axis / np.linalg.norm(rotation_axis)
    rotation_cos_angle_unsafe = np.dot(v_normalized, v_reference)
    rotation_cos_angle = np.clip(rotation_cos_angle_unsafe, -1, 1)
    rotation_angle_rad = np.arccos(rotation_cos_angle)
    r, _ = cv2.Rodrigues(rotation_axis_normalized * rotation_angle_rad)
    rt = np.eye(4)
    rt[:3, :3] = r
    return rt


def rpy2rotation_matrix(r, p, y):
    """
    new O_b calc by fix angle rotation in O_a, get a_T_b
    :param r:  x
    :param p:  y
    :param y:  z
    :return:
    """
    axis_x = np.array([1., 0, 0])
    axis_y = np.array([0, 1., 0])
    axis_z = np.array([0, 0, 1.])
    r_x, _ = cv2.Rodrigues(axis_x * r)
    r_y, _ = cv2.Rodrigues(axis_y * p)
    r_z, _ = cv2.Rodrigues(axis_z * y)
    rotation_matrix = r_z @ r_y @ r_x
    return rotation_matrix


def euler2rotation_matrix(x, y, z):
    """
    not fix axis, z-y-x rotation
    :param x:
    :param y:
    :param z:
    :return:
    """
    axis_x = np.array([1., 0, 0])
    axis_y = np.array([0, 1., 0])
    axis_z = np.array([0, 0, 1.])
    r_x, _ = cv2.Rodrigues(axis_x * x)
    r_y, _ = cv2.Rodrigues(axis_y * y)
    r_z, _ = cv2.Rodrigues(axis_z * z)
    rotation_matrix = r_z @ r_y @ r_x
    return rotation_matrix


def vector2rpy(vector):
    '''
    rotate from (1,0,0)
    :param vector:
    :return:
    '''
    unit_vector = vector / np.linalg.norm(vector)

    # Extract the components of the vector
    x, y, z = unit_vector

    # Calculate yaw (ψ) - rotation around the Z-axis
    yaw = np.arctan2(y, x)

    # Calculate pitch (θ) - rotation around the Y-axis
    pitch = np.arcsin(-z)

    # Roll (φ) is not defined by a single vector, usually calculated from rotation matrices or multiple vectors
    roll = 0.0  # Typically, roll is set to 0 or calculated from additional information

    return roll, pitch, yaw


def vector2rpy_shoudler(vector):
    '''
    rotate from (1,0,0)
    :param vector:
    :return:
    '''
    unit_vector = vector / np.linalg.norm(vector)

    # Extract the components of the vector
    x, y, z = unit_vector

    # Calculate yaw (ψ) - rotation around the Z-axis
    # v_xz=np.array([x,0,z])
    # axis,angle = find_axis_angle(v_xz,unit_vector)
    # roll=angle
    roll = np.arctan2(y, z)

    # Calculate pitch (θ) - rotation around the Y-axis
    pitch = np.arcsin(-x)

    # Roll (φ) is not defined by a single vector, usually calculated from rotation matrices or multiple vectors
    # roll = 0.0  # Typically, roll is set to 0 or calculated from additional information

    return roll, pitch, 0


def rotation_matrix_to_rpy(R):
    """
    Convert a rotation matrix to roll, pitch, and yaw angles.

    Parameters:
    - R: A 3x3 rotation matrix.

    Returns:
    - roll: The roll angle in degrees.
    - pitch: The pitch angle in degrees.
    - yaw: The yaw angle in degrees.
    """
    # Ensure the rotation matrix is a valid 3x3 matrix
    assert R.shape == (3, 3), "Rotation matrix must be 3x3"

    # Extract the rotation angles
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)

    singular = sy < 1e-6

    if not singular:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0

    # Convert the angles from radians to degrees
    roll = np.degrees(roll)
    pitch = np.degrees(pitch)
    yaw = np.degrees(yaw)

    return roll, pitch, yaw

def rotation_matrix_to_euler_angles(R):
    """
    Convert a rotation matrix to Euler angles (roll, pitch, yaw) using ZYX order.

    Parameters:
    - R: A 3x3 rotation matrix.

    Returns:
    - roll: Rotation angle around the X-axis in degrees.
    - pitch: Rotation angle around the Y-axis in degrees.
    - yaw: Rotation angle around the Z-axis in degrees.
    """
    assert R.shape == (3, 3), "Rotation matrix must be a 3x3 array"

    # Calculate the Euler angles from the rotation matrix
    if R[2, 0] < 1:
        if R[2, 0] > -1:
            pitch = np.arcsin(R[2, 0])
            yaw = np.arctan2(-R[2, 1], R[2, 2])
            roll = np.arctan2(-R[1, 0], R[0, 0])
        else:
            # R[2, 0] == -1
            pitch = -np.pi / 2
            yaw = -np.arctan2(R[1, 2], R[1, 1])
            roll = 0
    else:
        # R[2, 0] == 1
        pitch = np.pi / 2
        yaw = np.arctan2(R[1, 2], R[1, 1])
        roll = 0

    # Convert from radians to degrees
    roll = np.degrees(roll)
    pitch = np.degrees(pitch)
    yaw = np.degrees(yaw)

    return roll, pitch, yaw


def quaternion_to_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * y ** 2 - 2 * z ** 2, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
        [2 * x * y + 2 * z * w, 1 - 2 * x ** 2 - 2 * z ** 2, 2 * y * z - 2 * x * w],
        [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x ** 2 - 2 * y ** 2]
    ])


def xyz_quaternion_to_homogeneous(x, y, z,qw, qx, qy, qz):
    translation = np.array([x, y, z])
    quaternion = [qw, qx, qy, qz]
    rotation_matrix = quaternion_to_matrix(quaternion)

    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = rotation_matrix
    transformation_matrix[:3, 3] = translation

    return transformation_matrix

def calc_dist(a,b):
    return np.linalg.norm(a-b)

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
