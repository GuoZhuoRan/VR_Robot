import numpy as np


def calculate_normal_vector(A, B, C):
    """
    Calculate the normal vector of a plane defined by three points A, B, and C.

    Args:
    A, B, C (tuple): Coordinates of the points A, B, and C (x, y, z).

    Returns:
    numpy.ndarray: The normal vector of the plane.
    """
    # Convert points to numpy arrays
    # A = np.array(A)
    # B = np.array(B)
    # C = np.array(C)

    # Calculate vectors AB and AC
    AB = B - A
    AC = C - A
    # print(AB)
    # print(AC)
    # Calculate the cross product of AB and AC
    normal_vector = np.cross(AB, AC)

    # print(normal_vector)

    return normal_vector


def angle_between_planes(normal_vector1, normal_vector2):
    """
    Calculate the angle between two planes given their normal vectors.

    Args:
    normal_vector1, normal_vector2 (numpy.ndarray): Normal vectors of the two planes.

    Returns:
    float: The angle between the two planes in degrees.
    """
    # Normalize the vectors
    # print(np.linalg.norm(normal_vector1))
    normal_vector1 = normal_vector1 / np.linalg.norm(normal_vector1)
    normal_vector2 = normal_vector2 / np.linalg.norm(normal_vector2)

    # Calculate the dot product of the vectors
    dot_product = np.dot(normal_vector1, normal_vector2)

    # Calculate the angle in radians
    angle_rad = np.arccos(dot_product)

    # Convert the angle to degrees
    # angle_deg = np.degrees(angle_rad)

    return angle_rad


def angle_bixing_left(jian, zhou, wan):
    di = np.array([0, 0, 0])
    di[0] = -100.0
    di[1] = jian[1]
    di[2] = jian[2]
    vector1 = calculate_normal_vector(jian, wan, di)
    vector2 = calculate_normal_vector(jian, wan, zhou)
    # print(jian)
    # print(zhou)
    # print(wan)
    angless = angle_between_planes(vector1, vector2)
    return angless

def angle_bixing_right(jian, zhou, wan):
    di = np.array([0, 0, 0])
    di[0] = -100
    di[1] = jian[1]
    di[2] = jian[2]
    vector1 = calculate_normal_vector(jian, wan, di)
    vector2 = calculate_normal_vector(jian, wan, zhou)
    angless = angle_between_planes(vector1, vector2)
    return angless

#
# # Example usage
# # Replace these vectors with your own normal vectors
# # normal_vector1 = np.array([0, -150, 0])
# # normal_vector2 = np.array([0, -100, -50])
# b_shoulder_left = np.array([0, 0, 0])
# c_elbow_left = np.array([-10, 10, 10])
# a_hand_left = np.array([0, 20, 0])
#
# angle_left = angle_bixing_left(b_shoulder_left, c_elbow_left, a_hand_left)
# angle_deg_left = angle_left
# # angle_deg_left = np.degrees(angle_left)
# print("Angle between planes:", angle_deg_left)
#
# b_shoulder_right = np.array([0, 0, 0])
# c_elbow_right = np.array([-10, -10, 10])
# a_hand_right = np.array([0, -20, 0])
#
# angle_right = angle_bixing_left(b_shoulder_right, c_elbow_right, a_hand_right)
# angle_deg_right = angle_right
# # angle_deg_right = np.degrees(angle_right)
# print("Angle between planes:", angle_deg_right)