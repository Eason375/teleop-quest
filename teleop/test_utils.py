from fourier_grx_client import *
# from fourier_grx.sdk import ControlGroup, RobotClient
# import cv2
import numpy as np
import time
import os
import sys
import math
sys.path.append(os.path.join(os.path.dirname(__file__), "../"))
# from kinematics.gr1.fi_end_effector_gr1 import EndEffectorGR1
from rich import inspect
from rich.console import Console
console = Console()
log = console.log
print = console.print

# 定义相机参数

def FifthPoly(p0, p0_dot, p0_dotdot, p1, p1_dot, p1_dotdot, totalTime, currenttime):
    t = currenttime
    time = totalTime
    if t < totalTime:
        A = np.zeros((6, 6))
        A[0, 0] = 1
        A[1, 1] = 1
        A[2, 2] = 0.5
        A[3, 0] = -10 / (time ** 3)
        A[3, 1] = -6 / (time ** 2)
        A[3, 2] = -3 / (2 * time)
        A[3, 3] = 10 / (time ** 3)
        A[3, 4] = -4 / (time ** 2)
        A[3, 5] = 1 / (2 * time)
        A[4, 0] = 15 / (time ** 4)
        A[4, 1] = 8 / (time ** 3)
        A[4, 2] = 3 / (2 * (time ** 2))
        A[4, 3] = -15 / (time ** 4)
        A[4, 4] = 7 / (time ** 3)
        A[4, 5] = -1 / (time ** 2)
        A[5, 0] = -6 / (time ** 5)
        A[5, 1] = -3 / (time ** 4)
        A[5, 2] = -1 / (2 * (time ** 3))
        A[5, 3] = 6 / (time ** 5)
        A[5, 4] = -3 / (time ** 4)
        A[5, 5] = 1 / (2 * (time ** 3))

        x0 = np.array([p0, p0_dot, p0_dotdot, p1, p1_dot, p1_dotdot])
        a = np.dot(A, x0)

        pd = a[0] + a[1] * t + a[2] * t * t + a[3] * t * t * t + a[4] * t * t * t * t + a[5] * t * t * t * t * t
        pd_dot = a[1] + 2 * a[2] * t + 3 * a[3] * t * t + 4 * a[4] * t * t * t + 5 * a[5] * t * t * t * t
        pd_dotdot = 2 * a[2] + 6 * a[3] * t + 12 * a[4] * t * t + 20 * a[5] * t * t * t
    else:
        pd = p1
        pd_dot = p1_dot
        pd_dotdot = p1_dotdot

    return pd, pd_dot, pd_dotdot

def R_T2RT(R, T):
    RT = np.hstack((R, T))
    return RT

def RT2R_T(RT):
    R = RT[:3, :3]
    T = RT[:3, 3]
    return R, T

def print_matrix(matrix):
    print(matrix)

def eulerAngleToRotatedMatrix(eulerAngle, seq):
    rx, ry, rz = eulerAngle
    xs = math.sin(rx)
    xc = math.cos(rx)
    ys = math.sin(ry)
    yc = math.cos(ry)
    zs = math.sin(rz)
    zc = math.cos(rz)

    rotX = np.array([[1, 0, 0],
                     [0, xc, -xs],
                     [0, xs, xc]])

    rotY = np.array([[yc, 0, ys],
                     [0, 1, 0],
                     [-ys, 0, yc]])

    rotZ = np.array([[zc, -zs, 0],
                     [zs, zc, 0],
                     [0, 0, 1]])

    if seq == "zyx":
        rotMat = rotX @ rotY @ rotZ
    elif seq == "yzx":
        rotMat = rotX @ rotZ @ rotY
    elif seq == "zxy":
        rotMat = rotY @ rotX @ rotZ
    elif seq == "xzy":
        rotMat = rotY @ rotZ @ rotX
    elif seq == "yxz":
        rotMat = rotZ @ rotX @ rotY
    elif seq == "xyz":
        rotMat = rotZ @ rotY @ rotX
    return rotMat


def rotation_matrix_to_rpy(R):
    """
    将旋转矩阵R转换为RPY角Roll, Pitch, Yaw以度为单位
    
    参数:
    R -- 3x3的旋转矩阵
    
    返回:
    rpy -- 包含Roll, Pitch, Yaw的元组 度
    """
    # 确保R是numpy数组
    R = np.asarray(R)
    
    # 计算RPY角
    # 注意：这里使用的顺序是'zyx'，即先绕Z轴旋转Yaw，再绕Y轴旋转Pitch，最后绕X轴旋转Roll
    # 但由于我们的输入是旋转矩阵，而旋转矩阵是顺序相反的（先绕X轴，再绕Y轴，最后绕Z轴）
    # 所以我们实际上是在从旋转矩阵中提取'xyz'顺序的角，但输出时标记为'rpy'（Roll, Pitch, Yaw）
    # 这意味着返回的Roll实际上是绕x轴的旋转，Pitch是绕y轴的旋转，Yaw是绕z轴的旋转（尽管我们称之为Yaw）
    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6

    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0

    # 将弧度转换为度
    rpy = np.degrees([x, y, z])

    return rpy


def wxyzToRotatedMatrix(wxyz, seq):
    x, y, z, w = wxyz
    x2 = x * x
    y2 = y * y
    z2 = z * z
    w2 = w * w
    rotMat = np.array([[1 - 2 * (y2 + z2), 2 * x * y - 2 * w * z, 2 * x * z + 2 * w * y],
                       [2 * x * y + 2 * w * z, 1 - 2 * (x2 + z2), 2 * y * z - 2 * w * x],
                       [2 * x * z - 2 * w * y, 2 * y * z + 2 * w * x, 1 - 2 * (x2 + y2)]])
    return rotMat

# def attitudeVectorToMatrix(m, useQuaternion, seq):
#     if m.shape[1] == 1:
#         m = m.T
#     tmp = np.eye(4, dtype=np.float64)
#     if useQuaternion:
#         quaternionVec = m[3:0, 4:1]
#         # quaternionToRotatedMatrix(quaternionVec).copyTo(tmp[0:3, 0:3])
#         # print(norm(quaternionVec))
#     else:
#         if m.shape[1] == 6:
#             rotVec = m[0, 3:6]
#         else:
#             rotVec = m[0, 3:10]
#         if seq == "":
#             cv2.Rodrigues(rotVec, tmp[0:3, 0:3])
#         else:
#             eulerAngleToRotatedMatrix(rotVec, seq).copyTo(tmp[0:3, 0:3])
#     tmp[0:3, 3:4] = m[0, 0:3].T.reshape(3,1)
    return tmp

def calculateBasePosition(R_cam2base, T_cam2base, CAMERA_xyz):
    BASE_xyz = R_cam2base @ CAMERA_xyz + T_cam2base
    return BASE_xyz


def calculate_base_position(R_cam2base, T_cam2base, CAMERA_rot, CAMERA_xyz):
    BASE_xyz = R_cam2base @ CAMERA_xyz + T_cam2base
    Base_rot = R_cam2base @ CAMERA_rot
    return Base_rot, BASE_xyz

def calculate_rgb_position(R_depth2cam, T_depth2cam, Depth_rot, Depth_xyz):
    CAMERA_xyz = R_depth2cam @ Depth_xyz + T_depth2cam
    CAMERA_rot = R_depth2cam @ Depth_rot
    return CAMERA_rot, CAMERA_xyz

def TRANS(hand_r, hand_t):
    # 从文件中读取相机标定结果,相机坐标系到base坐标系
    calibrate_result = np.load('../Handeyecalibrate/calibrate_result.npy')
    R_cam2base = calibrate_result[:3, :3]
    T_cam2base = calibrate_result[:3, 3].reshape(3, 1)
    print("T_cam2base", T_cam2base)

    # realsense depth相机坐标系到相机坐标系 固定
    R_depth2cam = np.array([
        [0.99999934, -0.00049277581, -0.0010182461],
        [0.00049642508, 0.99999344, 0.0035867561],
        [0.001016472, -0.0035872592, 0.99999303]
    ])
    T_depth2cam = np.array([
        [0.014890525],
        [-3.2838438e-05],
        [0.00037542282]
    ])

    # 若已知depth坐标系的坐标值, 可得到相机坐标系下的坐标值和 GR-1 base坐标系下的坐标值

    # 1 计算depth坐标系下 到rgb相机坐标系下的位置
    Depth_xyz = hand_t
    Depth_rot = hand_r
    CAMERA_rot, CAMERA_xyz = calculate_rgb_position(R_depth2cam, T_depth2cam, Depth_rot, Depth_xyz)
    print("CAMERA_xyz :", CAMERA_xyz)

    # 2 通过给出rgb相机坐标系(color_coordinate)下的坐标 计算该坐标i在基坐标系下的位置
    Base_rot, BASE_xyz = calculate_base_position(R_cam2base, T_cam2base, CAMERA_rot, CAMERA_xyz)
    print("BASE_xyz :", BASE_xyz)
    return Base_rot, BASE_xyz