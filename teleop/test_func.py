from fourier_grx_client import *
# from fourier_grx.sdk import ControlGroup, RobotClient
# import cv2
import numpy as np
# import pyrealsense2 as rs
import time
import os
import sys
import math
# sys.path.append(os.path.join(os.path.dirname(__file__), "../"))
from rich import inspect
from rich.console import Console
console = Console()
log = console.log
print = console.print
from test_utils import *

FREQUENCY = 400

def dynamic_move(client: RobotClient, start, end, totalTime):
    p0 = start
    p1 = end
    p0_dot = np.zeros(7)
    p0_dotdot = np.zeros(7) 
    p1_dot = np.zeros(7) 
    p1_dotdot = np.zeros(7)



    for i in range(int(totalTime * FREQUENCY)):
        currenttime = i / FREQUENCY
        pd, _ , _ = FifthPoly(p0, p0_dot, p0_dotdot, p1, p1_dot, p1_dotdot, totalTime, currenttime)
        time.sleep(1 / FREQUENCY)
        client.move_joints(ControlGroup.RIGHT_ARM, pd, 0.0)

def dynamic_move_left(client: RobotClient, start, end, totalTime):
    p0 = start
    p1 = end
    p0_dot = np.zeros(7)
    p0_dotdot = np.zeros(7) 
    p1_dot = np.zeros(7) 
    p1_dotdot = np.zeros(7)



    for i in range(int(totalTime * FREQUENCY)):
        currenttime = i / FREQUENCY
        pd, _ , _ = FifthPoly(p0, p0_dot, p0_dotdot, p1, p1_dot, p1_dotdot, totalTime, currenttime)
        time.sleep(1 / FREQUENCY)
        client.move_joints(ControlGroup.LEFT_ARM, pd, 0.0)


def MOVE_to_START(client: RobotClient):

    # ctx = rs.context()
    # list = ctx.query_devices()
    # if len(list) == 0:
    #     raise RuntimeError("No device detected. Is it plugged in?")

    # R_cam2base = np.eye(3, dtype=np.float64)
    # T_cam2base = np.zeros((3, 1), dtype=np.float64)
    # calibrate_result = np.load('../Handeyecalibrate/calibrate_result.npy')
    # R_cam2base = calibrate_result[:3, :3]
    # T_cam2base = calibrate_result[:3, 3].reshape(3, 1)
    # print("R_cam2base: ", R_cam2base)
    # print("T_cam2base: ", T_cam2base)

    q_pos_start3 = np.array([70.0, -5.0, 0.0, -120.0, 0.0, -0.0, 0.0])
    start_pos = client.joint_positions.copy()[25:] 
    dynamic_move(client, start_pos, q_pos_start3, 4.0)
    # client.move_joints(ControlGroup.RIGHT_ARM, q_pos_start3, 3.0, blocking=True)

    q_pos_start4 = np.array([0.0, -5.0, 0.0, -110.0,  0.0, -0.0, 0.0])
    start_pos = client.joint_positions.copy()[25:] 
    dynamic_move(client, start_pos, q_pos_start4, 4.0)
    # client.move_joints(ControlGroup.RIGHT_ARM, q_pos_start4, 3.0, blocking=True)
