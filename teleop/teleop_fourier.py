import math
import numpy as np
import torch
import cv2
from pathlib import Path
import yaml
from multiprocessing import shared_memory, Queue, Event
import time
from pytransform3d import rotations
import pyzed.sl as sl
from TeleVision import OpenTeleVision
from Preprocessor import VuerPreprocessor
from constants_vuer import tip_indices
from dex_retargeting.retargeting_config import RetargetingConfig
import asyncio
import websockets
import json
class VuerTeleop:
    def __init__(self, config_file_path):
        self.resolution = (720, 1280)
        self.crop_size_w = 1
        self.crop_size_h = 0
        self.resolution_cropped = (self.resolution[0]-self.crop_size_h, self.resolution[1]-2*self.crop_size_w)

        self.img_shape = (self.resolution_cropped[0], 2 * self.resolution_cropped[1], 3)
        self.img_height, self.img_width = self.resolution_cropped[:2]

        self.shm = shared_memory.SharedMemory(create=True, size=np.prod(self.img_shape) * np.uint8().itemsize)
        self.img_array = np.ndarray((self.img_shape[0], self.img_shape[1], 3), dtype=np.uint8, buffer=self.shm.buf)
        image_queue = Queue()
        toggle_streaming = Event()
        self.tv = OpenTeleVision(self.resolution_cropped, self.shm.name, image_queue, toggle_streaming, ngrok=True)
        self.processor = VuerPreprocessor()

        RetargetingConfig.set_default_urdf_dir('../assets')
        with Path(config_file_path).open('r') as f:
            cfg = yaml.safe_load(f)
        left_retargeting_config = RetargetingConfig.from_dict(cfg['left'])
        right_retargeting_config = RetargetingConfig.from_dict(cfg['right'])
        self.left_retargeting = left_retargeting_config.build()
        self.right_retargeting = right_retargeting_config.build()

    # def step(self):
    #     head_mat, left_wrist_mat, right_wrist_mat, left_hand_mat, right_hand_mat = self.processor.process(self.tv)

    #     head_rmat = head_mat[:3, :3]

    #     left_pose = np.concatenate([left_wrist_mat[:3, 3] + np.array([-0.6, 0, 1.6]),
    #                                 rotations.quaternion_from_matrix(left_wrist_mat[:3, :3])[[1, 2, 3, 0]]])
    #     right_pose = np.concatenate([right_wrist_mat[:3, 3] + np.array([-0.6, 0, 1.6]),
    #                                  rotations.quaternion_from_matrix(right_wrist_mat[:3, :3])[[1, 2, 3, 0]]])
    #     left_qpos = self.left_retargeting.retarget(left_hand_mat[tip_indices])[[4, 5, 6, 7, 10, 11, 8, 9, 0, 1, 2, 3]]
    #     right_qpos = self.right_retargeting.retarget(right_hand_mat[tip_indices])[[4, 5, 6, 7, 10, 11, 8, 9, 0, 1, 2, 3]]

    #     return head_rmat, left_pose, right_pose, left_qpos, right_qpos
    
    def step(self):
        head_mat, left_wrist_mat, right_wrist_mat, left_hand_mat, right_hand_mat = self.processor.process(self.tv)
        head_rmat = head_mat[:3, :3]

        left_wrist_mat[2, 3] +=0.40
        right_wrist_mat[2,3] +=0.40
        left_wrist_mat[0, 3] +=0.15
        right_wrist_mat[0,3] +=0.15

        left_qpos = self.left_retargeting.retarget(left_hand_mat[tip_indices])[[4, 5, 6, 7, 10, 11, 8, 9, 0, 1, 2, 3]]
        right_qpos = self.right_retargeting.retarget(right_hand_mat[tip_indices])[[4, 5, 6, 7, 10, 11, 8, 9, 0, 1, 2, 3]]

        return head_rmat, left_wrist_mat, right_wrist_mat, left_qpos, right_qpos


async def send_hand_angles(right_angles, left_angles):
    """
    Sends the right and left hand angles to the WebSocket server.
    """
    async with websockets.connect("ws://localhost:8008/ws") as websocket:
        # Prepare the data payload
        right_hand_data = {
            "hand": "right",
            "angles": right_angles  # Send the right hand angles
        }

        left_hand_data = {
            "hand": "left",
            "angles": left_angles  # Send the left hand angles
        }

        # Send both left and right hand data to the server
        await websocket.send(json.dumps(right_hand_data))
        await websocket.send(json.dumps(left_hand_data))

async def send_pose_data(left_pose_data,right_pose_data):
    uri = "ws://localhost:8001/ws"  # Adjusted to match your server's port
    async with websockets.connect(uri) as websocket:
        # # Flatten the matrix to a 1D list for sending
        left_pose_data_flattened = np.array(left_pose_data).flatten().tolist()
        right_pose_data_flattened = np.array(right_pose_data).flatten().tolist()

        # Prepare the data to send as a JSON object
        data = {
            "left_pose_data": left_pose_data_flattened,
            "right_pose_data": right_pose_data_flattened
        }
        # Convert to JSON and send it
        await websocket.send(json.dumps(data))
#         right_pose_data = np.load('right_angles.npy') 
#         frame_count = right_pose_data.shape[0]  # Total number of frames
# #         left_target = np.array([
# # [1, 0, 0, 0.2],
# # [0, 1, 0, 0.2],
# # [0, 0, 1, 0.1],
# # [0, 0, 0, 1]
# # ])
# #         left_pose_data_flattened=left_target.flatten().tolist()
#         for i in range(frame_count-1):
#             right_pose_data_flattened=np.array(right_pose_data[i+1]).flatten().tolist()
#             data = {
#                 "left_pose_data": left_pose_data_flattened,
#                 "right_pose_data": right_pose_data_flattened
#             }    
#             await websocket.send(json.dumps(data))

#             print(f"Sent right_pose_data: {right_pose_data_flattened}")

#             # # Increment time variable
#             # t += 1 / 60

#             # Maintain 60 Hz rate
#             # await asyncio.sleep(1 / 60)
if __name__ == '__main__':
    # Initialize teleoperation using ZED camera instead of simulation
    teleoperator = VuerTeleop('inspire_hand.yml')
    # arm_ik = Arm_IK()
    # Create a Camera object for ZED
    zed = sl.Camera()

    # Create InitParameters object and set configuration parameters for ZED
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.camera_fps = 60  # Set fps at 60

    # Open the ZED camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print("Camera Open Error: " + repr(err) + ". Exiting program.")
        exit()

    # Initialize image containers
    image_left = sl.Mat()
    image_right = sl.Mat()
    runtime_parameters = sl.RuntimeParameters()

    # Prepare for storing right angles in a NumPy array
    right_angles_history = []
    left_angles_history = []
    try:
        user_input = input("Please enter the start signal (enter 's' to start the subsequent program):")
        if user_input.lower() == 's':
            while True:
                # Capture frames from ZED camera
                if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                    zed.retrieve_image(image_left, sl.VIEW.LEFT)
                    zed.retrieve_image(image_right, sl.VIEW.RIGHT)

                    # Combine left and right images and convert from BGRA to RGB
                    bgr = np.hstack((image_left.numpy()[teleoperator.crop_size_h:, teleoperator.crop_size_w:-teleoperator.crop_size_w],
                                    image_right.numpy()[teleoperator.crop_size_h:, teleoperator.crop_size_w:-teleoperator.crop_size_w]))
                    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGRA2RGB)

                    # Copy combined image to shared memory
                    np.copyto(teleoperator.img_array, rgb)

                    # Process the head and hand matrices (teleoperation logic)
                    head_rmat, left_pose, right_pose, left_qpos, right_qpos = teleoperator.step()
                    # sol_q ,tau_ff, flag = arm_ik.ik_fun(left_pose, right_pose)
                    print('left_pose',left_pose)
                    right_angles_history.append(right_pose)
                    np.save('right_angles.npy', np.array(right_angles_history))
                    if right_qpos is not None and left_qpos is not None:
                        # 4,5: index 6,7: middle, 0,1: pinky, 2,3: ring, 8,9: thumb
                        right_angles = [1.7 - right_qpos[i] for i in [4, 6, 2, 0]]
                        right_angles.append(1.2 - right_qpos[8])
                        right_angles.append(0.5 - right_qpos[9])

                        left_angles = [1.7 - left_qpos[i] for i in [4, 6, 2, 0]]
                        left_angles.append(1.2 - left_qpos[8])
                        left_angles.append(0.5 - left_qpos[9])

                        # # Append the right_angles to history and save it as a NumPy array


                        # Send the angles to the server via WebSocket
                        # asyncio.run(send_hand_angles(right_angles, left_angles))
                    asyncio.get_event_loop().run_until_complete(send_pose_data(left_pose,right_pose))
                    # print("right_qpos", right_qpos)

    except KeyboardInterrupt:
        # Close ZED camera on exit
        zed.close()
        exit(0)