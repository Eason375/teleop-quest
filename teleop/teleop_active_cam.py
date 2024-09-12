import math
import numpy as np

np.set_printoptions(precision=2, suppress=True)
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from pytransform3d import rotations

import time
import cv2
from constants_vuer import *
from TeleVision import OpenTeleVision
import pyzed.sl as sl
from dynamixel.active_cam import DynamixelAgent
from multiprocessing import Array, Process, shared_memory, Queue, Manager, Event, Semaphore

resolution = (720, 1280)
crop_size_w = 1
crop_size_h = 0
resolution_cropped = (resolution[0] - crop_size_h, resolution[1] - 2 * crop_size_w)

# agent = DynamixelAgent(port="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8IT033-if00-port0")
# agent._robot.set_torque_mode(True)

# Create a Camera object
zed = sl.Camera()

# Create a InitParameters object and set configuration parameters
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720  # Use HD720 opr HD1200 video mode, depending on camera type.
init_params.camera_fps = 60  # Set fps at 60

# Open the camera
err = zed.open(init_params)
if err != sl.ERROR_CODE.SUCCESS:
    print("Camera Open : " + repr(err) + ". Exit program.")
    exit()

# Capture 50 frames and stop
# i = 0
image_left = sl.Mat()
image_right = sl.Mat()
runtime_parameters = sl.RuntimeParameters()

img_shape = (resolution_cropped[0], 2 * resolution_cropped[1], 3)
# img_height, img_width = resolution_cropped[:2]
shm = shared_memory.SharedMemory(create=True, size=np.prod(img_shape) * np.uint8().itemsize)
img_array = np.ndarray((img_shape[0], img_shape[1], 3), dtype=np.uint8, buffer=shm.buf)
image_queue = Queue()
toggle_streaming = Event()
tv = OpenTeleVision(resolution_cropped, shm.name, image_queue, toggle_streaming,ngrok=True)

while True:
    start = time.time()

    # head_mat = grd_yup2grd_zup[:3, :3] @ tv.head_matrix[:3, :3] @ grd_yup2grd_zup[:3, :3].T
    # if np.sum(head_mat) == 0:
    #     head_mat = np.eye(3)
    # head_rot = rotations.quaternion_from_matrix(head_mat[0:3, 0:3])
    # try:
    #     ypr = rotations.euler_from_quaternion(head_rot, 2, 1, 0, False)
    #     # print(ypr)
    #     # agent._robot.command_joint_state([0., 0.4])
    #     # agent._robot.command_joint_state(ypr[:2])
    #     # print("success")
    # except:
    #     # print("failed")
    #     # exit()
    #     pass

    if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
        zed.retrieve_image(image_left, sl.VIEW.LEFT)
        zed.retrieve_image(image_right, sl.VIEW.RIGHT)
        timestamp = zed.get_timestamp(sl.TIME_REFERENCE.CURRENT)  # Get the timestamp at the time the image was captured
        # print("Image resolution: {0} x {1} || Image timestamp: {2}\n".format(image.get_width(), image.get_height(),
        #         timestamp.get_milliseconds()))

    bgr = np.hstack((image_left.numpy()[crop_size_h:, crop_size_w:-crop_size_w],
                     image_right.numpy()[crop_size_h:, crop_size_w:-crop_size_w]))
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGRA2RGB)

    np.copyto(img_array, rgb)

    end = time.time()
    # print(1/(end-start))
zed.close()



import numpy as np
import cv2
import time
from multiprocessing import shared_memory, Queue, Event
import pyzed.sl as sl
from TeleVision import OpenTeleVision

# Camera settings and resolution
resolution = (720, 1280)
# resolution = (480, 854)
crop_size_w = 1
crop_size_h = 0
resolution_cropped = (resolution[0] - crop_size_h, resolution[1] - 2 * crop_size_w)

# Initialize the ZED Camera
zed = sl.Camera()

# Create InitParameters object and set camera settings
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720  # HD720 resolution
init_params.camera_fps = 60  # Set FPS at 60

# Open the ZED camera
err = zed.open(init_params)
if err != sl.ERROR_CODE.SUCCESS:
    print(f"Camera Open Error: {repr(err)}. Exiting program.")
    exit()

# Create image holders for left and right frames
image_left = sl.Mat()
image_right = sl.Mat()
runtime_parameters = sl.RuntimeParameters()

# Setup shared memory and frame buffer
img_shape = (resolution_cropped[0], 2 * resolution_cropped[1], 3)
shm = shared_memory.SharedMemory(create=True, size=np.prod(img_shape) * np.uint8().itemsize)
img_array = np.ndarray(img_shape, dtype=np.uint8, buffer=shm.buf)
image_queue = Queue()
toggle_streaming = Event()

# Start OpenTeleVision with cropped resolution, shared memory, and image queue
tv = OpenTeleVision(resolution_cropped, shm.name, image_queue, toggle_streaming,ngrok=True)

# Main streaming loop
while True:
    start_time = time.time()

    # Grab images from ZED camera
    if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
        zed.retrieve_image(image_left, sl.VIEW.LEFT)   # Capture left image
        zed.retrieve_image(image_right, sl.VIEW.RIGHT) # Capture right image

        # Combine left and right images horizontally (side-by-side)
        bgr_combined = np.hstack((
            image_left.numpy()[crop_size_h:, crop_size_w:-crop_size_w],
            image_right.numpy()[crop_size_h:, crop_size_w:-crop_size_w]
        ))

        # Convert BGR to RGB (for VR display)
        rgb_combined = cv2.cvtColor(bgr_combined, cv2.COLOR_BGRA2RGB)

        # Copy the combined RGB image to shared memory for streaming
        np.copyto(img_array, rgb_combined)

    # Limit the loop rate (optional, depends on performance needs)
    end_time = time.time()
    print(f"FPS: {1 / (end_time - start_time)}")

# Close the camera after streaming ends
zed.close()
