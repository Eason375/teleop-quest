# import time
# from fourier_grx.sdk.end_effectors import InspireDexHand

# # Ensure that the server is running on the robot before executing this script.

# # Instantiate the hand object
# # For left hand use IP: 192.168.137.39
# # For right hand use IP: 192.168.137.19
# HAND_IP = '192.168.137.19'  # Change to the appropriate IP address
# hand = InspireDexHand(HAND_IP)
# time.sleep(1.0)  # Wait for the device to initialize

# # Finger control sequence:
# # [pinky, ring, middle, index, thumbending, thumbrotation]
# # Maximum position: 1000
# # Minimum position: 0
# positions = [
#     [700, 1000, 1000, 1000, 1000, 1000],
#     [1000, 700, 1000, 1000, 1000, 1000],
#     [1000, 1000, 700, 1000, 1000, 1000],
#     [1000, 1000, 1000, 700, 1000, 1000],
#     [1000, 1000, 1000, 1000, 700, 1000],
#     [1000, 1000, 1000, 1000, 1000, 700]
# ]

# # Execute the finger control sequence
# for pos in positions:
#     hand.set_positions(pos)
#     time.sleep(2.0)  # Wait for 2 seconds before moving to the next position


import numpy as np
import time
from fourier_grx.sdk.end_effectors import InspireDexHand

# Function to normalize qpos to the 0-1000 range
def normalize_qpos_to_robot_hand(qpos, min_angle=0.0, max_angle=np.pi/2, max_position=1000):
    """
    Normalize qpos values (assumed in radians) to robot hand's control values (0-1000).
    qpos: Array of joint angles in radians.
    min_angle: The minimum angle for the joint (e.g., 0 radians).
    max_angle: The maximum angle for the joint (e.g., pi/2 radians).
    max_position: The maximum position value for the robot hand (1000).
    """
    # Clip the qpos to be within the expected range
    qpos_clipped = np.clip(qpos, min_angle, max_angle)
    
    # Normalize to the 0-1000 range
    normalized_qpos = (qpos_clipped - min_angle) / (max_angle - min_angle) * max_position
    return normalized_qpos.astype(int)

# Instantiate the hand object
HAND_IP = '192.168.137.19'  # Right hand's IP address
hand = InspireDexHand(HAND_IP)
time.sleep(1.0)  # Wait for the device to initialize

# The qpos array from the teleoperation or simulation (example values)
right_qpos = np.array([1.66967746e+00, 4.27635017e-01, 1.67236209e+00, 4.30254114e-01,
                       1.68027558e+00, 5.33675943e-01, 1.67771733e+00, 5.07226367e-01,
                       8.08890487e-01, 1.87063007e-18, 3.83622334e-18, 1.77743661e-02])

# Map qpos to robot hand positions
# Assuming qpos elements correspond to: [pinky, ring, middle, index, thumb flex, thumb rotation]
# Normalize qpos values (assuming range from 0 to pi/2 for each joint)
right_qpos[:6]=[1000 ,1000,1000 ,1000,1000 ,1000]
robot_positions = normalize_qpos_to_robot_hand(right_qpos[:6])

# Send positions to the robot hand
print(f"Sending positions to robot hand: {robot_positions}")
hand.set_positions(robot_positions)
time.sleep(2.0)  # Allow some time for the hand to move

