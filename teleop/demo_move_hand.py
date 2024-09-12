import numpy as np
import asyncio
from fastapi import FastAPI, WebSocket
from fourier_grx.sdk.end_effectors import InspireDexHand

app = FastAPI()

# Function to map angles from radians to 0-1000 range
def normalize_angle(angle, min_angle, max_angle):
    """Normalize a radian angle to a range of [0, 1000]"""
    return int(1000 * (angle - min_angle) / (max_angle - min_angle))

# Instantiate the hand objects for both left and right hands
HAND_IP_LEFT = '192.168.137.39'
HAND_IP_RIGHT = '192.168.137.19'
left_hand = InspireDexHand(HAND_IP_LEFT)
right_hand = InspireDexHand(HAND_IP_RIGHT)

# Min and max angles for normalization (adjust as necessary)
min_angle = 0.0
max_angle = 1.7

# Assuming the joint order is the same for both hands
joint_mapping = [0, 1, 2, 3, 4, 5]
# WebSocket endpoint for receiving real-time joint angles
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    print("WebSocket connection established")

    try:
        while True:
            # Expecting to receive data as a dictionary with 'hand' and 'angles' keys
            data = await websocket.receive_json()
            
            hand_type = data.get("hand")  # 'left' or 'right'
            angles = data.get("angles")  # List of joint angles in radians
            
            # Normalize angles for the robot hand
            normalized_positions = [normalize_angle(angles[joint_mapping[j]], min_angle, max_angle) for j in range(6)]
            
            # Adjust the last two positions to 800 for the right hand
            if hand_type == 'right':
                # normalized_positions[-2:] = [800, 800]
                right_hand.set_positions(normalized_positions)
            # elif hand_type == 'left':
            #     left_hand.set_positions(normalized_positions)

            # Print for debugging
            print(f"Sent positions to {hand_type} hand: {normalized_positions}")

            # Sleep for 0.016667 seconds (~60 frames per second)
            await asyncio.sleep(0.016667)  # non-blocking sleep for streaming
    
    except Exception as e:
        print(f"Connection error: {e}")
    # finally:
    #     # Close the WebSocket connection if the client disconnects
    #     await websocket.close()

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8008)

# import numpy as np
# import asyncio
# from fastapi import FastAPI, WebSocket
# from fourier_grx.sdk.end_effectors import InspireDexHand

# app = FastAPI()

# # Function to map angles from radians to 0-1000 range
# def normalize_angle(angle, min_angle, max_angle):
#     """Normalize a radian angle to a range of [0, 1000]"""
#     return int(1000 * (angle - min_angle) / (max_angle - min_angle))

# # Instantiate the hand objects for both left and right hands
# HAND_IP_LEFT = '192.168.137.39'
# HAND_IP_RIGHT = '192.168.137.19'
# left_hand = InspireDexHand(HAND_IP_LEFT)
# right_hand = InspireDexHand(HAND_IP_RIGHT)

# # Min and max angles for normalization (adjust as necessary)
# min_angle = 0.0
# max_angle = 1.7

# # Assuming the joint order is the same for both hands
# joint_mapping = [0, 1, 2, 3, 4, 5]

# # WebSocket endpoint for receiving real-time joint angles
# @app.websocket("/ws")
# async def websocket_endpoint(websocket: WebSocket):
#     await websocket.accept()
#     print("WebSocket connection established")

#     try:
#         while True:
#             data = await websocket.receive_json()
            
#             hand_type = data.get("hand")  # 'left' or 'right'
#             angles = data.get("angles")  # List of joint angles in radians
            
#             if angles and hand_type:
#                 # Normalize angles for the robot hand
#                 normalized_positions = [normalize_angle(angles[joint_mapping[j]], min_angle, max_angle) for j in range(6)]
                
#                 # Adjust the last two positions to 800 for the right hand
#                 if hand_type == 'right':
#                     right_hand.set_positions(normalized_positions)
#                 elif hand_type == 'left':
#                     left_hand.set_positions(normalized_positions)

#                 # Print for debugging
#                 print(f"Sent positions to {hand_type} hand: {normalized_positions}")

#             # Sleep for 0.016667 seconds (~60 frames per second)
#             await asyncio.sleep(0.016667)  # non-blocking sleep for streaming

#     except Exception as e:
#         print(f"Connection error: {e}")
#     # finally:
#     #     # Close the WebSocket connection if the client disconnects
#     #     print("Connection closed")
#     #     await websocket.close()

# if __name__ == "__main__":
#     import uvicorn
#     uvicorn.run(app, host="0.0.0.0", port=8008)
