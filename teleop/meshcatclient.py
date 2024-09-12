import asyncio
import websockets
import json
import numpy as np
import time

async def send_pose_data():
    uri = "ws://localhost:8001/ws"
    try:
        async with websockets.connect(uri) as websocket:
            try:
                right_pose_data = np.load('right_angles.npy') 
                frame_count = right_pose_data.shape[0]  # Total number of frames
                left_target = np.array([
        [1, 0, 0, 0.2],
        [0, 1, 0, 0.2],
        [0, 0, 1, 0.1],
        [0, 0, 0, 1]
    ])
                left_pose_data_flattened=left_target.flatten().tolist()
                for i in range(frame_count-1):
                    right_pose_data_flattened=np.array(right_pose_data[i+1]).flatten().tolist()
                    data = {
                        "left_pose_data": left_pose_data_flattened,
                        "right_pose_data": right_pose_data_flattened
                    }    
                    await websocket.send(json.dumps(data))

                    print(f"Sent right_pose_data: {right_pose_data_flattened}")

                    # # Increment time variable
                    # t += 1 / 60

                    # Maintain 60 Hz rate
                    await asyncio.sleep(1 / 60)
            except websockets.ConnectionClosed as e:
                print(f"Connection closed error: {e}")


    #         t = 0
    #         while True:
    #             try:
    #                 # Generate some pose data
    #                 x_translation = 0.2 + 0.1 * np.sin(t)
    #                 y_translation = 0.3 + 0.1 * np.cos(t)
    #                 z_translation = 0.1

    #                 right_pose_data = np.array([
    #                     [1, 0, 0, x_translation],
    #                     [0, 1, 0, y_translation],
    #                     [0, 0, 1, z_translation],
    #                     [0, 0, 0, 1]
    #                 ])

    #                 right_pose_data_flattened = right_pose_data.flatten().tolist()

    #                 data = {
    #                     "left_pose_data": right_pose_data_flattened,
    #                     "right_pose_data": right_pose_data_flattened
    #                 }

    #                 # Send data
    #                 await websocket.send(json.dumps(data))

    #                 print(f"Sent right_pose_data: {right_pose_data_flattened}")

    #                 # Increment time variable
    #                 t += 1 / 60

    #                 # Maintain 60 Hz rate
    #                 await asyncio.sleep(1 / 60)
    #             except websockets.ConnectionClosed as e:
    #                 print(f"Connection closed error: {e}")
    #                 break
    except Exception as e:
        print(f"WebSocket connection failed: {e}")

# Start the WebSocket client
asyncio.get_event_loop().run_until_complete(send_pose_data())


