from fourier_core.end_effector.fi_end_effector_base import EndEffectorBase
from fastapi import FastAPI, WebSocket
import numpy as np
import pink
from pink.visualization import start_meshcat_visualizer
import pinocchio as pin
import math
import os
import time
import qpsolvers
from loop_rate_limiters import RateLimiter
from scipy.spatial.transform import Rotation as R
import meshcat_shapes
from test_func import *
from fourier_grx_client import *
import asyncio

app = FastAPI()

def smoothing_factor(t_e, cutoff):
    r = 2 * math.pi * cutoff * t_e
    return r / (r + 1)

def exponential_smoothing(a, x, x_prev):
    return a * x + (1 - a) * x_prev

class OneEuroFilter:
    def __init__(self, t0, x0, dx0=None, min_cutoff=1.0, beta=0.0, d_cutoff=1.0):
        """Initialize the one euro filter for a 14-dimensional numpy array."""
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.d_cutoff = float(d_cutoff)

        # Ensure x0 is a numpy array
        self.x_prev = np.array(x0, dtype=float)

        # If dx0 is not provided, initialize it as zeros with the same shape as x0
        if dx0 is None:
            self.dx_prev = np.zeros_like(self.x_prev)
        else:
            self.dx_prev = np.array(dx0, dtype=float)

        self.t_prev = float(t0)

    

    def __call__(self, t, x):
        """Compute the filtered signal for a 14-dimensional numpy array."""
        t_e = t - self.t_prev

        # The filtered derivative of the signal
        a_d = smoothing_factor(t_e, self.d_cutoff)
        dx = (x - self.x_prev) / t_e
        dx_hat = exponential_smoothing(a_d, dx, self.dx_prev)

        # The filtered signal
        cutoff = self.min_cutoff + self.beta * np.abs(dx_hat)
        a = smoothing_factor(t_e, cutoff)
        x_hat = exponential_smoothing(a, x, self.x_prev)

        # Memorize the previous values
        self.x_prev = x_hat
        self.dx_prev = dx_hat
        self.t_prev = t

        return x_hat

class EndEffectorGR1(EndEffectorBase):
    def __init__(self, robot_name: str, urdf_path: str, control_frequency: int, joints=None):
        super().__init__(joints)
        self.robot_name = robot_name
        self.frequency = control_frequency
        urdf_dir = os.path.dirname(urdf_path)
        package_dir = os.path.join(urdf_dir, "../meshes")
        package_dir = os.path.normpath(package_dir)
        self.robot = pin.RobotWrapper.BuildFromURDF(
            filename=urdf_path,
            package_dirs=[package_dir],
            root_joint=pin.JointModelFreeFlyer(),
        )
        self.left_ee_frame = self.robot.model.addFrame(
            pin.Frame(
                "left_ee_frame",
                self.robot.model.getJointId("l_wrist_pitch"),
                self.robot.model.getFrameId("l_hand_pitch"),
                pin.SE3(
                    rotation=np.eye(3),
                    translation=np.array([0.0, 0.0, 0.0]),
                ),
                pin.FrameType.OP_FRAME,
            )
        )
        self.right_ee_frame = self.robot.model.addFrame(
            pin.Frame(
                "right_ee_frame",
                self.robot.model.getJointId("r_wrist_pitch"),
                self.robot.model.getFrameId("r_hand_pitch"),
                pin.SE3(
                    rotation=np.eye(3),
                    translation=np.array([0.0, 0.0, 0.0]),
                ),
                pin.FrameType.OP_FRAME,
            )
        )
        self.robot.rebuildData()
        self.configuration = pink.Configuration(self.robot.model, self.robot.data, self.robot.q0, copy_data=False)
        self.left_hand_task = pink.tasks.RelativeFrameTask(
            "left_ee_frame",
            "link_torso",
            position_cost=0.95,
            orientation_cost=[0.5, 0.5, 0.5],
            gain=0.5,
        )
        self.right_hand_task = pink.tasks.RelativeFrameTask(
            "right_ee_frame",
            "link_torso",
            position_cost=0.95,
            orientation_cost=[0.5, 0.5, 0.5],
            gain=0.5,
        )
        self.tasks = [self.right_hand_task, self.left_hand_task]
        self.solver = "quadprog" if "quadprog" in qpsolvers.available_solvers else qpsolvers.available_solvers[0]
        self.rate = RateLimiter(frequency=self.frequency)
        self.dt = self.rate.period
        self.configuration.update(self.robot.q0)

    @property
    def frame_names(self) -> list[str]:
        return [f.name for f in self.robot.model.frames]

    def resetq(self, q: np.ndarray) -> None:
        # self.configuration.update(np.hstack((np.zeros(6), [1], np.zeros(6), q, np.zeros(39 - 7 - 14 - 6))))
        self.configuration.update(np.hstack((np.zeros(6), [1], q)))

    def get_transform(self, to_frame: str, from_frame: str) -> np.ndarray:
        if from_frame not in self.frame_names:
            raise KeyError(f"Frame {from_frame} not found in the robot.")
        if to_frame not in self.frame_names:
            raise KeyError(f"Frame {to_frame} not found in the robot.")
        transform = self.configuration.get_transform(to_frame, from_frame).np
        return transform

    def set_target_for_task(self, task, target, from_frame) -> None:
        target_transform = np.dot(self.get_transform("link_torso", from_frame), target)
        target_SE3 = pin.SE3(rotation=target_transform[:3, :3], translation=target_transform[:3, 3])
        task.set_target(target_SE3)

    def forward_kinematics(self):
        left_ee_transform = self.get_transform("left_ee_frame", "link_torso")
        right_ee_transform = self.get_transform("right_ee_frame", "link_torso")
        return left_ee_transform, right_ee_transform

    # def forward_kinematics(self,q_new: np.ndarray):
    #     temp_q = self.configuration.q[13:27]
    #     self.resetq(q_new)
    #     left_ee_transform = self.get_transform("left_ee_frame", "link_torso")
    #     right_ee_transform = self.get_transform("right_ee_frame", "link_torso")
    #     self.resetq(temp_q)
    #     return left_ee_transform, right_ee_transform

    def inverse_kinematics(self, q: np.ndarray, l_target: np.ndarray = None, r_target: np.ndarray = None) -> np.ndarray:
        self.resetq(q)
        if l_target is not None:
            self.set_target_for_task(self.left_hand_task, l_target, "link_torso")
        if r_target is not None:
            self.set_target_for_task(self.right_hand_task, r_target, "link_torso")

        tasks_to_solve = []
        if r_target is not None:
            tasks_to_solve.append(self.right_hand_task)
        if l_target is not None:
            tasks_to_solve.append(self.left_hand_task)

        if not tasks_to_solve:
            tasks_to_solve = self.tasks


        counts = 0

        while True:
            
            velocity = pink.solve_ik(self.configuration, tasks_to_solve, self.dt, solver=self.solver)
            self.configuration.integrate_inplace(velocity, self.dt)
            # if (r_target is None or np.linalg.norm(self.right_hand_task.compute_error(self.configuration)) < 0.0001) and (l_target is None or np.linalg.norm(self.left_hand_task.compute_error(self.configuration)) < 0.0001):
            if (r_target is None or np.linalg.norm(self.right_hand_task.compute_error(self.configuration)) < 0.005) and (l_target is None or np.linalg.norm(self.left_hand_task.compute_error(self.configuration)) < 0.0001):
                break
            counts+=1
            if counts>=100:
                # raise KeyError("can not reach!")
                print(("can not reach!"))
                return q[6:20]
        # print(self.configuration.model)

        return self.configuration.q[13:27]

        velocity = pink.solve_ik(self.configuration, tasks_to_solve, self.dt, solver=self.solver)
        self.configuration.integrate_inplace(velocity, self.dt)

        return self.configuration.q[13:27]

    def forward_dynamics(self):
        pass

    def inverse_dynamics(self):
        pass

def task_enable(client:RobotClient):
    client.set_enable(True)

def task_disable(client:RobotClient):
    client.set_enable(True)


def task_set_gains(client: RobotClient):
    # kp = np.array([0.1] * 32)
    # kd = np.array([0.01] * 32)
    control_mode = [
    ControlMode.NONE, ControlMode.NONE, ControlMode.NONE, ControlMode.NONE, ControlMode.NONE, ControlMode.NONE,  # left leg
    ControlMode.NONE, ControlMode.NONE, ControlMode.NONE, ControlMode.NONE, ControlMode.NONE, ControlMode.NONE,   # right leg
    ControlMode.NONE, ControlMode.NONE, ControlMode.NONE,  # waist
    ControlMode.NONE, ControlMode.NONE, ControlMode.NONE,  # head
    ControlMode.PD, ControlMode.PD, ControlMode.PD, ControlMode.PD, ControlMode.PD, ControlMode.PD, ControlMode.PD,  # left arm
    ControlMode.PD, ControlMode.PD, ControlMode.PD, ControlMode.PD, ControlMode.PD, ControlMode.PD, ControlMode.PD,  # right arm
        ]
    kp = np.array([
    251.625, 362.52, 200, 200, 10.98, 10.98,  # left leg
    251.625, 362.52, 200, 200, 10.98, 10.98,  # right leg
    251.625, 251.625, 251.625,  # waist
    112.06, 112.06, 112.06,  # head
    # 180.0, 180.0, 180.0, 180.0, 180.0, 90.0, 90.0,  # left arm
    800.5, 800.5, 455.0, 455.0, 455.0, 130.0, 130.0,
    # 180.0, 180.0, 180.0, 180.0, 180.0, 90.0, 90.0,  # right arm
    800.5, 800.5, 455.0, 455.0, 455.0, 130.0, 130.0,
    ])
    kd = np.array([
    14.72, 10.0833, 11, 11, 0.6, 0.6,  # left leg
    14.72, 10.0833, 11, 11, 0.6, 0.6,  # right leg
    14.72, 14.72, 14.72,  # waist
    3.1, 3.1, 3.1,  # head
    # 8, 8, 8, 8, 8, 5, 5,  # left arm
    # 8, 8, 8, 8, 8, 4, 4,  # right arm
    40.9, 40.9, 15, 15, 15, 5, 5,
    40.9, 40.9, 15, 15, 15, 5, 5,
    ])

    # Set PD parameters
    new_gains = client.set_gains(kp, kd, control_mode=control_mode)
    print(new_gains)

viz = None
viewer = None
@app.on_event("startup")
async def startup_event():
    global viz, viewer
    urdf_path = "/home/eason/LLM/TO/teleop_controller/assets/GR1T2/urdf/GR1T2.urdf"
    gr1kinematics = EndEffectorGR1("T2", urdf_path, control_frequency=50)

    # Start Meshcat visualizer only once during server startup
    viz = start_meshcat_visualizer(gr1kinematics.robot)
    if viz is None:
        raise RuntimeError("Could not start Meshcat visualizer")

    viz.display(gr1kinematics.configuration.q)
    viewer = viz.viewer

    # Visualize frames for targets
    meshcat_shapes.frame(viewer["left_ee_transform_target"], opacity=1.0)
    meshcat_shapes.frame(viewer["right_ee_transform_target"], opacity=1.0)


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    oef_min_cutoff = 0.02
    oef_beta = 1.5
    oef = None
    FREQUENCY_TELE = 60

    urdf_path = "/home/eason/LLM/TO/teleop_controller/assets/GR1T2/urdf/GR1T2.urdf"
    gr1kinematics = EndEffectorGR1("T2", urdf_path, control_frequency=50)
    global viz, viewer

    if viz is None or viewer is None:
        await websocket.close()
        return

    q_pos = np.zeros(32)

    while True:
        try:
            # Receive the real-time right hand pose data (4x4 transformation matrix)
            data = await websocket.receive_json()
            left_pose_data=np.array(data["left_pose_data"]).reshape(4, 4)
            right_pose_data = np.array(data["right_pose_data"]).reshape(4, 4)
            print(left_pose_data)
            # print(left_pose_data)


            # Visualize the right hand's transformation in Meshcat
            viewer["right_ee_transform_target"].set_transform(
                gr1kinematics.get_transform("link_torso", "base") @ right_pose_data
            )

            # Visualize the left hand's fixed target
            viewer["left_ee_transform_target"].set_transform(
                gr1kinematics.get_transform("link_torso", "base") @ left_pose_data
            )

            # Perform inverse kinematics for both hands
            q_total = gr1kinematics.inverse_kinematics(
                q=q_pos, l_target=left_pose_data, r_target=right_pose_data
            )

            if oef is None:
                oef = OneEuroFilter(time.time(), q_total, min_cutoff=oef_min_cutoff, beta=oef_beta)
                filtered_pos = q_total
            else:
                filtered_pos = oef(time.time(), q_total)

            viz.display(gr1kinematics.configuration.q)

            # Maintain the loop rate
            await asyncio.sleep(1 / FREQUENCY_TELE)

        except Exception as e:
            print(f"Error inside loop: {e}")
            await websocket.send_text(f"Error inside loop: {e}")
            break

    # await websocket.close()

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8001)
