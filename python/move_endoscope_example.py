import click
from pathlib import Path
import logging
from threading import Lock
from enum import Enum
from typing import Tuple

import time
import numpy as np
import PyKDL

import rclpy
from ros_abstraction_layer import ral
from ambf_msgs.msg import RigidBodyState

logger = logging.getLogger(__name__)
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)s | %(message)s",
    filename="experiment.log",
    filemode="w",  # overwrite each run
)
logger.addHandler(logging.StreamHandler())

class Status(Enum):
    IDLE = 0
    MOVING = 1
    HOLD = 2
    ERROR = 3


class MoveEndoscopeExample:
    def __init__(self, rosNode: str, EndoscopeName: str, rate: int = 100):
        self.ral = ral(rosNode)

        # Topic name is expected to be in the format "/ambf/env/Endoscope_tip/State"
        # where "Endoscope_tip" is the name of the endoscope object in AMBF
        self.topicName = f"/ambf/env/{EndoscopeName}/State"
        self.EndoscopeName = EndoscopeName
        
        self.endoscopePose = PyKDL.Frame()
        self.haveMsg = False
        self.latestMsgTime = time.time()

        self.posThreshold = 0.001  # 1 mm
        self.rotThreshold = 0.01  # ~0.57 degrees

        self.lock = Lock()
        self.state = Status.IDLE
        
        self.endoscopePoseSub = self.ral.subscriber(self.topicName,
                                                    RigidBodyState,
                                                    self.endoscope_state_callback)

        self.endoscopePosePub = self.ral.publisher(self.topicName.replace("State", "Command"),
                                                    RigidBodyState)

        self.ral.spin()
    

    def endoscope_state_callback(self, msg: RigidBodyState):
        with self.lock:
            self.endoscopePose.p = PyKDL.Vector(msg.pose.position.x, 
                                                msg.pose.position.y, 
                                                msg.pose.position.z)
            
            self.endoscopePose.M = PyKDL.Rotation.Quaternion(msg.pose.orientation.x, 
                                                            msg.pose.orientation.y,
                                                            msg.pose.orientation.z,
                                                            msg.pose.orientation.w)
            
            self.haveMsg = True
            self.latestMsgTime = time.time()


    def is_ready(self) -> bool:
        with self.lock:
            return self.haveMsg


    def get_endoscope_pose(self) -> PyKDL.Frame:
        with self.lock:
            return self.endoscopePose
        
    
    def check_frame_difference(self, 
                               frame1: PyKDL.Frame, 
                               frame2: PyKDL.Frame) -> Tuple[bool, int]:
        # Compute the difference between two frames in terms of position and rotation
        twist = PyKDL.diff(frame1, frame2)
        posDiff = twist.vel.Norm()
        rotDiff = twist.rot.Norm()

        if posDiff > self.posThreshold or rotDiff > self.rotThreshold:
            logger.warning(f"Frame difference exceeds threshold: posDiff={posDiff:.4f}, rotDiff={rotDiff:.4f}")
            
            # check if the posDiff is more than the threshold relative to the rotDiff, or vice versa, to determine if it's a large jump
            numSteps = max(int(posDiff / self.posThreshold), int(rotDiff / self.rotThreshold))
            
            return False, numSteps
        return True, 1
    

    def move_endoscope(self, targetPose: PyKDL.Frame):
        self.state = Status.MOVING

        # Interpolate if the target pose is within a certain threshold of the current pose
        needInterpolate, numSteps = self.check_frame_difference(self.get_endoscope_pose(), targetPose)
        if not needInterpolate:
            self.publish_endoscope_command(targetPose)

        # Interpolate between current pose and target pose
        else:
            currentPose = self.get_endoscope_pose()
            for i in range(1, numSteps + 1):
                alpha = i / numSteps
                interpPose = PyKDL.interpolate(currentPose, targetPose, alpha)
                self.publish_endoscope_command(interpPose)
                time.sleep(0.01)  # Sleep to allow time for the command to be processed

        self.state = Status.IDLE


    def hold_endoscope(self):
        self.state = Status.HOLD
        self.publish_endoscope_command(self.get_endoscope_pose())


    def publish_endoscope_command(self, targetPose: PyKDL.Frame):
        cmdMsg = RigidBodyState()
        cmdMsg.pose.position.x = targetPose.p.x()
        cmdMsg.pose.position.y = targetPose.p.y()
        cmdMsg.pose.position.z = targetPose.p.z()

        qx, qy, qz, qw = targetPose.M.GetQuaternion()
        cmdMsg.pose.orientation.x = qx
        cmdMsg.pose.orientation.y = qy
        cmdMsg.pose.orientation.z = qz
        cmdMsg.pose.orientation.w = qw

        self.endoscopePosePub.publish(cmdMsg)
    

    def shutdown(self):
        self.ral.shutdown()


def cleanup_ros_objects(*objects):
    for obj in objects:
        if obj is None:
            continue

        for method in ("close", "shutdown", "stop", "destroy", "destroy_node"):
            if hasattr(obj, method):
                try:
                    getattr(obj, method)()
                except Exception:
                    pass       
    
@click.command()
@click.option('--name', default='EndoscopeTip', help='Name of ROS topic to subscribe to for endoscope state messages.')
def main(name):
    # Initialize ROS node
    rclpy.init()

    # Create MoveEndoscopeExample instance
    moveEndoscopeExample = MoveEndoscopeExample("move_endoscope_example", name)

    # Predefined target pose for the endoscope (example values, adjust as needed)
    targetRelPoses = [PyKDL.Frame(PyKDL.Rotation.RPY(0.0, 0.0, 0.0), PyKDL.Vector(0.1, 0.0, 0.0)),
                      PyKDL.Frame(PyKDL.Rotation.RPY(0.0, 0.0, np.pi/4), PyKDL.Vector(0.1, 0.1, 0.0)),
                      PyKDL.Frame(PyKDL.Rotation.RPY(0.0, 0.0, np.pi/2), PyKDL.Vector(0.1, 0.1, 0.1)),
                      PyKDL.Frame(PyKDL.Rotation.RPY(0.0, np.pi, 0.0), PyKDL.Vector(0.1, 0.0, 0.1)),
                      PyKDL.Frame(PyKDL.Rotation.RPY(np.pi/4, 0.0, 0.0), PyKDL.Vector(0.0, 0.1, 0.1))]
    
    try:
        while not moveEndoscopeExample.is_ready():
            logger.warning(f"Waiting for endoscope state message: {name}...")
            time.sleep(0.5)
        
        logger.info("Received first endoscope state message. Starting to move endoscope pose...")
        
        while rclpy.ok() and moveEndoscopeExample.is_ready():
            currentPose = moveEndoscopeExample.get_endoscope_pose()
            for idx, targetRelPose in enumerate(targetRelPoses):
                logger.info(f"Moving endoscope to #{idx}/{len(targetRelPoses)} target pose: {currentPose * targetRelPose}")
                moveEndoscopeExample.move_endoscope(currentPose * targetRelPose)
                time.sleep(1)  # Sleep for a while before moving to the next target pose
            # After moving through all target poses, hold the endoscope at the last pose for a while
            logger.info("Holding endoscope at last target pose...")
            moveEndoscopeExample.hold_endoscope()
            time.sleep(5)  # Hold for 5 seconds before repeating the sequence
            break;

    except KeyboardInterrupt:
        logger.info("Shutting down...")

    finally:
        cleanup_ros_objects(
            moveEndoscopeExample
        )

        if rclpy.ok():
            rclpy.shutdown()

        
if __name__ == "__main__":
    main()