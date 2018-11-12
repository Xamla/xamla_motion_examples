import signal
import functools
import asyncio

import numpy as np

from pyquaternion import Quaternion

from xamla_motion.world_view_client import WorldViewClient
from xamla_motion.motion_client import MoveGroup, EndEffector
from xamla_motion.data_types import JointValues, Pose
from xamla_motion.gripper_client import WeissWsgGripperProperties, WeissWsgGripper
from xamla_motion.robot_chat_client import RobotChatClient, RobotChatSteppedMotion

def shutdown(loop, reason):
    """function to shutdown asyncio properly """
    print('shutdown asyncio due to : {}'.format(reason), flush=True)
    tasks = asyncio.gather(*asyncio.Task.all_tasks(loop=loop),
                           loop=loop, return_exceptions=True)
    tasks.add_done_callback(lambda t: loop.stop())
    tasks.cancel()
    # Keep the event loop running until it is either destroyed or all
    # tasks have really terminated
    while not tasks.done() and not loop.is_closed():
        loop.run_forever()

def get_gripper(move_group: MoveGroup):
    # create instance of wsg gripper by name
    properties = WeissWsgGripperProperties('wsg50')
    return  WeissWsgGripper(properties, move_group.motion_service)


def main(loopCount: int):
    # create move group instance targeting the right arm of the robot
    move_group = MoveGroup("/sda10f/right_arm_torso")

    # get the corresponding end effector
    end_effector = move_group.get_end_effector()

    # get the gripper attached at the right arm
    wsg_gripper = get_gripper(move_group)

    # get a client to communicate with the robot
    robot_chat = RobotChatClient()

    # create a instance of WorldViewClient to get access to rosvita world view
    world_view_client = WorldViewClient()

    # get the example joint values from world view
    jv_home = world_view_client.get_joint_values("Home","")
    jv_prePick = world_view_client.get_joint_values("01_PrePick","example_01_pick_place")
    jv_pick = world_view_client.get_joint_values("02_Pick","example_01_pick_place")
    jv_prePlace = world_view_client.get_joint_values("03_PrePlace","example_01_pick_place")
    jv_place = world_view_client.get_joint_values("04_Place","example_01_pick_place") 

    loop = asyncio.get_event_loop()
    loop.add_signal_handler(signal.SIGTERM,
                            functools.partial(shutdown, loop, signal.SIGTERM))
    loop.add_signal_handler(signal.SIGINT,
                            functools.partial(shutdown, loop, signal.SIGINT))

    async def move_supervised(joint_values: JointValues, velocity_scaling=1):
        """Opens a window in rosvita to let the user supervise the motion to joint values """
        stepped_client = move_group.move_joints_collision_free_supervised(joint_values, 
                                                        velocity_scaling = velocity_scaling)
        robot_chat_stepped_motion = RobotChatSteppedMotion(robot_chat,
                                                        stepped_client,
                                                        move_group.name)
        await robot_chat_stepped_motion.handle_stepwise_motions()

    async def prepare_gripper():
        print("Home the gripper") 
        # Allows parallel homing and supervised movement
        t1 = wsg_gripper.homing()
        t2 = move_supervised(jv_home)
        await asyncio.gather(t1, t2)

    async def go_to_prepick():
        print("Go to prepick position and open gripper")
        # Allows parallel adjustment of the gripper and movement of the end effector
        t1 = move_group.move_joints_collision_free(jv_prePick, velocity_scaling=0.5)
        t2 = wsg_gripper.grasp(0.02, 1, 0.05 )
        await asyncio.gather(t1, t2)

    async def pick_up():
        """Simple example of a "picking up" motion """
        print("Pick up")
        await move_group.move_joints_collision_free(jv_pick, velocity_scaling=0.04)
        await wsg_gripper.grasp(0.002, 0.1, 0.05)
        await move_group.move_joints_collision_free(jv_prePick)

    async def go_to_preplace():
        print("Go to preplace position")
        await move_group.move_joints_collision_free(jv_prePlace)   

    async def place():
        """Simple example of a "placing" motion """
        print("Place")
        await move_group.move_joints_collision_free(jv_place, velocity_scaling=0.04)
        await wsg_gripper.grasp(0.04, 1, 0.05)
        await move_group.move_joints_collision_free(jv_prePlace)  

    async def pick_and_place():
        """Actions to be repeated in loop"""
        await go_to_prepick()
        await pick_up()
        await go_to_preplace()
        await place()

    try:
        # plan a trajectory to the begin pose
        loop.run_until_complete(prepare_gripper())
        for i in range(loopCount):
            loop.run_until_complete(pick_and_place())      
    finally:
        loop.close()
    

if __name__ == '__main__':
    loopCount = 2
    main(loopCount)