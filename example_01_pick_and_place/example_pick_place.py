import signal
import functools
import asyncio

import numpy as np

from pyquaternion import Quaternion

from xamla_motion.world_view_client import WorldViewClient
from xamla_motion.motion_client import MoveGroup, EndEffector
from xamla_motion.data_types import Pose
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


def main():
    # create move group instance
    move_group = MoveGroup()

    # get default endeffector of the movegroup
    end_effector = move_group.get_end_effector()

    # get the gripper of the current move group
    wsg_gripper = get_gripper(move_group)

    # get a client to communicate with the robot
    robot_chat = RobotChatClient()

    # create a instance of WorldViewClient to get access to rosvita world view
    world_view_client = WorldViewClient()

    # define a couple of poses to which the robot should move
    t_begin = [0.502522, 0.2580, 0.3670]
    t_prepick = [0.4, 0.4, 0.15]
    t_pick = [0.4, 0.4, 0.1]
    t_preplace = [0.4, 0, 0.15]
    t_place = [0.4, 0, 0.1]

    # apply the same upright positions to all the poses
    q_upright = Quaternion(w=0, x=0, y=-1, z=0)

    pose_begin = Pose(t_begin, q_upright)
    pose_prepick = Pose(t_prepick, q_upright)
    pose_pick = Pose(t_pick, q_upright)
    pose_preplace = Pose(t_preplace, q_upright)
    pose_place = Pose(t_place, q_upright)

    # calculate the joint values out of the poses
    begin = end_effector.inverse_kinematics(pose_begin, collision_check=True)
    prepick = end_effector.inverse_kinematics(pose_prepick, collision_check=True)
    pick = end_effector.inverse_kinematics(pose_pick, collision_check=True)
    preplace = end_effector.inverse_kinematics(pose_preplace, collision_check=True)
    place = end_effector.inverse_kinematics(pose_place, collision_check=True)


    print(begin)
    print(type(begin))

    loop = asyncio.get_event_loop()
    loop.add_signal_handler(signal.SIGTERM,
                            functools.partial(shutdown, loop, signal.SIGTERM))
    loop.add_signal_handler(signal.SIGINT,
        functools.partial(shutdown, loop, signal.SIGINT))

    async def move_supervised_to_pose(pose: Pose, velocity_scaling=1):
        """Opens a window in rosvita to let the user supervise the motion to pose """
        stepped_client = end_effector.move_poses_supervised(pose,
                                                            collision_check=True)
        robot_chat_stepped_motion = RobotChatSteppedMotion(robot_chat,
                                                       stepped_client,
                                                       move_group.name)
        await robot_chat_stepped_motion.handle_stepwise_motions()

    async def prepare_gripper():
        print("Home the gripper") 
        # Allows parallel homing and supervised movement
        t1 = wsg_gripper.homing()
        t2 = move_supervised_to_pose(pose_begin)
        await asyncio.gather(t1, t2)

    async def go_to_prepick():
        print("Go to prepick position and open gripper")
        # Allows parallel adjustment of the gripper and movement of the end effector
        t1 = end_effector.move_poses_collision_free(pose_prepick, velocity_scaling=0.1)
        t2 = wsg_gripper.grasp(0.02, 1, 0.05 )
        await asyncio.gather(t1, t2)

    async def pick_up():
        """Simple example of a "picking up" motion """
        print("Pick up")
        await end_effector.move_poses_collision_free(pose_pick, velocity_scaling=0.02)
        await wsg_gripper.grasp(0.002, 0.1, 0.05)
        await end_effector.move_poses_collision_free(pose_prepick)

    async def go_to_preplace():
        print("Go to preplace position")
        await end_effector.move_poses_collision_free(pose_preplace)   

    async def place():
        """Simple example of a "placing" motion """
        print("Place")
        await end_effector.move_poses_collision_free(pose_place, velocity_scaling=0.02)
        await wsg_gripper.grasp(0.04, 1, 0.05)
        await end_effector.move_poses_collision_free(pose_preplace)  

    async def pick_and_place():
        """Actions to be repeated in loop"""
        await go_to_prepick()
        await pick_up()
        await go_to_preplace()
        await place()

    try:
        # plan a trajectory to the begin pose
        loop.run_until_complete(prepare_gripper())
        for i in range(2):
            loop.run_until_complete(pick_and_place())      
        print("Finished. Go to begin position") 
        loop.run_until_complete(end_effector.move_poses_collision_free(pose_begin))
    finally:
        loop.close()
    

if __name__ == '__main__':
    main()