import asyncio

from xamla_motion.world_view_client import WorldViewClient
from xamla_motion.motion_client import MoveGroup
from xamla_motion.data_types import JointValues, JointPath
from xamla_motion.gripper_client import WeissWsgGripperProperties, WeissWsgGripper
from xamla_motion.robot_chat_client import RobotChatClient, RobotChatSteppedMotion

from xamla_motion.utility import register_asyncio_shutdown_handler 

import example_utils 

def main(loopCount: int):
    # create move group instance targeting the right arm of the robot
    move_group = example_utils.get_move_group()

    # get the gripper attached at the right arm
    wsg_gripper = example_utils.get_gripper(move_group)

    # get a client to communicate with the robot
    robot_chat = RobotChatClient()

    # create a instance of WorldViewClient to get access to rosvita world view
    world_view_client = WorldViewClient()

    # get the example joint values from world view
    jv_home = world_view_client.get_joint_values("Home","example_01_pick_place")
    jv_prePick = world_view_client.get_joint_values("01_PrePick","example_01_pick_place")
    jv_pick = world_view_client.get_joint_values("02_Pick","example_01_pick_place")
    jv_prePlace = world_view_client.get_joint_values("03_PrePlace","example_01_pick_place")
    jv_place = world_view_client.get_joint_values("04_Place","example_01_pick_place") 

    loop = asyncio.get_event_loop()
    # register the loop handler to be shutdown appropriately
    register_asyncio_shutdown_handler(loop)

    async def move_supervised(joint_values: JointValues, velocity_scaling=1):
        """Opens a window in rosvita to let the user supervise the motion to joint values """
        stepped_client = move_group.move_joints_collision_free_supervised(joint_values, 
                                                        velocity_scaling = velocity_scaling)
        robot_chat_stepped_motion = RobotChatSteppedMotion(robot_chat,
                                                        stepped_client,
                                                        move_group.name)
        await robot_chat_stepped_motion.handle_stepwise_motions()

    async def prepare_gripper():
        print("Home the gripper and move to joints") 
        # Allows parallel homing and supervised movement
        t1 = wsg_gripper.homing()
        t2 = move_supervised(jv_home)
        await asyncio.gather(t1, t2)

    async def go_to_prepick():
        print("Go to prepick position and open gripper")
        # Allows parallel adjustment of the gripper and movement of the end effector
        t1 = move_group.move_joints(jv_prePick, velocity_scaling=0.5)
        t2 = wsg_gripper.grasp(0.02, 1, 0.05 )
        await asyncio.gather(t1, t2)

    async def pick_up():
        """Simple example of a "picking up" motion """
        print("Pick up")
        await move_group.move_joints(jv_pick, velocity_scaling=0.04)
        await wsg_gripper.grasp(0.002, 0.1, 0.05)
        await move_group.move_joints(jv_prePick)

    async def go_to_preplace():
        print("Go to preplace position")
        await move_group.move_joints(jv_prePlace)   

    async def place():
        """Simple example of a "placing" motion """
        print("Place")
        await move_group.move_joints(jv_place, velocity_scaling=0.04)
        await wsg_gripper.grasp(0.04, 1, 0.05)
        await move_group.move_joints(jv_prePlace)  

    async def pick_and_place():
        """Actions to be repeated in loop"""
        await go_to_prepick()
        await pick_up()
        await go_to_preplace()
        await place()

    async def return_home():
        print("Return home")
        await move_group.move_joints_collision_free(jv_home)

    try:
        # plan a trajectory to the begin pose
        loop.run_until_complete(prepare_gripper())
        for i in range(loopCount):
            loop.run_until_complete(pick_and_place())      
        loop.run_until_complete(return_home())
    finally:
        loop.close()

if __name__ == '__main__':
    loopCount = 2
    main(loopCount)
