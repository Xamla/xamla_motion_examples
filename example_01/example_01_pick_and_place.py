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

    # Plan all the movement beforehand, so that they needn't be calculated anew all the time
    prepick_to_pick_plan = move_group.move_joints(jv_pick, velocity_scaling=0.04)\
        .with_start(jv_prePick).plan() 
    pick_to_prepick_plan = move_group.move_joints(jv_prePick, velocity_scaling=1)\
        .with_start(jv_pick).plan() 
    prepick_to_preplace_plan = move_group.move_joints(jv_prePlace, velocity_scaling=1)\
        .with_start(jv_prePick).plan() 
    preplace_to_place_plan = move_group.move_joints(jv_place, velocity_scaling=0.04)\
        .with_start(jv_prePlace).plan() 
    place_to_preplace_plan = move_group.move_joints(jv_prePlace, velocity_scaling=0.04)\
        .with_start(jv_place).plan()  
    preplace_to_prepick_plan = move_group.move_joints(jv_prePick, velocity_scaling=0.5)\
        .with_start(jv_prePlace).plan() 

    loop = asyncio.get_event_loop()
    # register the loop handler to be shutdown appropriately
    register_asyncio_shutdown_handler(loop)

    async def run_supervised(stepped_motion_client):
        print('start supervised execution')

        task_next = asyncio.ensure_future(next(stepped_motion_client))

        await stepped_motion_client.action_done_future
        task_next.cancel()

        print('finished supervised execution')

    async def move_supervised(joint_values: JointValues, velocity_scaling=1):
        """Opens a window in rosvita to let the user supervise the motion to joint values """
        move_joints = move_group.move_joints(joint_values)
        # Set velocity scaling
        move_joints = move_joints.with_velocity_scaling(velocity_scaling)
        # Plan 
        move_joints_plan = move_joints.plan()
        # Get a stepped motion client from move joints 
        stepped_motion_client = move_joints_plan.execute_supervised()
        robot_chat_stepped_motion = RobotChatSteppedMotion(robot_chat,
                                                        stepped_motion_client,
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
        # Go to prepick from current configuration
        t1 = move_group.move_joints(jv_prePick, velocity_scaling=0.5).plan().execute_async()
        t2 = wsg_gripper.grasp(0.02, 1, 0.05 )
        await asyncio.gather(t1, t2)

    async def pick_up():
        """Simple example of a "picking up" motion. Use precalculated plans."""
        print("Pick up")
        await prepick_to_pick_plan.execute_async()
        await wsg_gripper.grasp(0.002, 0.1, 0.05)
        await pick_to_prepick_plan.execute_async()

    async def go_to_preplace():
        print("Go to preplace position")
        #  Use precalculated plans
        await prepick_to_preplace_plan.execute_async()   

    async def place():
        """Simple example of a "placing" motion.  Use precalculated plans."""
        print("Place")
        await preplace_to_place_plan.execute_async()
        await wsg_gripper.grasp(0.04, 1, 0.05)
        await place_to_preplace_plan.execute_async()  

    async def pick_and_place():
        """Actions to be repeated in loop."""
        await go_to_prepick()
        await pick_up()
        await go_to_preplace()
        await place()

    try:
        # Home the gripper and move to home joint values
        loop.run_until_complete(prepare_gripper())
        for i in range(loopCount):
            loop.run_until_complete(pick_and_place())     
        # Go to home configuration
        loop.run_until_complete(
            move_group.move_joints(jv_home).plan().execute_async()
            )
    finally:
        loop.close()

if __name__ == '__main__':
    loopCount = 2
    main(loopCount)
