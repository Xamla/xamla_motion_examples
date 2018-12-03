from xamla_motion.motion_client import MoveGroup
from xamla_motion.gripper_client import WeissWsgGripperProperties, WeissWsgGripper

"""Some utility functions specific to currently used robot and gripper """

def get_move_group() -> MoveGroup:
    return MoveGroup("/sda10f/right_arm_torso")

def get_gripper(move_group: MoveGroup) -> WeissWsgGripper:
    # create instance of wsg gripper by name
    properties = WeissWsgGripperProperties('wsg50')
    return  WeissWsgGripper(properties, move_group.motion_service)

def get_right_move_group() -> MoveGroup:
    return MoveGroup("/sda10f/sda10f_r2_controller")

def get_left_move_group() -> MoveGroup:
    return MoveGroup("/sda10f/sda10f_r1_controller")

def get_full_body_move_group() -> MoveGroup:
    return MoveGroup("/sda10f")

def get_torso_joint_name() -> str:
    return "torso_joint_b1"

