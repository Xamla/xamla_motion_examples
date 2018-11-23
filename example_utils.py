from xamla_motion.motion_client import MoveGroup
from xamla_motion.gripper_client import WeissWsgGripperProperties, WeissWsgGripper

"""Some utility functions specific to currently used robot and gripper """

def get_move_group() -> MoveGroup:
    return MoveGroup("/sda10f/right_arm_torso")

def get_gripper(move_group: MoveGroup) -> WeissWsgGripper:
    # create instance of wsg gripper by name
    properties = WeissWsgGripperProperties('wsg50')
    return  WeissWsgGripper(properties, move_group.motion_service)