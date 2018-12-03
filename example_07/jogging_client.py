
import rospy
import actionlib

from datetime import timedelta

from xamla_motion.data_types import Pose, JointValues
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from xamlamoveit_msgs.srv import SetString

from std_srvs.srv import SetBool

from xamla_motion.utility import ROSNodeSteward
from xamla_motion.xamla_motion_exceptions.exceptions import ServiceException


class JoggingClient(object):

    __setpoint_topic = "/xamlaJointJogging/jogging_setpoint"
    __jogging_command_topic = "/xamlaJointJogging/jogging_command"
    
    __set_move_group_service_name = "/xamlaJointJogging/set_movegroup_name"
    __toggle_tracking_service_name = "xamlaJointJogging/start_stop_tracking"

    def __init__(self):
        self.__ros_node_steward = ROSNodeSteward()
        self._create_connect_topics()
        self._create_connect_services()


    def _create_connect_topics(self):
        self._set_point = rospy.Publisher(self.__setpoint_topic, 
                                        PoseStamped,
                                        queue_size=5)
        self._jogging_command = rospy.Publisher(self.__jogging_command_topic, 
                                            JointTrajectory, 
                                            queue_size=5)

    def _create_connect_services(self):
        try:
            self.__set_move_group_service = rospy.ServiceProxy(
                self.__set_move_group_service_name,
                SetString)
        except rospy.ServiceException as exc:
            raise ServiceException('connection for service with name: ' +
                                   self.__set_move_group_service_name +
                                   ' could not be established') from exc

        try:
            self.__toggle_tracking_service = rospy.ServiceProxy(
                self.__toggle_tracking_service_name,
                SetBool)
        except rospy.ServiceException as exc:
            raise ServiceException('connection for service with name: ' +
                                   self.__toggle_tracking_service_name +
                                   ' could not be established') from exc


    def send_set_point(self, setPoint: Pose):
        pose_msg  = setPoint.to_posestamped_msg() 
        self._set_point.publish(pose_msg)


    def send_velocities(self, velocities: JointValues):
        point = JointTrajectoryPoint(
            time_from_start = rospy.Duration.from_sec(0.008), #  timedelta(seconds=0.008),
            velocities = velocities.values)
        trajectory = JointTrajectory(
            joint_names = velocities.joint_set.names,
            points= [point])
        self._jogging_command.publish(trajectory)


    def set_move_group_name(self, name: str):
        try:
            response = self.__set_move_group_service(name)
        except rospy.ServiceException as exc:
            raise ServiceException('service call for query'
                                   ' set move group'
                                   ' failed, abort') from exc
        print(response)


    def toggle_tracking(self, toggle: bool):
        try:
            response = self.__toggle_tracking_service(toggle)
        except rospy.ServiceException as exc:
            raise ServiceException('service call for query'
                                   ' set toggle'
                                   ' failed, abort') from exc
        print(response)
