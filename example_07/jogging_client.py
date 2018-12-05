
import rospy
import actionlib

from datetime import timedelta

from xamla_motion.data_types import Pose, JointValues # , Twist
#TODO: move this to xamla_motion.data_types
from example_07.twist import Twist

from geometry_msgs.msg import PoseStamped, TwistStamped
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from xamlamoveit_msgs.msg import ControllerState 
from xamlamoveit_msgs.srv import GetSelected, SetString
from xamlamoveit_msgs.srv import GetFloat, SetFloat 
from xamlamoveit_msgs.srv import GetFlag, SetFlag 

from std_srvs.srv import SetBool

from xamla_motion.utility import ROSNodeSteward
from xamla_motion.xamla_motion_exceptions.exceptions import ServiceException


class JoggingClient(object):

    __setpoint_topic = "/xamlaJointJogging/jogging_setpoint"
    __jogging_command_topic = "/xamlaJointJogging/jogging_command"
    __jogging_twist_topic = "/xamlaJointJogging/jogging_twist"
    __jogging_feedback_topic = "/xamlaJointJogging/feedback"

    __toggle_tracking_service_id = "/xamlaJointJogging/start_stop_tracking"
    __get_move_group_name_service_id = "/xamlaJointJogging/get_movegroup_name"
    __set_move_group_name_service_id = "/xamlaJointJogging/set_movegroup_name"
    __get_endeffector_name_service_id = "/xamlaJointJogging/get_endeffector_name"
    __set_endeffector_name_service_id = "/xamlaJointJogging/set_endeffector_name"
    __status_service_id = "xamlaJointJogging/status"
    __get_velocity_scaling_service_id = "/xamlaJointJogging/get_velocity_scaling"
    __set_velocity_scaling_service_id = "/xamlaJointJogging/set_velocity_scaling"
    __get_flag_service_id= "xamlaJointJogging/get_flag"
    __set_flag_service_id= "xamlaJointJogging/set_flag"

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
        self._jogging_twist = rospy.Publisher(self.__jogging_twist_topic, 
                                            TwistStamped, 
                                            queue_size=5)
        self.__feedback_sub = rospy.Subscriber(self.__jogging_feedback_topic,
                                            ControllerState,
                                            callback=self._handle_jogging_feedback,
                                            queue_size=1)

    def _create_connect_services(self):
        
        # utility function for dry purpose
        def exc_wrap_call(name, msg_type):
            try:
                return rospy.ServiceProxy(name, msg_type)
            except rospy.ServiceException as exc:
                raise ServiceException('connection for service with name: ' +
                                   name +
                                   ' could not be established') from exc

        self.__toggle_tracking_service = exc_wrap_call(
                self.__toggle_tracking_service_id, SetBool)
        self.__get_velocity_scaling_service = exc_wrap_call(
                self.__get_velocity_scaling_service_id, GetFloat)
        self.__set_velocity_scaling_service = exc_wrap_call(
                self.__set_velocity_scaling_service_id, SetFloat)
        self.__get_move_group_name_service = exc_wrap_call(
                self.__get_move_group_name_service_id,GetSelected)
        self.__set_move_group_name_service = exc_wrap_call(
            self.__set_move_group_name_service_id, SetString)
            
        self.__get_endeffector_name_service = exc_wrap_call(
            self.__get_endeffector_name_service_id,GetSelected)
        self.__set_endeffector_name_service = exc_wrap_call(
            self.__set_endeffector_name_service_id, SetString)
        self.__get_flag_service = exc_wrap_call(self.__get_flag_service_id, GetFlag)
        self.__set_flag_service = exc_wrap_call(self.__set_flag_service_id,SetFlag)

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

    def send_twist(self, twist: Twist):
        twist_stamped = twist.to_twiststamped_msg()
        self._jogging_twist.publish(twist_stamped)


    @staticmethod
    def _exc_wrap_service_call(service_call_func, query_desc, *argv):
        """ Utility function for a exception handling when calling a service"""
        try:
            response = service_call_func(*argv)
        except rospy.ServiceException as exc:
            raise ServiceException('service call for query' +
                                   query_desc +
                                   ' failed, abort') from exc
        return response

    def get_velocity_scaling(self) -> float:
        response = self._exc_wrap_service_call(self.__get_velocity_scaling_service, ' get velocity scaling ')
        return response.data

    def set_velocity_scaling(self, value: float) -> None:
        response = self._exc_wrap_service_call(self.__set_velocity_scaling_service, 
                                            ' set velocity scaling',  value)

    def get_move_group_name(self) -> str:
        response = self._exc_wrap_service_call(self.__get_move_group_name_service, 
                                            ' get move group')
        response.selected
        return response.collection

    def set_move_group_name(self, name: str) -> None:
        response = self._exc_wrap_service_call(self.__set_move_group_name_service, 
                                        ' set move group with name {} '.format(name), name)

    def get_endeffector_name(self) -> str:
        response = self._exc_wrap_service_call(self.__get_endeffector_name_service, 
                                            ' get endeffector')
        response.selected
        return response.collection

    def set_endeffector_name(self, name: str) -> None:
        response = self._exc_wrap_service_call(self.__set_endeffector_name_service, 
                                        ' set endeffector with name {} '.format(name), name)

    def get_flag(self, name: "str") -> bool:
        response = self._exc_wrap_service_call(self.__get_flag_service, 
                                    ' get flag with name {} '.format(name), name)
        return response.value

    def set_flag(self, name: str, value: bool) -> None:
        response = self._exc_wrap_service_call(self.__set_flag_service, 
                                    ' set flag with name {} '.format(name), name, value)

    def toggle_tracking(self, toggle: bool) -> None:
        response = self._exc_wrap_service_call(self.__toggle_tracking_service, 
                                    ' toggle tracking ', toggle)

    def start(self):
        self.toggle_tracking(True)

    def stop(self):
        self.toggle_tracking(False)

    def _handle_jogging_feedback(self, state):
        # TODO: implement this properly
        if not state.error_code == 1:
            print("COLLISION OCCURRED") 