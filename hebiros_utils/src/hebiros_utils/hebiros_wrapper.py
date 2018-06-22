"""
# Name: hebiros_wrapper.py
# Company: MetaMorph, Inc.
# Author(s): Joseph Coombe
# Create Date: 04/13/2018
# Edit Date: 06/20/2018
#
# Description:
#   Convenience class to cut down on boilerplate code when using hebiros package (http://wiki.ros.org/hebiros)
#   from Python.
"""
from functools import wraps

import rospy
from sensor_msgs.msg import JointState
from actionlib import SimpleActionClient
from hebiros.srv import AddGroupFromNamesSrv, AddGroupFromURDFSrv, EntryListSrv, SendCommandWithAcknowledgementSrv, \
    SetCommandLifetimeSrv, SetFeedbackFrequencySrv, SizeSrv
from hebiros.msg import CommandMsg, EntryListMsg, EntryMsg, FeedbackMsg, PidGainsMsg, SettingsMsg, WaypointMsg
from hebiros.msg import TrajectoryAction, TrajectoryGoal


class HebirosWrapper(object):
    def __init__(self, hebi_group_name, hebi_families=None, hebi_names=None, from_urdf=False, lite_mode=False):
        """
        :type hebi_group_name: str
        :param hebi_group_name: Unique name for set of HEBI Actuators

        :type hebi_families: list[str]
        :param hebi_families: Ordered list of HEBI Actuator Families, e.g.['LeftArm', 'RightArm', 'HEBI']

        :type hebi_names: list[str]
        :param hebi_names: Ordered list of HEBI Actuators Families / Name, e.g.['Hip', 'Knee', 'Ankle']

        :type from_urdf: bool
        :param from_urdf: Create HEBI Group from URDF on ROS Parameter Server (/robot_description)

        :type lite_mode: bool
        :param lite_mode: If True, do not create any common Subscribers, Publishers, or Service/Action Clients
        """

        self.hebi_group_name = hebi_group_name

        if not from_urdf:
            if not isinstance(hebi_families, list) or not isinstance(hebi_names, list):
                raise TypeError("hebi_families and hebi_names must be of type list!")
            if len(hebi_families) != 1 and len(hebi_families) != len(hebi_names):
                raise ValueError("Invalid number of families and names for group {}".format(hebi_group_name))

            self.hebi_families = hebi_families
            self.hebi_names = hebi_names

            add_group_from_names = rospy.ServiceProxy('/hebiros/add_group_from_names', AddGroupFromNamesSrv)
            rospy.loginfo("  Waiting for AddGroupFromNamesSrv at %s ...", '/hebiros/add_group_from_names')
            rospy.wait_for_service('/hebiros/add_group_from_names')  # block until service server starts
            rospy.loginfo("  AddGroupFromNamesSrv AVAILABLE.")
            add_group_from_names(hebi_group_name, hebi_names, hebi_families)

        else:  # from_urdf
            add_group_from_urdf = rospy.ServiceProxy('/hebiros/add_group_from_urdf', AddGroupFromURDFSrv)
            rospy.loginfo("  Waiting for AddGroupFromURDFSrv at %s ...", '/hebiros/add_group_from_urdf')
            rospy.wait_for_service('/hebiros/add_group_from_urdf')  # block until service server starts
            rospy.loginfo("  AddGroupFromURDFSrv AVAILABLE.")
            add_group_from_urdf(hebi_group_name)

            entry_list = rospy.ServiceProxy('/hebiros/entry_list', EntryListSrv)
            rospy.wait_for_service('/hebiros/entry_list')  # block until service server starts
            srv_reponse = entry_list()
            self.hebi_families = [entry.family for entry in srv_reponse.entry_list.entries]
            self.hebi_names = [entry.name for entry in srv_reponse.entry_list.entries]

        self.hebi_mapping = [family + "/" + name for family, name in zip(self.hebi_families, self.hebi_names)]
        self.hebi_count = len(self.hebi_mapping)

        # Topics
        self.feedback_topic = '/hebiros/'+hebi_group_name+'/feedback'
        rospy.loginfo("  feedback_topic: %s", self.feedback_topic)
        self.feedback_joint_state_topic = '/hebiros/'+hebi_group_name+'/feedback/joint_state'
        rospy.loginfo("  feedback_joint_state_topic: %s", self.feedback_joint_state_topic)
        self.command_topic = '/hebiros/'+hebi_group_name+'/command'
        rospy.loginfo("  command_topic: %s", self.command_topic)
        self.command_joint_state_topic = '/hebiros/'+hebi_group_name+'/command/joint_state'
        rospy.loginfo("  command_joint_state_topic: %s", self.command_joint_state_topic)

        # Services
        self.entry_list_srv = '/hebiros/entry_list'
        rospy.loginfo("  entry_list_srv: %s", self.entry_list_srv)
        self.send_command_with_acknowledgement_srv = '/hebiros/'+hebi_group_name+'/send_command_with_acknowledgement'
        rospy.loginfo("  send_command_with_acknowledgement_srv: %s", self.send_command_with_acknowledgement_srv)
        self.set_command_lifetime_srv = '/hebiros/'+hebi_group_name+'/set_command_lifetime'
        rospy.loginfo("  set_command_lifetime_srv: %s", self.set_command_lifetime_srv)
        self.set_feedback_frequency_srv = '/hebiros/'+hebi_group_name+'/set_feedback_frequency_srv'
        rospy.loginfo("  set_feedback_frequency_srv: %s", self.set_feedback_frequency_srv)
        self.size_srv = '/hebiros/'+hebi_group_name+'/size'
        rospy.loginfo("  size_srv: %s", self.size_srv)

        # Actions
        self.trajectory_action = '/hebiros/'+hebi_group_name+'/trajectory'
        rospy.loginfo("  trajectory_action: %s", self.trajectory_action)

        self._lite_mode = lite_mode

        if not lite_mode:
            # Service Clients
            self.entry_list = rospy.ServiceProxy(self.entry_list_srv, EntryListSrv)
            rospy.loginfo("  Waiting for EntryListSrv at %s ...", self.entry_list)
            rospy.wait_for_service(self.entry_list_srv)  # block until service server starts
            rospy.loginfo("  EntryListSrv AVAILABLE.")

            self.send_command_with_acknowledgement = rospy.ServiceProxy(self.send_command_with_acknowledgement_srv,
                                                                        SendCommandWithAcknowledgementSrv)
            rospy.loginfo("  Waiting for SendCommandWithAcknowledgementSrv at %s ...",
                          self.send_command_with_acknowledgement)
            rospy.wait_for_service(self.send_command_with_acknowledgement_srv)  # block until service server starts
            rospy.loginfo("  SendCommandWithAcknowledgementSrv AVAILABLE.")

            self.set_command_lifetime = rospy.ServiceProxy(self.set_command_lifetime_srv, SetCommandLifetimeSrv)
            rospy.loginfo("  Waiting for SetCommandLifetimeSrv at %s ...", self.set_command_lifetime)
            rospy.wait_for_service(self.set_command_lifetime_srv)  # block until service server starts
            rospy.loginfo("  SetCommandLifetimeSrv AVAILABLE.")

            self.set_feedback_frequency = rospy.ServiceProxy(self.set_feedback_frequency_srv, SetFeedbackFrequencySrv)
            rospy.loginfo("  Waiting for SetFeedbackFrequencySrv at %s ...", self.set_feedback_frequency)
            rospy.wait_for_service(self.set_command_lifetime_srv)  # block until service server starts
            rospy.loginfo("  SetFeedbackFrequencySrv AVAILABLE.")

            self.size = rospy.ServiceProxy(self.size_srv, SizeSrv)
            rospy.loginfo("  Waiting for SizeSrv at %s ...", self.size)
            rospy.wait_for_service(self.size_srv)  # block until service server starts
            rospy.loginfo("  SizeSrv AVAILABLE.")

            # Simple Action Client
            self.trajectory_action_client = SimpleActionClient(self.trajectory_action, TrajectoryAction)
            rospy.loginfo("  Waiting for TrajectoryActionServer at %s ...", self.trajectory_action)
            self.trajectory_action_client.wait_for_server()  # block until action server starts
            rospy.loginfo("  TrajectoryActionServer AVAILABLE.")

            # Publishers
            self.joint_state_publisher = rospy.Publisher(self.command_joint_state_topic, JointState, queue_size=1)

            # Subscribers
            self.joint_state_subscriber = rospy.Subscriber(self.feedback_joint_state_topic, JointState,
                                                           self._feedback_joint_state_cb)
            self.joint_state_feedback = JointState()

            # Additional callback functions
            self._additional_feedback_callbacks = []  # NOTE: Careful. Should be short & sweet (i.e. non-blocking).

    @classmethod
    def from_names(cls, hebi_group_name, hebi_families=None, hebi_names=None, lite_mode=False):
        return cls(hebi_group_name, hebi_families, hebi_names, lite_mode=lite_mode)

    @classmethod
    def from_urdf(cls, hebi_group_name, lite_mode=False):
        return cls(hebi_group_name, from_urdf=True, lite_mode=lite_mode)

    def _feedback_joint_state_cb(self, msg):
        assert isinstance(msg, JointState)
        self.joint_state_feedback = msg
        # Execute additional callbacks
        for cb in self._additional_feedback_callbacks:
            cb(msg)

    def lite_mode(self):
        return self._lite_mode

    def _not_available_in_lite_mode(method):
        @wraps(method)
        def wrapper(self, *args, **kwargs):
            if self.lite_mode():
                rospy.logwarn("Not available in Lite mode...")
                return None
            else:
                return method(self, *args, **kwargs)

        return wrapper

    @_not_available_in_lite_mode
    def add_feedback_callback(self, cb):
        self._additional_feedback_callbacks.append(cb)

    @_not_available_in_lite_mode
    def remove_feedback_callback(self, cb):
        self._additional_feedback_callbacks.remove(cb)

    @_not_available_in_lite_mode
    def get_joint_state_feedback(self):
        return self.joint_state_feedback

    @_not_available_in_lite_mode
    def get_joint_positions(self):
        return self.get_joint_state_feedback().position

    @_not_available_in_lite_mode
    def get_joint_velocities(self):
        return self.get_joint_state_feedback().velocity

    @_not_available_in_lite_mode
    def get_joint_efforts(self):
        return self.get_joint_state_feedback().effort
