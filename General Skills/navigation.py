#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy

from uchile_skills.robot_skill import RobotSkill

## CORE imports
# knowledge base
from maqui_skills.core.knowledge import KnowledgeSkill

# hardware
from maqui_skills.core.base import BaseSkill


## messages and services
from std_srvs.srv import Empty
from uchile_msgs.msg import SemanticObject
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from uchile_srvs.srv import NavGoal
from uchile_srvs.srv import PoseStamped as PoseStampedSRV
from uchile_srvs.srv import Transformer


class NavigationSkill(RobotSkill):
    """
    The NavigationSkill

    TODO
    """
    _type = "navigation"

    def __init__(self):
        super(NavigationSkill, self).__init__()
        self._description = "the navigation skill"
        self.register_dependency(KnowledgeSkill.get_type())
        self.register_dependency(BaseSkill.get_type())

        self.search_positions = []
        self.search_positions_name = []
        self.detected_positions = []
        self.threshold_pose = 1.5

        # clients
        self.go_client        = rospy.ServiceProxy('/go', NavGoal)
        self.look_client      = rospy.ServiceProxy('/look', NavGoal)
        #self.approach_client  = rospy.ServiceProxy('/maqui/nav/goal_server/approach', NavGoal)
        self.cancel_client    = rospy.ServiceProxy('/cancel', Empty)
        self.arrived_client   = rospy.ServiceProxy('/has_arrived', NavGoal)
        #self.transform_client = rospy.ServiceProxy('/maqui/tf/simple_pose_transformer/transform', Transformer)
        self.current_pose_client   = rospy.ServiceProxy('/get_current_pose', PoseStampedSRV)
        #self.clear_costmaps_client = rospy.ServiceProxy('/maqui/nav/move_base/clear_costmaps', Empty)

        # Set Subscribers when node is ready
        #self.sub_localization = rospy.Subscriber("/maqui/nav/amcl_pose", PoseWithCovarianceStamped, self.localization_cb, queue_size=1)

    def check(self):
        rospy.loginfo("{skill: %s}: check()." % self._type)
        try:
            self.go_client.wait_for_service(timeout=2)
            self.look_client.wait_for_service(timeout=2)
            #self.approach_client.wait_for_service(timeout=2)
            self.cancel_client.wait_for_service(timeout=2)
            self.arrived_client.wait_for_service(timeout=2)
            #self.transform_client.wait_for_service(timeout=2)
            self.current_pose_client.wait_for_service(timeout=2)
            #self.clear_costmaps_client.wait_for_service(timeout=2)
        except rospy.ROSException:
            rospy.logwarn('[navigation]: Check failed. PoseServer services not found. e.g. /maqui/nav/goal_server/go')
            return False
        return True

    def setup(self):
        rospy.loginfo("{skill: %s}: setup()... waiting for services" % self._type)
        self.go_client.wait_for_service()
        self.look_client.wait_for_service()
        #self.approach_client.wait_for_service()
        self.cancel_client.wait_for_service()
        self.arrived_client.wait_for_service()
        #self.transform_client.wait_for_service()
        self.current_pose_client.wait_for_service()
        #self.clear_costmaps_client.wait_for_service()
        return True

    def shutdown(self):
        rospy.loginfo("{skill: %s}: shutdown()." % self._type)
        return self.cancel()

    def start(self):
        rospy.loginfo("{skill: %s}: start()." % self._type)
        return True

    def pause(self):
        rospy.loginfo("{skill: %s}: pause()." % self._type)
        return self.cancel()

    # =========================================================================
    # MISC
    # =========================================================================
    """
    map analyzer stuff
    prepare kinect
    validate input poses
    input in Roll Pitch yaw and X,Y.Z.W
    """
    def where_i_am(self):
        """
        returns the current robot pose as a Pose object
        relative to the map frame
        returns None on failure
        """
        try:
            res = self.current_pose_client()
            return res.pose_out.pose
        except rospy.ServiceException:
            print("FALLO!")
            return None
        return None

    def save_current_position(self, name, map_name=None):
        """
        Saves the current robot position into the sem_map file.

        It requires the pose name as a string, but pls, do not use spaces!

        returns True on success, False otherwise.
        """
        curr_pose = self.robot.navigation.where_i_am()
        if curr_pose is not None:
            place           = SemanticObject()
            place.id        = name
            place.type      = 'map_pose'
            place.frame_id  = 'map'
            place.pose      = curr_pose
            if self.robot.knowledge.pose.set(place) and self.robot.knowledge.pose.save_to_map(map_name):
                return True
        return False

    # =========================================================================
    # MAPPING
    # =========================================================================
    """
    save map
    reset map
    """

    # =========================================================================
    # LOCALIZATION
    # =========================================================================
    """
    pose estimation
    """

    # =========================================================================
    # PLANNING
    # =========================================================================
    """
    - cuanto falta?
    - darse cuenta si ha dado muchas vueltas en la misma posicion.
    """

    #def clear_costmaps(self):
    #    """
    #    clears the planning costmaps. It is useful when you have kidnapped
    #    the robot, or when you see in RVIZ that costmaps are too dirty
    #    returns True on success
    #    """
    #    try:
    #        self.clear_costmaps_client()
    #    except rospy.ServiceException:
    #        return False
    #    return True

    def wait_for_result(self, timeout=None, cancel_on_timeout=True):
        """
        waits for the robot to stop moving or a timeout [s]
        If there is a timeout, the goal will be cancelled (by default). You
        can override this behavior by setting cancel_on_timeout=False

        The robot could be stopped by 3 reasons:
        - robot has never moved
        - robot has reached the goal (see: navigation.reached())
        - robot failed to reach the goal and aborted the mission

        returns True if the robot has stopped
        returns False on timeout
        """
        elapsed_time = 0.0
        while not rospy.is_shutdown():
            rospy.sleep(0.5)

            # ready
            if not self.is_moving():
                if not self.reached():
                    return False
                return True

            elapsed_time += 0.5
            if timeout is not None and elapsed_time > timeout:
                if cancel_on_timeout:
                    self.cancel()
                return False

    def reached(self):
        """
        returns True if the robot has reached the last queued goal
        returns False otherwise
        """
        try:
            res = self.arrived_client()
            if res.state == 3:
                return True
        except rospy.ServiceException:
            return False
        return False

    def is_moving(self):
        """
        returns True if the robot is currently in motion
        returns False otherwise
        """
        try:
            res = self.arrived_client()

            if res.state in [0, 3, 4, 5]:
                return False
            return True

        except rospy.ServiceException:
            return False

    def cancel(self):
        """
        cancels a previous sended goal if any
        """
        try:
            self.cancel_client()
        except rospy.ServiceException:
            return False
        # use robot.base.stop()
        return True

    def go(self, location_name):
        """
        Sets a goal for the robot to move on and returns immediately.

        This method is non-blocking.

        The goal pose is retrieved from the knowledge.pose skill
        through the robot pose server

        returns True on success
        returns False on failed (place is unknown or service call failed)
        """
        knowledge = self.robot.knowledge
        location = knowledge.pose.get(location_name)
        if location is None:
            return False
        goal = PoseStamped()
        goal.header.frame_id = location.frame_id
        goal.pose = location.pose
        return self._set_go_goal(goal)

    def approach(self, location_name):
        """
        Sets an approach goal for the robot to move on and returns immediately.
        See also, navigation.go()
        """
        knowledge = self.robot.knowledge
        location = knowledge.pose.get(location_name)
        if location is None:
            return False
        goal = PoseStamped()
        goal.header.frame_id = location.frame_id
        goal.pose = location.pose
        return self._set_approach_goal(goal)

    def look(self, location_name):
        """
        Sets a goal in-place, but poiting to the desired location
        See also, navigation.go()
        """
        knowledge = self.robot.knowledge
        location = knowledge.pose.get(location_name)
        if location is None:
            return False
        goal = PoseStamped()
        goal.header.frame_id = location.frame_id
        goal.pose = location.pose
        return self._set_look_goal(goal)

    def go_to_pose(self, pose, use_robot_frame=False):
        """
        Sets a goal from a Pose object relative to the map frame (default)

        Setting use_robot_frame to True, will make the goal relative to the
        robot. Then, the performed motion will be equivalent as if the robot
        moved in a straight line and then performs the rotation.

        See also, navigation.go()
        """
        goal = PoseStamped()
        goal.header.frame_id = self._parse_frame_id(use_robot_frame)
        goal.pose = pose
        return self._set_go_goal(goal)

    def transform_pose_stamped(self, pose_stamped):
        """
        Transforms a PoseStamped object to the map frame

        Args:
            pose_stamped (PoseStamped): The pose to transform.

        Returns:
            PoseStamped: Transformed pose

        Raises:
            Exception: When the transform fails
        """
        transform = Transformer()
        transform.pose_in = pose_stamped
        transform.frame_out = 'map'

        try:
            return self._set_transform_goal(transform)
        except Exception as e:
            rospy.logwarn("Failed to transform pose from '%s' to the /map frame.", pose_stamped.header.frame_id)
            raise e

    def go_to_pose_stamped(self, pose_stamped):
        """
        Sets a goal from a PoseStamped object

        See also, navigation.go()
        """
        try:
            goal = self.transform_pose_stamped(pose_stamped)
            self._set_go_goal(goal)
        except Exception:
            return False
        return True

    def look_to_pose(self, pose, use_robot_frame=False):
        """
        Sets a look goal from a Pose object, relative to the map frame (default)

        Setting use_robot_frame to True, will make the goal relative to the
        robot.

        See also, navigation.look()
        """
        goal = PoseStamped()
        goal.header.frame_id = self._parse_frame_id(use_robot_frame)
        goal.pose = pose
        return self._set_look_goal(goal)

    def approach_to_pose(self, pose, use_robot_frame=False):
        """
        Sets an approach goal from a Pose object relative to the map frame (default)

        See also, navigation.go_to_pose()
        """
        goal = PoseStamped()
        goal.header.frame_id = self._parse_frame_id(use_robot_frame)
        goal.pose = pose
        return self._set_approach_goal(goal)

    def go_to_point(self, x=0.0, y=0.0, degrees=0, use_robot_frame=False):
        """
        See navigation.go_to_pose()
        """
        goal = PoseStamped()
        goal.header.frame_id = self._parse_frame_id(use_robot_frame)
        goal.pose = self._build_pose(x, y, degrees)
        return self._set_go_goal(goal)

    def look_to_point(self, x=0.0, y=0.0, use_robot_frame=False):
        """
        See navigation.look_to_pose()
        """
        goal = PoseStamped()
        goal.header.frame_id = self._parse_frame_id(use_robot_frame)
        goal.pose = self._build_pose(x, y, 0)
        return self._set_look_goal(goal)

    def approach_to_point(self, x=0.0, y=0.0, degrees=0, use_robot_frame=False):
        """
        See navigation.go_to_pose()
        """
        goal = PoseStamped()
        goal.header.frame_id = self._parse_frame_id(use_robot_frame)
        goal.pose = self._build_pose(x, y, degrees)
        return self._set_approach_goal(goal)

    def rotate(self, degrees):
        """
        Changes the orientation of the robot
        """
        radians = math.radians(degrees)
        rotate_x = math.cos(radians)
        rotate_y = math.sin(radians)
        return self.look_to_point(x=rotate_x, y=rotate_y, use_robot_frame=True)

    def _build_pose(self, x, y, degrees):
        radians = math.radians(degrees)
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.orientation.z = math.sin(radians / 2.0)
        pose.orientation.w = math.cos(radians / 2.0)
        return pose

    def _parse_frame_id(self, use_robot_frame=False):
        return "maqui/base_link" if use_robot_frame else "map"

    def _set_go_goal(self, pose_stamped):
        pose_stamped.header.stamp = rospy.Time.now()
        return self._client_caller(self.go_client, pose_stamped)

    def _set_look_goal(self, pose_stamped):
        pose_stamped.header.stamp = rospy.Time.now()
        return self._client_caller(self.look_client, pose_stamped)

    def _set_approach_goal(self, pose_stamped):
        pose_stamped.header.stamp = rospy.Time.now()
        return self._client_caller(self.approach_client, pose_stamped)

    def _set_transform_goal(self, transformer):
        #transformer.pose_in.header.stamp = rospy.Time.now()
        return self._client_transform_caller(self.transform_client, transformer)

    def _client_transform_caller(self, client, transformer):
        #transformer.pose_in.header.stamp = rospy.Time.now()
        if transformer.pose_in.pose.orientation.w == 0.0:
            transformer.pose_in.pose.orientation.w = 1.0

#       try:
#           transformed_pose = client(transformer.pose_in, transformer.frame_out)
#           return transformed_pose.pose_out
#       except rospy.ServiceException:
#           return False

        transformed_pose = client(transformer.pose_in, transformer.frame_out)
        return transformed_pose.pose_out

    def _client_caller(self, client, pose_stamped):
        pose_stamped.header.stamp = rospy.Time.now()
        try:
            client(goal=pose_stamped)
            return True
        except rospy.ServiceException:
            return False

    # =========================================================================
    # Methods to see if maqui passes through certain map points
    # =========================================================================
    def set_search_position(self, list_name=[], list_pose=[]):
        self.search_positions_name = list_name
        self.search_positions = list_pose

    def get_search_position(self):
        return self.detected_positions

    def localization_cb(self, posecovar):
        bender_x = posecovar.pose.pose.position.x
        bender_y = posecovar.pose.pose.position.y

        d = -1
        for id, position in enumerate(self.search_positions):
            if d == -1 or d > math.sqrt((bender_x - position[0]) ** 2 + (bender_y - position[1]) ** 2):
                d = math.sqrt((bender_x - position[0]) ** 2 + (bender_y - position[1]) ** 2)
            if math.sqrt((bender_x - position[0]) ** 2 + (bender_y - position[1]) ** 2) < self.threshold_pose:
                if not self.search_positions_name[id] in self.detected_positions:
                    self.detected_positions.append(self.search_positions_name[id])

############### UNUSED ROS INTERFACE #######################################
## Services
# /maqui/nav/move_base/NavfnROS/make_plan
# /maqui/nav/move_base/TrajectoryPlannerROS/set_parameters
# /maqui/nav/move_base/clear_costmaps
# /maqui/nav/move_base/global_costmap/inflation_layer/set_parameters
# /maqui/nav/move_base/global_costmap/obstacle_layer/set_parameters
# /maqui/nav/move_base/global_costmap/set_parameters
# /maqui/nav/move_base/global_costmap/static_layer/set_parameters
# /maqui/nav/move_base/local_costmap/inflation_layer/set_parameters
# /maqui/nav/move_base/local_costmap/obstacle_layer/set_parameters
# /maqui/nav/move_base/local_costmap/set_parameters
# /maqui/nav/move_base/make_plan
# /maqui/nav/move_base/set_parameter
# /maqui/nav/request_nomotion_update
# /maqui/nav/set_map

## TOPICS
# /maqui/nav/amcl/parameter_descriptions
# /maqui/nav/amcl/parameter_updates
# /maqui/nav/amcl/scan
# /maqui/nav/amcl_pose
# /maqui/nav/base/cmd_vel
# /maqui/nav/cmd_vel
# /maqui/nav/cmd_vel_mux/active
# /maqui/nav/cmd_vel_mux/input/safety
# /maqui/nav/cmd_vel_mux/parameter_descriptions
# /maqui/nav/cmd_vel_mux/parameter_updates
# /maqui/nav/cmd_vel_mux_nav_nodelet_manager/bond
# /maqui/nav/goal_server/goal
# /maqui/nav/goal_server/points_approach
# /maqui/nav/goal_server/polygon_approach
# /maqui/nav/initialpose
# /maqui/nav/move_base/NavfnROS/plan
# /maqui/nav/move_base/TrajectoryPlannerROS/cost_cloud
# /maqui/nav/move_base/TrajectoryPlannerROS/global_plan
# /maqui/nav/move_base/TrajectoryPlannerROS/local_plan
# /maqui/nav/move_base/cancel
# /maqui/nav/move_base/cmd_vel
# /maqui/nav/move_base/current_goal
# /maqui/nav/move_base/feedback
# /maqui/nav/move_base/global_costmap/costmap
# /maqui/nav/move_base/global_costmap/footprint
# /maqui/nav/move_base/global_costmap/inflated_obstacles
# /maqui/nav/move_base/goal
# /maqui/nav/move_base/local_costmap/costmap
# /maqui/nav/move_base/local_costmap/footprint
# /maqui/nav/move_base/result
# /maqui/nav/move_base/status
# /maqui/nav/move_base/twist_recovery
# /maqui/nav/move_base_simple/goal
# /maqui/nav/mux/cmd_vel
# /maqui/nav/odom
# /maqui/nav/particlecloud


if __name__ == '__main__':

    rospy.init_node("nav_test")
    nav = NavigationSkill()

    goal = PoseStamped()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "map"
    goal.pose.orientation.w = 0.0

    #nav.go_to_pose_stamped(goal)
