from copy import copy
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSLivelinessPolicy, qos_profile_sensor_data
from rclpy.logging import LoggingSeverity
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import ParameterValue, ParameterType, Parameter
import shapely.geometry as sg
from turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator, TaskResult
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Quaternion
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
import math
from irobot_create_msgs.msg import KidnapStatus
from .change_parameters import ChangeParameters
from collections import deque
from .navigation_parameters import navigation_parameters

from . import topological_map
from .signpost import Signpost
from autonomous_navigation_challenge_2024_msgs.msg import PerceptedSignpostList, PerceptedSignpost
from . import change_parameters
from enum import IntEnum
from .signpost import Signpost

class State(IntEnum):
    INITIAL_STATE = -1
    GO_TO_NEXT_NODE = 0
    WAIT_AND_OBSERVE = 1
    APPROACH_TO_CENTROID = 2
    SPIN_AND_OBSERVE = 3
    OBSERVE_LEFT_RIGHT = 4
    KIDNAPPED = 5
    WAIT_NEAR_SIGNPOST = 6

class NavigationState:
    def __init__(self, state, navigation_node):
        # Global parameters
        self.state = state
        self.navigation_node = navigation_node
        self.entry_time = navigation_node.get_clock().now()
        self.navigation_pending = False

        # Approach to centroid parameters
        self.approach_steps = 3
        self.approach_index = 0

        # Spin and observe parameters
        self.turn_index = 0

        # Kidnap parameters
        self.kidnap_calculated = False
        self.new_goal_node = None
        self.new_goal_direction = None
        self.new_last_node = None
        self.new_initial_pose_stamped = None
        self.new_initial_pose_node = None
        self.kidnap_with_one_element = False

        # Near signpost parameters
        #self.has_stopped_near_signpost = False
        self.last_sopped_at_signpost = None
        self.last_navigation_state = None
    
    def can_stop_at_signpost(self):
        if self.last_sopped_at_signpost is None:
            print("last stopped at sign is none")
            return True
        curr_time = self.navigation_node.get_clock().now()
        elapsed_time = curr_time - self.last_sopped_at_signpost
        print(f"can stop? {curr_time} {self.last_sopped_at_signpost} ")
        return elapsed_time > self.navigation_node.near_signpost_delay
    
    def get(self):
        return self.state
    
    def restore_previous_state(self):
        self.navigation_node._state = self.last_navigation_state
        self.navigation_node._state.last_navigation_state = None
    
    def change_state(self, new_state):
        if new_state == State.WAIT_NEAR_SIGNPOST:
            #self.has_stopped_near_signpost = True
            self.last_sopped_at_signpost = self.navigation_node.get_clock().now()
            self.last_navigation_state = copy(self)
            print("Saved last state")
            self.navigation_node.navigator.cancelTask()
            self.navigation_pending = False
        self.state = new_state
        self.entry_time = self.navigation_node.get_clock().now()
        self.navigation_node.get_logger().info(f"State changed to {State(new_state).name}")
        self.navigation_node.get_logger().info(f"Navigation pending: {self.navigation_pending}")
        self.kidnap_with_one_element = False
        if self.state == State.APPROACH_TO_CENTROID:
            self.approach_index = (self.approach_index + 1) % (self.approach_steps + 1)
            self.turn_index = 0
        elif self.state == State.OBSERVE_LEFT_RIGHT:
            self.turn_index = (self.turn_index + 1) % 4
        elif self.state == State.GO_TO_NEXT_NODE:
            #self.has_stopped_near_signpost = False
            self.approach_index = 0
            self.turn_index = 0
        elif self.state == State.SPIN_AND_OBSERVE:
            self.turn_index = (self.turn_index + 1) % 3

class NavigationNode(Node):
    def __init__(self):
        super().__init__("navigation_node") #Init node
        self.get_logger().set_level(LoggingSeverity.INFO)
        self.get_logger().info("Initializing navigation node")

        self.param_listener = navigation_parameters.ParamListener(self)

        self.mock_kidnap = self.get_parameter("mock_kidnap").get_parameter_value().bool_value

        # Initialize TopologicalMap and Nav2
        self.navigator = TurtleBot4Navigator()
        #initial_pose = self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH) #change initial pose with the correct values from rviz
        #self.navigator.setInitialPose(initial_pose)

        self.topological_map = topological_map.RectilinearGridMap()
        self.topological_map = self.topological_map.read_from_file("./src/mobile-robots-project-work/autonomous_navigation_challenge_2024/autonomous_navigation_challenge_2024/topological_maps/navigation_map_on_door2.dat")
        self.topological_map.build()
        self._percepted_signposts = None

        self.kidnapping_map = topological_map.RectilinearGridMap()
        self.kidnapping_map = self.kidnapping_map.read_from_file("./src/mobile-robots-project-work/autonomous_navigation_challenge_2024/autonomous_navigation_challenge_2024/topological_maps/kidnapping_map3.dat")
        self.kidnapping_map.build()

        self._state = NavigationState(State.INITIAL_STATE, self)
        self._robot_starting_direction = None
        self.wait_and_observe_time = Duration(seconds=1.5)
        self.wait_after_kidnap_time = Duration(seconds=1.0)
        self._curr_pose_with_covariance = None
        self._next_node = None
        self._next_direction = None
        self._next_cardinal_point_direction = None
        self._kidnapped = False
        self._kidnap_node = deque(maxlen=100)
        self._last_node = None
        # Callback groups
        # self._display_cg = ReentrantCallbackGroup()
        # self._run_executor = ReentrantCallbackGroup()
        self._last_interpreted_signpost = None
        self.near_signpost_stop_distance = 3.0
        self.near_signpost_stop_time = Duration(seconds=1.2)
        self.near_signpost_delay = Duration(seconds=10.0)

        # ROS2 interfaces
        curr_pose_qos = QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, history=QoSHistoryPolicy.KEEP_LAST, depth=2)
        self.create_timer(0.1, self.run_step)
        self.create_timer(0.1, self.display_callback)
        self.create_subscription(PerceptedSignpostList,'/perception',self.perception_callback,10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.curr_pose_callback, qos_profile=curr_pose_qos)
        kidnap_qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=2)
        kidnap_qos.durability = QoSDurabilityPolicy.VOLATILE
        kidnap_qos.history = QoSHistoryPolicy.KEEP_LAST
        kidnap_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        kidnap_qos.liveliness = QoSLivelinessPolicy.AUTOMATIC
        if self.mock_kidnap:
            self.get_logger().info("Mocking kidnap")
            self.create_subscription(KidnapStatus, '/kidnap_status_mock', self.kidnap_callback, 10)
        else:
            self.create_subscription(KidnapStatus, '/kidnap_status', self.kidnap_callback, qos_profile=kidnap_qos)

        self.marker_publisher = self.create_publisher(MarkerArray, "poly", 10)
        self.get_logger().info("Waiting for amcl to become active...")
        self.navigator._waitForNodeToActivate('amcl')
        self.get_logger().info("Waiting for initial_pose")
        while not self.navigator.initial_pose_received:
            rclpy.spin_once(self.navigator, timeout_sec=0.1)
        self.get_logger().info("Waiting for bt_navigator to become active...")
        self.navigator._waitForNodeToActivate('bt_navigator')
        self._jailbreak()
        self.navigator.clearAllCostmaps()

    # Display topological map
    def display_callback(self):
        display_topological_map = self.get_parameter("show_topological_map").get_parameter_value().bool_value
        if not display_topological_map:
            return
        if not self.topological_map:
            return
        if not self.topological_map.built:
            return
        marker_array = self.topological_map.get_all_relative_directions_msg(self.get_clock().now().to_msg())
        self.marker_publisher.publish(marker_array)
        marker_array = self.topological_map.get_all_cardinal_points_msg(self.get_clock().now().to_msg())
        self.marker_publisher.publish(marker_array)

    def curr_pose_callback(self, msg):
        if self._state.state == State.KIDNAPPED:
            return
        curr_pose = msg.pose.pose
        curr_point = sg.Point(curr_pose.position.x, curr_pose.position.y)
        curr_node = self.topological_map.localize(curr_point)
        if self._curr_pose_with_covariance is None and curr_node is not None:
            observer_pose = curr_pose
            observer_angle = euler_from_quaternion([observer_pose.orientation.x, observer_pose.orientation.y, observer_pose.orientation.z, observer_pose.orientation.w])[2]
            observer_forward = np.array([np.cos(observer_angle), np.sin(observer_angle)])
            self._robot_starting_direction = curr_node.get_closest_direction(observer_forward)
            self.get_logger().info(f"Initial robot direction set to {topological_map.Direction(self._robot_starting_direction).name}")
        self._curr_pose_with_covariance = msg.pose

    def perception_callback(self, msg):
        self._percepted_signposts = msg

    def kidnap_callback(self, msg):
        self._kidnapped = msg.is_kidnapped

    def _jailbreak(self):
        node=ChangeParameters("/motion_control", "safety_override", "full")
        rclpy.spin_once(node)
        node.destroy_node()
    
    def get_curr_state(self):
        pass
    
    def point_vect_to_pose(self, point, vect):
        #new_goal_orientation_quat = quaternion_from_euler(0, 0, np.arctan2(new_goal_orientation_vect[1], new_goal_orientation_vect[0]))
        #euler = euler_from_quaternion(new_goal_orientation_quat)
        new_goal_orientation_angle = np.arctan2(vect[1], vect[0])
        print(f"New goal orientation: {vect}, angle {new_goal_orientation_angle}")
        return self.navigator.getPoseStamped(point, new_goal_orientation_angle * 180 / np.pi)
    
    def split_segment(self, start_point, end_point, n_split, index):
        vect = end_point-start_point
        vect = vect / n_split
        return start_point + vect * index
    
    def start_navigation_to_node(self, next_node, next_direction, cardinal_point_direction):
        """Navigate to a given cardinal point in a node with a certain direction. If cardinal_point_direction is None the centroid is selected"""
        self._state.navigation_pending = True
        self._state.approach_index = 0
        self.navigator.clearAllCostmaps()
        self._next_node, self._next_direction, self._next_cardinal_point_direction = next_node, next_direction, cardinal_point_direction
        if cardinal_point_direction is None:
            new_goal_position = self._next_node.centroid
            self.get_logger().info(f"Navigating to centroid of node {next_node}")
        else:
            new_goal_position = self._next_node.cardinal_points[cardinal_point_direction]
            self.get_logger().info(f"Navigating to cardinal point {topological_map.Direction(cardinal_point_direction).name} of node {next_node}")
        new_goal_orientation_vect = self._next_node.relative_directions[self._next_direction]
        new_pose = self.point_vect_to_pose(new_goal_position, new_goal_orientation_vect)
        # Set new navigation goal
        self.navigator.goToPose(new_pose)
        if cardinal_point_direction is None:
            self._state.change_state(State.APPROACH_TO_CENTROID)
        else:
            self._state.change_state(State.GO_TO_NEXT_NODE)
    
    def start_navigation_to_point_direction(self, next_point, next_direction):
        """Navigate to a given point in a node. If cardinal_point_direction is None the centroid is selected"""
        self._state.navigation_pending = True
        self.navigator.clearAllCostmaps()
        new_goal_orientation_vect = self._next_node.relative_directions[next_direction]
        new_pose = self.point_vect_to_pose(next_point, new_goal_orientation_vect)
        self.navigator.goToPose(new_pose)
    
    def should_percieve(self):
        if not self._curr_pose_with_covariance:
            self.get_logger().warn("No current pose available, ignoring signpost!")
            return False
        # If the next node is not set, we are in the initial state and we can accept the signpost
        if self._next_node is None:
            return True
        curr_pose = self._curr_pose_with_covariance.pose
        curr_point = sg.Point(curr_pose.position.x, curr_pose.position.y)
        curr_node = self.topological_map.localize(curr_point)
        # if curr_node is None:
        #     self.get_logger().warn(f"No current node found from {curr_pose}, ignoring signpost!")
        #     return False
        # If we are in a node which is not the next node, we already accepted a signpost and we ignore the new one
        if curr_node is not None and curr_node != self._next_node:
            self.get_logger().warn(f"Already accepted a signpost, ignoring signpost as the robot is in a node that is not the target")
            return False
        # If the last node is not a neighbor of the next node, we already accepted a signpost and we ignore the new one
        if self._last_node is not None and self._next_node is not None:
            is_next_node_neighbor = self._last_node == self._next_node or self._last_node in self.topological_map.get_neighbors(self._next_node).values()
            print(f"{self._last_node == self._next_node} or {self._last_node in self.topological_map.get_neighbors(self._next_node).values()}")
            if not is_next_node_neighbor:
                self.get_logger().warn(f"Already accepted a signpost, ignoring signpost as last node is not a neighbor of next node")
                return False
        # A special logic is required if the last signpost was a backward signpost as both of the above conditions fail
        # If the last signpost was a backward signpost, we can accept a new signpost only if the robot direction is the opposite of the last detection direction
        if self._last_interpreted_signpost == PerceptedSignpost.SIGNPOST_BACKWARD:
            self.get_logger().warn(f"Special handling of backward signpost")
            next_direction_vect = self._next_node.relative_directions[self._next_direction]
            curr_orientation_angle = euler_from_quaternion([curr_pose.orientation.x, curr_pose.orientation.y, curr_pose.orientation.z, curr_pose.orientation.w])[2]
            curr_orientation_vect = np.array([np.cos(curr_orientation_angle), np.sin(curr_orientation_angle)])
            dot_prod = np.dot(next_direction_vect, curr_orientation_vect)
            return dot_prod > 0.0
        return True
    
    def run_step(self):
        # Update the position of the robot in the kidnapping map
        if self._state.get() == State.KIDNAPPED:# and self._state.kidnap_calculated == False:
            self._curr_pose_with_covariance = None
            if not self._state.kidnap_calculated:
                self.navigator.cancelTask()
                if len(self._kidnap_node) <=  1:
                    self.get_logger().warn("No previous node available, replace the robot in the starting position!")
                    self._state.new_initial_pose_node = self._kidnap_node[-1]
                    self._state.new_goal_direction = self._robot_starting_direction
                    new_initial_pose = self._state.new_initial_pose_node.get_center_pose(self._robot_starting_direction)
                    self._state.new_initial_pose_stamped = PoseStamped()
                    self._state.new_initial_pose_stamped.header.stamp = self.get_clock().now().to_msg()
                    self._state.new_initial_pose_stamped.header.frame_id = "map"
                    self._state.new_initial_pose_stamped.pose = new_initial_pose
                    self._state.kidnap_calculated = True
                    self._state.kidnap_with_one_element = True
                    return
                print("Current kindap queue before kidnap state: ", [node.centroid for node in self._kidnap_node])
                new_initial_pose_node = self._kidnap_node[-2]
                new_initial_pose_direction = None
                # Calculate the new_intial_pose_direction
                for direction, neighbor in self.kidnapping_map.edges[new_initial_pose_node].items():
                    if neighbor == self._kidnap_node[-1]:
                        new_initial_pose_direction = direction
                        break
                if new_initial_pose_direction is None:
                    self.get_logger().error(f"Critical error: kidnapping failed as node {new_initial_pose_node.centroid} has no neighbor {self._kidnap_node[-1].centroid}")
                    self.destroy_node()
                    exit(1)

                self.get_logger().info(f"Repositioning robot in previous node {new_initial_pose_node} with direction {topological_map.Direction(new_initial_pose_direction).name}")
                
                # Get the pose where we have to position the robot
                new_initial_pose = new_initial_pose_node.get_center_pose(new_initial_pose_direction)
                self._state.new_initial_pose_stamped = PoseStamped()
                self._state.new_initial_pose_stamped.header.stamp = self.get_clock().now().to_msg()
                self._state.new_initial_pose_stamped.header.frame_id = "map"
                self._state.new_initial_pose_stamped.pose = new_initial_pose
                self._state.new_initial_pose_node = new_initial_pose_node

                # Calculate the new goal (node and direction)
                next_kidnap_node = self._kidnap_node[-1]
                new_goal_point = sg.Point(next_kidnap_node.centroid[0],next_kidnap_node.centroid[1])
                new_goal_node = self.topological_map.localize(new_goal_point)
                while new_goal_node is None:
                    next_kidnap_node = self.kidnapping_map.edges[next_kidnap_node][new_initial_pose_direction]
                    if next_kidnap_node is None:
                        self.get_logger().error(f"Critical error: kidnapping failed as kidnap node {next_kidnap_node.centroid} has no neighbor in direction {topological_map.Direction(new_initial_pose_direction).name}")
                        self.destroy_node()
                        exit(1)
                    new_goal_point = sg.Point(next_kidnap_node.centroid[0],next_kidnap_node.centroid[1])
                    new_goal_node = self.topological_map.localize(new_goal_point)
                self._state.new_goal_node = new_goal_node
                self._state.new_goal_direction = new_initial_pose_direction
                
                # Calculate the previous node
                prev_kidnap_node = self._kidnap_node[-2]
                new_last_point = sg.Point(prev_kidnap_node.centroid[0],prev_kidnap_node.centroid[1])
                new_last_node = self.topological_map.localize(new_last_point)
                while new_last_node is None:
                    prev_kidnap_node = self.kidnapping_map.edges[prev_kidnap_node][topological_map.Direction.get_opposite(new_initial_pose_direction)]
                    if prev_kidnap_node is None:
                        self.get_logger().error(f"Node {prev_kidnap_node} has no neighbor in direction {topological_map.Direction.get_opposite(new_initial_pose_direction).name}")
                        self.destroy_node()
                        exit(1)
                    new_last_point = sg.Point(prev_kidnap_node.centroid[0],prev_kidnap_node.centroid[1])
                    new_last_node = self.topological_map.localize(new_last_point)
                self._state.new_last_node = new_last_node
                
                # Pop the last node in the kidnap list
                self._kidnap_node.pop()
                print("Current kindap queue after pop: ", [node.centroid for node in self._kidnap_node])
                self._state.kidnap_calculated = True
                
            if self._state.kidnap_calculated:
                self.draw_arrow(self._state.new_initial_pose_node, self._state.new_goal_direction)
                
            if not self._kidnapped and self._state.kidnap_calculated:
                self.navigator.setInitialPose(self._state.new_initial_pose_stamped)
                time = self.get_clock().now()
                self.navigator.clearAllCostmaps()
                while not self.navigator.initial_pose_received or self.get_clock().now() - time < self.wait_after_kidnap_time:
                    self.get_logger().info("Waiting for initial_pose...")
                    rclpy.spin_once(self.navigator, timeout_sec=0.1)
                self.navigator.clearAllCostmaps()
                if self._state.kidnap_with_one_element:
                    self._state.change_state(State.INITIAL_STATE)
                    return
                self._state.navigation_pending = False
                self._last_node = self._state.new_last_node
                self.get_logger().info(f"New last node is {self._last_node}")
                self.start_navigation_to_node(self._state.new_goal_node, self._state.new_goal_direction, topological_map.Direction.get_opposite(self._state.new_goal_direction))
                print("Current kindap queue after kidnap state: ", [node.centroid for node in self._kidnap_node])
            return 
        
        
        # Check if the robot is kidnapped
        if self._kidnapped:
            self.get_logger().info("Kidnapped!")
            self._state.kidnap_calculated = False
            self._state.change_state(State.KIDNAPPED)
            return
        
        if not self.navigator.initial_pose_received:
            self.get_logger().warn("No initial pose set, waiting...")
            return
        if not self._curr_pose_with_covariance:
            self.get_logger().warn("No current pose available, waiting...")
            return
        
        # Localize the robot on the kidnap map
        # Get current pose and current point
        curr_pose = self._curr_pose_with_covariance.pose
        curr_point = sg.Point(curr_pose.position.x, curr_pose.position.y)
        curr_node_kidnapped = self.kidnapping_map.localize(curr_point)
        if curr_node_kidnapped is not None:
            if len(self._kidnap_node) == 0 or curr_node_kidnapped != self._kidnap_node[-1]:
                self._kidnap_node.append(curr_node_kidnapped)
        
        # If the first node has not been reached, go to the first node
        if self._state.get() == State.INITIAL_STATE:
            curr_node = self.topological_map.localize(curr_point)
            if curr_node is None:
                self.get_logger().error("The robot must be in a node to start the navigation!")
                self.destroy_node()
                exit(1)
            self._last_node = curr_node
            next_node, next_direction = self.topological_map.interpret_signpost_by_pose(curr_pose, PerceptedSignpost.SIGNPOST_FORWARD)
            if next_node is None or next_direction is None:
                self.get_logger().error("Invalid starting orientation for the robot. Please rotate the robot to face a node")
                self.destroy_node()
                exit(1)
            self._last_interpreted_signpost = PerceptedSignpost.SIGNPOST_FORWARD
            cardinal_point_direction = topological_map.Direction.get_opposite(next_direction)
            self.start_navigation_to_node(next_node, next_direction, cardinal_point_direction)
            return
        elif self._state.get() == State.GO_TO_NEXT_NODE:
            if self._state.navigation_pending and self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                if result != TaskResult.SUCCEEDED:
                    self.get_logger().info(f"GO_TO_NEXT_NODE failed. Retrying to go to next node")
                    self.start_navigation_to_node(self._next_node, self._next_direction, self._next_cardinal_point_direction)
                    return
                self.last_sopped_at_signpost = None
                self._state.navigation_pending = False
                self._state.change_state(State.WAIT_AND_OBSERVE)
        elif self._state.get() == State.APPROACH_TO_CENTROID:
            if self._state.approach_index == 0:
                self.get_logger().info("APPROACH_TO_CENTER finished")
                self._state.change_state(State.SPIN_AND_OBSERVE)
                return
            if self._state.navigation_pending and self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                if result != TaskResult.SUCCEEDED:
                    self.get_logger().info(f"APPROACH_TO_CENTROID failed. Retrying to go to next goal point")
                    self.navigator.clearAllCostmaps()
                    cardinal_point_direction = topological_map.Direction.get_opposite(self._next_direction)
                    cardinal_point = self._next_node.cardinal_points[cardinal_point_direction]
                    self.start_navigation_to_point_direction(self.split_segment(cardinal_point, self._next_node.centroid, self._state.approach_steps, self._state.approach_index), self._next_direction)
                    return
                self._state.navigation_pending = False
                self.get_logger().info(f"APPROACH_TO_CENTER navigation {self._state.approach_index} ended")
                if self._state.approach_index == self._state.approach_steps:
                    self._state.change_state(State.SPIN_AND_OBSERVE)
                else:
                    self._state.change_state(State.OBSERVE_LEFT_RIGHT)
            elif not self._state.navigation_pending: #and self.get_clock().now() - self._state.entry_time >= self.wait_and_observe_time:
                self.get_logger().info(f"APPROACH_TO_CENTER starting navigation {self._state.approach_index}")
                cardinal_point_direction = topological_map.Direction.get_opposite(self._next_direction)
                cardinal_point = self._next_node.cardinal_points[cardinal_point_direction]
                self.start_navigation_to_point_direction(self.split_segment(cardinal_point, self._next_node.centroid, self._state.approach_steps, self._state.approach_index), self._next_direction)
        elif self._state.get() == State.WAIT_AND_OBSERVE:
            if self.get_clock().now() - self._state.entry_time >= self.wait_and_observe_time:
                #self.start_navigation_to_node(self._next_node, self._next_direction, None)
                self._state.change_state(State.APPROACH_TO_CENTROID)
                return
        elif self._state.get() == State.SPIN_AND_OBSERVE:
            if self.get_clock().now() - self._state.entry_time >= self.wait_and_observe_time:
                if self._state.turn_index == 0:
                    self.navigator.spin(spin_dist=math.radians(40))
                elif self._state.turn_index == 1:
                    self.navigator.spin(spin_dist=-math.radians(80))
                elif self._state.turn_index == 2:
                    self.navigator.spin(spin_dist=math.radians(40))
                self._state.navigation_pending = True
                while self._state.navigation_pending and self.navigator.isTaskComplete():
                    if self._kidnapped:
                        return
                    feedback = self.navigator.getFeedback()
                self._state.navigation_pending = False
                result = self.navigator.getResult()
                self._state.change_state(State.SPIN_AND_OBSERVE) #Reset timer
        elif self._state.get() == State.OBSERVE_LEFT_RIGHT:
            if self.get_clock().now() - self._state.entry_time >= self.wait_and_observe_time or self._state.turn_index == 0:
                self.get_logger().info(f"Observing left right, turn index: {self._state.turn_index}")
                if self._state.turn_index == 0:
                    self._state.change_state(State.APPROACH_TO_CENTROID)
                    return
                if self._state.turn_index == 1:
                    self.navigator.spin(spin_dist=math.radians(30))
                elif self._state.turn_index == 2:
                    self.navigator.spin(spin_dist=-math.radians(60))
                elif self._state.turn_index == 3:
                    self.navigator.spin(spin_dist=math.radians(30))
                self._state.navigation_pending = True
                while self._state.navigation_pending and self.navigator.isTaskComplete():
                    if self._kidnapped:
                        return
                    feedback = self.navigator.getFeedback()
                self._state.navigation_pending = False
                result = self.navigator.getResult()
                if self._state.turn_index == 1 or self._state.turn_index == 2 or self._state.turn_index == 3:
                    self._state.change_state(State.OBSERVE_LEFT_RIGHT)
                return
        elif self._state.get() == State.WAIT_NEAR_SIGNPOST:
            if self.get_clock().now() - self._state.entry_time >= self.near_signpost_stop_time:
                self._state.restore_previous_state()
                self.get_logger().info(f"State changed from {State(State.WAIT_NEAR_SIGNPOST).name} to {State(self._state.get()).name}")
                return
        
        curr_pose = self._curr_pose_with_covariance.pose
        curr_point = sg.Point(curr_pose.position.x, curr_pose.position.y)
        curr_node = self.topological_map.localize(curr_point)
        if not curr_node is None:
            if self._last_node != curr_node:
                self.get_logger().info(f"Last node changed to {curr_node} with centroid {curr_node.centroid}")
            self._last_node = curr_node
            
        
        if not self._percepted_signposts or len(self._percepted_signposts.signposts) == 0:
            return
        # Ordering signposts should be done in the perception node
        #signpost = min(self._percepted_signposts.signposts, key=lambda x: x.distance)
        signpost = self._percepted_signposts.signposts[0]
        if not signpost:
            return
        self.get_logger().info(f"Closest percepted signpost: {signpost}")
        
        # Call interpret_signpost method

        if not self.should_percieve():
            return
        
        if signpost.signpost_type == Signpost.INVALID and signpost.distance < self.near_signpost_stop_distance \
            and self._state.can_stop_at_signpost():
                #and self._state.has_stopped_near_signpost == False:
            self._state.change_state(State.WAIT_NEAR_SIGNPOST)

        if self._next_node is None:
            self.get_logger().warn(f"Ignoring signpost as next_node is None")
            return
        
        # If we should percieve, we can always interpret the signpost with respect to the next node
        next_node, next_direction = self.topological_map.interpret_signpost_by_direction(self._next_node, self._next_direction, signpost.signpost_type)
        if next_node is None:
            self.get_logger().warn(f"No next node found from {self._next_node.centroid if self._next_node else None} with signpost {signpost}, ignoring signpost!")
            return
        
        
        self._last_interpreted_signpost = signpost.signpost_type
        # BACKWARD signpost requires a special handling as the robot could never pass in the node with the signpost
        if signpost.signpost_type == PerceptedSignpost.SIGNPOST_BACKWARD:
            self._last_node = self._next_node
            self.get_logger().info(f"Last node changed to {self._next_node} with centroid {self._next_node.centroid}")
        elif signpost.signpost_type == PerceptedSignpost.SIGNPOST_STOP:
            self.get_logger().info("Route completed successfully")
            self.destroy_node()
            exit(0)
        cardinal_point_direction = topological_map.Direction.get_opposite(next_direction)
        self.get_logger().info(f"Successfully interpreted a new signpost.\n\t- Next node: {next_node}\n\t- Next direction: {topological_map.Direction(next_direction).name}\n\t- Next cardinal point: {topological_map.Direction(cardinal_point_direction).name}")
        self.start_navigation_to_node(next_node, next_direction, cardinal_point_direction)

    def get_centroid_arrow_msg(self, node, direction):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = hash((self, direction, 1)) % 2147483647 # Let's hope this is unique!
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 1.0 # Length of the arrow
        marker.scale.y = 0.2 # Width of the arrow
        marker.scale.z = 0.2 # Height of the arrow
        marker.color.a = 1.0 
        if direction == topological_map.Direction.NORTH:
            rgb = (1.0, 0.0, 0.0)
        elif direction == topological_map.Direction.EAST:
            rgb = (0.0, 1.0, 0.0)
        elif direction == topological_map.Direction.SOUTH:
            rgb = (0.0, 0.0, 1.0)
        elif direction == topological_map.Direction.WEST:
            rgb = (1.0, 1.0, 0.0)
        marker.color.r = rgb[0]
        marker.color.g = rgb[1]
        marker.color.b = rgb[2]
        marker.lifetime = Duration(seconds=0.2).to_msg()
        marker.pose.position.x = node.centroid[0]
        marker.pose.position.y = node.centroid[1]
        marker.pose.position.z = 0.1
        vect = node.relative_directions[direction]
        q = quaternion_from_euler(0, 0, np.arctan2(vect[1], vect[0]))
        marker.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return marker
    
    def draw_arrow(self, node, direction):
        marker = self.get_centroid_arrow_msg(node, direction)
        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        self.marker_publisher.publish(marker_array)

def main():
    rclpy.init()
    executor = SingleThreadedExecutor()
    
    #change_parameters.change_parameters("safety_override", ParameterType.PARAMETER_STRING, "full", "motion_control")
    node = NavigationNode()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()

if __name__ == '__main__':
    main()
