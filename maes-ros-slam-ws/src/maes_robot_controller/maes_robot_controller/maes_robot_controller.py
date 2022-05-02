import math
import random
from dataclasses import dataclass
from typing import Callable

import geometry_msgs.msg
from maes_msgs.msg import State
from maes_msgs.srv import BroadcastToAll, DepositTag

import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav2_msgs.msg import *
from nav_msgs.msg import OccupancyGrid
from rclpy.impl.rcutils_logger import RcutilsLogger
from tf2_msgs.msg import TFMessage

import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, ReliabilityPolicy, HistoryPolicy
from rclpy.qos import QoSProfile


class RobotController(Node):

    def __init__(self):
        # The name and namespace is usually overridden be the launch file
        super().__init__(node_name="maes_robot_controller")

        # All topics have prefixed with the namespace of the node e.g. /robot0
        self.topic_namespace_prefix = self.get_namespace()

        # Used to print to console running ros2
        self.logger = MaesLogger(logger=self.get_logger())

        # Register fields for navigation
        self.nav_goal_handle = None
        self.nav_result_future = None
        self.nav_feedback = None
        self.nav_status = None

        # Interface to use global costmap for nagivation
        self.global_costmap: MaesCostmap = MaesCostmap(self.logger)

        # Robot position set by a callback function from the tf topic
        self.robot_position: TransformStamped = None

        # Registers subscribers, services and actions and assigns to variables below.
        # Subscribers
        self.state_subscriber = None
        self.global_costmap_sub = None
        self.tf_sub = None
        # Service clients
        self.broadcast_srv = None
        self.deposit_env_tag_srv = None
        # Navigation action client
        self.nav_to_pose_client = None
        self._register_subs_srvs_actions()  # Assign to variables

        # Logic variables for YOUR algorithm below here
        self.next_target: Coord2D = None
        self.next_target_costmap_index: int = None

    def logic_loop_callback(self, state: State):
        '''
        This function is called every time the robot receives a new state msg from MAES
        Use this function to express the logic that makes the robot move.

        Examples of using services:
        self.broadcast_msg(msg="Testing broadcasting")
        self.deposit_tag(tag_msg="Content of env_tag")

        Example of moving:
        goal_pose = self.create_goal_pose(pose_x=10.0, pose_y=2.0, pose_z=0.0,
                                          ori_x=0.0, ori_y=0.0, ori_z=0.0, ori_w=1.0)
        self.go_to_pose(goal_pose)

        Get current time in ticks
        state.tick

        The navigator can also to used to cancel moves and/or check status of the current goal e.g.
        self.cancel_nav()
        self.is_nav_complete()
        self.get_feedback()
        '''

        # Wait for map to be initialised
        if self.global_costmap.costmap is None:
            self.logger.log_debug("Robot with namespace {0} has no global map".format(self.topic_namespace_prefix))
            return

        # If no target found or current nav cancelled by nav2 stack
        if self.next_target is None or self.is_nav_complete():
            # Find index of first tile in costmap that is a frontier
            all_frontiers = [index for index, value in enumerate(self.global_costmap.costmap.data) if self.is_frontier(index)]

            # Choose random frontier from the list of all frontiers
            if self.topic_namespace_prefix == "/robot0":
                target_frontier_tile_index = max(all_frontiers) if len(all_frontiers) > 0 else None
            else:
                target_frontier_tile_index = min(all_frontiers) if len(all_frontiers) > 0 else None

            if target_frontier_tile_index is not None:
                self.next_target = self.global_costmap.costmap_index_to_pos(target_frontier_tile_index)
                self.next_target_costmap_index = target_frontier_tile_index
                self.move_to_pos(self.next_target.x, self.next_target.y)
                self.logger.log_info("Robot with namespace {0} found new target at ({1},{2})".format(self.topic_namespace_prefix,
                                                                                                     self.next_target.x,
                                                                                                     self.next_target.y))
            else:
                self.logger.log_info("Robot with namespace {0} is has found no more frontiers".format(self.topic_namespace_prefix))


        # If target is no yet reached, i.e. it is  still a frontier
        elif self.is_frontier(self.next_target_costmap_index):
            # This section allows for logging feedback etc.
            #self.logger.log_info("Frontier value: {0}".format(self.global_costmap.costmap.data[self.next_target_costmap_index]))
            pass
        # If target is reached
        else:
            self.logger.log_info("Robot with namespace {0} reached its target at ({1},{2})".format(self.topic_namespace_prefix,
                                                                                        self.next_target.x,
                                                                                        self.next_target.y))
            self.next_target_costmap_index = None
            self.next_target = None
            self.cancel_nav()

    # This method returns true if the tile is not itself unknown, but has a neighbor, that is unknown
    def is_frontier(self, map_index: int):
        # -1 = unknown, 0 = certain to be open, 100 = certain to be obstacle
        # It is itself unknown
        if self.global_costmap.costmap.data[map_index] == -1:
            return False
        # It is itself a wall
        if self.global_costmap.costmap.data[map_index] >= 65:
            return False

        return self.global_costmap.has_unknown_neighbor(map_index)

    def move_to_pos(self, pose_x, pose_y):
        goal = self.create_goal_pose(pose_x=pose_x, pose_y=pose_y,
                                     pose_z=0.0, ori_x=0.0, ori_y=0.0,
                                     ori_z=0.0, ori_w=1.0)
        self.go_to_pose(goal)

    def create_goal_pose(self, pose_x: float, pose_y: float, pose_z: float, ori_x: float, ori_y: float, ori_z: float,
                         ori_w: float) -> PoseStamped:
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = pose_x
        goal_pose.pose.position.y = pose_y
        goal_pose.pose.position.z = pose_z
        goal_pose.pose.orientation.x = ori_x
        goal_pose.pose.orientation.y = ori_y
        goal_pose.pose.orientation.z = ori_z
        goal_pose.pose.orientation.w = ori_w

        return goal_pose

    def deposit_tag(self, tag_msg):
        request = DepositTag.Request()
        request.msg = tag_msg
        self.deposit_env_tag_srv.call_async(request=request)

    def broadcast_msg(self, msg):
        request = BroadcastToAll.Request()
        request.msg = msg
        self.broadcast_srv.call_async(request=request)

    def go_to_pose(self, pose: PoseStamped):
        # Sends a `NavToPose` action request and waits for completion
        self.logger.log_debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.logger.log_info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.logger.log_info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                  str(pose.pose.position.y) + '...')
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg,
                                                                   self._feedback_callback)
        # In case we want to wait for action to complete before returning to logic loop
        # rclpy.spin_until_future_complete(self, send_goal_future)
        # self.goal_handle = send_goal_future.result()

        # if not self.goal_handle.accepted:
        #     self.error('Goal to ' + str(pose.pose.position.x) + ' ' +
        #                str(pose.pose.position.y) + ' was rejected!')
        #     return False
        #
        # self.result_future = self.goal_handle.get_result_async()
        # return True

    def cancel_nav(self):
        self.logger.log_info('Canceling current goal.')
        if self.nav_result_future:
            future = self.nav_goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return

    def is_nav_complete(self):
        if not self.nav_result_future:
            # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(self, self.nav_result_future, timeout_sec=0.10)
        if self.nav_result_future.result():
            self.nav_status = self.nav_result_future.result().status
            if self.nav_status != GoalStatus.STATUS_SUCCEEDED:
                self.logger.log_info('Goal with failed with status code: {0}'.format(self.nav_status))
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        self.logger.log_info('Goal succeeded!')
        return True

    def _register_subs_srvs_actions(self):
        # Declare topics
        state_topic = self.topic_namespace_prefix + "/maes_state"
        broadcast_srv_topic = self.topic_namespace_prefix + "/maes_broadcast"
        deposit_env_tag_srv_topic = self.topic_namespace_prefix + "/maes_deposit_tag"
        nav_to_pose_topic = self.topic_namespace_prefix + "/navigate_to_pose"
        global_costmap_2d_topic = self.topic_namespace_prefix + "/global_costmap/costmap"
        tf2_topic = self.topic_namespace_prefix + "/tf"

        # Quality of service profile for subscriptions
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2
        )

        # Create subscribers
        self.state_subscriber = self.create_subscription(msg_type=State,
                                                         topic=state_topic,
                                                         callback=self.logic_loop_callback,
                                                         qos_profile=qos_profile)
        self.global_costmap_sub = self.create_subscription(msg_type=OccupancyGrid,
                                                           topic=global_costmap_2d_topic,
                                                           callback=self.global_costmap.update_costmap,
                                                           qos_profile=qos_profile)
        self.tf_sub = self.create_subscription(msg_type=TFMessage,
                                               topic=tf2_topic,
                                               callback=self.save_robot_position_callback,
                                               qos_profile=qos_profile)

        # Register service clients
        self.broadcast_srv = self.create_client(srv_type=BroadcastToAll, srv_name=broadcast_srv_topic)
        self.deposit_env_tag_srv = self.create_client(srv_type=DepositTag, srv_name=deposit_env_tag_srv_topic)

        # Wait for services to be active
        while not self.broadcast_srv.wait_for_service(timeout_sec=1) or not self.deposit_env_tag_srv.wait_for_service(timeout_sec=1):
            self.logger.log_info("{0} waiting for either broadcast or deposit tag services".format(self.topic_namespace_prefix))

        # Create navigation action client
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, nav_to_pose_topic)

        # Wait for action service to be active
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1):
            self.logger.log_info("{0} waiting for either nav action client to start".format(self.topic_namespace_prefix))

    """
    Call back functions from here
    """
    def save_robot_position_callback(self, msg: TFMessage):
        odom = list(filter(lambda e: e.header.frame_id == "odom", msg.transforms))
        if len(odom) == 1:
            self.robot_position = odom[0]

    def _feedback_callback(self, msg):
        self.nav_feedback = msg.feedback
        return


def main(args=None):
    rclpy.init(args=args)

    controller = RobotController()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

"""
Helper classes from here
"""
class MaesLogger:
    """
    This class was made to pass a simple logging interface to both the costmap and the controller
    """
    def __init__(self, logger) -> None:
        super().__init__()
        self._logger: RcutilsLogger = logger

    def log_info(self, msg):
        self._logger.info(msg)

    def log_warn(self, msg):
        self._logger.warn(msg)

    def log_error(self, msg):
        self._logger.error(msg)

    def log_debug(self, msg):
        self._logger.debug(msg)

@dataclass
class Coord2D:
    x: float
    y: float


class MaesCostmap:
    def __init__(self, logger: MaesLogger) -> None:
        super().__init__()
        self.logger = logger
        # The map data, in row-major order, starting with (0, 0).  Occupancy
        # probabilities are in the range [0,100].  Unknown is -1.
        self.costmap: OccupancyGrid = None


    def update_costmap(self, costmap: OccupancyGrid):
        self.costmap = costmap

    def distance_to_costmap_index(self, from_coord: Coord2D, index: int) -> float:
        index_pos = self.costmap_index_to_pos(index)
        return math.sqrt(math.pow(from_coord.x - index_pos.x, 2) +
                         math.pow(from_coord.y - index_pos.y, 2))


    def costmap_index_to_pos(self, index: int) -> Coord2D:
        y_tile: int = int(index / self.costmap.info.width)
        x_tile: int = index % self.costmap.info.width
        return self.cost_map_tiles_to_pos(x_tile=x_tile, y_tile=y_tile)

    def cost_map_tiles_to_pos(self, x_tile: int, y_tile: int) -> Coord2D:
        x = (x_tile - (self.costmap.info.width / 2)) * self.costmap.info.resolution
        y = (y_tile - (self.costmap.info.height / 2)) * self.costmap.info.resolution
        return Coord2D(x, y)

    def pos_to_costmap_index(self, pos: Coord2D) -> int:
        x_tile, y_tile = self.pos_to_costmap_tile(pos)
        return y_tile * self.costmap.info.width + x_tile

    def pos_to_costmap_tile(self, pos: Coord2D) -> (int, int):
        x_tile = int((pos.x - self.costmap.info.origin.position.x) / self.costmap.info.resolution)
        y_tile = int((pos.y - self.costmap.info.origin.position.y) / self.costmap.info.resolution)
        return x_tile, y_tile

    def get_costmap_coord_status(self, x: int, y: int) -> float:
        index = y * self.costmap.info.width + x
        return self.costmap.data[index]

    def get_unknown_neighbor_if_avail(self, index: int):
        width = self.costmap.info.width
        height = self.costmap.info.height
        up_left = index + width - 1
        up = index + width
        up_right = index + width + 1
        left = index - 1
        right = index + 1
        down_left = index - width - 1
        down = index - width
        down_right = index - width + 1
        neighbors = [up_left, up, up_right, left, right, down_left, down, down_right]
        # Filter out neighbors, that are out of bounds
        neighbors = list(filter(lambda tile_index: 0 <= tile_index <= width * height - 1, neighbors))
        for neighbor_index in neighbors:
            # If neighbor is unknown, return index of that neighbor
            if self.costmap.data[neighbor_index] == -1:
                return neighbor_index

        return None

    def has_unknown_neighbor(self, index: int):
        return self.get_unknown_neighbor_if_avail(index) is not None