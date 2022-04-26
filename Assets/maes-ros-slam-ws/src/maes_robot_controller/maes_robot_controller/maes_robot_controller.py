import math
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
from tf2_msgs.msg import TFMessage

import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, ReliabilityPolicy, HistoryPolicy
from rclpy.qos import QoSProfile


@dataclass
class Coord2D:
    x: float
    y: float


class RobotController(Node):

    def __init__(self):
        # The name and namespace is usually overridden be the launch file
        super().__init__(node_name="maes_robot_controller")

        self.topic_namespace_prefix = self.get_namespace()  # All topics have prefixed with the namespace of the node e.g. /robot0

        # Declare topics
        self.state_topic = self.topic_namespace_prefix + "/maes_state"
        self.broadcast_srv_topic = self.topic_namespace_prefix + "/maes_broadcast"
        self.deposit_env_tag_srv_topic = self.topic_namespace_prefix + "/maes_deposit_tag"
        self.goal_pose2D_topic = self.topic_namespace_prefix + "/goal_pose"
        self.nav_to_pose_topic = self.topic_namespace_prefix + "/navigate_to_pose"
        self.global_costmap_2d_topic = self.topic_namespace_prefix + "/global_costmap/costmap"
        self.tf2_topic = self.topic_namespace_prefix + "/tf"

        # Quality of service profile for subscriptions
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2
        )

        # Create subscribers
        self.state_subscriber = self.create_subscription(msg_type=State,
                                                         topic=self.state_topic,
                                                         callback=self.logic_loop,
                                                         qos_profile=qos_profile)
        self.global_costmap_sub = self.create_subscription(msg_type=OccupancyGrid,
                                                           topic=self.global_costmap_2d_topic,
                                                           callback=self.save_costmap,
                                                           qos_profile=qos_profile)
        self.tf_sub = self.create_subscription(msg_type=TFMessage,
                                               topic=self.tf2_topic,
                                               callback=self.save_robot_position,
                                               qos_profile=qos_profile)

        # Register service clients
        self.broadcast_srv = self.create_client(srv_type=BroadcastToAll, srv_name=self.broadcast_srv_topic)
        self.deposit_env_tag_srv = self.create_client(srv_type=DepositTag, srv_name=self.deposit_env_tag_srv_topic)

        # Register fields for navigation
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None
        # The map data, in row-major order, starting with (0, 0).  Occupancy
        # probabilities are in the range [0,100].  Unknown is -1.
        self.costmap: OccupancyGrid = None
        self.robot_position: TransformStamped = None

        # Create navigation action client
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, self.topic_namespace_prefix + '/navigate_to_pose')

        # Logic variables below here
        self.next_target: Coord2D = None
        self.next_target_costmap_index: int = None

    def logic_loop(self, state: State):
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

        The navigator can also to used to cancel moves and/or check status of the current goal e.g.
        self.cancel_nav()
        self.is_nav_complete()
        self.get_feedback()
        '''

        # -1 = unknown, 0 = certain to be open, 100 = certain to be obstacle
        # We assume anything between 10 and 90 to be uncertain, and thus a frontier
        # If no target found
        if self.next_target is None:
            target_frontier_tile_index = next((index for index, value in enumerate(self.costmap.data) if self.is_frontier(index)), None)
            # frontier_points = [index for index, value in enumerate(self.costmap.data) if is_frontier(value)]
            # frontier_points.sort(key=lambda e1: self.distance_to_costmap_index(e1), reverse=True)
            # frontier_points = filter(lambda e: self.distance_to_costmap_index(e) > 2.0, frontier_points)
            # target_frontier_tile_index = next(iter(frontier_points), None)

            # fp = list(map(self.costmap_index_to_pos, frontier_points))
            # fp_string = ""
            # for point in fp:
            #     fp_string = fp_string + "," + str(point)
            # self.info(fp_string)

            if target_frontier_tile_index is None:
                self.info("Robot with namespace {0} is has found no more frontiers".format(self.topic_namespace_prefix))
            else:
                self.next_target = self.costmap_index_to_pos(target_frontier_tile_index)
                self.next_target_costmap_index = target_frontier_tile_index
                self.info("Robot with namespace {0} found new target at ({1},{2})".format(self.topic_namespace_prefix,
                                                                                          self.next_target.x,
                                                                                          self.next_target.y))
                # self.move_to_pos(self.next_target.x, self.next_target.y)
        # If target is no yet reached, i.e. it is  still a frontier
        # elif is_frontier(self.costmap.data[self.next_target_costmap_index]):
        elif self.is_frontier(self.next_target_costmap_index):
            self.info("Frontier value: {0}".format(self.costmap.data[self.next_target_costmap_index]))
            # self.info("Feedback from action server: {0}".format(self.get_feedback()))
            self.get_feedback()
            # Print feedback from action server or something, idk?
            pass

        # elif self.status in [GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED]:
        #     self.info("Goal status appears to be {0}. Resetting targets".format(self.status))
        #     self.next_target_costmap_index = None
        #    self.next_target = None
        # If target is reached
        else:
            self.info("Robot with namespace {0} reached its target at ({1},{2})".format(self.topic_namespace_prefix,
                                                                                        self.next_target.x,
                                                                                        self.next_target.y))
            self.next_target_costmap_index = None
            self.next_target = None

    # This method returns true if the tile is not itself unknown, but has a neighbor, that is unknown
    def is_frontier(self, index: int):
        # It is itself unknown
        if self.costmap.data[index] == -1:
            return False
        # It is itself a wall
        if self.costmap.data[index] > 65:
            return False

        return self.has_unknown_neighbor(index)

    def get_unknown_neighbor(self, index: int):
        up_left = index + self.costmap.info.width - 1
        up = index + self.costmap.info.width
        up_right = index + self.costmap.info.width + 1
        left = index - 1
        right = index + 1
        down_left = index - self.costmap.info.width - 1
        down = index - self.costmap.info.width
        down_right = index - self.costmap.info.width + 1
        all = [up_left, up, up_right, left, right, down_left, down, down_right]
        all = list(filter(lambda e: e > 0, all))
        for neighbor in all:
            if self.costmap.data[neighbor] == -1:
                return neighbor

        return None

    def has_unknown_neighbor(self, index: int):
        return self.get_unknown_neighbor(index) is not None

    def distance_to_costmap_index(self, index: int) -> float:
        robot_x = self.robot_position.transform.translation.x
        robot_y = self.robot_position.transform.translation.y
        index_pos = self.costmap_index_to_pos(index)
        return math.sqrt(math.pow(robot_x - index_pos.x, 2) +
                         math.pow(robot_y - index_pos.y, 2))

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

    def save_robot_position(self, msg: TFMessage):
        odom = list(filter(lambda e: e.header.frame_id == "odom", msg.transforms))
        if len(odom) == 1:
            self.robot_position = odom[0]

    def save_costmap(self, occ_map: OccupancyGrid):
        self.costmap = occ_map

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
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                  str(pose.pose.position.y) + '...')
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg,
                                                                   self._feedback_callback)
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
        self.info('Canceling current goal.')
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return

    def is_nav_complete(self):
        if not self.result_future:
            # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.info('Goal with failed with status code: {0}'.format(self.status))
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        self.info('Goal succeeded!')
        return True

    def get_feedback(self):
        return self.feedback

    def get_result(self):
        return self.status

    def wait_until_nav2_active(self):
        self._wait_for_node_to_activate(self.topic_namespace_prefix + 'bt_navigator')
        self.info('Nav2 is ready for use!')
        return

    def _wait_for_node_to_activate(self, node_name):
        # Waits for the node within the tester namespace to become active
        self.debug('Waiting for ' + node_name + ' to become active..')
        node_service = node_name + '/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(node_service + ' service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while (state != 'active'):
            self.debug('Getting ' + node_name + ' state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.debug('Result of get_state: %s' % state)
            time.sleep(2)
        return

    def _feedback_callback(self, msg):
        self.feedback = msg.feedback
        return

    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
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
