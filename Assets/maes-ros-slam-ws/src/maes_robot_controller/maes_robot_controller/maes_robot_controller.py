import geometry_msgs.msg
from maes_msgs.msg import State
from maes_msgs.srv import BroadcastToAll, DepositTag

import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose

import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, ReliabilityPolicy, HistoryPolicy
from rclpy.qos import QoSProfile


class RobotController(Node):

    def __init__(self):
        # The name and namespace is usually overridden be the launch file
        super().__init__(node_name="maes_robot_controller")

        robot_id = self.get_namespace()[1:]  # Remove '/' prefix from namespace

        # Define topics
        self.topic_prefix = self.get_namespace() # All topics have prefixed with the namespace of the node
        self.state_topic = "{0}/maes_state".format(self.topic_prefix)
        self.broadcast_srv_topic = "{0}/maes_broadcast".format(self.topic_prefix)
        self.deposit_env_tag_srv_topic = "{0}/maes_deposit_tag".format(self.topic_prefix)
        self.goal_pose2D_topic = "{0}/goal_pose".format(self.topic_prefix)
        self.nav_to_pose_topic = "{0}/navigate_to_pose".format(self.topic_prefix)

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
        # Register service clients
        self.broadcast_srv = self.create_client(srv_type=BroadcastToAll, srv_name=self.broadcast_srv_topic)
        self.deposit_env_tag_srv = self.create_client(srv_type=DepositTag, srv_name=self.deposit_env_tag_srv_topic)

        # Register fields for navigation
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, self.topic_prefix + '/navigate_to_pose')


    def logic_loop(self, msg: State):
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

        # self.get_logger().info('Robot0 heard: "%s"' % msg)
        # self.broadcast_msg(msg="Testing broadcasting")
        # self.deposit_tag(tag_msg="Content of env_tag")
        if(msg.tick == 1):
            goal_pose = self.create_goal_pose(pose_x=10.0, pose_y=2.0, pose_z=0.0,
                                              ori_x=0.0, ori_y=0.0, ori_z=0.0, ori_w=1.0)

            self.go_to_pose(goal_pose)
        if(msg.tick == 100):
            self.cancel_nav()

        self.info(str(self.is_nav_complete()) + " " + str(self.get_feedback()))


    def create_goal_pose(self, pose_x: float, pose_y: float, pose_z: float, ori_x: float, ori_y: float, ori_z: float, ori_w: float) -> PoseStamped:
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
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Goal to ' + str(pose.pose.position.x) + ' ' +
                       str(pose.pose.position.y) + ' was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

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
        self._wait_for_node_to_activate(self.topic_prefix + 'bt_navigator')
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
