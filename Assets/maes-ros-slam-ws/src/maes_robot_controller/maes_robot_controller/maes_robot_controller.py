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

    def __init__(self, robot_id):
        super().__init__(robot_id)
        # Define topics
        self.topic_prefix = "/{0}".format(robot_id)
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

        # Register nav2 navigator
        self.navigator = BasicNavigator(topic_prefix=self.topic_prefix)


    def logic_loop(self, msg: State):
        # self.get_logger().info('Robot0 heard: "%s"' % msg)
        # self.broadcast_msg(msg="Testing broadcasting")
        # self.deposit_tag(tag_msg="Content of env_tag")
        if(msg.tick == 1):
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = 10.0
            goal_pose.pose.position.y = -2.0
            goal_pose.pose.position.z = 0.0
            goal_pose.pose.orientation.x = 0.0
            goal_pose.pose.orientation.y = 0.0
            goal_pose.pose.orientation.z = 0.0
            goal_pose.pose.orientation.w = 1.0

            self.navigator.go_to_pose(goal_pose)
        if(msg.tick == 100):
            self.navigator.cancel_nav()

        self.info(str(self.navigator.is_nav_complete()) + " " + str(self.navigator.get_feedback()))


    def deposit_tag(self, tag_msg):
        request = DepositTag.Request()
        request.msg = tag_msg
        self.deposit_env_tag_srv.call_async(request=request)

    def broadcast_msg(self, msg):
        request = BroadcastToAll.Request()
        request.msg = msg
        self.broadcast_srv.call_async(request=request)


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

    controller = RobotController('robot0')

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


class BasicNavigator(Node):
    def __init__(self, topic_prefix: str):
        super().__init__(node_name=topic_prefix[1:] + '_basic_navigator')
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None
        self.topic_prefix = topic_prefix

        self.initial_pose_received = False
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, topic_prefix + '/navigate_to_pose')

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