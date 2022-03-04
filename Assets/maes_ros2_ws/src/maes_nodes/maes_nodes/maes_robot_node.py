import rclpy
from maes_interface.msg import RobotState
from rclpy.node import Node

from std_msgs.msg import String


class MaesRobot(Node):

    def __init__(self):
        super().__init__(node_name='maes_robot_0')
        self.create_subscription(
            RobotState,
            '/robot1/robot_state',
            self.robot_state_callback,
            10)


    def robot_state_callback(self, msg: RobotState):
        print(msg)


def main(args=None):
    rclpy.init(args=args)

    maes_robot = MaesRobot()

    rclpy.spin(maes_robot)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    maes_robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()