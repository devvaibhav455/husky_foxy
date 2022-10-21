#! /usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Joy



class ModeControl(Node):

    def __init__(self):
        super().__init__('mode_control')
        self.analog_axes = [0 , 1, 3, 4]
        self.button_gps_mode = 4
        self.button_local_mode = 5
        self.subscription = self.create_subscription(
            Joy,
            'joy_teleop/joy',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg)
        if(msg.buttons[self.button_gps_mode] == 1):
            # print("Received 4")
            pass


def main(args=None):
    rclpy.init(args=args)

    mode_control = ModeControl()

    rclpy.spin(mode_control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mode_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()