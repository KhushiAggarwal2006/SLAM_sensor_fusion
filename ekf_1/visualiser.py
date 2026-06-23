#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

import matplotlib.pyplot as plt

class OdomVisualizer(Node):

    def __init__(self):
        super().__init__('odom_visualizer')

        self.subscription = self.create_subscription(
            Odometry,
            '/zed/zed_node/odom',
            self.odom_callback,
            10)

        self.x_data = []
        self.y_data = []

        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [])

    def odom_callback(self, msg):

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.x_data.append(x)
        self.y_data.append(y)

        self.line.set_xdata(self.x_data)
        self.line.set_ydata(self.y_data)

        self.ax.relim()
        self.ax.autoscale_view()

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)

    node = OdomVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
