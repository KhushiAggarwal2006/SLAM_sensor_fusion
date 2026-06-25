import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped

from qt_gui.plugin import Plugin
from python_qt_binding.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QCheckBox
)

import pyqtgraph as pg


# ================= ROS NODE ================= #

class TrajectoryNode(Node):

    def __init__(self):
        super().__init__('trajectory_rqt_node')

        # buffers
        self.zed_x, self.zed_y = [], []
        self.imu_x, self.imu_y = [], []
        self.filt_x, self.filt_y = [], []
        self.tf_x, self.tf_y = [], []
        self.uni_x, self.uni_y = [], []

        # subscribers
        self.create_subscription(
            PoseStamped,
            '/zed/zed_node/pose',
            self.cb_zed,
            10
        )

        self.create_subscription(
            Odometry,
            '/imu_state',
            self.cb_imu,
            10
        )

        self.create_subscription(
            Odometry,
            '/filtered_state',
            self.cb_filtered,
            10
        )

        self.create_subscription(
            Odometry,
            '/unicycle_dynamics',
            self.cb_uni,
            10
        )

        # Direct TF topic subscription
        self.create_subscription(
            TFMessage,
            '/tf',
            self.cb_tf,
            100
        )

    # -------- callbacks -------- #

    def cb_zed(self, msg):
        self.zed_x.append(msg.pose.position.x)
        self.zed_y.append(msg.pose.position.y)

    def cb_imu(self, msg):
        self.imu_x.append(msg.pose.pose.position.x)
        self.imu_y.append(msg.pose.pose.position.y)

    def cb_filtered(self, msg):
        self.filt_x.append(msg.pose.pose.position.x)
        self.filt_y.append(msg.pose.pose.position.y)

    def cb_uni(self, msg):
        self.uni_x.append(msg.pose.pose.position.x)
        self.uni_y.append(msg.pose.pose.position.y)

    def cb_tf(self, msg):
        for transform in msg.transforms:

            if (
                transform.header.frame_id == 'world'
                and transform.child_frame_id == 'base_link'
            ):
                self.tf_x.append(transform.transform.translation.x)
                self.tf_y.append(transform.transform.translation.y)
                break


# ================= RQT PLUGIN ================= #

class TrajectoryWidget(Plugin):

    def __init__(self, context):
        super(TrajectoryWidget, self).__init__(context)

        self.setObjectName('TrajectoryWidget')

        pg.setConfigOption('background', 'w')
        pg.setConfigOption('foreground', 'k')

        # -------- QWidget -------- #
        self._widget = QWidget()
        self._layout = QVBoxLayout(self._widget)

        # DO NOT call rclpy.init() in rqt
        self.node = TrajectoryNode()

        # -------- toggle checkboxes -------- #
        self._toggle_layout = QHBoxLayout()

        self.zed_checkbox = QCheckBox("ZED Odom")
        self.imu_checkbox = QCheckBox("IMU State")
        self.filt_checkbox = QCheckBox("Filtered State")
        self.tf_checkbox = QCheckBox("TF world→base_link")
        self.uni_checkbox = QCheckBox("Unicycle Dynamics")

        for cb in (
            self.zed_checkbox,
            self.imu_checkbox,
            self.filt_checkbox,
            self.tf_checkbox,
            self.uni_checkbox
        ):
            cb.setChecked(True)
            self._toggle_layout.addWidget(cb)

        self._layout.addLayout(self._toggle_layout)

        # -------- plot -------- #
        self.plot = pg.PlotWidget(title="Multi Trajectory Viewer")
        self.plot.showGrid(x=True, y=True)
        self.plot.addLegend()
        self.plot.setAspectLocked(True)

        self._layout.addWidget(self.plot)

        # curves
        self.zed_curve = self.plot.plot(
            pen=pg.mkPen(color='#c0392b', width=2),
            name="ZED Odom"
        )

        self.imu_curve = self.plot.plot(
            pen=pg.mkPen(color='#196f3d', width=2),
            name="IMU State"
        )

        self.filt_curve = self.plot.plot(
            pen=pg.mkPen(color='#1a5276', width=2),
            name="Filtered State"
        )

        self.tf_curve = self.plot.plot(
            pen=pg.mkPen(color='#b9770e', width=2),
            name="TF world→base_link"
        )

        self.uni_curve = self.plot.plot(
            pen=pg.mkPen(color='#6c3483', width=2),
            name="Unicycle Dynamics"
        )

        # wire checkboxes to curve visibility
        self.zed_checkbox.stateChanged.connect(
            lambda state: self.zed_curve.setVisible(bool(state))
        )

        self.imu_checkbox.stateChanged.connect(
            lambda state: self.imu_curve.setVisible(bool(state))
        )

        self.filt_checkbox.stateChanged.connect(
            lambda state: self.filt_curve.setVisible(bool(state))
        )

        self.tf_checkbox.stateChanged.connect(
            lambda state: self.tf_curve.setVisible(bool(state))
        )

        self.uni_checkbox.stateChanged.connect(
            lambda state: self.uni_curve.setVisible(bool(state))
        )

        # Qt timer (drives both ROS + UI)
        self.timer_id = self.startTimer(50)

        context.add_widget(self._widget)

    # ================= MAIN LOOP ================= #

    def timerEvent(self, event):

        # ---- spin ROS callbacks safely ---- #
        rclpy.spin_once(self.node, timeout_sec=0.0)

        # ---- update plots ---- #

        if self.zed_curve.isVisible() and len(self.node.zed_x) > 1:
            self.zed_curve.setData(
                self.node.zed_x,
                self.node.zed_y
            )

        if self.imu_curve.isVisible() and len(self.node.imu_x) > 1:
            self.imu_curve.setData(
                self.node.imu_x,
                self.node.imu_y
            )

        if self.filt_curve.isVisible() and len(self.node.filt_x) > 1:
            self.filt_curve.setData(
                self.node.filt_x,
                self.node.filt_y
            )

        if self.tf_curve.isVisible() and len(self.node.tf_x) > 1:
            self.tf_curve.setData(
                self.node.tf_x,
                self.node.tf_y
            )

        if self.uni_curve.isVisible() and len(self.node.uni_x) > 1:
            self.uni_curve.setData(
                self.node.uni_x,
                self.node.uni_y
            )

    # ================= LIFECYCLE ================= #

    def shutdown_plugin(self):
        if self.node:
            self.node.destroy_node()

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass