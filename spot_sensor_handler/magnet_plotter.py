#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import matplotlib.pyplot as plt
from collections import deque

class MagnetPlotter(Node):
    def __init__(self):
        super().__init__('magnet_plotter')

        # buffer length
        self.buffer_size = 200

        # deques for timestamps and each axis
        self.times = deque(maxlen=self.buffer_size)
        self.x_vals = deque(maxlen=self.buffer_size)
        self.y_vals = deque(maxlen=self.buffer_size)
        self.z_vals = deque(maxlen=self.buffer_size)

        # record start time in seconds
        self.start_time = self.get_clock().now().nanoseconds / 1e9

        # subscriptions with best-effort QoS to match micro-ROS
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Int32, 'bmm150_x', self.cb_x, qos_profile=qos)
        self.create_subscription(Int32, 'bmm150_y', self.cb_y, qos_profile=qos)
        self.create_subscription(Int32, 'bmm150_z', self.cb_z, qos_profile=qos)

        # interactive mode on
        plt.ion()

        # Figure for X
        self.fig_x = plt.figure()
        self.ax_x = self.fig_x.add_subplot(1, 1, 1)
        self.line_x, = self.ax_x.plot([], [], color='blue')
        self.ax_x.set_xlabel('Time (s)')
        self.ax_x.set_ylabel('Mag X (Int32)')
        self.ax_x.set_title('Live BMM150 X-axis')

        # Figure for Y
        self.fig_y = plt.figure()
        self.ax_y = self.fig_y.add_subplot(1, 1, 1)
        self.line_y, = self.ax_y.plot([], [], color='green')
        self.ax_y.set_xlabel('Time (s)')
        self.ax_y.set_ylabel('Mag Y (Int32)')
        self.ax_y.set_title('Live BMM150 Y-axis')

        # Figure for Z
        self.fig_z = plt.figure()
        self.ax_z = self.fig_z.add_subplot(1, 1, 1)
        self.line_z, = self.ax_z.plot([], [], color='red')
        self.ax_z.set_xlabel('Time (s)')
        self.ax_z.set_ylabel('Mag Z (Int32)')
        self.ax_z.set_title('Live BMM150 Z-axis')

        # one timer to refresh all plots
        self.create_timer(0.2, self.update_plots)

    def record_time(self):
        t = self.get_clock().now().nanoseconds / 1e9 - self.start_time
        # only append a new timestamp once per callback
        if not self.times or t != self.times[-1]:
            self.times.append(t)
        return t

    def cb_x(self, msg):
        self.record_time()
        self.x_vals.append(msg.data)

    def cb_y(self, msg):
        self.record_time()
        self.y_vals.append(msg.data)

    def cb_z(self, msg):
        self.record_time()
        self.z_vals.append(msg.data)

    def update_plots(self):
        if not self.times:
            return

        t = list(self.times)

        # --- X plot ---
        x = list(self.x_vals)
        self.line_x.set_data(t[-len(x):], x)
        self.ax_x.set_xlim(t[0], t[-1])
        if x:
            ymin, ymax = min(x), max(x)
            self.ax_x.set_ylim(ymin - 5, ymax + 5)
        self.fig_x.canvas.draw()
        self.fig_x.canvas.flush_events()

        # --- Y plot ---
        y = list(self.y_vals)
        self.line_y.set_data(t[-len(y):], y)
        self.ax_y.set_xlim(t[0], t[-1])
        if y:
            ymin, ymax = min(y), max(y)
            self.ax_y.set_ylim(ymin - 5, ymax + 5)
        self.fig_y.canvas.draw()
        self.fig_y.canvas.flush_events()

        # --- Z plot ---
        z = list(self.z_vals)
        self.line_z.set_data(t[-len(z):], z)
        self.ax_z.set_xlim(t[0], t[-1])
        if z:
            ymin, ymax = min(z), max(z)
            self.ax_z.set_ylim(ymin - 5, ymax + 5)
        self.fig_z.canvas.draw()
        self.fig_z.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = MagnetPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
