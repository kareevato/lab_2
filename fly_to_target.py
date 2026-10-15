import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped

from .flight_test.flight_state import FlightState
from .flight_test.flight_test_node import FlightTestNode


class FlyToTargetNode(FlightTestNode):
    HOVER_DURATION = 5.0
    DX = 4.0
    DY = 0.5
    TARGET_TOLERANCE = 0.25
    MIN_ALTITUDE = 1.8

    def __init__(self):
        super().__init__()
        self.start_x = None
        self.start_y = None
        self.start_z = None
        self.target_x = None
        self.target_y = None
        self.target_reached = False
        self.last_log = 0.0
        self.cmd_vel_publisher = self.create_publisher(
            TwistStamped, "/ap/v1/cmd_vel", 10
        )
        self.get_logger().info("variant 33")

    def odometry_callback(self, message):
        super().odometry_callback(message)
        if self.start_x is None:
            p = message.pose.pose.position
            self.start_x, self.start_y, self.start_z = p.x, p.y, p.z
            self.target_x = self.start_x + self.DX
            self.target_y = self.start_y + self.DY
            self.get_logger().info(f"start x={self.start_x:.3f} y={self.start_y:.3f}")
            self.get_logger().info(f"target x={self.target_x:.3f} y={self.target_y:.3f}")

    def control_loop(self):
        if self.state == FlightState.LANDING and not self.target_reached:
            self.fly_to_target()
            return
        super().control_loop()
        if self.state == FlightState.FINISHED and self.odometry is not None:
            x, y, _ = self.position()
            self.get_logger().info(
                f"after land x={x:.3f} y={y:.3f} e={self.distance():.3f}"
            )

    def fly_to_target(self):
        if self.odometry is None:
            return
        x, y, z = self.position()
        if z - self.start_z < self.MIN_ALTITUDE:
            return
        d = self.distance()
        now = time.monotonic()
        if now - self.last_log > 1:
            self.last_log = now
            self.get_logger().info(f"x={x:.3f} y={y:.3f} dist={d:.3f}")
        if d <= self.TARGET_TOLERANCE:
            self.send_velocity(0.0, 0.0)
            self.get_logger().info(f"before land x={x:.3f} y={y:.3f} e={d:.3f}")
            self.target_reached = True
            return
        v = max(0.15, min(1.0, 0.6 * d))
        self.send_velocity(
            v * (self.target_x - x) / d,
            v * (self.target_y - y) / d,
        )

    def position(self):
        p = self.odometry.pose.pose.position
        return p.x, p.y, p.z

    def distance(self):
        x, y, _ = self.position()
        return math.hypot(self.target_x - x, self.target_y - y)

    def send_velocity(self, vx, vy):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        self.cmd_vel_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FlyToTargetNode()
    try:
        while (
            rclpy.ok()
            and node.state != FlightState.FINISHED
            and node.state != FlightState.ERROR
        ):
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
