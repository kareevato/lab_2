#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger
from ardupilot_msgs.msg import Status
from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff


class FlyToTarget(Node):
    def __init__(self):
        super().__init__("fly_to_target")
        self.dx, self.dy = 4.0, 0.5
        self.alt = 2.0
        self.ok = 0.25
        self.state = "wait"
        self.busy = False
        self.status = None
        self.odom = None
        self.x0 = self.y0 = self.z0 = None
        self.xt = self.yt = None
        self.t0 = None
        self.last = 0.0
        self.create_subscription(Status, "/ap/v1/status", self.on_st, 10)
        self.create_subscription(Odometry, "/odometry", self.on_od, 10)
        self.cmd = self.create_publisher(TwistStamped, "/ap/v1/cmd_vel", 10)
        self.prearm = self.create_client(Trigger, "/ap/v1/prearm_check")
        self.mode = self.create_client(ModeSwitch, "/ap/v1/mode_switch")
        self.armc = self.create_client(ArmMotors, "/ap/v1/arm_motors")
        self.toff = self.create_client(Takeoff, "/ap/v1/experimental/takeoff")
        self.create_timer(0.2, self.loop)
        self.get_logger().info("variant 33")

    def on_st(self, m):
        self.status = m

    def on_od(self, m):
        self.odom = m
        if self.x0 is None:
            p = m.pose.pose.position
            self.x0, self.y0, self.z0 = p.x, p.y, p.z
            self.xt, self.yt = self.x0 + self.dx, self.y0 + self.dy
            self.get_logger().info(f"start {self.x0:.3f} {self.y0:.3f}")
            self.get_logger().info(f"target {self.xt:.3f} {self.yt:.3f}")

    def xyz(self):
        p = self.odom.pose.pose.position
        return p.x, p.y, p.z

    def dist(self):
        x, y, _ = self.xyz()
        return math.hypot(self.xt - x, self.yt - y)

    def vel(self, vx, vy):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "map"
        m.twist.linear.x = float(vx)
        m.twist.linear.y = float(vy)
        self.cmd.publish(m)

    def call(self, client, req, nxt, check):
        self.busy = True
        fut = client.call_async(req)
        fut.add_done_callback(lambda f, n=nxt, c=check: self.got(f, n, c))

    def got(self, fut, nxt, check):
        self.busy = False
        try:
            r = fut.result()
        except Exception as e:
            self.get_logger().error(str(e))
            self.state = "err"
            return
        if not check(r):
            self.get_logger().error("fail " + self.state)
            self.state = "err"
            return
        self.state = nxt

    def loop(self):
        if self.state == "wait":
            if self.status and self.prearm.service_is_ready() and self.mode.service_is_ready() and self.armc.service_is_ready() and self.toff.service_is_ready():
                self.state = "prearm"
        elif self.state == "prearm" and not self.busy:
            self.call(self.prearm, Trigger.Request(), "guided", lambda r: r.success)
        elif self.state == "guided" and not self.busy:
            req = ModeSwitch.Request(); req.mode = 4
            self.call(self.mode, req, "arm", lambda r: r.status and r.curr_mode == 4)
        elif self.state == "arm" and not self.busy:
            req = ArmMotors.Request(); req.arm = True
            self.call(self.armc, req, "takeoff", lambda r: r.result)
        elif self.state == "takeoff" and not self.busy:
            req = Takeoff.Request(); req.alt = float(self.alt)
            self.call(self.toff, req, "hover", lambda r: r.status)
            self.t0 = time.monotonic()
        elif self.state == "hover":
            if self.odom is None:
                return
            _, _, z = self.xyz()
            if z - self.z0 < 1.8:
                self.t0 = time.monotonic()
                return
            if time.monotonic() - self.t0 < 5:
                self.vel(0, 0)
                return
            self.get_logger().info("go")
            self.t0 = time.monotonic()
            self.state = "fly"
        elif self.state == "fly":
            x, y, _ = self.xyz()
            d = self.dist()
            now = time.monotonic()
            if now - self.last > 1:
                self.last = now
                self.get_logger().info(f"x={x:.3f} y={y:.3f} dist={d:.3f}")
            if d <= self.ok:
                self.vel(0, 0)
                self.t0 = now
                self.state = "hold"
                return
            v = max(0.15, min(1.0, 0.6 * d))
            self.vel(v * (self.xt - x) / d, v * (self.yt - y) / d)
        elif self.state == "hold":
            self.vel(0, 0)
            if time.monotonic() - self.t0 < 2:
                return
            if self.dist() > self.ok:
                self.state = "fly"
                return
            x, y, z = self.xyz()
            self.get_logger().info(f"before land x={x:.3f} y={y:.3f} e={self.dist():.3f}")
            self.state = "land"
        elif self.state == "land" and not self.busy:
            req = ModeSwitch.Request(); req.mode = 9
            self.call(self.mode, req, "wait_land", lambda r: r.status and r.curr_mode == 9)
        elif self.state == "wait_land":
            if self.status and not self.status.flying:
                x, y, z = self.xyz()
                self.get_logger().info(f"after land x={x:.3f} y={y:.3f} e={self.dist():.3f}")
                self.state = "done"


def main(args=None):
    rclpy.init(args=args)
    n = FlyToTarget()
    try:
        while rclpy.ok() and n.state not in ("done", "err"):
            rclpy.spin_once(n, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
