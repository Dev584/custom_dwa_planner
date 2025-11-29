#!/usr/bin/env python3

import rclpy
import math
import random
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from tf_transformations import euler_from_quaternion

class DWAPlannerNode(Node):
    def __init__(self):
        super().__init__('dwa_planner_node')
        
        # Variables Initialization
        self.goal_x = None
        self.goal_y = None
        self.goal_reached = False
        self.odom_data = None
        self.scan_data = None
        self.pose = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0]

        # Pubs and Subs
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.cmd_pub = self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(MarkerArray, '/dwa_path', 10)

        # Parameters
        self.declare_parameter('goal_threshold', 0.15)
        self.declare_parameter('robot_radius', 0.22)
        self.declare_parameter('max_speed', 0.3)
        self.declare_parameter('max_yaw_rate', 2.5)
        self.declare_parameter('max_accel', 0.2)
        self.declare_parameter('max_delta_yaw_rate', 3.2)
        self.declare_parameter('v_resolution', 0.02)
        self.declare_parameter('yaw_rate_resolution', 0.2)
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('predict_time', 1.5)

        self.goal_threshold = self.get_parameter('goal_threshold').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_yaw_rate = self.get_parameter('max_yaw_rate').value
        self.max_accel = self.get_parameter('max_accel').value
        self.max_delta_yaw_rate = self.get_parameter('max_delta_yaw_rate').value
        self.v_resolution = self.get_parameter('v_resolution').value
        self.yaw_rate_reso = self.get_parameter('yaw_rate_resolution').value
        self.dt = self.get_parameter('dt').value
        self.predict_time = self.get_parameter('predict_time').value

        # Timer Loop
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info("DWA Planner Node has been started.")
        self.get_logger().info("Set goal using RViz '2D Pose Goal' or publish to /pose_goal")


    def goal_callback(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_reached = False
        self.get_logger().info(f"New goal received: ({self.goal_x:.2f}, {self.goal_y:.2f})")

    def odom_callback(self, msg):
        self.odom_data = msg
        self.pose[0] = msg.pose.pose.position.x
        self.pose[1] = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.pose[2] = yaw
        self.velocity[0] = msg.twist.twist.linear.x
        self.velocity[1] = msg.twist.twist.angular.z

    def scan_callback(self, msg):
        self.scan_data = msg

    def predict_motion(self, v, w):
        traj = []
        x, y, yaw = self.pose
        time = 0.0
        while time <= self.predict_time:
            x += v * math.cos(yaw) * self.dt
            y += v * math.sin(yaw) * self.dt
            yaw += w * self.dt
            traj.append((x, y))
            time += self.dt
        return traj

    def calc_goal_cost(self, traj):
        dx = self.goal_x - traj[-1][0]
        dy = self.goal_y - traj[-1][1]
        return math.hypot(dx, dy)
    
    def calc_obstacle_cost(self, traj):
        if self.scan_data is None:
            return 0.0, float('inf')
        min_dist = float('inf')

        for traj_x, traj_y in traj:
            for i, r in enumerate(self.scan_data.ranges):
                if math.isinf(r) or math.isnan(r) or r < self.robot_radius:
                    continue
                angle = self.scan_data.angle_min + i * self.scan_data.angle_increment
                obs_x = self.pose[0] + r * math.cos(angle + self.pose[2] + angle)
                obs_y = self.pose[1] + r * math.sin(angle + self.pose[2] + angle)
                dist = math.hypot(traj_x - obs_x, traj_y - obs_y)
                if dist < min_dist:
                    min_dist = dist
        if min_dist == float('inf'):
            return 0.0, min_dist
        if min_dist < self.robot_radius +0.1:
            return 10.0 / min_dist, min_dist
        return 1.0 / min_dist, min_dist
    
    def dwa_planning(self):
        min_v = max(0.0, self.velocity[0] - self.max_accel * self.dt)
        max_v = min(self.max_speed, self.velocity[0] + self.max_accel * self.dt)
        min_w = max(-self.max_yaw_rate, self.velocity[1] - self.max_delta_yaw_rate * self.dt)
        max_w = min(self.max_yaw_rate, self.velocity[1] + self.max_delta_yaw_rate * self.dt)
        best_cost = float('inf')
        best_u = [0.0, 0.0]
        best_traj = []
        for v in np.arange(min_v, max_v, self.v_resolution):
            for w in np.arange(min_w, max_w, self.yaw_rate_reso):
                traj = self.predict_motion(v, w)
                goal_cost = self.calc_goal_cost(traj)
                obs_cost, min_dist = self.calc_obstacle_cost(traj)
                speed_cost = self.max_speed - v

                total_cost = (goal_cost * 1.0 + obs_cost * 1.0 + speed_cost * 0.3)

                if total_cost < best_cost and min_dist > self.robot_radius:
                    best_cost = total_cost
                    best_u = [v, w]
                    best_traj = traj
        return best_u, best_traj

    def control_loop(self):
        if (self.odom_data is None or
            self.scan_data is None or
            self.goal_x is None or
            self.goal_y is None):
            return

        dist_to_goal = math.hypot(self.goal_x - self.pose[0], self.goal_y - self.pose[1])
        if dist_to_goal < self.goal_threshold:
            if not self.goal_reached:
                self.goal_reached = True
                self.get_logger().info("Goal reached!")
            twist = Twist()
            self.cmd_pub.publish(twist)
            return
        best_cmd, best_traj = self.dwa_planning()
        twist = Twist()
        twist.linear.x = best_cmd[0]
        twist.angular.z = best_cmd[1]
        self.cmd_pub.publish(twist)

        self.get_logger().info(f"Cmd: v={best_cmd[0]:.2f}, w={best_cmd[1]:.2f}, Dist to Goal: {dist_to_goal:.2f}")
        self.publish_traj(best_traj)

    def publish_traj(self, traj):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.id = 0
        marker.scale.x = 0.02
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        marker.pose.orientation.w = 1.0

        for x, y in traj:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.0
            marker.points.append(p)
        
        marker_array.markers.append(marker)
        self.path_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = DWAPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()