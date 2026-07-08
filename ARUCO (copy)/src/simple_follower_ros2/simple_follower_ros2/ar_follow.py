#!/usr/bin/env python3
# coding=utf-8

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy._rclpy_pybind11 import RCLError
from visualization_msgs.msg import Marker

#from dynamic_reconfigure.server import Server
#from simple_follower.cfg import arPIDConfig


class ArFollower(Node):
	def __init__(self):
		
		super().__init__('arfollower')
		
		#参数初始化
		self.linearfront_p = 0.7
		self.linearback_p = 1.0
		self.angularleft_p = 0.375
		self.angularright_p = 0.375
		self.d_param = 0.8
		
		self.max_angular_speed = 0.42
		self.min_angular_speed = -0.42
		self.max_linear_speed = 0.35
		self.min_linear_speed = -0.21
		self.goal_x =0.6
		self.goal_y =0.0
		self.lateral_deadband = 0.05
		self.marker_timeout = 0.4
		self.stopped_for_lost_marker = True
		self.last_marker_time = None
				
		#订阅AR标签位姿信息，发布速度话题
		qos = QoSProfile(depth=10)
		self.cmdvelpublisher=self.create_publisher(Twist,'/cmd_vel',qos)
		self.arposesubscriber=self.create_subscription(
			Marker,
			'/aruco_single/marker',
			self.set_cmd_vel,
			qos)					

		self.move_cmd = Twist()
		self.move_cmd.linear.x = 0.0
		self.move_cmd.angular.z = 0.0
		self.stop_timer = self.create_timer(0.1, self.stop_if_marker_lost)
		
	def clamp(self, value, min_value, max_value):
		return max(min(value, max_value), min_value)

	def publish_stop(self):
		self.move_cmd = Twist()
		self.cmdvelpublisher.publish(self.move_cmd)

	def calculate_angular_speed(self, target_offset_y):
		if abs(target_offset_y - self.goal_y) < self.lateral_deadband:
			return 0.0

		if target_offset_y > self.goal_y:
			angularspeed = target_offset_y * self.angularleft_p
			angularspeed = self.clamp(angularspeed, 0.0, self.max_angular_speed)
		else:
			angularspeed = target_offset_y * self.angularright_p
			angularspeed = self.clamp(angularspeed, self.min_angular_speed, 0.0)
		return -angularspeed

	def stop_if_marker_lost(self):
		if self.last_marker_time is None:
			self.publish_stop()
			if not self.stopped_for_lost_marker:
				self.stopped_for_lost_marker = True
			return

		elapsed = self.get_clock().now() - self.last_marker_time
		if elapsed > Duration(seconds=self.marker_timeout):
			self.publish_stop()
			if not self.stopped_for_lost_marker:
				self.get_logger().warn('ArUco marker lost, stop robot and wait for detection.')
			self.stopped_for_lost_marker = True

	def set_cmd_vel(self, msg):
		self.last_marker_time = self.get_clock().now()
		if self.stopped_for_lost_marker:
			self.get_logger().info('ArUco marker detected, resume following.')
		self.stopped_for_lost_marker = False
				
		offset_y = 0.15 #小车中心与摄像头检测到的AR标签中心的偏差
		target_offset_y = msg.pose.position.x - offset_y #AR标签位姿信息x方向(已校正)-对应ROS中y方向
		target_offset_x = msg.pose.position.z #AR标签位姿信息z方向-对应ROS中x方向		

		#当AR标签和小车的距离与设定距离存在偏差时
		if target_offset_x > self.goal_x:
			linearspeed = target_offset_x * self.linearfront_p 
			if linearspeed < 0.01:
				linearspeed = 0.0
				#极低速置零，避免小车摇摆
			linearspeed = self.clamp(linearspeed, 0.0, self.max_linear_speed)
			self.move_cmd.linear.x = linearspeed
			self.move_cmd.angular.z = self.calculate_angular_speed(target_offset_y)
		else:
			linearspeed = (target_offset_x - self.goal_x) * self.linearback_p
			if abs(linearspeed) < 0.01:
				linearspeed = 0.0
			linearspeed = self.clamp(linearspeed, self.min_linear_speed, 0.0)
			self.move_cmd.linear.x = linearspeed
			self.move_cmd.angular.z = self.calculate_angular_speed(target_offset_y)
		
		self.cmdvelpublisher.publish(self.move_cmd)
	

def main(args=None):
	
	rclpy.init(args=args)
	print('ar following start!')
	arfollower=ArFollower()
	try:
		rclpy.spin(arfollower)
	except (ExternalShutdownException, KeyboardInterrupt, RCLError):
		pass
	finally:
		arfollower.destroy_node()
		if rclpy.ok():
			rclpy.shutdown()
		

if __name__ == '__main__':
    main()

