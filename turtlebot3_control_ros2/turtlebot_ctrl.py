#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import random

from example_interfaces.msg import String

class TurtlebotCtrl(Node):
	def __init__(self):
		super().__init__("TurtlebotCtrl")

		self.laser = LaserScan()
		self.odom = Odometry()


		self.map = np.array([	[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
								[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
								[0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
								[0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
								[0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
								[0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
								[0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0],
								[0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
								[0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
								[0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
								[0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0],
								[0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0],
								[0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
								[0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
								[0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
								[0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
								[0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0],
								[0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
								[0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
								[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
						])

		self.publish_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)
		self.subscriber_odom = self.create_subscription(Odometry, "/odom", self.callback_odom, 10)
		self.subscriber_laser = self.create_subscription(LaserScan, "/scan", self.callback_laser, 10)
		self.timer = self.create_timer(0.5, self.cmd_vel_pub)
		
		self.direction = 1  # 1 for forward, -1 for backward
		self.rotate = False
		self.rotation_time = 0
		self.turn_duration = 5  # Adjust as necessary for 90 degree turn

	def cmd_vel_pub(self):

		map_resolution = 4

		index_x = -int(self.odom.pose.pose.position.x*map_resolution)
		index_y = -int(self.odom.pose.pose.position.y*map_resolution)

		index_x += int(self.map.shape[0]/2)
		index_y += int(self.map.shape[0]/2)

		if (index_x < 1): index_x = 1
		if (index_x > self.map.shape[0]-1): index_x = self.map.shape[0]-1
		if (index_y < 1): index_y = 1
		if (index_y > self.map.shape[0]-1): index_y = self.map.shape[0]-1

		if (self.map[index_x][index_y] == 1):
			self.map[index_x][index_y] = 2

			#self.get_logger().info("Another part reached ... percentage total reached...." + str(100*float(np.count_nonzero(self.map == 2))/(np.count_nonzero(self.map == 1) + np.count_nonzero(self.map == 2))) )
		#	self.get_logger().info("Discrete Map")
		#	self.get_logger().info("\n"+str(self.map))
			print("           PORCENTAGEM: ", 100*float(np.count_nonzero(self.map == 2))/(np.count_nonzero(self.map == 1) + np.count_nonzero(self.map == 2)))

        # Desenvlva seu codigo aqui

        # Avoid obstacles based on laser scan data
		msg = Twist()
		if self.rotate:
			msg.angular.z = random.uniform(0.1, 0.4)  # Rotate to avoid obstacle
			self.rotation_time += random.uniform(1, 3)
			if self.rotation_time > self.turn_duration:
				self.rotation_time = 0
				self.rotate = False
		else:
			if min(self.laser.ranges[0:20]) < random.uniform(0.3, 0.5) or min(self.laser.ranges[-20:]) < random.uniform(0.3, 0.5):
				msg.linear.x = 0.0
				self.rotate = True
			else:
				msg.linear.x = random.uniform(0.1, 0.3)
				msg.angular.z = random.uniform(-0.3, 0.3)

		self.publish_cmd_vel.publish(msg)



	def callback_laser(self, msg):
		self.laser = msg


	def callback_odom(self, msg):
		self.odom = msg

def main(args=None):
	rclpy.init(args=args)
	node = TurtlebotCtrl()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == "__main__":
	main()
