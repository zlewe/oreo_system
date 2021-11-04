import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time
import random
import numpy as np
from math import pi as PI

class Combine_laser:
	def __init__(self):
		rospy.init_node('combine_laser')
		self.sub1 = rospy.Subscriber('/scan3', LaserScan, self.callback)
		self.sub2 = rospy.Subscriber('/scan_back', LaserScan, self.back_cb)
		self.pub = rospy.Publisher('/scan_com', LaserScan, queue_size=100)
		# self.laser = LaserScan()
		rospy.spin()

	def callback(self, data):
		laser = LaserScan()
		laser.header = data.header
		laser.header.stamp = 
		laser.header.frame_id = 'laser_com'
		laser.angle_min = data.angle_min
		laser.angle_max = data.angle_max
		laser.time_increment = data.angle_increment
		laser.scan_time = data.scan_time
		laser.range_min = data.range_min
		laser.range_max = data.range_max
		# laser.intensities = data.intensities
		# print(data)
		# print(self.back_laser)
		try:
			laser.ranges.append(data.ranges)
			# laser.ranges.append(self.back_laser.ranges)
			laser.ranges = list(laser.ranges[0])
			laser.ranges += list(self.back_laser.ranges)
			# print(laser.ranges)
			# laser.ranges.append(self.back_laser.ranges)
		# laser.intensities = data.intensities.append(self.back_laser.intensities)
			self.pub.publish(laser)
			print("fucks")
		except:
			pass
		# print laser.ranges
		# print (list(laser.ranges))
		# print len(laser.ranges)

	def back_cb(self, data):
		self.back_laser = data
		# print(data.ranges)
		# print(self.back_laser)

Combine_laser()