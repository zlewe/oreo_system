#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import numpy as np

def cmd_callback(data):
	global pub, multiplier
	L = 400.0 * multiplier
	A = 0.5
	velocity = Twist()
	velocity.linear.x = data.linear.x * L
	velocity.linear.y = data.linear.y * L
	velocity.linear.z = data.linear.z * L

	velocity.angular.x = data.angular.x * A
	velocity.angular.y = data.angular.y * A
	velocity.angular.z = data.angular.z * A

	pub.publish(velocity)
	
    

print("scale_velocity")
rospy.init_node('scale_velocity')
multiplier = rospy.get_param('vel_mult', default=1.0)
rospy.Subscriber("/temp_cmd_vel", Twist, cmd_callback)
pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
rospy.spin()

