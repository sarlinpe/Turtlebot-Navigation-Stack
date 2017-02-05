#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, Pose, TwistWithCovariance, Twist

robot_name = "mobile_base"

def callback(model_states):
	rospy.loginfo("Received ModelStates")
	try:
		idx = model_states.name.index(robot_name)
	except ValueError:
		rospy.logerr("[ModelStates] Could not find model with name %s", robot_name)
		return

	model_pose = model_states.pose[idx]
	model_twist = model_states.twist[idx]

	msg = Odometry()
	msg.pose.pose = model_pose
	msg.twist.twist = model_twist

	pub.publish(msg)
	

def initialize():
	global pub
	rospy.init_node("odom_correcter", anonymous=True)	
	rospy.Subscriber("/gazebo/model_states", ModelStates, callback)
	pub = rospy.Publisher("/odom_true", Odometry, queue_size=1)
	rospy.spin()


if __name__ == "__main__":
	initialize()
