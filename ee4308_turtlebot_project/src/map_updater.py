#!/usr/bin/env python


import rospy
#from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField

def process_pc(pc_msg):
    h = pc_msg.height
    w = pc_msg.width

    rospy.loginfo("Dimesions of PL array: %s", (h,w))
    rospy.loginfo("Point step: %s", pc_msg.point_step)
    print pc_msg.fields


if __name__ == "__main__":
    rospy.init_node("map_updater", anonymous=True)
    rospy.Subscriber("/camera/depth/points_throttle", PointCloud2, process_pc)
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down node: %s", rospy.get_name())
