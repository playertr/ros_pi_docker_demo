#! /usr/bin/python3

import rospy
from std_msgs.msg import Time

pub = rospy.Publisher('/time', Time, queue_size=10)
rospy.init_node('talker')

rate = rospy.Rate(10) # 10hz
while not rospy.is_shutdown():
    pub.publish(rospy.Time.now())
    rate.sleep()