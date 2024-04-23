#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32

#this function publishes an integers on a topic called anomaly
if __name__ == '__main__':
    rospy.init_node("anomaly_detection")
    rospy.loginfo("program started")
    pub = rospy.Publisher("/anomaly",Int32,queue_size=10)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        msg = Int32()
        msg = 1
        pub.publish(msg)
        rate.sleep()
