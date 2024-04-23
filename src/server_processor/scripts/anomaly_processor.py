#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import json



#Callback function
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data) #displays the anomaly message
    msg = json.loads(data.data)

    #sets the priority level
    priority_msg = f"priority_{'1' if msg['type'] == 'Fire' else '1' if msg['type'] == 'unidentified' else '2' if msg['type'] == 'garbage' else '3'}:{data.data}"
    pub.publish(priority_msg)

#this function starts a publisher on the display_topic and starts a listener on the anomaly_topic
def listener():
    global pub
    pub = rospy.Publisher('display_topic', String, queue_size=10)
    rospy.init_node('anomaly_processor', anonymous=True)
    rospy.Subscriber("anomaly_topic", String, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
