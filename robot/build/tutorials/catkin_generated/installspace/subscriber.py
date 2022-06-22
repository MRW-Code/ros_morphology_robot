#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool, String


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


def listener():
    rospy.init_node('subscriber', anonymous=True)
    rospy.Subscriber('do_inference_topic', String, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
