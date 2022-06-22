#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, String
# from main import pictureSaved


def talker():
    # Publisher
    pub = rospy.Publisher('do_inference_topic', String, queue_size=10)
    # Init node
    rospy.init_node('inference_trigger', anonymous=True)
    # Sleep object lets us wait
    rate = rospy.Rate(10)  # 10hz
    pictureSaved=False
    while not rospy.is_shutdown():
        # Check if an image has been captured
        if pictureSaved:
            # get the file path of the image that has been captured
            img_path = f'testtesttest'

        else:
            img_path = f'Waiting for image to classify'

        pub.publish(img_path)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
