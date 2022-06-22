#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, String
import os
from natsort import natsorted



def talker():
    # Publisher
    pub = rospy.Publisher('do_inference_topic', String, queue_size=10)
    # Init node
    rospy.init_node('inference_trigger', anonymous=True)
    # Sleep object lets us wait
    rate = rospy.Rate(0.25)  # 10hz
    
    num_files = 0
    while not rospy.is_shutdown():
        # print(os.getcwd())
        dir_path = './src/test1/scripts/src/fileName'
        img_files = natsorted([f'{dir_path}/{x}' for x in os.listdir(dir_path)])
        len_file_list = len(img_files)

        if len_file_list > num_files and num_files is not None:
            print(f'there are now {len_file_list} images in the file')
            num_files = len_file_list
            test_img_path = img_files[-1]

        else:
            test_img_path = None
        
        pub.publish(test_img_path)
        rate.sleep()
        


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
