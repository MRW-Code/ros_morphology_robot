#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool, String
import cv2
import torch
from fastai.vision.all import *


def infer_shape(path, model):
    img = torch.tensor(cv2.imread(str(path)))
    pred = model.predict(img)
    return pred

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    # print(data.data)
    # print(type(data.data))

    if data.data != '':
        # print('loading model')
        model = load_learner('./src/test1/scripts/src/model.pkl', cpu=True)
        pred = infer_shape(path=data.data, model=model)
        print(f'THIS IMAGE IS {pred}')
    # else: 
        # print('waiting for new image')



def listener():
    rospy.init_node('subscriber', anonymous=True)
    rospy.Subscriber('do_inference_topic', String, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()