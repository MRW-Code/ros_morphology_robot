#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool, String
import cv2
import torch
from fastai.vision.all import *
import os



class PubSub:

    def __init__(self):
        rospy.init_node('echoer')
        self.move = None
        self.model = load_learner('./src/robot_v2/scripts/src/model.pkl', cpu=True)
        self.sub = rospy.Subscriber('do_inference_topic', String, self.callback)
        self.pub = rospy.Publisher('ready_move', Bool, queue_size=10)
        rospy.spin()

    def infer_shape(self, path):
        img = torch.tensor(cv2.imread(str(path)))
        pred = self.model.predict(img)
        return pred


    def callback(self, data):
        if data.data != '':
            pred = self.infer_shape(path=data.data)
            print(f'THIS IMAGE IS {pred}')
            self.move = True
            self.pub.publish(self.move)
        else:
            print('waiting for something to do')






# def infer_shape(path, model):
#     img = torch.tensor(cv2.imread(str(path)))
#     pred = model.predict(img)
#     return pred

# def callback(data):
#     # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
#     # print(data.data)
#     # print(type(data.data))

#     if data.data != '':
#         # print('loading model')
#         print(os.getcwd())
#         model = load_learner('./src/robot_v2/scripts/src/model.pkl', cpu=True)
#         pred = infer_shape(path=data.data, model=model)
#         print(f'THIS IMAGE IS {pred}')
#     else: 
#         print('waiting for new image')



# def listener():
#     rospy.init_node('subscriber', anonymous=True)
#     rospy.Subscriber('do_inference_topic', String, callback)
#     rospy.spin()


if __name__ == '__main__':
    PubSub()