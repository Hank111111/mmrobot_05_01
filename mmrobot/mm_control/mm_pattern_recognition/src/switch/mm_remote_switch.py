#!/usr/bin/env python

from mm_pattern_recognition.srv import *
import rospy
from cv_bridge import CvBridge, CvBridgeError
from train import preprocess
from sklearn.externals import joblib
from os.path import join
import rospkg
class RemoteSwitchRecognizeServer:
    def __init__(self):
        self.bridge = CvBridge()
        rospack = rospkg.RosPack()
        model_path = join(rospack.get_path('mm_pattern_recognition'), "src/switch/model/train_model.m")
        print(model_path)
        self.model = joblib.load(model_path)
        
        rospy.init_node('recognize_remote_switch_status')
        s = rospy.Service('recognize_remote_switch_status', RecognizeStatus, self.handle_remote_switch_status_request)
        print("Ready to recognize_remote_switch_status.")
        rospy.spin()

    def handle_remote_switch_status_request(self, req):    
        response = RecognizeStatusResponse()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(req.image_message, "bgr8")
            data = preprocess(cv_image)
            status = self.model.predict(data.reshape(1,-1))
            response.status.append(status)
            print("remote switch status: {}".format(status))
        except CvBridgeError as e:
            print(e)

        return response

if __name__ == "__main__":
    server = RemoteSwitchRecognizeServer()