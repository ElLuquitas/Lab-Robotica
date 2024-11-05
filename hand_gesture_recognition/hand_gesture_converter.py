#!/usr/bin/env python3.9
from __future__ import print_function

import roslib
roslib.load_manifest('cv_bridge')
import sys
import rospy
import cv2
from hand_gesture_detector import Hand_Gesture_Detector
from std_msgs.msg import String
from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError
import hardcoded_bridge as wtf

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/maqui/interactions/hand_gesture",Image, queue_size=10)
    #self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/maqui/camera/front/image_raw",Image,self.callback, queue_size=10)
    self.rate = rospy.Rate(10)

  def callback(self,data):

    t_o = rospy.rostime.get_time()
    cv_image = wtf.imgmsg_to_cv2(data)

    #Haz tu transformacion de la imagen aqui
    detector = Hand_Gesture_Detector()
    result = detector.detect_single_frame(cv_image)
    
    
    try:
      if type(result) != type(None):
        self.image_pub.publish(wtf.cv2_to_imgmsg(result))
        print(rospy.rostime.get_time()-t_o)
      else:
        self.image_pub.publish(data)
        print(rospy.rostime.get_time()-t_o)

    except:
      with Exception as e:
        print(e)
    
    self.rate.sleep()

def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except: 
    with Exception as e:
      print(e)
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)