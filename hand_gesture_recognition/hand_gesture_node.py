#!/usr/bin/env python3.9
import rospy
import time
import hardcoded_bridge as wtf
from hand_gesture_detector import Hand_Gesture_Detector
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class hand_gesture_node():
    def __init__(self):
        self.subscriber = rospy.Subscriber('/maqui/camera/front/image_raw', Image, self._callback)
        self.publisher = rospy.Publisher('/maqui/interactions/hand_gestures', String , queue_size=10, latch=True)
        self.sphr_subscriber = rospy.Subscriber('/maqui/interactions/semaphore', Bool, self._callback2)
        self.semaphore = True

    def _callback(self, data):
        if self.semaphore:
            cv_image = wtf.imgmsg_to_cv2(data)

            #Haz tu transformacion de la imagen aqui
            detector = Hand_Gesture_Detector()
            result = detector.get_proba(cv_image)
            
            output = String()

            try:
                if type(result) != type(None) and max(result[1]) > 0.6:
                    output.data = str({'time': time.time(), 'data': f'$GestureS {result[0]}'})
                    print(f'Sending: {output.data}')
                    self.publisher.publish(output)
                else:
                    pass

            except:
                with Exception as e:
                    print(e)

    def _callback2(self, msg):
        sphr = msg.data
        self.semaphore = sphr

def main():

    rospy.init_node('hand_gesture_node')
    hand_gesture_node()
    rospy.spin()

if __name__ == '__main__':
    main()