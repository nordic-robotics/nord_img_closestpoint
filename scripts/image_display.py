import sys
import rospy
import cv
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:
    def __init__(self):
        cv2.namedWindow("Image window", 1)
        cv2.namedWindow("Depth Image window", 1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)
        self.depth_image_sub = rospy.Subscriber("/camera/depth/image_raw",Image,self.callbackDepth)


    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e
          
        (rows,cols,channels) = cv_image.shape

        if cols > 60 and rows > 60 :
            cv2.circle(cv_image, (50,50), 10, 255)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

    def callbackDepth(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
        except CvBridgeError, e:
            print e
          
        (rows,cols,channels) = cv_image.shape

        if cols > 60 and rows > 60 :
            cv2.circle(cv_image, (50,50), 10, 60000)
        

        cv2.imshow("Depth Image window", cv_image*10)
        cv2.waitKey(3)




def main(args):
    print "main started"

    ic = image_converter()

    print "initialize ..."
    rospy.init_node('image_converter', anonymous=True)
    print "init complete"

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
