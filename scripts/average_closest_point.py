#!/usr/bin/python
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nord_messages.msg import RelativePoint

class image_converter:
    def __init__(self):
        #cv2.namedWindow("Image window", 1)
        #cv2.namedWindow("Depth Image window", 1)
        self.bridge = CvBridge()
        #self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)
        self.depth_image_sub = rospy.Subscriber("/camera/depth/image_raw",Image,self.callbackDepth)
	self.pub = rospy.Publisher('/nord/img/closest', RelativePoint, queue_size=100)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e
          
        (rows,cols,channels) = cv_image.shape

        #cv2.imshow("Image window", cv_image)
        #cv2.waitKey(3)

    def callbackDepth(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
        except CvBridgeError, e:
            print e
          
        (rows,cols,channels) = cv_image.shape

        # Move undesired points to infinity and smooth
        filter_size = 21
        cv_image[cv_image < 300] = 60001
        cv_image_blur = cv2.GaussianBlur(cv_image,(filter_size,filter_size),0)
        
        # The average closest pixel
        dist=np.min(cv_image_blur)/1000.0
        indxarr=np.argwhere(cv_image_blur == np.min(cv_image_blur))
        avg=np.average(indxarr, axis = 0)
#        print np.min(cv_image_blur)
        #  print indxarr

        # calculate the angle relative to the center of the camera
        angle_to_min = (((avg[1]-320.0)/320.0) * 57.5/2.0 * np.pi/180.0)

        # prepair the message
        msgs=RelativePoint()
        msgs.angle=angle_to_min
        msgs.distance=dist
        self.pub.publish(msgs)

        # uncomment for visualization        
#        cv2.circle(cv_image_blur, (int(avg[1]),int(avg[0])), 10, 6000, thickness=1, lineType=8)
#        cv2.imshow("Depth Image window", cv_image_blur*10)
#        cv2.waitKey(3)
	



def main(args):
    #print "main started"


    #print "initialize ..."
    rospy.init_node('image_converter', anonymous=True)
    #print "init complete"
    ic = image_converter()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    #cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
