#!/usr/bin/env python3
import string
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ImageShow:
    def __init__(self, image_topic: string) -> None:
        self.cv_bridge = CvBridge()
        rospy.init_node("image_show", anonymous=True)

        self.image_sub = rospy.Subscriber(image_topic, Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(data, "mono8")
        except CvBridgeError as e:
            print(e)

        (rows, cols) = cv_image.shape
        if cols > 60 and rows > 60:
            cv2.circle(cv_image, (50, 50), 10, 255)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)


if __name__ == "__main__":
    topic = "/ping360_node/sonar/images"
    img_show = ImageShow(topic)
    rospy.spin()
