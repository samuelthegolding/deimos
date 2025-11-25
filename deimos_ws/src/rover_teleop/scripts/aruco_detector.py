#!/usr/bin/env python3

import cv2
import cv2.aruco as aruco
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int32MultiArray  # To publish detected marker IDs

class ArucoDetector:
    def __init__(self):
        rospy.init_node("aruco_detector", anonymous=True)

        # Initialize OpenCV bridge
        self.bridge = CvBridge()

        # Define ArUco dictionary and parameters
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
        self.aruco_params = aruco.DetectorParameters()

        # Publishers
        self.image_pub = rospy.Publisher("/aruco_detected", Image, queue_size=10)
        self.id_pub = rospy.Publisher("/aruco_ids", Int32MultiArray, queue_size=10)

        # Subscriber to image_raw topic
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.image_callback)

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")
            return

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        # Draw markers
        if ids is not None:
            frame = aruco.drawDetectedMarkers(frame, corners, ids)

            # Publish detected IDs
            id_msg = Int32MultiArray()
            id_msg.data = ids.flatten().tolist()
            self.id_pub.publish(id_msg)

        # Publish the processed image
        try:
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.image_pub.publish(image_msg)
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")

        # Display for debugging
        cv2.imshow("ArUco Detection", frame)
        cv2.waitKey(1)

if __name__ == "__main__":
    rospy.loginfo("Starting ArUco detector node...")
    detector = ArucoDetector()
    print("Detector running...")
    rospy.spin()  # Keep the node running
