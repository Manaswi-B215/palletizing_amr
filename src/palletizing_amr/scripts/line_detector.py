#!/usr/bin/env python3

"""
line_detector.py

ROS2 Jazzy Line Detection Node
Author: ChatGPT (Customized for Palletizing AMR)

Subscribes:
    /camera/image_raw

Publishes:
    /line/error   (Float32)

Description:
    Detects the coloured line using HSV thresholding,
    computes the centroid of the largest contour and
    publishes the horizontal error from the image centre.
"""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from rclpy.qos import qos_profile_sensor_data


class LineDetector(Node):

    def __init__(self):

        super().__init__("line_detector")

        ####################################################
        # Parameters
        ####################################################

        self.declare_parameter("crop_ratio", 0.35)

        self.declare_parameter("min_contour_area", 500)

        # HSV lower values
        self.declare_parameter("h_low", 0)
        self.declare_parameter("s_low", 0)
        self.declare_parameter("v_low", 0)

        # HSV upper values
        self.declare_parameter("h_high", 180)
        self.declare_parameter("s_high", 255)
        self.declare_parameter("v_high", 60)

        ####################################################
        # Read parameters
        ####################################################

        self.crop_ratio = self.get_parameter(
            "crop_ratio").value

        self.min_area = self.get_parameter(
            "min_contour_area").value

        self.lower = np.array([

            self.get_parameter("h_low").value,
            self.get_parameter("s_low").value,
            self.get_parameter("v_low").value

        ])

        self.upper = np.array([

            self.get_parameter("h_high").value,
            self.get_parameter("s_high").value,
            self.get_parameter("v_high").value

        ])

        ####################################################
        # ROS
        ####################################################

        self.bridge = CvBridge()

        self.error_pub = self.create_publisher(
            Float32,
            "/line/error",
            10
        )

        self.image_sub = self.create_subscription(

            Image,

            "/camera/image_raw",

            self.image_callback,

            qos_profile_sensor_data

        )

        ####################################################
        # Variables
        ####################################################

        self.kernel = np.ones((5, 5), np.uint8)

        self.get_logger().info("Line Detector Started")

    ########################################################

    def image_callback(self, msg):

        image = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="bgr8"
        )

        height, width, _ = image.shape

        crop_start = int(
            height * (1 - self.crop_ratio)
        )

        crop = image[crop_start:, :]

        ####################################################
        # HSV
        ####################################################

        hsv = cv2.cvtColor(
            crop,
            cv2.COLOR_BGR2HSV
        )

        ####################################################
        # Threshold
        ####################################################

        mask = cv2.inRange(
            hsv,
            self.lower,
            self.upper
        )

        ####################################################
        # Morphology
        ####################################################

        mask = cv2.morphologyEx(

            mask,

            cv2.MORPH_OPEN,

            self.kernel

        )

        mask = cv2.morphologyEx(

            mask,

            cv2.MORPH_CLOSE,

            self.kernel

        )

        ####################################################
        # Find contours
        ####################################################

        contours, _ = cv2.findContours(

            mask,

            cv2.RETR_EXTERNAL,

            cv2.CHAIN_APPROX_SIMPLE

        )

        ####################################################
        # No contour
        ####################################################

        if len(contours) == 0:

            cv2.imshow("Mask", mask)
            cv2.imshow("Line Detector", crop)
            cv2.waitKey(1)

            return

        ####################################################
        # Largest contour
        ####################################################

        largest = max(
            contours,
            key=cv2.contourArea
        )

        area = cv2.contourArea(largest)

        if area < self.min_area:

            cv2.imshow("Mask", mask)
            cv2.imshow("Line Detector", crop)
            cv2.waitKey(1)

            return

        ####################################################
        # Centroid
        ####################################################

        M = cv2.moments(largest)

        if M["m00"] == 0:
            return

        cx = int(
            M["m10"] / M["m00"]
        )

        cy = int(
            M["m01"] / M["m00"]
        )

        ####################################################
        # Error Calculation
        ####################################################

        image_center = width // 2

        error = float(cx - image_center)

        ####################################################
        # Publish Error
        ####################################################

        error_msg = Float32()
        error_msg.data = error

        self.error_pub.publish(error_msg)

        ####################################################
        # Draw Debug Information
        ####################################################

        cv2.drawContours(
            crop,
            [largest],
            -1,
            (255, 0, 0),
            2
        )

        cv2.circle(
            crop,
            (cx, cy),
            8,
            (0, 255, 0),
            -1
        )

        cv2.line(
            crop,
            (image_center, 0),
            (image_center, crop.shape[0]),
            (0, 0, 255),
            2
        )

        cv2.putText(
            crop,
            f"Error : {error:.1f}",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 0),
            2
        )

        cv2.putText(
            crop,
            f"Area : {area:.0f}",
            (20, 75),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 0),
            2
        )

        ####################################################
        # Display
        ####################################################

        cv2.imshow("Mask", mask)
        cv2.imshow("Line Detector", crop)

        cv2.waitKey(1)


############################################################


def main(args=None):

    rclpy.init(args=args)

    node = LineDetector()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        cv2.destroyAllWindows()

        node.destroy_node()

        rclpy.shutdown()


############################################################

if __name__ == "__main__":

    main()
