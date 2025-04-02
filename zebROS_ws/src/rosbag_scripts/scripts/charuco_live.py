#!/usr/bin/env python3

import cv2 as cv
import numpy as np
import rospy

class CharucoCalibrator:

    def __init__(
        self,
        chessboard_size=(5,4),
        frame_size_h=1300,
        frame_size_w=1600,
        square_mm=120,
        marker_mm=90,
        aruco_dict=cv.aruco.DICT_4X4_250,
    ):
        self.chessboard_size = chessboard_size
        self.frame_size_h = frame_size_h
        self.frame_size_w = frame_size_w
        self.square_mm = square_mm
        self.marker_mm = marker_mm
        self.aruco_dict = aruco_dict


        self.predefined_dict = cv.aruco.getPredefinedDictionary(self.aruco_dict)

        self.board = cv.aruco.CharucoBoard(
            self.chessboard_size, self.square_mm, self.marker_mm, self.predefined_dict
        )
        self.board.setLegacyPattern(True)

        # Create detector
        self.detector = cv.aruco.CharucoDetector(self.board)
        self.arucoparams = cv.aruco.DetectorParameters()
        self.arucoparams.cornerRefinementMethod = cv.aruco.CORNER_REFINE_SUBPIX
        self.detector.setDetectorParameters(self.arucoparams)

        self.coords = []

    def plot_coords(self):
        img = np.zeros((self.frame_size_h, self.frame_size_w, 3), dtype=np.uint8)

        for c in self.coords:
            cv.circle(img, (int(c[0]), int(c[1])), 5, (0, 0, 255), -1)
        
        cv.imshow("Detections", img)
        cv.waitKey(1)

    def callback(self, img):
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        # Detect Charuco board
        charuco_corners, charuco_ids, _, _ = self.detector.detectBoard(gray)
        if charuco_ids is not None:
            cv.aruco.drawDetectedCornersCharuco(
                    img, charuco_corners, charuco_ids
                )
            if len(charuco_corners) >= 6:
                for c in charuco_corners:
                    self.coords.append(c[0])


        # cv.imshow("Charuco Detection", img)
        # cv.waitKey(1)

if __name__ == "__main__":
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge

    rospy.init_node("charuco_live")
    bridge = CvBridge()
    calibrator = CharucoCalibrator()

    def image_callback(msg):
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        calibrator.callback(img)

    sub = rospy.Subscriber("/ov2311_10_9_0_10_video1/image_raw", Image, image_callback)


    while not rospy.is_shutdown():
        calibrator.plot_coords()
        rospy.sleep(2)
