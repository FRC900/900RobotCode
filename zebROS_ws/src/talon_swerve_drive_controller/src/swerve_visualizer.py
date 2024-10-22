#!/usr/bin/env python3

import cv2
from math import pi
import numpy as np
import rospy

from talon_state_msgs.msg import TalonFXProState

class SwerveVisualizer:
    speeds = []
    angles = []
    WINDOW_SIZE = 500
    WHEEL_SIZE = 50

    def __init__(self) -> None:
        self.speed_names = rospy.get_param("swerve_drive_controller/speed")
        self.steering_names = rospy.get_param("swerve_drive_controller/steering")
        self.wheel_coords = rospy.get_param("swerve_drive_controller/wheel_coords")
        self.max_magnitude = max(max([[abs(y) for y in x] for x in self.wheel_coords])) * 3. / 2.
        self.wheel_coords = [[self.WINDOW_SIZE / 2. + (-pt / self.max_magnitude) * (self.WINDOW_SIZE / 2.) for pt in reversed(coord)] for coord in self.wheel_coords]
        
        max_speed = rospy.get_param("/teleop/teleop_params/max_speed")
        wheel_radius = rospy.get_param("swerve_drive_controller/wheel_radius")
        ratio_encoder_to_rotations = rospy.get_param("swerve_drive_controller/ratio_encoder_to_rotations")

        self.speed_scale = wheel_radius * ratio_encoder_to_rotations / max_speed

        self.talon_states_sub = rospy.Subscriber('talonfxpro_states', TalonFXProState, self.callback, tcp_nodelay=True, queue_size=1) # subscribing here so that we can figure out the actual speed of said motor at a given time, talonfxpro_states gives us these values

    def callback(self, msg: TalonFXProState) -> None:
        self.speeds = [None] * len(self.speed_names)
        self.angles = [None] * len(self.steering_names)
        for i in range(len(self.speed_names)):
            for j in range(len(msg.name)):
                if msg.name[j] == self.speed_names[i]:
                    self.speeds[i] = msg.velocity[j]
                if msg.name[j] == self.steering_names[i]:
                    self.angles[i] = msg.position[j]

    # Given length of a line segment starting at the origin, find the endpoint of the line segment
    # after rotating it by angle
    def get_line_endpoints(self, length: float, angle: float):

        c = np.cos(angle)
        s = np.sin(angle)
        R = np.matrix([[c, -s],
                       [s,  c]])

        C = np.matrix([[0.0, length]]).T

        D = R * C

        # Rotate because screen display coords are off by 90
        # And positive x&y are in opposite directions
        return D[0, 0], -D[1, 0]


    def update(self) -> None:
        if len(self.speeds) == 0 or len(self.angles) == 0:
            return
        img = np.zeros((self.WINDOW_SIZE, self.WINDOW_SIZE, 3), np.uint8)
        for c, a, s, n in zip(self.wheel_coords, self.angles, self.speeds, self.steering_names):
            end_x, end_y = self.get_line_endpoints(self.WHEEL_SIZE, a)
            cv2.line(img, (int(c[0] + end_x), int(c[1] + end_y)), (int(c[0] - end_x), int(c[1] - end_y)), (255, 255, 0), 15)
            cv2.putText(img, n, (int(c[0] - self.WINDOW_SIZE/10.), int(c[1] - 1.2 * abs(end_y))), cv2.FONT_HERSHEY_SIMPLEX, .75, (255, 255, 255), 2)
            if abs(s) > 0. :
                end_x, end_y = self.get_line_endpoints(self.WHEEL_SIZE * s * self.speed_scale, a)
                cv2.arrowedLine(img, (int(c[0]), int(c[1])), (int(c[0] + end_x), int(c[1] + end_y)), (0, 0, 255), 5, tipLength=0.5)
        cv2.imshow("Swerve Drive", img)
        cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('swerve_visualizer')

    visualizer = SwerveVisualizer()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        visualizer.update()
        r.sleep()

    rospy.spin()