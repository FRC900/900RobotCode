# base class of sim balance, shared code between the cli and sim version


from collections import namedtuple
from enum import Enum
import cv2 # for visualizing
import numpy as np
import time
import math
import rospy
import std_msgs.msg
import rosgraph_msgs.msg

class States(Enum):
    OFF_LEFT = 1
    ON_RAMP_LEFT_1_WHEEL = 3
    ON_MIDDLE_LEFT_1_WHEEL = 4
    ON_LEFT_2_WHEEL_1_RAMP = 5
    
    ON_MIDDLE_2_WHEEL = 6
    
    ON_RIGHT_2_WHEEL_1_RAMP = 7
    ON_MIDDLE_RIGHT_1_WHEEL = 8
    ON_RAMP_RIGHT_1_WHEEL = 9
    OFF_RIGHT = 2

BLACK = (0,0,0)
WHITE = (255,255,255)
IMAGE_X = 1280
IMAGE_Y = 720
MAX_ACCEL = 1
FLOOR_OFFSET = 200 # pixels for visualizing
# meters to pixels
METER_TO_PIXEL = 300 # pixels per meter, very arbitrary

STATION_OFFSET = 1.5 # meters
STATION_OFFSET_PIXEL = int(STATION_OFFSET * METER_TO_PIXEL)
WHEEL_RADIUS = 0.05 # meters
# distance between center of wheel to center of other wheel
WHEEL_TO_WHEEL = 0.3302 * 2

# flat part of the charging station
MIDDLE_LENGHT = 1.2192 

# lenght of a single ramp part the robot would drive up
RAMP_LEN_X = 0.312285 # not the hypot of the ramp, just the lenght of the ramp X
DIAG_LENGHT = 0.377800 # meters, hypot of the ramp

# how high the station is when the robot is not on it
BALANCED_HEIGHT = 0.231775 # meters
# angle that the ramps are at when the station is perfectly balance 
BALANCED_ANGLE = 0.59777527 # rad
# lowest angle the charging station gets to when a robot is driven on it
LOWEST_ANGLE = 0.191986 # radians, 11 degrees
LOWEST_MIDDLE = BALANCED_HEIGHT * 2 - 0.4064
# how high the highest point of the station is when a robot is driven on it
# occurs at same time as LOWEST_ANGLE
MAX_HEIGHT = 0.4064 # meters 
FUDGE_FACTOR = 0.01 # meters
FLAT_X_LEFT = STATION_OFFSET + RAMP_LEN_X
FLAT_X_RIGHT = STATION_OFFSET + RAMP_LEN_X + MIDDLE_LENGHT
FLAT_X_LEFT_PIXEL = int(FLAT_X_LEFT * METER_TO_PIXEL)
FLAT_X_RIGHT_PIXEL = int(FLAT_X_RIGHT * METER_TO_PIXEL)
TIME_STEP = 0.01
ROBOT_MASS = 45.35 # kg
LEVER_2_LEVER = 0.2794 # distance between the two points that the charging station can rotate in M
END_2_LEVER = 0.8027 # distance between the end of the station to one of the rotating points in M
LEVER_L_X = STATION_OFFSET + END_2_LEVER
LEVER_R_X = STATION_OFFSET + END_2_LEVER + LEVER_2_LEVER
class Point:
    def __init__(self, x=0, y=0) -> None:
        self.x = x
        self.y = y

    def __repr__(self) -> str:
        return f"Point({self.x}, {self.y})"

class ChargingStationSim:
    def __init__(self) -> None:
        self.previous_vel = 0
        self.center_X = None
        self.center_Y = None
        self.left_wheel = {}
        self.left_wheel["x"] = 0.7
        self.left_wheel["y"] = 0
        self.right_wheel = {}
        self.right_wheel["x"] = self.left_wheel["x"] + WHEEL_TO_WHEEL
        self.right_wheel["y"] = 0
        self.angle = 0
        self.theta = -999
        self.state = States.OFF_LEFT
        self.height = 0
        self.time = 0
        self.prev_robot_vel = 0
        self.left_ramp_start_XY = (STATION_OFFSET, FUDGE_FACTOR)
        self.left_ramp_end_XY = (STATION_OFFSET + RAMP_LEN_X, BALANCED_HEIGHT)
        self.right_ramp_start_XY = (STATION_OFFSET + RAMP_LEN_X*2+MIDDLE_LENGHT, FUDGE_FACTOR)
        self.right_ramp_end_XY = (STATION_OFFSET+RAMP_LEN_X+MIDDLE_LENGHT, BALANCED_HEIGHT)
        self.center_line_angle_rad = -999
        self.center_line_accels = [] # evenly spaced by TIME_STEP accelerations, should be able to integrate to get velocites
        self.center_line_left_XY = (STATION_OFFSET + RAMP_LEN_X, BALANCED_HEIGHT)
        self.center_line_right_XY = (STATION_OFFSET + RAMP_LEN_X+MIDDLE_LENGHT, BALANCED_HEIGHT)
        self.noise_angle = -999
        self.x_cmd_vel = 0
        #self.df = pd.DataFrame(columns=["force", "torque", "inertia", "angular_accel", "velocity", "radius", "angle"])

    def round_dict(self, d):
        new_d = {}
        for key, value in d.items():
            new_d[key] = round(value, 3)
        return new_d
    
    def visualize_lines(self, img):
        # draw left ramp using the two points provided by self.left_ramp_start_XY and self.left_ramp_end_XY
        img = cv2.line(img, (int(self.left_ramp_start_XY[0]*METER_TO_PIXEL), IMAGE_Y-FLOOR_OFFSET-int(self.left_ramp_start_XY[1]*METER_TO_PIXEL)), (int(self.left_ramp_end_XY[0]*METER_TO_PIXEL), IMAGE_Y-FLOOR_OFFSET-int(self.left_ramp_end_XY[1]*METER_TO_PIXEL)), (0,165,255), 3)
        # same for right ramp
        img = cv2.line(img, (int(self.right_ramp_start_XY[0]*METER_TO_PIXEL), IMAGE_Y-FLOOR_OFFSET-int(self.right_ramp_start_XY[1]*METER_TO_PIXEL)), (int(self.right_ramp_end_XY[0]*METER_TO_PIXEL), IMAGE_Y-FLOOR_OFFSET-int(self.right_ramp_end_XY[1]*METER_TO_PIXEL)), (0,165,255), 3)
        # draw center line
        img = cv2.line(img, (int(self.center_line_left_XY[0]*METER_TO_PIXEL), IMAGE_Y-FLOOR_OFFSET-int(self.center_line_left_XY[1]*METER_TO_PIXEL)), (int(self.center_line_right_XY[0]*METER_TO_PIXEL), IMAGE_Y-FLOOR_OFFSET-int(self.center_line_right_XY[1]*METER_TO_PIXEL)), (255,128,0), 3)
        return img

    def visualize(self):
        # make a blank image
        img = np.zeros((IMAGE_Y,IMAGE_X,3), dtype=np.uint8)
        img.fill(255)
        ###print(img)
        # draw the floor as a line
        img = cv2.line(img, (0, IMAGE_Y-FLOOR_OFFSET), (IMAGE_X, IMAGE_Y-FLOOR_OFFSET), (0,255,0), 3)
        img = cv2.rectangle(img, (0, IMAGE_Y-FLOOR_OFFSET), (IMAGE_X, IMAGE_Y), (0,255,0), -1)
        
        # add text of the state and angle of the robot and position of the wheels
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(img, f"State: {self.state}", (10, 50), font, 1, BLACK, 2, cv2.LINE_AA)
        cv2.putText(img, f"Time (s): {round(self.time, 4)}", (800, 50), font, 1, BLACK, 2, cv2.LINE_AA)
        cv2.putText(img, f"Angle: {round(np.rad2deg(self.angle), 3) * -1} Fake angle {round(np.rad2deg(self.noise_angle), 3)}", (10, 100), font, 1, BLACK, 2, cv2.LINE_AA)
        cv2.putText(img, f"Left Wheel: {self.round_dict(self.left_wheel)}, Right Wheel: {self.round_dict(self.right_wheel)}", (10, 150), font, 1, BLACK, 2, cv2.LINE_AA)
        cv2.putText(img, f"Given x_cmd_vel {round(self.x_cmd_vel, 3)}", (10, 200), font, 1, BLACK, 2, cv2.LINE_AA)
        # draw vertical lines every meter for reference
        for i in range(0, IMAGE_X, METER_TO_PIXEL):
            img = cv2.line(img, (i, IMAGE_Y-FLOOR_OFFSET), (i, IMAGE_Y), (0,0,255), 3)

        # draw the robot
        # draw the left wheel
        left_wheel_x = int(self.left_wheel["x"] * METER_TO_PIXEL)
        left_wheel_y = int(self.left_wheel["y"] * METER_TO_PIXEL)
        ##print(left_wheel_x, left_wheel_y)
        WHEEL_RADIUS_PIXEL = int(WHEEL_RADIUS * METER_TO_PIXEL)
        ##print(f"Drawing circle at {left_wheel_x}, {IMAGE_Y - left_wheel_y - FLOOR_OFFSET}")
        img = cv2.circle(img, (left_wheel_x, IMAGE_Y - left_wheel_y - FLOOR_OFFSET - WHEEL_RADIUS_PIXEL), WHEEL_RADIUS_PIXEL, (255,0,0), -1)

        # draw the right wheel
        right_wheel_x = int(self.right_wheel["x"] * METER_TO_PIXEL)
        right_wheel_y = int(self.right_wheel["y"] * METER_TO_PIXEL)
        img = cv2.circle(img, (right_wheel_x, IMAGE_Y - right_wheel_y - FLOOR_OFFSET - WHEEL_RADIUS_PIXEL), WHEEL_RADIUS_PIXEL, (255,0,0), -1)
        
        # draw the line between the wheels as the robot
        img = cv2.line(img, (left_wheel_x, IMAGE_Y - left_wheel_y - FLOOR_OFFSET - WHEEL_RADIUS_PIXEL), (right_wheel_x, IMAGE_Y - right_wheel_y - FLOOR_OFFSET - WHEEL_RADIUS_PIXEL), (255,0,0), 3)
        
        # draw line at X of LEFT_MIDDLE
        #img = cv2.line(img, (0, IMAGE_Y - FLOOR_OFFSET - int(LOWEST_MIDDLE*METER_TO_PIXEL)), (IMAGE_X, IMAGE_Y - FLOOR_OFFSET-int(LOWEST_MIDDLE*METER_TO_PIXEL)), (0,0,255), 3)
        # draw the charging station
        # draw the base as a line
        #img = self.draw_rotated_ramp_left(img)
        img = self.visualize_lines(img)
        # draw a point at center_X and center_Y
        if self.center_X is not None and self.center_Y is not None:
            middle_X = (self.center_line_right_XY[0] + self.center_line_left_XY[0]) / 2
            middle_Y = (self.center_line_right_XY[1] + self.center_line_left_XY[1]) / 2
            #img = cv2.line(img, (int(middle_X*METER_TO_PIXEL), 0), (int(middle_X*METER_TO_PIXEL), IMAGE_Y), (0,0,255), 3)
            #img = cv2.line(img, (0, IMAGE_Y -FLOOR_OFFSET - int(middle_Y*METER_TO_PIXEL)), (IMAGE_X, IMAGE_Y - FLOOR_OFFSET- int(middle_Y*METER_TO_PIXEL)), (0,0,255), 3)
        cv2.imshow("Test", img)
        cv2.waitKey(10)
        #_ = input("Press [enter] to continue.")

    def get_pitch(self,):
        return self.angle

    def off_left(self, velocity, timestep):
        ''' 
        v / R = rotations/second
        circumference of wheel = 2 * pi * R
        x1_new = rotations/second * dT * circumference
        y1_new = 0
        '''
        ##print("off left")

        self.linear_move(velocity, timestep)
        x_diff = self.left_ramp_start_XY[0] - self.right_wheel["x"]
         
        height = WHEEL_RADIUS
        hypot = np.sqrt(x_diff**2 + height**2)
        ##print(f"hypot: {hypot} height: {height}, x_diff: {x_diff}, wheel radius: {WHEEL_RADIUS}")
        if self.right_wheel["x"] >= STATION_OFFSET - FUDGE_FACTOR * 3 and self.state == States.OFF_LEFT:
            self.state = States.ON_RAMP_LEFT_1_WHEEL
    
    def on_ramp_left_2_wheels(self, velocity, timestep):
        # on flat plane but roatated, still easy
        # we know angle of ramp and velocity (hypot)
        RAMP_LEN_Y = self.left_ramp_end_XY[1]
        # calculate ramp angle based on the two points of the ramp
        RAMP_ANGLE = (np.arctan((RAMP_LEN_Y) / (RAMP_LEN_X)))
        #print(f"Wanted velocity {velocity}")
        velocity = self.clamp(self.prev_robot_vel-MAX_ACCEL*timestep, self.prev_robot_vel+MAX_ACCEL*timestep, velocity)
        #print(f"Actual velocity {velocity}")
        self.prev_robot_vel = velocity
        y_diff = velocity * timestep * math.sin(RAMP_ANGLE) 
        x_diff = velocity * timestep * math.cos(RAMP_ANGLE)
        
        self.left_wheel["y"] += y_diff
        self.right_wheel["y"] += y_diff
        self.left_wheel["x"] += x_diff
        self.right_wheel["x"] += x_diff
        if self.left_wheel["x"] >= FLAT_X_LEFT and self.state == States.ON_LEFT_2_WHEEL_1_RAMP:
            #print("UPDATED STATE TO ON_LEFT_2_WHEEL_1_RAMP---------------")
            self.state = States.ON_MIDDLE_2_WHEEL
            

    def calc_ramp_angle(self):
        RAMP_LEN_Y = self.left_ramp_end_XY[1]
        # calculate ramp angle based on the two points of the ramp
        RAMP_ANGLE = (np.arctan((RAMP_LEN_Y) / (RAMP_LEN_X)))
        return RAMP_ANGLE

    def calc_center_angle(self):
        middle_x_dist = self.center_line_right_XY[0] - self.center_line_left_XY[0]
        middle_y_dist = self.center_line_right_XY[1] - self.center_line_left_XY[1]
        # calculate ramp angle based on the two points of the ramp
        RAMP_ANGLE = (np.arctan((middle_y_dist) / (middle_x_dist)))
        return RAMP_ANGLE

    def slip(self, velocity, timestep):
        RAMP_ANGLE = self.calc_center_angle()
        ##print(f"RAMP Angle {RAMP_ANGLE}")
        # move the robot back depending on the angle of it
        y_diff = -0.1 * velocity * timestep * math.sin(RAMP_ANGLE) 
        x_diff = -0.1 * velocity * timestep * math.cos(RAMP_ANGLE)
        ##print(f"x_diff {x_diff}, y_diff {y_diff}")
        # apply slip 
        self.right_wheel["x"] += x_diff
        self.right_wheel["y"] += y_diff
        self.left_wheel["x"] += x_diff
        self.left_wheel["y"] += y_diff

    # get center of robot
    def get_center_x(self):
        center_x = (self.left_wheel["x"] + self.right_wheel["x"]) / 2
        return center_x
    
    # get center of ramp
    def get_center_ramp_x(self):
        center_x = (self.left_ramp_start_XY[0] + self.right_ramp_start_XY[0]) / 2
        return center_x

    def update_ramp(self, timestep):
        '''
        I want to get angular velocity.
        
        torque=force*r*sin(θ)
        let only force be gravity on our robot
        force should be constant then because the ramp will not snap
        force = 9.8 * 45.3592kg (100 lbs)
        theta = whatever the angle of ramp currently is, we keep track of that
        r is whichever of the two levers we are closer two, find that

        torque/inertia = angular accel
        just need to find inertia and then integrate for velocity
        I=mr^2
        I=45.3592*r^2 where R is from before... I think?

        we now have angular accel and timesteps integrate for velocity and appliy it * dT
        '''
        if self.center_line_angle_rad == -999:
            self.center_line_angle_rad = self.angle
            ##print(f"center line angle {self.center_line_angle_rad} just set")
            #time.sleep(1)
        force = ROBOT_MASS * 9.8
        c_X = self.get_center_x()
        radius = -(c_X - self.get_center_ramp_x())

        #radius = abs(c_X - self.get_center_ramp_x())
        ##print(f"Radius {radius}, self.right_wheel['x'] {self.right_wheel['x']}, c_X {c_X}")
        #_ = input()
        torque = force * radius * abs(math.sin(self.angle)) # ramp angle before update is also just robot angle
        # basically multiplying by angle because sin of small x = x 
        inertia = ROBOT_MASS * (radius ** 2)
        #inertia = 1
        angular_accel = torque/inertia
    
        #print(f"Desired AA {angular_accel}\nTorque {torque}\nRadius {radius}\nsin {math.sin(self.angle)}\nAngle {self.angle}")
        #angular_accel = self.clamp(-1, 1, angular_accel)
        if -0.05 <= radius <= 0.05:
            #print("Applying 'to center' force")
            #print(f"Previous final vel = {self.previous_vel + angular_accel * timestep }")
            ##print(f"Adding {2 * self.angle * radius}")
            
            final_vel = -100 * self.angle * abs(radius)
            #self.previous_vel = final_vel

            #print(f"New final vel = {final_vel}")
        else:
            # apply angular accel to previous velocity
            final_vel = self.previous_vel + angular_accel * timestep
            #print(f"Final Vel {final_vel} ")
            self.previous_vel = final_vel
        '''
        self.center_line_accels.append(angular_accel)
        # make dataFrame with force, torque, inertia, angular accel, and velocity, radius and angle
        #self.df = pd.DataFrame(columns=["force", "torque", "inertia", "angular_accel", "velocity", "radius", "angle"])
        #acceleration = [1, 2, 3, 4, 5, 6, 7, 8, 9, -1, -2, -3, -4, -5, -6, -7, -8, -9]
        # TODO, when the ramp is fully extended, one way or the other, it does not accelerate anymore, so clear the list
        velocitylist = [0]
        for acc in self.center_line_accels:
            velocitylist.append(velocitylist[-1] + acc * TIME_STEP)
        final_vel = velocitylist[-1]
        '''
        #print(f"Time {self.time}")
        #self.df.loc[len(self.df)] = [force, torque, inertia, angular_accel, final_vel, radius, self.angle]
        #self.fig, ax = render_mpl_table(self.df)
        #self.fig.savefig("table.png")
        ###print(f"Velocity list {velocitylist}")
        ##print(f"Angular accel {angular_accel} final vel {final_vel * timestep}")
        #time.sleep(1)
        center_pt = self.get_center_ramp_x()
        self.update_center_with_angle(final_vel, center_pt, timestep)


    def clamp(self, low, high, n):
        if n < low: 
            #self.center_line_accels = []
            return low
        if n > high: 
            #self.center_line_accels = []
            return high
        return n
    
    def update_center_with_angle(self, angular_vel, center_point, timestep):
        if (self.center_line_left_XY[1] <= LOWEST_MIDDLE) and angular_vel > 0:
            ##print("Ramp is too low with angular vel ", angular_vel)
            self.center_line_accels = []
            #time.sleep(1)
            return
        
        if (self.center_line_right_XY[1] <= LOWEST_MIDDLE) and angular_vel < 0:
            ##print("Ramp is too low with angular vel ", angular_vel)
            self.center_line_accels = []
            #time.sleep(1)
            return
        
        ###print("Starting left = ", self.center_line_left_XY)
        ###print("Starting right = ", self.center_line_right_XY)
        ###print(f"Center point {center_point}")
        self.center_line_angle_rad += timestep * angular_vel
        if self.center_line_angle_rad < -LOWEST_ANGLE: 
            self.center_line_angle_rad = -LOWEST_ANGLE
            self.previous_vel = 0
        if self.center_line_angle_rad > LOWEST_ANGLE: 
            self.center_line_angle_rad = LOWEST_ANGLE
            self.previous_vel = 0
        
        self.angle = self.center_line_angle_rad
        # find x and y from angle and hypot which is half the length of the ramp
        self.center_line_left_XY = [center_point - (MIDDLE_LENGHT / 2) * math.cos(self.center_line_angle_rad), (MIDDLE_LENGHT / 2) * -math.sin(self.center_line_angle_rad) + BALANCED_HEIGHT]
        self.center_line_right_XY = [center_point + (MIDDLE_LENGHT / 2) * math.cos(self.center_line_angle_rad), (MIDDLE_LENGHT / 2) * math.sin(self.center_line_angle_rad) + BALANCED_HEIGHT]
        # move the wheels to be on the ramp use same math as before
        # find new radius for the wheels, use pythagorean theorem
        ###print(f"self.center_line_left_XY {self.center_line_left_XY} self.center_line_right_XY {self.center_line_right_XY}")
        self.center_X = abs(self.center_line_left_XY[0] + self.center_line_right_XY[0]) / 2
        self.center_Y = abs(self.center_line_left_XY[1] + self.center_line_right_XY[1]) / 2
        r_left_wheel = math.sqrt((self.center_X - self.left_wheel["x"]) ** 2 + (self.center_Y - self.left_wheel['y']) ** 2)
        r_right_wheel = math.sqrt((self.center_X - self.right_wheel['x']) ** 2 + (self.center_Y - self.right_wheel['y']) ** 2)
        # find new position for the wheels
        self.center_line_left_XY = [center_point - (MIDDLE_LENGHT / 2) * math.cos(self.center_line_angle_rad), (MIDDLE_LENGHT / 2) * -math.sin(self.center_line_angle_rad) + BALANCED_HEIGHT]
        self.center_line_right_XY = [center_point + (MIDDLE_LENGHT / 2) * math.cos(self.center_line_angle_rad), (MIDDLE_LENGHT / 2) * math.sin(self.center_line_angle_rad) + BALANCED_HEIGHT]
        if self.left_wheel["x"] < self.get_center_ramp_x():
            self.left_wheel['x'] = center_point - r_left_wheel * math.cos(self.center_line_angle_rad)
        else:
            self.left_wheel['x'] = center_point + r_left_wheel * math.cos(self.center_line_angle_rad)

        if self.right_wheel["x"] < self.get_center_ramp_x():
            self.right_wheel['x'] = center_point - r_right_wheel * math.cos(self.center_line_angle_rad)
        else:
            self.right_wheel['x'] = center_point + r_right_wheel * math.cos(self.center_line_angle_rad)
            
        self.left_wheel['y'] = r_left_wheel * -math.sin(self.center_line_angle_rad) + BALANCED_HEIGHT
        self.right_wheel['y'] = r_right_wheel * math.sin(self.center_line_angle_rad) + BALANCED_HEIGHT
        ##print(f"r_left_wheel {r_left_wheel} r_right_wheel {r_right_wheel}")
        ##print(f"Center X {self.center_X} Center Y {self.center_Y}")
        ##print("Angle in deg", self.center_line_angle_rad * 180 / np.pi)

       # _ = input("Press enter to continue")
        
        # assert if the hypot is not the same as the length of the ramp


        #time.sleep(2)

    def on_middle_2_wheels(self, velocity, timestep):
        self.slip(velocity, timestep)
        self.update_ramp(timestep)
        self.on_ramp_left_2_wheels(velocity, timestep)
        

    def on_middle_left_1_wheel(self, velocity, timestep):
        ##print("on ramp left 1 wheel")
        # move the right wheel forward
        rotations = velocity / (WHEEL_RADIUS * 2 * np.pi) # i am still not sure about this 2 * pi, because it simplifies to velocity*timestep * sin/cos(RAMP_ANGLE)
        circumference = 2 * np.pi * WHEEL_RADIUS          # it looks fine though, and for a flat ramp it is correct as it simplifies to velocity * timestep
        hypot = rotations * circumference * timestep
        # ramp len y using pythagorean theorem
        
        RAMP_LEN_Y = self.left_ramp_end_XY[1]
        # calculate ramp angle based on the two points of the ramp
        RAMP_ANGLE = (np.arctan((RAMP_LEN_Y) / (RAMP_LEN_X)))
        #RAMP_ANGLE = np.deg2rad(34.5)
        ##print(f"RAMP_ANGLE DEG = {np.rad2deg(RAMP_ANGLE)}")
        #RAMP_ANGLE =
        y_diff = hypot * np.sin(RAMP_ANGLE)
        x_diff = hypot * np.cos(RAMP_ANGLE)
        self.right_wheel["x"] += x_diff
        self.right_wheel["y"] += y_diff
        ##print(f"X diff = {x_diff} Y diff = {y_diff}, hypot = {hypot}, sin = {np.sin(RAMP_ANGLE)}, cos = {np.cos(RAMP_ANGLE)}")
        # calculate new angle
        self.angle = (np.arcsin(self.right_wheel["y"] / WHEEL_TO_WHEEL))
        #assert self.angle <= 31.5
        # move the left wheel forward
        #assert self.left_wheel["x"] <= self.right_wheel["x"] - np.sqrt(WHEEL_TO_WHEEL**2 - self.right_wheel["y"]**2)
        self.left_wheel["x"] = self.right_wheel["x"] - np.sqrt(WHEEL_TO_WHEEL**2 - self.right_wheel["y"]**2)
        #self.left_wheel["y"] = 0
        if self.left_wheel["x"] >= STATION_OFFSET - FUDGE_FACTOR * 5 and self.state == States.ON_MIDDLE_LEFT_1_WHEEL:
            self.state = States.ON_LEFT_2_WHEEL_1_RAMP


    def on_ramp_left_1_wheel(self, velocity, timestep):
        '''x, y of where wheel touches ramp
            velocity -> convert to radians per second, should be a constant conversion
            v / R # of rotations per second
            circumference of wheel = 2 * pi * R
            angle of ramp = 31.5 degrees
            length of robot = const
            timestep = 
            distance_between_wheels = const


            Process:
            timestep * rotations/second = rotations 
            circumference * rotations = distance traveled on ramp = hypot
            y_diff = hypot * sin(angle of ramp)
            x_diff = hypot * cos(angle of ramp)
            x_new = x + x_diff
            y_new = y + y_diff

            # calculate new angle
            sin^-1(y_new / distance_between_wheels) = new angle

            x1_new = x_new - sqrt(w^2 - y_new^2)
            y1_new = 0
        '''
        ##print("on ramp left 1 wheel")
        # move the right wheel forward
        rotations = velocity / (WHEEL_RADIUS * 2 * np.pi) # i am still not sure about this 2 * pi, because it simplifies to velocity*timestep * sin/cos(RAMP_ANGLE)
        circumference = 2 * np.pi * WHEEL_RADIUS          # it looks fine though, and for a flat ramp it is correct as it simplifies to velocity * timestep
        hypot = rotations * circumference * timestep
        # ramp len y using pythagorean theorem

        RAMP_LEN_Y = np.sqrt(DIAG_LENGHT**2 - (RAMP_LEN_X)**2)
        # calculate ramp angle based on the two points of the ramp
        RAMP_ANGLE = (np.arctan((RAMP_LEN_Y) / (RAMP_LEN_X)))
        #RAMP_ANGLE = np.deg2rad(34.5)
        ##print(f"RAMP_ANGLE DEG = {np.rad2deg(RAMP_ANGLE)}")
        #RAMP_ANGLE =
        y_diff = hypot * np.sin(RAMP_ANGLE)
        x_diff = hypot * np.cos(RAMP_ANGLE)
        self.right_wheel["x"] += x_diff
        self.right_wheel["y"] += y_diff
        ##print(f"X diff = {x_diff} Y diff = {y_diff}, hypot = {hypot}, sin = {np.sin(RAMP_ANGLE)}, cos = {np.cos(RAMP_ANGLE)}")
        # calculate new angle
        self.angle = (np.arcsin(self.right_wheel["y"] / WHEEL_TO_WHEEL))
        #assert self.angle <= 31.5
        # move the left wheel forward
        self.left_wheel["x"] = self.right_wheel["x"] - np.sqrt(WHEEL_TO_WHEEL**2 - self.right_wheel["y"]**2)
        self.left_wheel["y"] = 0
        if self.right_wheel["x"] >= STATION_OFFSET + RAMP_LEN_X - WHEEL_RADIUS:
            ##print("on middle left 1 wheel")

            #exit()
            self.state = States.ON_MIDDLE_LEFT_1_WHEEL
            # move right wheel down to the floor
            self.right_wheel["y"] = 0.05715 + 0.0254 # found by taking max hieght on the other side and subtracting the max hieght of both
            self.left_ramp_end_XY = (self.right_wheel["x"], self.right_wheel["y"])
            self.left_ramp_start_XY = (self.left_ramp_start_XY[0], FUDGE_FACTOR)
            
            self.center_line_left_XY = (self.right_wheel["x"], self.right_wheel["y"])
            # use the angle of the robot to find 
            self.center_line_right_XY = (FLAT_X_RIGHT, MAX_HEIGHT)
            # update the angle
            self.angle = (np.arcsin(self.right_wheel["y"] / WHEEL_TO_WHEEL))
            #self.visualize()
            
    def on_left_2_wheel_1_ramp(self, velocity, timestep):
        # we are on flat plane now, # easy peasy
        # move both wheels forward 
        vt = velocity * timestep
        self.left_wheel["x"] += vt
        self.left_wheel["y"] += vt
        self.right_wheel["x"] += vt
        self.right_wheel["y"] += vt
    
    def linear_move(self, velocity, timestep):
        rotations = velocity / (WHEEL_RADIUS * 2 * np.pi)
        circumference = 2 * np.pi * WHEEL_RADIUS
        ##print(f"Rotations {rotations * circumference * timestep}, Circumference {circumference}")
        self.right_wheel["x"] += rotations * circumference * timestep # simplifies to velocity * timestep but it is what will be used for the ramp
        # move the left wheel forward
        self.left_wheel["x"] += rotations * circumference * timestep

    def step(self, velocity, timestep):
        self.time += timestep
        #clock_pub.publish(rospy.Time.from_sec(charging_station.time))
        ##print(f"Time: {charging_station.time}")
        #time.sleep(TIME_STEP)
        if self.state == States.OFF_LEFT:
            self.off_left(velocity, timestep)
        elif self.state == States.ON_RAMP_LEFT_1_WHEEL:
            self.on_ramp_left_1_wheel(velocity, timestep)
        elif self.state == States.ON_MIDDLE_LEFT_1_WHEEL:
            self.on_middle_left_1_wheel(velocity, timestep)
        elif self.state == States.ON_LEFT_2_WHEEL_1_RAMP:
            self.on_ramp_left_2_wheels(velocity, timestep)
        elif self.state == States.ON_MIDDLE_2_WHEEL:
            self.on_middle_2_wheels(velocity, timestep)

            
