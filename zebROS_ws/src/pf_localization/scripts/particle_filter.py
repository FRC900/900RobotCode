#!/usr/bin/python
"""

Particle Filter localization sample

original author: Atsushi Sakai (@Atsushi_twi)

"""
"""
Big list of TODOs :

    This is a lot of work, but take it step by step.

    - Adding data from ROS
       - Figure out a way to publish a transform between an odom frame and the calculate map frame location.  The result of the pf will be the distance from the robot to the origin of the map frame, or alternatively, the coordinates of the robot in the map frame (same thing).  But since the realsense camera (or something else) will be publishing odom->robot, the code just needs to express this as an offset between the existing odom frame origin and the calculate map frame origin.  Subtracting the origin of the odom frame from the calculated map frame should do it, or maybe a ROS transform call mgiht be possible?
       - If the code ends up keeping track of multiple cmd_vel messages that happened between goal detect messages, change the function which calculates motion to account for that. Basically, step through the list and iteratively add cmd_vel * dt for that message to the robot's position.  Be careful with the start and end entries in the list - the dt there will depend on the relative arrival times of the cmd_vel messages along with the previous and current goal detect message.  e.g. if cmd_vel comes in at 10 msec intervals most of the update will be loc = cmd_vel * 0.01. The first and last timestep will not typically be 10msec though, because there the timestep is actually the time between the last overall update (last arrival of a goal detect message) and the time the next cmd_vel arrived. Same with the last message.  A cmd_vel will come in, then before the next cmd_vel arrives, a goal detect message shows up, screwing up the regular spacing of those messages. Only add enough distance for the robot traveling cmd_vel for the time between the final cmd_vel message and the goal detect message.

     - Update closest_feature to include wall occlusion.  Make it check to see if the RFID tag being compared against could not possibly be seen because it is behind a wall.  If so, exclude it from being a potential match
     
     - Add robot orientation. This will let the code better filter out which targets are seen at each timestep.
       - Update robot state vector to hold robot rotation
       - For sim, need to update the motion model to model rotation
       - For ROS, add a navX subscriber. The motion model should simply copy this value into robot state when called (we trust the navX as much as anything).
       - In the sim code which simulates target detection, update it to make sure the camera field of view can actually see the target
       - Same in closest_feature - update it to ignore targets which are outside of the camera's field of view when trying to find a closest match
       - Add code which rotates all particles to the navX orientation, possibly including some noise
       - For bonus points, update the gui to show which way the robot is facing (arrow, line, whatever)

     - Experiment with clustering.  This supposedly helps with cases where the targets are symmetric. The problem is that leads to cases where one observation could come from several different distinct positions.  If the model picks the wrong one and converges on it, as written it will be hard for the code to get to the correct position.  clustering seems to keep some particles from other less than optimal locations, and if further robot motion turns these suboptimal locations into a much better guess, will use them (as would be the case if a slight change in position made it obvious which of the various symetric alternatives the robot was located at).  See e.g. https://github.com/teammcr192/activity-indoor-localization/blob/master/ParticleFilter/pf.py for an example.

     - Another way to fix the problem above is to occasinally add random samples back in during resampling. This will help overcome the "everything converged to the wrong spot" problem, at the expense of replacing probably good estimates with random ones.  Having it happen infrequently and with only a small percent of the particles will help. Need to test and see what happens.

     - Another way to fix this is to have a tigher range of possible starting locations. For 2019, we know the robot has to be on the 1st level of the hab. Figure out what that means for a potential range of starting center coords of the robot.  Also, if we always start against the wall at the back of the hab that further constrains us (probably add some margin for error, but it will still be smaller than assuming the robot center can be literally anywhere on the 1st level of the hab.
        - Add even tigher restrictions for left / center / right, and add a service call to set them?  This would require restarting the particle filter, or at least re-initing the particle locations.

	  
"""

import sys
import numpy as np
import math
import matplotlib.pyplot as plt
import pandas as pd

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from goal_detection.msg import GoalDetection
import tf
import tf2_geometry_msgs
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3Stamped
import message_filters


DT = 0.1   # time tick [s]
SIM_TIME = 50.0  # simulation time [s]
MAX_RANGE = 4.75 # maximum observation range

show_animation = True

# history - start empty
hxEst  = np.empty(shape=[4,0])
hxTrue = hxEst
hxDR   = hxEst

# Beacon - holds field-centric x,y,angle coordinates
#          of targets on the field
class Beacon:
    def __init__(self, coord):
        self.coord = coord # numpy array [x,y,angle]

    def __str__(self):
        return "Beacon : coord" + np.array2string(self.coord)

    def get_coords(self):
        return np.array([self.coord[0], self.coord[1]])

    # Compute and return [distance, bearing to target] 
    # between the requested beacon and the input
    # location.  loc is [x,y] coordinates
    def distance(self, loc, reverse_angle = False):
        dx = self.coord[0] - loc[0]
        dy = self.coord[1] - loc[1]
        d = math.sqrt(dx**2 + dy**2) # distance
        #if (reverse_angle == True):
            #dy = -dy
            #dx = -dx
        b = math.atan2(dy, dx)       # bearing
        return np.array([d, b])
 
    # If this beacon would be visible to a robot at loc,
    # return true distance and bearing for it.
    # Used by simulation to generate a list of simulated
    # detected targets this time-step
    def visible_in_sim(self, loc, max_range, walls):
        db = self.distance(loc, True)
        if (db[0] <= max_range):
            # Make sure target isn't at too oblique an angle to the
            # vision target - TODO : measure how far off we can get and still see
            delta_angle = abs(self.coord[2] - db[1])
            if ((delta_angle < .9) or (delta_angle > (2 * math.pi - .9))):

                if walls.intersect([loc[0], loc[1]],[self.coord[0], self.coord[1]]) == False:
                    detection = Detection(np.array([db[0], db[1]]))
                    detection.set_actual(self)
                    return detection

        return None


# Holds a list of Beacon objects, plus accessor functions
class Beacons:
    def __init__(self, beacons): # Initialize from a numpy array of [x,y,theta] coords
        self.beacons = []
        for i in range(len(beacons[:,0])):
            self.beacons.append(Beacon(beacons[i]))

    def append(self, beacon):
        self.beacons.append(beacon)

    def clear(self):
        self.beacons = []

    def visible_in_sim(self, loc, max_range, walls):
        ret = Detections()
        for i in range(len(self.beacons)):
           d = self.beacons[i].visible_in_sim(loc, max_range, walls)
           if (d != None):
               ret.append(d)
        return ret

    # For a detection at beacondist / beaconangle from robot_loc,
    # where robot_loc is [x,y] in field-centric coords,
    # find the closest actual beacon and return it
    def closest_beacon(self, robot_loc, beacondist, beaconangle):
        mindist = sys.float_info.max
        best_idx = -1

        for i in range(len(self.beacons)):
            #print("beacon=", self.beacons[i].get_coords())
            db = self.beacons[i].distance(robot_loc)
            #print("db = ", db)
            deltarange = abs(beacondist - db[0])
            deltaangle = abs(beaconangle - db[1])
            #print("deltarange ", deltarange, " deltaangle ", deltaangle)
            delta = deltarange + deltaangle * 20
            #print("delta", delta)
            if (delta < mindist):
                # TODO - add checks from sim code for wall occlusion
                # and target angle to rule out additional potential targets
                mindist = delta
                best_idx = i
                #print("----------- new best", best_idx)

        if (best_idx == -1): # Won't happen, maybe later after adding a bound
            return None      # on acceptable min distance
        return self.beacons[best_idx]

class Walls:
    def __init__(self, walls):
        self.walls = walls

    def _ccw(self,A,B,C):
        return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

    # Return true if line segments AB and CD intersect
    def _one_intersect(self,A,B,C,D):
        return self._ccw(A,C,D) != self._ccw(B,C,D) and self._ccw(A,B,C) != self._ccw(A,B,D)

    # Returns true if line segment CD intersects with any defined walls
    # returns false otherwise
    def intersect(self, C, D):
        for i in range(len(self.walls)):
            if self._one_intersect([self.walls[i][0],self.walls[i][2]],[self.walls[i][1],self.walls[i][3]],C,D):
                return True
        return False

# Collect walls plus beacons into a single conatiner
class FieldMap:
    def __init__(self, beacons, walls):
        self.beacons = beacons
        self.walls = walls

    def _dist_to_beacon(self, beacon, location):
        return self.beacons[beacon].distance(location)

    # Given the predicted robot position and the measured distance
    # to a beacon, find the coordinates of the closest beacon to that
    # position.
    def closest_beacon(self, robot_loc, beacondist, beaconangle):
        return self.beacons.closest_beacon(robot_loc, beacondist, beaconangle)

    # Get a list of (distance, bearing, realX, realY, realAngle) of all
    # beacons within max_range away from location
    # Make sure they are visible, both not obscured by an intervening
    # wall as well as at an angle that can be seen
    def visible_in_sim(self, location, max_range):
        return self.beacons.visible_in_sim(location, max_range, self.walls)

# Detection -
#  Holds robot-relative distance / angle coordinates
#    of a detected target
class Detection:
    def __init__(self, coord):
        self.coord = coord # numpy array [dist / angle]
        self.actual = None

    def __str__(self):
        return "Detection : coord" + np.array2string(self.coord) + " actual," + str(self.actual)

    # set actual field beacon associated with this detection - for sim and debugging
    def set_actual(self, actual): 
        self.actual = actual

    def get_actual(self):
        return self.actual;

    # Add noise to dist/ angle measurement. Use by simulation to make
    # detections look more like real world measurements
    def add_noise(self, Q):
        self.coord[0] += np.random.randn() * Q[0, 0]  # add noise to distance
        self.coord[1] += np.random.randn() * Q[1, 1]  # add noise to bearing

    def weight(self, loc, field_map, Q):
        closest_beacon = field_map.closest_beacon(loc, self.coord[0], self.coord[1]) # Todo - change to use coord[]
        if (closest_beacon == None):
            return 0

        # distance / angle to best predicted ideal detection
        loc_to_closest = closest_beacon.distance(loc) 

        # difference in distance and angle between predicted and actual detection
        delta = loc_to_closest - self.coord
        
        # Weight is the product of the probabilities that this
        # position would give the measured distances
        # Alternatively, it is the likelhood that the given position
        # is accurate given the distance / angle to target
        w =     self._gauss_likelihood(delta[0], math.sqrt(Q[0, 0]))
        w = w * self._gauss_likelihood(delta[1], math.sqrt(Q[1, 1]))

        return w

    # probability of x given a 0-centered normal distribution 
    def _gauss_likelihood(self, x, sigma):
        p = 1.0 / math.sqrt(2.0 * math.pi * sigma ** 2.) * \
            math.exp(-x ** 2. / (2. * sigma ** 2.))

        return p

    def guess_actual(self, loc, field_map):
        #print("guess actual loc", loc, "self.coord", self.coord)
        self.actual = field_map.closest_beacon(loc, self.coord[0], self.coord[1])

# List of detection objects plus helper functions
class Detections:
    def __init__(self):
        self.detections = []

    def __str__(self):
        s = ""
        for i in range(len(self.detections)):
            s += "\t" + str(self.detections[i]) + "\n"
        return s

    def append(self, detection):
        if (detection != None):
            self.detections.append(detection)

    def get_actual(self):
        actuals = []
        for i in range(len(self.detections)):
            a = self.detections[i].get_actual()
            if (a != None):
                actuals.append(a)
        return actuals

    def weights(self, loc, field_map, Q):
        w = 1
        for i in range(len(self.detections)):
            w *= self.detections[i].weight(loc, field_map, Q)

        return w;

    def add_noise(self, Q):
        for i in range(len(self.detections)):
            self.detections[i].add_noise(Q)


    def guess_actuals(self, loc, field_map):
        for i in range(len(self.detections)):
            self.detections[i].guess_actual(loc, field_map)


class PFLocalization(object):
    def __init__(self, NP, F, B, field_map, Q, R, starting_xmin, starting_xmax, starting_ymin, starting_ymax):
        self.NP = NP
        self.NTh = NP / 2.0 # Number of particle for re-sampling
        self.F = F
        self.B = B
        self.field_map = field_map
        self.Q = Q
        self.R = R

        xpos = np.random.uniform(starting_xmin, starting_xmax, (1, self.NP))
        ypos = np.random.uniform(starting_ymin, starting_ymax, (1, self.NP))
        self.px = np.concatenate((xpos, ypos, np.zeros((1, self.NP)), np.zeros((1, NP))), axis=0)

        #px = np.zeros((4, self.NP))  # Particle store
        self.pw = np.zeros((1, self.NP)) + 1.0 / NP  # Particle weight

    def set_B(self, B):
        self.B = B

    # Get particle coords - for viz only
    def get_px(self):
        return self.px

    # Given state matrix x and transition matrix u,
    # calculate and return a state matrix for the next timestep
    def motion_model(self, x, u):
        x = self.F.dot(x) + self.B.dot(u)

        return x

    def calc_covariance(self, xEst, px, pw):
        cov = np.zeros((3, 3))

        for i in range(px.shape[1]):
            dx = (px[:, i] - xEst)[0:3]
            cov += pw[0, i] * dx.dot(dx.T)

        return cov

    """
    Localization with Particle filter
    """
    def localize(self, z, u):

        for ip in range(self.NP):
            # Grab location and weight of this particle
            x = np.array([self.px[:, ip]]).T
            w = self.pw[0, ip]

            # Predict with random input sampling - update the position of
            # the particle using the input robot velocity. Add noise to the
            # commanded robot velocity to account for tracking error
            ud1 = u[0, 0] + np.random.randn() * self.R[0, 0]
            ud2 = u[1, 0] + np.random.randn() * self.R[1, 1]
            ud = np.array([[ud1, ud2]]).T
            x = self.motion_model(x, ud)

            # x is now the new predicted robot position for this particle

            # Update particle list with new position & weight values
            self.px[:, ip]  = x[:, 0]
            self.pw[0, ip] = z.weights(x[:,0].T, self.field_map, self.Q) # TODO - or maybe just straight assignment

        self.pw = self.pw / self.pw.sum()  # normalize

        # estimated position is the weighted average of all particles 
        xEst = self.px.dot(self.pw.T)
        PEst = self.calc_covariance(xEst, self.px, self.pw)

        self.resampling()

        return xEst, PEst

    # Resample particles - grab a new set of particles
    # selected from the current list. The odds of a particle
    # being selected each iteration is its weight - so more
    # likely predictions have a better chance of being picked
    def resampling(self):
        """
        low variance re-sampling
        """

        Neff = 1.0 / (self.pw.dot(self.pw.T))[0, 0]  # Effective particle number
        if Neff < self.NTh:
            wcum = np.cumsum(self.pw)
            base = np.cumsum(self.pw * 0.0 + 1. / self.NP) - 1. / self.NP
            resampleid = base + np.random.rand(base.shape[0]) / self.NP

            inds = []
            ind = 0
            for ip in range(self.NP):
                while resampleid[ip] > wcum[ind]:
                    ind += 1
                inds.append(ind)

            self.px = self.px[:, inds]
            self.pw = np.zeros((1, self.NP)) + 1. / self.NP  # init weight

    def guess_detection_actuals(self, detections, loc):
        detections.guess_actuals(loc, self.field_map)


class PFLocalizationSim(PFLocalization):
    def __init__(self, NP, F, B, field_map, Q, R, Qsim, Rsim, starting_xmin, starting_xmax, starting_ymin, starting_ymax):
        super(PFLocalizationSim, self).__init__(NP, F, B, field_map, Q, R, starting_xmin, starting_xmax, starting_ymin, starting_ymax)
        self.Qsim = Qsim
        self.Rsim = Rsim

    # Return motion model transition vector
    # For diff drive, linear and angular velocity
    # For swerve, use x and y velocity
    # This is used to estimate position using dead reckoning (integrating velocity to get
    # position over time)
    def get_input(self, time):
        #v = 1.0  # [m/s]
        #yawrate = 0.1  # [rad/s]
        #u = np.array([[v, yawrate]]).T

        if (time > 2):
            if ((time % (SIM_TIME / 5)) < (SIM_TIME / 10)):
                yvel = -1.2  # [m/s]
            else:
                yvel = 1.2  # [m/s]

            xvel = 0.3  # [m/s]
        else:
            xvel = 0
            yvel = 0

        u = np.array([[xvel, yvel]]).T
        return u

    # Sim-specific
    def update_ground_truth(self, xTrue, u):
        return self.motion_model(xTrue, u)

    # Return a list of Detections for targets visible at this time
    def get_detections(self, xTrue):
        # Get list of beacons in range and visible,
        # add noise to them
        z = self.field_map.visible_in_sim(xTrue[:,0].T, MAX_RANGE)
        z.add_noise(self.Qsim)
        return z

    def update_dead_reckoning(self, xd, u):
        # add noise to input to simulate dead reckoning
        ud1 = u[0, 0] + np.random.randn() * self.Rsim[0, 0]
        ud2 = u[1, 0] + np.random.randn() * self.Rsim[1, 1]
        ud = np.array([[ud1, ud2]]).T

        return self.motion_model(xd, ud)


def plot_covariance_ellipse(xEst, PEst):  # pragma: no cover
    Pxy = PEst[0:2, 0:2]
    eigval, eigvec = np.linalg.eig(Pxy)

    if eigval[0] >= eigval[1]:
        bigind = 0
        smallind = 1
    else:
        bigind = 1
        smallind = 0

    t = np.arange(0, 2 * math.pi + 0.1, 0.1)

    # eigval[bigind] or eiqval[smallind] were occassionally negative numbers extremely
    # close to 0 (~10^-20), catch these cases and set the respective variable to 0
    try:
        a = math.sqrt(eigval[bigind])
    except ValueError:
        a = 0

    try:
        b = math.sqrt(eigval[smallind])
    except ValueError:
        b = 0

    x = [a * math.cos(it) for it in t]
    y = [b * math.sin(it) for it in t]
    angle = math.atan2(eigvec[bigind, 1], eigvec[bigind, 0])
    R = np.array([[math.cos(angle), math.sin(angle)],
                  [-math.sin(angle), math.cos(angle)]])
    fx = R.dot(np.array([[x, y]]))
    px = np.array(fx[0, :] + xEst[0, 0]).flatten()
    py = np.array(fx[1, :] + xEst[1, 0]).flatten()
    plt.plot(px, py, "--r")

# TODO : these should be in a class along with pf and the various 
# ROS callbacks
cmd_vel_u = np.zeros((2,1))
imu_yaw   = 0
last_time = None
start_time = None

def imu_cmd_vel_callback(imu_data, cmd_vel_data):
    global last_time
    if (last_time == None):
        start_time = last_time = rospy.get_time()

    global cmd_vel_u
    cmd_vel_u = np.array([[cmd_vel_data.linear.x, cmd_vel_data.linear.y]]).T
    q = tf.transformations.quaternion_inverse((imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w))
    global imu_yaw # Make global for debugging for now
    (roll, pitch, imu_yaw) = tf.transformations.euler_from_quaternion(q)
    #print("cmd_vel_u before")
    #print(cmd_vel_u)
    #print("imu_yaw")
    #print(imu_yaw)
    new_x = math.cos(imu_yaw) * cmd_vel_u[0] - math.sin(imu_yaw) * cmd_vel_u[1]
    new_y = math.sin(imu_yaw) * cmd_vel_u[0] + math.cos(imu_yaw) * cmd_vel_u[1]
    cmd_vel_u[0] = new_x
    cmd_vel_u[1] = new_y
    #print("cmd_vel_u after")
    #print(cmd_vel_u.T)

def goal_detection_callback(data, args):
    pf = args
    global cmd_vel_u
    z = Detections()
    for i in range(len(data.location)):
        # Fake transform to center of robot
        x = data.location[i].x - .1576
        y = -data.location[i].y + .234
        #y = -y
        #print("x", x)
        #print("y", y)
        d = math.hypot(x, y)
        b = math.atan2(y, x)
        z.append(Detection(np.array([d, b])))
    
    #print("z")
    #print(z)
    global last_time
    dt = rospy.get_time() - last_time
    print ("dt", dt)
    print (cmd_vel_u.T)
    last_time = rospy.get_time()
    now = last_time - start_time
    B = np.array([[dt, 0.0],
                  [0.0, dt],
                  [1.0, 0.0],
                  [0.0, 1.0]])

    pf.set_B(B)
    #xDR        = pf.update_dead_reckoning(xDR, cmd_vel_u)
    xEst, PEst = pf.localize(z, cmd_vel_u)
    pf.guess_detection_actuals(z, np.array([xEst[0,0], xEst[1,0]]))

    debug_plot(None, xEst, PEst, None, z, pf.get_px(), now)

# Beacon positions [x, y, orientation]
# in this case, beacons are vision targets
# Add / subtract various small values to make sure they
# aren't lying directly on a wall - bias them slightly
# towards the interior of the field. This prevents wall
# intersection code from thinking they're behind a wall accidentally
beacon_coords = np.array([
                          [2.07-.005,3.63-0.005, math.radians(270 - 60)],
                          [2.44,3.39-.001, 3. * math.pi / 2.],
                          [2.81+0.0075,3.63-0.0075, math.radians(270 + 60)],
                          [2.66,0.28, 0],
                          [1.64,0.71, math.pi / 2.],
                          [1.08,0.71, math.pi / 2.],
                          [0.53,0.71, math.pi / 2.],
                          [8.26 - 0.021,3.44, math.pi]
                            ])
# Assume map is symmetric around the origin in 
# both horizontal and vertical directions
# x, y, are -1, 1, but subtracting them from 0 inverts again
# subtract angles from pi to mirror horizontally
mirror = np.array(np.array([0, 0, math.pi]) - np.array([1,-1,1]) * beacon_coords)
beacon_coords = np.append(beacon_coords, mirror, 0)

# x, y, are 1, -1, but subtracting them from 0 inverts again
# subtract angles from 2pi to mirror vertically
mirror = np.array(np.array([0, 0, 2 * math.pi]) - np.array([-1,1,1]) * beacon_coords)
beacon_coords = np.append(beacon_coords, mirror, 0)

# normalize angles between 0 and 2pi
beacon_coords[beacon_coords[:,2] <= -math.pi] += np.array([0,0,2 * math.pi])
beacon_coords[beacon_coords[:,2] > math.pi] -= np.array([0,0,2 * math.pi])
print ("beacon_coords", beacon_coords)

#Wall positions [x1,x2,y1,y2]
wall_coords = np.array([[-1.94,1.94,.709,.709],
                  [-1.94,1.94,-.709,-.709],
                  [1.94,2.66,0.71,0.58],
                  [-1.94,-2.66,0.71,0.58],
                  [1.94,2.66,-0.71,-0.58],
                  [-1.94,-2.66,-0.71,-0.58],
                  [2.659,2.659,0.58,-0.58],
                  [-2.659,-2.659,0.58,-0.58],
                  [8.24,-8.24,4.10,4.10],
                  [8.24,-8.24,-4.10,-4.10],
                  [8.24,8.24,4.10,-4.10],
                  [-8.24,-8.24,4.10,-4.10]])    

# Rocket - 4 copies mirrored around origin
wall_coords1 = np.array([[2.89,2.94,4.10,3.86],
                   [2.94,2.69,3.86,3.39],
                   [2.69,2.20,3.39,3.39],
                   [2.20,1.94,3.39,3.86],
                   [1.94,1.94,3.86,3.86],
                   [1.94,1.99,3.86,4.10]]) 
    
wall_coords1 = np.append(wall_coords1, wall_coords1 * [-1,-1,1,1], 0) 
wall_coords1 = np.append(wall_coords1, wall_coords1 * [1,1,-1,-1], 0) 

wall_coords  = np.append(wall_coords,wall_coords1,0)

def debug_plot(xTrue, xEst, PEst, xDR, z, px, now):
    # store data history
    global hxEst
    global hxDR
    global hxTrue
    hxEst = np.hstack((hxEst, xEst))
    if (xDR is not None):
        hxDR = np.hstack((hxDR, xDR))
    if (xTrue is not None):
        hxTrue = np.hstack((hxTrue, xTrue))

    if show_animation:
        plt.cla()

        actual = z.get_actual();
        plot_x = xEst
        if (xTrue is not None):
            plot_x = xTrue
        for i in range(len(actual)):
            beacon = actual[i].get_coords()
            plt.plot([plot_x[0, 0], beacon[0]], [plot_x[1, 0], beacon[1]], "-k")

        for i in range(len(wall_coords)):
            plt.plot(wall_coords[i][:2],wall_coords[i][2:],color='purple')

        u = np.cos(beacon_coords[:,2])
        v = np.sin(beacon_coords[:,2])
        #plt.plot(beacon_coords[:, 0], beacon_coords[:, 1], ".k")
        plt.quiver(beacon_coords[:, 0], beacon_coords[:, 1], u, v)
        plt.plot(px[0, :], px[1, :], ".r")
        plt.plot(np.array(hxTrue[0, :]).flatten(),
                 np.array(hxTrue[1, :]).flatten(), "-b")
        plt.plot(np.array(hxDR[0, :]).flatten(),
                 np.array(hxDR[1, :]).flatten(), "-k")
        plt.plot(np.array(hxEst[0, :]).flatten(),
                 np.array(hxEst[1, :]).flatten(), "-r")
        #plot_covariance_ellipse(xEst, PEst)
        plt.axis("equal")
        plt.grid(True)
        plt.text(0, 0, str(now))
        plt.pause(0.001)

def main():
    print(__file__ + " start!!")

    time = 0.0

    # Particle filter parameter
    NP = 200  # Number of Particles

    # State Vectors [x y x' y']'
    xTrue = np.array([[-6.75,0,0,0]]).T
    xDR = xTrue  # Dead reckoning - only for animation

    field_map = FieldMap(Beacons(beacon_coords), Walls(wall_coords))

    # For swerve drive, use a very simple model
    # next position is based on current position plus current velocity
    # next velocity is just requested velocity
    F = np.array([[1.0, 0.0, 0.0, 0.0],
                  [0.0, 1.0, 0.0, 0.0],
                  [0.0, 0.0, 0.0, 0.0],
                  [0.0, 0.0, 0.0, 0.0]])
    B = np.array([[DT, 0.0],
                  [0.0, DT],
                  [1.0, 0.0],
                  [0.0, 1.0]])

    # Estimation parameter of PF
    Q = np.diag([0.05, 0.1])**2  # range error
    #R = np.diag([1.0, np.deg2rad(40.0)])**2  # input error
    R = np.diag([0.5, 0.5])**2  # input error

    # Simulation parameters
    Qsim = np.diag([0.1, 0.2])**2
    #Rsim = np.diag([1.0, np.deg2rad(30.0)])**2
    Rsim = np.diag([0.75, 0.75])**2

    # Important for at least some particles to be initialized to be near the true starting
    # coordinates. Resampling can only pick from existing position proposals and if
    # none are close to the starting position, the particles will never converge
    starting_xmin = -8.0
    starting_xmax = -6.5
    starting_ymin = -2.5
    starting_ymax = 2.5
    #pf = PFLocalizationSim(NP, F, B, field_map, Q, R, Qsim, Rsim, starting_xmin, starting_xmax, starting_ymin, starting_ymax)
    pf = PFLocalization(NP, F, B, field_map, Q, R, starting_xmin, starting_xmax, starting_ymin, starting_ymax)

    #This code is a copy of the code below it, with removed visualization. However, this code saves data in a pickle to be visualized separately.
    
    columns = ['wall_coords','beacon_coords','xTrue','z','px','hxEst','hxDR','hxTrue','time']
    total_values = []
    """
    while SIM_TIME >= time:
        time += DT
        u          = pf.get_input(time)
        xTrue      = pf.update_ground_truth(xTrue, u)
        z          = pf.get_detections(xTrue)
        xDR        = pf.update_dead_reckoning(xDR, u)
        xEst, PEst = pf.localize(z, u)

        # store data history
        hxEst = np.hstack((hxEst, xEst))
        hxDR = np.hstack((hxDR, xDR))
        hxTrue = np.hstack((hxTrue, xTrue))

        total_values.append([wall_coords,beacon_coords,xTrue,z,px,hxEst,hxDR,hxTrue,time])

    df = pd.DataFrame(total_values,columns=columns)
    df.to_pickle('data.p')
    """

    rospy.init_node('pf_localization', anonymous=True)

    cmd_vel_sub = message_filters.Subscriber("/frcrobot_jetson/swerve_drive_controller/cmd_vel", Twist)
    imu_sub = message_filters.Subscriber("/frcrobot_rio/navx_mxp", Imu)
    ts = message_filters.ApproximateTimeSynchronizer([cmd_vel_sub, imu_sub], 10, 0.1)
    ts.registerCallback(imu_cmd_vel_callback)
    rospy.Subscriber("/goal_detection_node/goal_detect_msg", GoalDetection, goal_detection_callback, (pf))
    global last_time
    last_time = rospy.get_time()

    # spin() simply keeps python from exiting until this node is stopped
    print "ROS Spinning";
    rospy.spin()
    
    """
    
    #Reset time for the standard code visualization
    time = 0

    while SIM_TIME >= time:
        time      += DT
        u          = pf.get_input(time)
        xTrue      = pf.update_ground_truth(xTrue, u)
        z          = pf.get_detections(xTrue)
        xDR        = pf.update_dead_reckoning(xDR, u)
        xEst, PEst = pf.localize(z, u)

        debug_plot(xTrue, xEst, PEst, xDR, z, pf.get_px(), time)
        
        total_values.append([wall_coords,beacon_coords,xTrue,z,pf.get_px(),hxEst,hxDR,hxTrue,time])

    #Saving pickle file
    columns = ['wall_coords','beacon_coords','xTrue','z','px','hxEst','hxDR','hxTrue','time']
    df = pd.DataFrame(total_values,columns=columns)
    df.to_pickle('data.p')
    """

if __name__ == '__main__':
    main()
