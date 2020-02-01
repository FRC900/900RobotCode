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

     - Update closest_feature to include wall occlusion.  Make it check to see if the RFID tag being compared against could not possibly be seen because it is behind a wall.  If so, exclude it from being a potential match
     
     - Add robot orientation. This will let the code better filter out which targets are seen at each timestep.
       - For sim, need to update the motion model to model rotation
       - In the sim code which simulates target detection, update it to make sure the camera field of view can actually see the target
       - Same in closest_feature - update it to ignore targets which are outside of the camera's field of view when trying to find a closest match

     - Experiment with clustering.  This supposedly helps with cases where the targets are symmetric. The problem is that leads to cases where one observation could come from several different distinct positions.  If the model picks the wrong one and converges on it, as written it will be hard for the code to get to the correct position.  clustering seems to keep some particles from other less than optimal locations, and if further robot motion turns these suboptimal locations into a much better guess, will use them (as would be the case if a slight change in position made it obvious which of the various symetric alternatives the robot was located at).  See e.g. https://github.com/teammcr192/activity-indoor-localization/blob/master/ParticleFilter/pf.py for an example.

     - Another way to fix the problem above is to occasinally add random samples back in during resampling. This will help overcome the "everything converged to the wrong spot" problem, at the expense of replacing probably good estimates with random ones.  Having it happen infrequently and with only a small percent of the particles will help. Need to test and see what happens.

     - Another way to fix this is to have a tigher range of possible starting locations. For 2019, we know the robot has to be on the 1st level of the hab. Figure out what that means for a potential range of starting center coords of the robot.  Also, if we always start against the wall at the back of the hab that further constrains us (probably add some margin for error, but it will still be smaller than assuming the robot center can be literally anywhere on the 1st level of the hab.
        - Add even tigher restrictions for left / center / right, and add a service call to set them?  This would require restarting the particle filter, or at least re-initing the particle locations.

	  
"""

import sys
import numpy as np
from scipy.optimize import linear_sum_assignment
import math
import matplotlib.pyplot as plt
#import pandas as pd

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, TwistStamped
from goal_detection.msg import GoalDetection
import tf
import tf2_geometry_msgs
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3Stamped
#import message_filters
from threading import Lock
import time


DT = 0.1   # time tick [s]
SIM_TIME = 50.0  # simulation time [s]
MAX_RANGE = 4.75 # maximum observation range

show_animation = True

# history - start empty
hxEst  = np.empty(shape=[6,0])
hxTrue = hxEst
hxDR   = hxEst

def normalize_angle(a):
    a %= 2. * math.pi
    if a > math.pi:
        a -= 2. * math.pi
    return a

# Beacon - holds field-centric x,y,angle coordinates
#          of targets on the field
class Beacon:
    def __init__(self, coord):
        self.coord = coord # numpy array [x,y,angle]

    def __str__(self):
        return "Beacon : coord" + np.array2string(self.coord)

    def get_coords(self):
        return np.array([self.coord[0], self.coord[1], self.coord[2]])

    # Compute and return [distance, bearing to target] 
    # between the requested beacon and the input
    # location.  loc is [x,y] coordinates
    def distance(self, loc, reverse_angle = False):
        dx = self.coord[0] - loc[0]
        dy = self.coord[1] - loc[1]
        d = math.sqrt(dx**2 + dy**2) # distance
        if (reverse_angle == True):
            dy = -dy
            dx = -dx
        b = normalize_angle(math.atan2(dy, dx))      # bearing
        return np.array([d, b])

    # Given an angle from the robot to this beacon
    # in field-centric coords, return the angle from
    # the beacon to the robot in beacon-centric coords
    def robot_bearing(self, angle):
        abs_bearing = math.pi - angle
        rel_bearing = abs_bearing - self.coord[2]
        return(normalize_angle(rel_bearing))
 
    # If this beacon would be visible to a robot at loc,
    # return true distance and bearing for it.
    # Used by simulation to generate a list of simulated
    # detected targets this time-step
    def visible_in_sim(self, loc, max_range, walls):
        db = self.distance(loc, True)
        if (db[0] <= max_range):
            # Make sure target isn't at too oblique an angle to the
            # vision target - TODO : measure how far off we can get and still see
            delta_angle = abs(normalize_angle(self.coord[2] - db[1]))
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

    def at(self, i):
        if i < len(self.beacons):
            return self.beacons[i]
        return None

    def visible_in_sim(self, loc, max_range, walls):
        ret = Detections()
        for i in range(len(self.beacons)):
           d = self.beacons[i].visible_in_sim(loc, max_range, walls)
           if (d != None):
               ret.append(d)
        return ret

    def get_costs(self, robot_loc, detection):
        #print ("robot_loc")
        #print (robot_loc)
        #print ("detection coord")
        #print(detection)
        costs = np.zeros(len(self.beacons))
        beacondist, beaconangle = detection.get_dist_angle()
        for i in range(len(self.beacons)):
            #print(self.beacons[i])
            db = self.beacons[i].distance(robot_loc)
            #print("db = %f %f" % (db[0], db[1]))
            # Assume a max range for actual detections, screen out
            # beacons further away than that
            if db[0] > 5.0:
                #print("robot to beacon dist of %f too far" % db[0])
                costs[i] = 1e5
                continue
            # Check that potential target is actually in the field of view of 
            # the cameras
            if abs(normalize_angle(db[1] - robot_loc[2])) > 0.85:
                #print("robot to beacon angle %f too large" % db[0])
                costs[i] = 1e5
                continue
            #print("beacondist = %f, beaconangle = %f, robot angle = %f" % (beacondist, beaconangle, robot_loc[2]))
            deltarange = abs(beacondist - db[0])
            #print ("deltarange = %f" % deltarange)
            deltaangle = abs(normalize_angle((beaconangle - db[1])))
            #print ("deltaangle = %f" % deltaangle)

            # Check that the angle from the target to the robot is
            # between +/- pi/2.  This is a quick way to rule out
            # cases where it would be looking through a wall to see the 
            # target
            reverse_angle = abs(self.beacons[i].robot_bearing(beaconangle))
            #print ("reverseangle = %f" % reverse_angle)
            if (reverse_angle >= (math.pi / 2.0 - .2)):
                costs[i] = 1e5
                continue

            #print("deltarange = %f deltaangle = %f" % (deltarange , deltaangle))
            costs[i] = deltarange + deltaangle * 5
        return costs

class Walls:
    def __init__(self, walls):
        self.walls = walls
        wall_max = np.amax(np.absolute(walls), axis=0)
        self.maxx = max(wall_max[0], wall_max[1])
        self.maxy = max(wall_max[2], wall_max[3])
        print(wall_max)
        print(self.maxx)
        print(self.maxy)

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

    def outside_field(self, loc):
        if abs(loc[0]) > self.maxx:
            return True
        if abs(loc[1]) > self.maxy:
            return True
        return False

# Collect walls plus beacons into a single conatiner
class FieldMap:
    def __init__(self, beacons, walls):
        self.beacons = beacons
        self.walls = walls

    # Get a list of (distance, bearing, realX, realY, realAngle) of all
    # beacons within max_range away from location
    # Make sure they are visible, both not obscured by an intervening
    # wall as well as at an angle that can be seen
    def visible_in_sim(self, location, max_range):
        return self.beacons.visible_in_sim(location, max_range, self.walls)

    def outside_field(self, loc):
        return self.walls.outside_field(loc) 

# Detection -
#  Holds robot-relative distance / angle coordinates
#    of a detected target
class Detection:
    def __init__(self, coord):
        self.coord = coord # numpy array [dist / angle]
        self.actual = None

    def __str__(self):
        return "Detection : coord" + np.array2string(self.coord) + " actual," + str(self.actual)

    # set actual field beacon associated with this detection
    def set_actual(self, beacon): 
        self.actual = beacon

    def get_actual(self):
        return self.actual;

    def get_dist_angle(self):
        return (self.coord[0], self.coord[1])

    # Add noise to dist/ angle measurement. Use by simulation to make
    # detections look more like real world measurements
    def add_noise(self, Q):
        self.coord[0] += np.random.randn() * Q[0, 0]  # add noise to distance
        self.coord[1] += np.random.randn() * Q[1, 1]  # add noise to bearing

    def weight(self, loc, Q):
        # TODO - maybe make this a small number rather than
        # zero to assign a penalty to particles which produce
        # cases where a detection can't be mapped to a beacon
        if (self.actual == None):
            return 0

        # distance / angle to best predicted ideal detection
        loc_to_closest = self.actual.distance(loc) 

        # difference in distance and angle between predicted and actual detection
        delta = loc_to_closest - self.coord
        
        # Weight is the product of the probabilities that this
        # position would give the measured distances
        # Alternatively, it is the likelhood that the given position
        # is accurate given the distance / angle to target
        w  = self._gauss_likelihood(delta[0], math.sqrt(Q[0, 0]))
        w *= self._gauss_likelihood(delta[1], math.sqrt(Q[1, 1]))

        return w

    # probability of x given a 0-centered normal distribution 
    def _gauss_likelihood(self, x, sigma):
        p = 1.0 / math.sqrt(2.0 * math.pi * sigma ** 2.) * \
            math.exp(-x ** 2. / (2. * sigma ** 2.))

        return p

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
        if field_map.outside_field(loc):
            print("Outside field")
            #print(loc)
            return 0

        # Assign each detection to a best-guess for
        # which beacon it actually is
        self.guess_actuals(loc, field_map)

        w = 1
        for i in range(len(self.detections)):
            w *= self.detections[i].weight(loc, Q)
            
        return w

    def add_noise(self, Q):
        for i in range(len(self.detections)):
            self.detections[i].add_noise(Q)

    def guess_actuals(self, loc, field_map):
        # Generate a cost between each [detection, beacon] pair
        # The cost is basically just the difference between the
        # detection distance and beacon distance from the robot
        # location - beacons closer to the detection coords
        # have a lower cost
        for i in range(len(self.detections)):
            cost = field_map.beacons.get_costs(loc, self.detections[i])
            if (i == 0):
                costs = np.vstack([cost])
            else:
                costs = np.vstack([costs, cost])

        # Assign detections to actual beacons such that
        # the cost of all mappings is minimized.  
        row_ind, col_ind = linear_sum_assignment(costs)
        if False:
            print ("costs")
            print (costs)
            #row_ind[i] == detection index
            #col_ind[i] == matched beacon index
            print (row_ind)
            print (col_ind)

        # Set the "actual" field in each detection to the 
        # real beacon matched up with it.  Set the actual
        # to None if no beacon could possibly match (i.e. if
        # they are all out of range or behind walls)
        for i in range(len(row_ind)):
            if costs[row_ind[i], col_ind[i]] > 1e4:
                self.detections[row_ind[i]].set_actual(None)
            else:
                self.detections[row_ind[i]].set_actual(field_map.beacons.at(col_ind[i]))
            #print(field_map.beacons.at(col_ind[i]))

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
        # Particle state - x, y, theta, x', y', theta'
        self.px = np.concatenate((xpos,
                                  ypos, 
                                  np.zeros((1, self.NP)),
                                  np.zeros((1, self.NP)),  
                                  np.zeros((1, self.NP)), 
                                  np.zeros((1, self.NP))),
                                 axis=0)

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

    # TODO : update me now that state matrix includes orientation
    def calc_covariance(self, xEst, px, pw):
        cov = np.zeros((3, 3))

        for i in range(px.shape[1]):
            dx = (px[:, i] - xEst)[0:3]
            cov += pw[0, i] * dx.dot(dx.T)

        return cov

    def move_particles(self, u):
        for ip in range(self.NP):
            # Grab location of this particle
            x = np.array([self.px[:, ip]]).T

            # Predict with random input sampling - update the position of
            # the particle using the input robot velocity. Add noise to the
            # commanded robot velocity to account for tracking error
            ud1 = u[0, 0] + np.random.randn() * self.R[0, 0]
            ud2 = u[1, 0] + np.random.randn() * self.R[1, 1]
            ud3 = u[2, 0] + np.random.randn() * self.R[2, 2]
            ud = np.array([[ud1, ud2, ud3]]).T
            x = self.motion_model(x, ud)

            # x is now the new predicted robot position for this particle
            self.px[:, ip]  = x[:, 0]
 
    # Since we have a trusted angle measurement, simply
    # set all particles to the desired angle plus
    # some noise
    def rotate_particles(self, angle):
        for ip in range(self.NP):
            self.px[2, ip]  = angle + np.random.randn() * self.R[2,2]

    def get_estimate(self):
        return self.px.dot(self.pw.T)

    def localize(self, z):
        print("localize")
        print(z)
        pw = np.zeros((1, self.NP))
        for ip in range(self.NP):
            # Grab location and weight of this particle
            x = np.array([self.px[:, ip]]).T

            # Update particle list with new position & weight values
            pw[0, ip] = z.weights(x[:,0].T, self.field_map, self.Q) # TODO - or maybe just straight assignment

        pw_sum = pw.sum()
        if (pw_sum != 0): # only update if there's at least one non-zero weight
            self.pw = pw / pw_sum  # normalize

        # estimated position is the weighted average of all particles 
        xEst = self.get_estimate()
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
        ud3 = u[2, 0] + np.random.randn() * self.Rsim[2, 2]
        ud = np.array([[ud1, ud2, ud3]]).T

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
last_time = None
start_time = None
pf = None
imu_offset = -math.pi / 2.0
imu_yaw = 0
lock = Lock()

def cmd_vel_callback(cmd_vel_data):
    fn_start_time = time.time()
    global lock
    if lock.acquire(False):
        try:
            print ("==========================")
            global last_time
            global start_time
            if (last_time == None):
                start_time = last_time = cmd_vel_data.header.stamp.to_sec()

            global cmd_vel_u
            cmd_vel_u = np.array([[cmd_vel_data.twist.linear.x, cmd_vel_data.twist.linear.y, cmd_vel_data.twist.angular.z]]).T
            #q = tf.transformations.quaternion_inverse((imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w))
            global imu_yaw # Make global for debugging for now
            #(roll, pitch, imu_yaw) = tf.transformations.euler_from_quaternion(q)
            #print("cmd_vel_u before")
            #print(cmd_vel_u.T)
            #print("imu_yaw %f "% imu_yaw)
            new_x = math.cos(imu_yaw) * cmd_vel_u[0] - math.sin(imu_yaw) * cmd_vel_u[1]
            new_y = math.sin(imu_yaw) * cmd_vel_u[0] + math.cos(imu_yaw) * cmd_vel_u[1]
            cmd_vel_u[0] = new_x
            cmd_vel_u[1] = new_y
            #print("cmd_vel_u after")
            #print(cmd_vel_u.T)

            now = cmd_vel_data.header.stamp.to_sec()
            dt = now - last_time
            #print ("imu_stamp time %f cmd_vel_stamp time %f" % (imu_data.header.stamp.to_sec(), cmd_vel_data.header.stamp.to_sec()))
            #print("cmd_vel : start_time %f, last_time %f, now, %f, dt %f" % (start_time, last_time, now, dt))
            last_time = now # force imu callback to reset last time
            if (dt > 0.2):
                dt = 0.03
                print ("adjusted dt %f" % dt)

            if (cmd_vel_u[0,0] != 0 or cmd_vel_u[1,0] != 0):
                #print ("moving particles");
                B = np.array([[dt,  0.0, 0.0],
                              [0.0, dt,  0.0],
                              [0.0, 0.0,  dt], # TODO - do we care about commanded z-rotation?
                              [1.0, 0.0, 0.0],
                              [0.0, 1.0, 0.0],
                              [0.0, 0.0, 1.0]])

                global pf
                pf.set_B(B)
                pf.move_particles(cmd_vel_u)

        finally:
            lock.release()
            print "cmd_vel callback runtime = %f"%(time.time() - fn_start_time)

def imu_callback(data):
    global lock
    fn_start_time = time.time()
    if lock.acquire(False):
        try:
            print ("**************************")
            q = tf.transformations.quaternion_inverse((data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w))
            global imu_yaw
            (roll, pitch, imu_yaw) = tf.transformations.euler_from_quaternion(q)
            global pf
            global imu_offset
            #print("imu_yaw %f (%f)"% (imu_yaw, imu_yaw + imu_offset))
            pf.rotate_particles(imu_yaw + imu_offset)

        finally:
            lock.release()
            print "IMU callback runtime = %f"%(time.time() - fn_start_time)

"""
def cmd_vel_callback(data, args):
    global lock
    if lock.acquire(False):
        try:
            global last_time
            global start_time
            global imu_yaw

            #print ("cmd_vel_callback : rospy.get_time() = %f" % rospy.get_time())
            cmd_vel_u = np.array([[data.linear.x, data.linear.y]]).T
            #print("cmd_vel_u before")
            #print(cmd_vel_u.T)
            #print("imu_yaw %f" % imu_yaw)
            new_x = math.cos(imu_yaw) * cmd_vel_u[0] - math.sin(imu_yaw) * cmd_vel_u[1]
            new_y = math.sin(imu_yaw) * cmd_vel_u[0] + math.cos(imu_yaw) * cmd_vel_u[1]
            cmd_vel_u[0] = new_x
            cmd_vel_u[1] = new_y
            #print("cmd_vel_u after")
            #print(cmd_vel_u.T)
            now = rospy.get_time()
            dt = now - last_time
            #print("cmd_vel : start_time %f, last_time %f, dt %f" % (start_time, last_time, dt))
            last_time = None # force imu callback to reset last time
            #if (dt > 0.3):
                #dt = 0.02
                #print ("adjusted dt %f" % dt)
            if (cmd_vel_u[0,0] != 0 or cmd_vel_u[1,0] != 0):
                #print ("moving particles");
                B = np.array([[dt, 0.0],
                    [0.0, dt],
                    [1.0, 0.0],
                    [0.0, 1.0]])

                pf = args
                pf.set_B(B)
                pf.move_particles(cmd_vel_u)
            pf.rotate_particles(imu_yaw + imu_offset)

        finally:
            lock.release()
"""

def goal_detection_callback(data, args):
    global lock
    fn_start_time = time.time()
    if lock.acquire(False):
        try:
            print ("ooooooooooooooooo")
            global last_time
            global start_time
            global imu_yaw
            global imu_offset
            if (last_time == None):
                start_time = last_time = data.header.stamp.to_sec();
            pf = args
            now = data.header.stamp.to_sec() - start_time
            print "1:goal detect callback runtime = %f"%(time.time() - fn_start_time)
            if (len(data.location) == 0):
                debug_plot(None, pf.get_estimate(), None, None, Detections(), pf.get_px(), now)
                return
            print "2:goal detect callback runtime = %f"%(time.time() - fn_start_time)
            z = Detections()
            print "3:goal detect callback runtime = %f"%(time.time() - fn_start_time)
            for i in range(len(data.location)):
                # Fake transform to center of robot
                x =  data.location[i].x + .1576
                y = -data.location[i].y - .234
                #print ("Raw detection loc.x = %f, loc.y = %f, x = %f, y = %f, imu_yaw = %f, imu_offset = %f" % (data.location[i].x, data.location[i].y, x, y, imu_yaw, imu_offset))
                #print("x", x)
                #print("y", y)
                d = math.hypot(x, y)
                b = normalize_angle(math.atan2(y, x) + imu_yaw + imu_offset)
                z.append(Detection(np.array([d, b])))
           
            #xDR        = pf.update_dead_reckoning(xDR, cmd_vel_u)
            xEst, PEst = pf.localize(z)
            print "4:goal detect callback runtime = %f"%(time.time() - fn_start_time)
            #print ("Guess actuals, loc =")
            #print (xEst)
            pf.guess_detection_actuals(z, np.array([xEst[0,0], xEst[1,0], xEst[2,0]]))
            print "5:goal detect callback runtime = %f"%(time.time() - fn_start_time)

            debug_plot(None, xEst, PEst, None, z, pf.get_px(), now)
            print "6:goal detect callback runtime = %f"%(time.time() - fn_start_time)

        finally:
            lock.release()
            print "7:goal detect callback runtime = %f"%(time.time() - fn_start_time)

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
for i in range(len(beacon_coords[:,2])):
    beacon_coords[i,2] = normalize_angle(beacon_coords[i,2])
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

debug_plot_count = 0
def debug_plot(xTrue, xEst, PEst, xDR, z, px, now):
    global debug_plot_count
    fn_start_time = time.time()
    # store data history
    global hxEst
    global hxDR
    global hxTrue
    hxEst = np.hstack((hxEst, xEst))
    if (xDR is not None):
        hxDR = np.hstack((hxDR, xDR))
    if (xTrue is not None):
        hxTrue = np.hstack((hxTrue, xTrue))
    print "1. debug_plot runtime = %f"%(time.time() - fn_start_time)
    debug_plot_count += 1
    if debug_plot_count < 20:
        return
    debug_plot_count = 0

    if show_animation:
        print (".................")
        plt.cla()

        actual = z.get_actual();
        print "2. debug_plot runtime = %f"%(time.time() - fn_start_time)
        plot_x = xEst
        if (xTrue is not None):
            plot_x = xTrue
        print "3. debug_plot runtime = %f"%(time.time() - fn_start_time)
        for i in range(len(actual)):
            beacon = actual[i].get_coords()
            plt.plot([plot_x[0, 0], beacon[0]], [plot_x[1, 0], beacon[1]], "-k")
        print "4. debug_plot runtime = %f"%(time.time() - fn_start_time)

        for i in range(len(wall_coords)):
            plt.plot(wall_coords[i][:2],wall_coords[i][2:],color='purple')
        print "5. debug_plot runtime = %f"%(time.time() - fn_start_time)

        u = np.cos(beacon_coords[:,2])
        v = np.sin(beacon_coords[:,2])
        #plt.plot(beacon_coords[:, 0], beacon_coords[:, 1], ".k")
        plt.quiver(beacon_coords[:, 0], beacon_coords[:, 1], u, v)
        print "6. debug_plot runtime = %f"%(time.time() - fn_start_time)
        if px is not None:
            plt.plot(px[0, :], px[1, :], ".r")
        plt.plot(np.array(hxTrue[0, :]).flatten(),
                 np.array(hxTrue[1, :]).flatten(), "-b")
        plt.plot(np.array(hxDR[0, :]).flatten(),
                 np.array(hxDR[1, :]).flatten(), "-k")
        plt.plot(np.array(hxEst[0, :]).flatten(),
                 np.array(hxEst[1, :]).flatten(), "-r")
        print "7. debug_plot runtime = %f"%(time.time() - fn_start_time)
        #print("xEst")
        #print(xEst)
        u = np.cos(xEst[2,0])
        v = np.sin(xEst[2,0])
        plt.quiver(xEst[0, 0], xEst[1, 0], u, v)
        print "8. debug_plot runtime = %f"%(time.time() - fn_start_time)
        #if (PEst is not None):
            #plot_covariance_ellipse(xEst, PEst)
        plt.axis("equal")
        plt.grid(True)
        plt.text(-2, 0, str(now))
        plt.pause(0.0001)
        print "9. debug_plot runtime = %f"%(time.time() - fn_start_time)

def main():
    print(__file__ + " start!!")

    time = 0.0

    # Particle filter parameter
    NP = 500  # Number of Particles

    # State Vectors [x y theta x' y' theta']'
    xTrue = np.array([[-7.00,0,0,0,0,0]]).T
    xDR = xTrue  # Dead reckoning - only for animation

    field_map = FieldMap(Beacons(beacon_coords), Walls(wall_coords))

    if False:
        z = Detections()
        z.append(Detection(np.array([1.0860, .14437])))
        loc = np.array([-3.372, 3.589, 2.078 - math.pi / 2., 0, 0, 0])
        z.guess_actuals(loc, field_map)
        print(z)

        debug_plot(None, np.array([loc]).T, None, None, z, None, -1)

    # For swerve drive, use a very simple model
    # next position is based on current position plus current velocity
    # next velocity is just requested velocity
    #              x    y    th   x'   y'   th'

    F = np.array([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                  [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                  [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
    B = np.array([[DT, 0.0, 0,0],
                  [0.0, DT, 0,0],
                  [0.0, 0.0, DT],
                  [1.0, 0.0, 0.0],
                  [0.0, 1.0, 0.0],
                  [0.0, 0.0, 1.0]])

    # Estimation parameter of PF
    Q = np.diag([0.05, 0.1])**2  # range error
    #R = np.diag([1.0, np.deg2rad(40.0)])**2  # input error
    R = np.diag([1.8, 1.8, np.deg2rad(10.0)])**2  # x, y, theta velocity error

    # Simulation parameters
    Qsim = np.diag([0.1, 0.2])**2
    #Rsim = np.diag([1.0, np.deg2rad(30.0)])**2
    Rsim = np.diag([0.75, 0.75, np.deg2rad(5)])**2

    # Important for at least some particles to be initialized to be near the true starting
    # coordinates. Resampling can only pick from existing position proposals and if
    # none are close to the starting position, the particles will never converge
    starting_xmin = -8.0
    starting_xmax = -6.5
    starting_ymin = -2.5
    starting_ymax = 2.5
    #pf = PFLocalizationSim(NP, F, B, field_map, Q, R, Qsim, Rsim, starting_xmin, starting_xmax, starting_ymin, starting_ymax)
    global pf
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

    #cmd_vel_sub = message_filters.Subscriber("/frcrobot_jetson/swerve_drive_controller/cmd_vel_out", TwistStamped, queue_size = 2)
    #imu_sub     = message_filters.Subscriber("/imu/zeroed_imu", Imu, queue_size = 2)
    #ts          = message_filters.ApproximateTimeSynchronizer([imu_sub, cmd_vel_sub], 10, 0.1)
    #ts.registerCallback(imu_cmd_vel_callback)

    cmd_vel_sub = rospy.Subscriber("/frcrobot_jetson/swerve_drive_controller/cmd_vel_out", TwistStamped, cmd_vel_callback, queue_size = 2)
    imu_sub = rospy.Subscriber("/imu/zeroed_imu", Imu, imu_callback, queue_size = 1)
    rospy.Subscriber("/goal_detection/goal_detect_msg", GoalDetection, goal_detection_callback, (pf), queue_size = 1)

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
