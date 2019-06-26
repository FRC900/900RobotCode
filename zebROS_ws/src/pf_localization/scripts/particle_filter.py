"""

Particle Filter localization sample

oritinal author: Atsushi Sakai (@Atsushi_twi)

"""
"""
Big list of TODOs :

    This is a lot of work, but take it step by step.

    - Make this more object oriented. Right now, sim is tied pretty tightly to the rest of the code. I can see splitting this up so that isn't the case.
       - Particle filter should be a class. There can be a constructor, data required are num particles, RFID locations, wall coords, starting bounding box, Q and R matricies
         Methods would be localization and all the things it calls - gauss likelyhood, calc_covariance, and so on
	 I could see things like RFID location (done), wall coords, motion model being separate objects that are part of the
	  main pf object.  
	  Wall locations would have a constructor with field map (wall coords), and then provide an intersect() call
	  Robot motion model could be yet another object, although there isn't much interesting there to break out into a separate object

       - then sim could use this, factoring out things which are sim specific.  The DT, sim time, simulated sensor stuff, anyting related to xTrue, and so on.
       - not sure how python handles polymorphism, but if I were doing this in C++, I'd create a top level class which has the framework for running PF.  That is,
         it has methods to get robot motion, sensor input, run an iteration of PF, update the state and so on.  Then turn sim into a derived class which specializes those
	 methods using the sim-only functions below.  Once that was working, create a real-hardware derived class which fills in those same methods, only
	 using stuff read from ROS.  If done correctly, the top-level driver code could be shared?

    - Adding data from ROS
       - I'd start with the simplest - calc_input should just read and store the latest swerve cmd_vel message.  From there, expand it to store a list of cmd_vel messages with associated time.  The second one would handle the case where there were multiple cmd_vel messages set between goal detection messages.  
       - The grab and parse goal detection results, replacing the observation funtion.  It actually looks like observation does 3 separate things, 2 of which we don't really need.  Split those two things out somewhere else and make observation just a "get_detected_beacons" function?  The other two things - updating true location (impossible with real robot) and updating dead reckoning (possible but not really useful) should end up in sim-only code?  In any case, remove the code which adds noise to each detection - the real data will already be noisy. 
       - Also, note that the camera detection are relative tto the camera, but the coordinates of the RFID tags are relative to the field coordinate system.  This will have to be handled in the pf_loc code itself.  First, translate from camera-frame to robot base frame. This can be done when the data is read from the message since it is a constant translation.  Translation from robot base frame to map frame has to happen for each particle.  Since each particle has the location of the robot base relative to the map, adding the robot coords (in the map frame) to the displacement between robot center and target will give the coords of the target in the map frame. Each particle's guess at the robot location is different, though, so this will have to be done separately for each particle.
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

     - In sim, add a check for angle from target to robot. If the angle is too far to the side, we'd have trouble detecting it so model it as not-seen.  This will be the case for e.g. where the robot is just barely unblocked by the cargo ship wall but still at a really oblique angle to the targets on the side - those will be impossible to see in real life even though they're not technically blocked by a wall

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

# Estimation parameter of PF
Q = np.diag([0.05, 0.1])**2  # range error
#R = np.diag([1.0, np.deg2rad(40.0)])**2  # input error
R = np.diag([0.5, 0.5])**2  # input error

#  Simulation parameter
Qsim = np.diag([0.1, 0.2])**2
#Rsim = np.diag([1.0, np.deg2rad(30.0)])**2
Rsim = np.diag([0.5, 0.5])**2

DT = 0.1   # time tick [s]
SIM_TIME = 50.0  # simulation time [s]
MAX_RANGE = 4.75 # maximum observation range

# Particle filter parameter
NP = 100  # Number of Particles
NTh = NP / 2.0  # Number of particle for re-sampling

show_animation = True

# Return motion model transition vector
# For diff drive, linear and angular velocity
# For swerve, use x and y velocity
# This is used to estimate position using dead reckoning (integrating velocity to get
# position over time)

class Beacons:
    def __init__(self, coords):
        self.coords = coords

    # Given the predicted robot position and the measured distance
    # to a beacon, find the coordinates of the closest beacon to that
    # position.
    def closest_feature(self, robotpos, beacondist, beaconangle):
        mindist = sys.float_info.max
        featid = -1

        for i in range(len(self.coords[:, 0])):
            dx = robotpos[0] - self.coords[i, 0]
            dy = robotpos[1] - self.coords[i, 1]
            d = math.sqrt(dx**2 + dy**2) # distance
            b = math.atan2(dy, dx)       # bearing
            deltarange = abs(beacondist - d)
            deltaangle = abs(beaconangle - b)
            if ((deltarange + deltaangle) < mindist):
                mindist = deltarange + deltaangle
                featid = i

        return self.coords[featid]

    # Get a list of (distance, bearing, realX, realY) of all
    # beacons within max_range away from location
    def in_range(self, location, max_range):
        z = np.zeros((0, 4))

        for i in range(len(self.coords[:, 0])):
            dx = location[0, 0] - self.coords[i, 0]
            dy = location[1, 0] - self.coords[i, 1]
            d  = math.sqrt(dx**2 + dy**2)

            if d <= max_range:
                b  = math.atan2(dy, dx)
                zi = np.array([[d, b, self.coords[i, 0], self.coords[i, 1]]])
                z  = np.vstack((z, zi))

        return z

#Wall positions [x1,x2,y1,y2]
Walls = np.array([[-1.94,1.94,.709,.709],
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

Walls1 = np.array([[2.89,2.94,4.10,3.86],
                   [2.94,2.69,3.86,3.39],
                   [2.69,2.20,3.39,3.39],
                   [2.20,1.94,3.39,3.86],
                   [1.94,1.94,3.86,3.86],
                   [1.94,1.99,3.86,4.10]]) 
    
Walls2 = Walls1 * [-1,-1,1,1] 
Walls3 = Walls1 * [-1,-1,-1,-1] 
Walls4 = Walls1 * [1,1,-1,-1] 

Walls = np.append(Walls,Walls1,0)
Walls = np.append(Walls,Walls2,0)
Walls = np.append(Walls,Walls3,0)
Walls = np.append(Walls,Walls4,0)

def calc_input(time):
    #v = 1.0  # [m/s]
    #yawrate = 0.1  # [rad/s]
    #u = np.array([[v, yawrate]]).T

    if (time > 2):
        if ((time % (SIM_TIME / 5)) < (SIM_TIME / 10)):
            yvel = -1.2  # [m/s]
        else:
            yvel = 1.2  # [m/s]

        xvel = 0.4  # [m/s]
    else:
        xvel = 0
        yvel = 0

    u = np.array([[xvel, yvel]]).T
    return u

def ccw(A,B,C):
    return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

# Return true if line segments AB and CD intersect
def intersect(A,B,C,D):
    return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)

def observation(xTrue, xd, u, beacons):

    xTrue = motion_model(xTrue, u)

    # Get list of beacons in range and visible,
    # add noise to them
    z_initial = beacons.in_range(xTrue, MAX_RANGE)

    z = np.zeros((0, 4))
    for i in range(len(z_initial[:, 0])):
        does_intersect = False
        for j in range(len(Walls)):
            if intersect([Walls[j][0],Walls[j][2]],[Walls[j][1],Walls[j][3]],[xTrue[0,0],xTrue[1,0]],[z_initial[i,2],z_initial[i,3]]):
                does_intersect = True
                break

        if does_intersect == False:
            d = z_initial[i, 0] + np.random.randn() * Qsim[0, 0]  # add noise to distance
            b = z_initial[i, 1] + np.random.randn() * Qsim[1, 1]  # add noise to bearing
            zi = np.array([[d, b, z_initial[i, 2], z_initial[i, 3]]])
            z = np.vstack((z, zi))

    #print ("z", z)
    # add noise to input to simulate dead reckoning
    ud1 = u[0, 0] + np.random.randn() * Rsim[0, 0]
    ud2 = u[1, 0] + np.random.randn() * Rsim[1, 1]
    ud = np.array([[ud1, ud2]]).T

    xd = motion_model(xd, ud)

    return xTrue, z, xd, ud


# Given state matrix x and transition matrix u,
# calculate and return a state matrix for the next timestep
def motion_model(x, u):

    #F = np.array([[1.0, 0, 0, 0],
                  #[0, 1.0, 0, 0],
                  #[0, 0, 1.0, 0],
                  #[0, 0, 0, 0]])

    #B = np.array([[DT * math.cos(x[2, 0]), 0],
                  #[DT * math.sin(x[2, 0]), 0],
                  #[0.0, DT],
                  #[1.0, 0.0]])

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

    x = F.dot(x) + B.dot(u)

    return x


# probability of x given a 0-centered normal distribution 
def gauss_likelihood(x, sigma):
    p = 1.0 / math.sqrt(2.0 * math.pi * sigma ** 2) * \
        math.exp(-x ** 2 / (2 * sigma ** 2))

    return p


def calc_covariance(xEst, px, pw):
    cov = np.zeros((3, 3))

    for i in range(px.shape[1]):
        dx = (px[:, i] - xEst)[0:3]
        cov += pw[0, i] * dx.dot(dx.T)

    return cov

def pf_localization(px, pw, xEst, PEst, z, u, beacons):
    """
    Localization with Particle filter
    """

    for ip in range(NP):
        # Grab location and weight of this particle
        x = np.array([px[:, ip]]).T
        w = pw[0, ip]

        # Predict with random input sampling - update the position of
        # the particle using the input robot velocity. Add noise to the
        # commanded robot velocity to account for tracking error
	# TODO - should this use R rather than Rsim?
        ud1 = u[0, 0] + np.random.randn() * Rsim[0, 0]
        ud2 = u[1, 0] + np.random.randn() * Rsim[1, 1]
        ud = np.array([[ud1, ud2]]).T
        x = motion_model(x, ud)

        # x is now the new predicted robot position for this particle

        #  Calc Importance Weight
        for i in range(len(z[:, 0])):
            # Find the index of the landmark which most
            # closely matches the distance of the reported 
            # target
            beacon_coord = beacons.closest_feature(x, z[i, 0], z[i, 1])
            dx = x[0, 0] - beacon_coord[0]
            dy = x[1, 0] - beacon_coord[1]
            prez = math.sqrt(dx**2 + dy**2)
            dz = prez - z[i, 0]

            b = math.atan2(dy, dx)
            db = b - z[i, 1]
            
            #if ((abs(beacon_coord[j, 0] - z[i, 2]) > 0.0001) or (abs(beacon_coord[j, 1] - z[i, 3]) > 0.0001)):
                #print("Misidentified beacon : dz", dz, " db", db)
                #print("robot predicted pos", x)
                #print("beacon_coord[", j, "]", beacon_coord[j]);
                #print("z[", i, "]", z[i])

            # Weight is the product of the probabilities that this
            # position would give the measured distances
            w = w * gauss_likelihood(dz, math.sqrt(Q[0, 0]))
            w = w * gauss_likelihood(db, math.sqrt(Q[1, 1]))

        # Update particle list with new position & weight values
        px[:, ip] = x[:, 0]
        pw[0, ip] = w

    pw = pw / pw.sum()  # normalize

    # estimated position is the weighted average of all particles 
    xEst = px.dot(pw.T)
    PEst = calc_covariance(xEst, px, pw)

    px, pw = resampling(px, pw)

    return xEst, PEst, px, pw

# Resample particles - grab a new set of particles
# selected from the current list. The odds of a particle
# being selected each iteration is its weight - so more
# likely predictions have a better chance of being picked
def resampling(px, pw):
    """
    low variance re-sampling
    """

    Neff = 1.0 / (pw.dot(pw.T))[0, 0]  # Effective particle number
    if Neff < NTh:
        print("resampling")
        wcum = np.cumsum(pw)
        base = np.cumsum(pw * 0.0 + 1 / NP) - 1 / NP
        resampleid = base + np.random.rand(base.shape[0]) / NP

        inds = []
        ind = 0
        for ip in range(NP):
            while resampleid[ip] > wcum[ind]:
                ind += 1
            inds.append(ind)

        px = px[:, inds]
        pw = np.zeros((1, NP)) + 1.0 / NP  # init weight

    return px, pw


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


def main():
    print(__file__ + " start!!")

    time = 0.0

    # State Vectors [x y x' y']'
    xTrue = np.array([[-6.75,0,0,0]]).T
    xEst = np.zeros((4,1))
    PEst = np.eye(4)

    # Beacon positions [x, y]
    # in this case, beacons are vision targets
    # Add / subtract various small values to make sure they
    # aren't lying directly on a wall - bias them slightly
    # towards the interior of the field. This prevents wall
    # intersection code from thinking they're behind a wall accidentally
    beacon_coords = np.array([
                              [2.07-.005,3.63-0.005],
                              [2.44,3.39-.001],
                              [2.81+0.0075,3.63-0.0075],
                              [2.66,0.28],
                              [1.64,0.71],
                              [1.08,0.71],
                              [.53,.71],
                              [8.26 - 0.021,3.44]
                                ])
    # Assume map is symmetric around the origin in 
    # both horizontal and vertical directions
    beacon_coords = np.append(beacon_coords, np.array([-1,1]) * beacon_coords, 0)
    beacon_coords = np.append(beacon_coords, np.array([1,-1]) * beacon_coords, 0)
    print ("beacon_coords", beacon_coords)

    beacons = Beacons(beacon_coords)

    # Important for at least some particles to be initialized to be near the true starting
    # coordinates. Resampling can only pick from existing position proposals and if
    # none are close to the starting position, the particles will never converge
    starting_xmin = -8.0
    starting_xmax = -6.5
    starting_ymin = -2.5
    starting_ymax = 2.5
    xpos = np.random.uniform(starting_xmin, starting_xmax, (1, NP))
    ypos = np.random.uniform(starting_ymin, starting_ymax, (1, NP))
    px = np.concatenate((xpos, ypos, np.zeros((1, NP)), np.zeros((1, NP))), axis=0)

    #px = np.zeros((4, NP))  # Particle store
    pw = np.zeros((1, NP)) + 1.0 / NP  # Particle weight
    xDR = xTrue             # Dead reckoning - only for animation

    # history
    hxEst = xEst
    hxTrue = xTrue
    hxDR = xTrue

    #This code is a copy of the code below it, with removed visualization. However, this code saves data in a pickle to be visualized separately.
    
    columns = ['beacon_coords','xTrue','z','px','hxEst','hxDR','hxTrue','time']
    total_values = []
    """
    while SIM_TIME >= time:
        time += DT
        u = calc_input(time)
        xTrue, z, xDR, ud = observation(xTrue, xDR, u, beacons)

        xEst, PEst, px, pw = pf_localization(px, pw, xEst, PEst, z, ud, beacons)
        #print(xEst)

        # store data history
        hxEst = np.hstack((hxEst, xEst))
        hxDR = np.hstack((hxDR, xDR))
        hxTrue = np.hstack((hxTrue, xTrue))

        total_values.append([beacon_coords,xTrue,z,px,hxEst,hxDR,hxTrue,time])

    df = pd.DataFrame(total_values,columns=columns)
    df.to_pickle('data.p')
    """
    
    #Reset time for the standard code visualization
    time = 0
    

    while SIM_TIME >= time:
        time += DT
        u = calc_input(time)

        xTrue, z, xDR, ud = observation(xTrue, xDR, u, beacons)

        xEst, PEst, px, pw = pf_localization(px, pw, xEst, PEst, z, ud, beacons)
        #print(xEst)

        # store data history
        hxEst = np.hstack((hxEst, xEst))
        hxDR = np.hstack((hxDR, xDR))
        hxTrue = np.hstack((hxTrue, xTrue))
        
        total_values.append([Walls,beacon_coords,xTrue,z,px,hxEst,hxDR,hxTrue,time])

        if show_animation:
            plt.cla()

            for i in range(len(z[:, 0])):
                plt.plot([xTrue[0, 0], z[i, 2]], [xTrue[1, 0], z[i, 3]], "-k")

            for i in range(len(Walls)):
                plt.plot(Walls[i][:2],Walls[i][2:],color='purple')

            
            plt.plot(beacon_coords[:, 0], beacon_coords[:, 1], ".k")
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
            plt.pause(0.001)
            #if (time <= DT):
            #    input("Press any ket to continue...")

    #Saving pickle file
    columns = ['Walls','beacon_coords','xTrue','z','px','hxEst','hxDR','hxTrue','time']
    df = pd.DataFrame(total_values,columns=columns)
    df.to_pickle('data.p')

if __name__ == '__main__':
    main()
