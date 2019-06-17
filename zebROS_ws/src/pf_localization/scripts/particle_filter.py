"""

Particle Filter localization sample

oritinal author: Atsushi Sakai (@Atsushi_twi)

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
MAX_RANGE = 6.0  # maximum observation range

# Particle filter parameter
NP = 300  # Number of Particles
NTh = NP / 2.0  # Number of particle for re-sampling

show_animation = False

# Return motion model transition vector
# For diff drive, linear and angular velocity
# For swerve, use x and y velocity
# This is used to estimate position using dead reckoning (integrating velocity to get
# position over time)



#RFID positions [x, y]
RFID = np.array([[2.07,3.63],
                [2.44,3.39],
                [2.81,3.63],
                [2.66,0.28],
                [1.64,0.71],
                [1.08,0.71],
                [.53,.71],
                [8.26,3.44]])

RFID0 = np.array([-1,1])*RFID
RFID1 = np.array([1,-1])*RFID
RFID2 = np.array([-1,-1])*RFID

RFID = np.append(RFID,RFID0,0)
RFID = np.append(RFID,RFID1,0)
RFID = np.append(RFID,RFID2,0)

#Wall positions [x1,x2,y1,y2]
Walls = np.array([[-1.94,1.94,.71,.71],
                  [-1.94,1.94,-.71,-.71],
                  [1.94,2.66,0.71,0.58],
                  [-1.94,-2.66,0.71,0.58],
                  [1.94,2.66,-0.71,-0.58],
                  [-1.94,-2.66,-0.71,-0.58],
                  [2.66,2.66,0.58,-0.58],
                  [-2.66,-2.66,0.58,-0.58],
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

    if (time > 5):
        xvel = 1.5  # [m/s]
        if ((time % (SIM_TIME/5)) < (SIM_TIME / 10)):
            yvel = 1  # [m/s]
        else:
            yvel = -1  # [m/s]

        xvel = 0.3  # [m/s]
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

def observation(xTrue, xd, u, RFID):

    xTrue = motion_model(xTrue, u)

    # add noise to gps x-y
    z = np.zeros((0, 4))

    for i in range(len(RFID[:, 0])):
        dx = xTrue[0, 0] - RFID[i, 0]
        dy = xTrue[1, 0] - RFID[i, 1]
        d = math.sqrt(dx**2 + dy**2)
        b = math.atan2(dy, dx)
        


        if d <= MAX_RANGE:
            
            does_intersect=False
            for j in range(len(Walls)):
                if intersect([Walls[j][0],Walls[j][2]],[Walls[j][1],Walls[j][3]],[xTrue[0,0],xTrue[1,0]],[RFID[i,0],RFID[i,1]]):
                    does_intersect=True

            if does_intersect == False:
                dn = d + np.random.randn() * Qsim[0, 0]  # add noise
                bn = b + np.random.randn() * Qsim[1, 1]  # add noise
                zi = np.array([[dn, bn, RFID[i, 0], RFID[i, 1]]])
                z = np.vstack((z, zi))

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

# Given the predicted robot position and the measured distance
# to a beacon, find the index of the closest beacon to that
# position.  This simulates sensor returns from N identical
# beacons positioned at fixed locations in the map
def closest_feature(robotpos, beacondist, beaconangle, RFID):
    mindist = sys.float_info.max
    featid = -1

    for i in range(len(RFID[:, 0])):
        dx = robotpos[0] - RFID[i, 0]
        dy = robotpos[1] - RFID[i, 1]
        d = math.sqrt(dx**2 + dy**2)
        b = math.atan2(dy, dx)
        deltarange = abs(beacondist - d)
        deltaangle = abs(beaconangle - b)
        if ((deltarange +deltaangle)< mindist):
            mindist = deltarange + deltaangle
            featid = i

    return featid

def pf_localization(px, pw, xEst, PEst, z, u, RFID):
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
            j = closest_feature(x, z[i, 0], z[i, 1], RFID)
            dx = x[0, 0] - RFID[j, 0]
            dy = x[1, 0] - RFID[j, 1]
            prez = math.sqrt(dx**2 + dy**2)
            dz = prez - z[i, 0]

            b = math.atan2(dy, dx)
            db = b - z[i, 1]
            
            #if ((abs(RFID[j, 0] - z[i, 2]) > 0.0001) or (abs(RFID[j, 1] - z[i, 3]) > 0.0001)):
                #print("Misidentified beacon : dz", dz, " db", db)
                #print("robot predicted pos", x)
                #print("RFID[", j, "]", RFID[j]);
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

    # RFID positions [x, y]
    #RFID = np.array([[10.0, 0.0],
    #                 [10.0, 10.0],
    #                 [0.0, 15.0],
    #                 [-5.0, 20.0]])

    '''
    RFID = np.array([
            [-0.8231083414,1.0982357152],
            [-1.682597162,1.0982357152],
            [-2.523895743,1.0982357152],
            [-4.1337319466,0.6571224052],
            [-3.169649248,5.6094151333],
            [-3.7744747147,5.2156103415],
            [-4.3474672618,5.6094151333],
            [ 0.8231083414,1.0982357152],
            [ 1.682597162,1.0982357152],
            [ 2.523895743,1.0982357152],
            [ 4.1337319466,0.6571224052],
            [ 3.169649248,5.6094151333],
            [ 3.7744747147,5.2156103415],
            [ 4.3474672618,5.6094151333],
            [-0.8231083414,-1.0982357152],
            [-1.682597162,-1.0982357152],
            [-2.523895743,-1.0982357152],
            [-4.1337319466,-0.6571224052],
            [-3.169649248,-5.6094151333],
            [-3.7744747147,-5.2156103415],
            [-4.3474672618,-5.6094151333],
            [ 0.8231083414,-1.0982357152],
            [ 1.682597162,-1.0982357152],
            [ 2.523895743,-1.0982357152],
            [ 4.1337319466,-0.6571224052],
            [ 3.169649248,-5.6094151333],
            [ 3.7744747147,-5.2156103415],
            [ 4.3474672618,-5.6094151333]])
    '''    
    # State Vectors [x y x' y']'
    xTrue = np.array([[-7.5,0,0,0]]).T
    xEst = np.zeros((4,1))
    PEst = np.eye(4)

    # Important for at least some particles to be initialized to be near the true starting
    # coordinates. Resampling can only pick from existing position proposals and if
    # none are close to the starting position, the particles will never converge
    starting_xmin = -8.3
    starting_xmax = -7.3
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
    
    columns = ['RFID','xTrue','z','px','hxEst','hxDR','hxTrue','time']
    total_values = []
    while SIM_TIME >= time:
        time += DT
        u = calc_input(time)
        xTrue, z, xDR, ud = observation(xTrue, xDR, u, RFID)

        xEst, PEst, px, pw = pf_localization(px, pw, xEst, PEst, z, ud, RFID)
        #print(xEst)

        # store data history
        hxEst = np.hstack((hxEst, xEst))
        hxDR = np.hstack((hxDR, xDR))
        hxTrue = np.hstack((hxTrue, xTrue))

        total_values.append([RFID,xTrue,z,px,hxEst,hxDR,hxTrue,time])

    df = pd.DataFrame(total_values,columns=columns)
    df.to_pickle('data.p')
    
    #Reset time for the standard code visualization
    time = 0
    

    while SIM_TIME >= time:
        time += DT
        u = calc_input(time)

        xTrue, z, xDR, ud = observation(xTrue, xDR, u, RFID)

        xEst, PEst, px, pw = pf_localization(px, pw, xEst, PEst, z, ud, RFID)
        #print(xEst)

        # store data history
        hxEst = np.hstack((hxEst, xEst))
        hxDR = np.hstack((hxDR, xDR))
        hxTrue = np.hstack((hxTrue, xTrue))
        
        total_values.append([Walls,RFID,xTrue,z,px,hxEst,hxDR,hxTrue,time])

        if show_animation:
            plt.cla()

            for i in range(len(z[:, 0])):
                plt.plot([xTrue[0, 0], z[i, 2]], [xTrue[1, 0], z[i, 3]], "-k")

            for i in range(len(Walls)):
                plt.plot(Walls[i][:2],Walls[i][2:],color='purple')

            
            plt.plot(RFID[:, 0], RFID[:, 1], ".k")
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
    columns = ['Walls','RFID','xTrue','z','px','hxEst','hxDR','hxTrue','time']
    df = pd.DataFrame(total_values,columns=columns)
    df.to_pickle('data.p')

if __name__ == '__main__':
    main()
