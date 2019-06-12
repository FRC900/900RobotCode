import matplotlib.pyplot as plt
from matplotlib import animation
import pickle
import numpy as np
import pandas as pd
import math
plt.style.use('ggplot')

df = pd.read_pickle('data.p')
fig = plt.figure()
ax = fig.add_subplot(211,xlim=(-10,10),ylim=(-5,5))
ax1 = fig.add_subplot(212)
error_bound = []

for i in range(500):
    error_bound.append(math.sqrt((df['hxTrue'][i][0,:][-1]-df['hxEst'][i][0,:][-1])**2 + (df['hxTrue'][i][1,:][-1]-df['hxEst'][i][1,:][-1])**2))

def animate(i):

    ax.cla()
    ax.set_xlim(-10,10)
    ax.set_ylim(-5,5)
    ax.scatter(df['RFID'][i][:,0],df['RFID'][i][:,1],color='black')
    ax.scatter(df['px'][i][0,:],df['px'][i][1,:],color='red')
    
    ax.plot(np.array(df['hxTrue'][i][0,:]).flatten(),np.array(df['hxTrue'][i][1,:]).flatten(),color='green',label='true')
    ax.plot(np.array(df['hxDR'][i][0,:]).flatten(),np.array(df['hxDR'][i][1,:]).flatten(),color='black',label='odom')
    ax.plot(np.array(df['hxEst'][i][0,1:]).flatten(),np.array(df['hxEst'][i][1,1:]).flatten(),color='orange',label='estimate')

    ax1.plot(np.arange(0,i/10,.1),error_bound[0:i],color='red')
    for j in range(len(df['z'][i][:,0])):
        ax.plot([df['xTrue'][i][0,0],df['z'][i][j,2]],[df['xTrue'][i][1,0],df['z'][i][j,3]],color='blue')

    return

ani = animation.FuncAnimation(fig, animate, frames=500,
                              interval=1, blit=False)
ani.save('vid.mp4',fps=10,dpi=300,bitrate=128000000)
