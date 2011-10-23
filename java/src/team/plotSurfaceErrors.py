from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import numpy as np
import os
from pylab import *


fig = figure()
ax = fig.gca(projection='3d')

odomNoise = array([1,2,4,8])
landNoise = array([1,2,4,8])

X, Y = np.meshgrid(odomNoise, landNoise)

Z = zeros((odomNoise.size, landNoise.size))
Z[0,2] = .5
Z[1,3] = .9


surf = ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap=cm.jet,
                       linewidth=0, antialiased=True)


# cset = ax.contour(X, Y, Z, zdir='z', offset=ax.get_zlim()[0], alpha=0.5)
# cset = ax.contour(X, Y, Z, zdir='x', offset=1, alpha=0.5)
# cset = ax.contour(X, Y, Z, zdir='y', offset=1, alpha=0.5)

# Set our labels
ax.set_xlabel('Odometry Noise')
ax.set_ylabel('Landmark Sensor Noise')
ax.set_zlabel('Number of Errors')

ax.set_title("Number of Data Association Errors versus Noise")

# This seems nice
# ax.view_init(elev=25, azim=45)

# savefig("%sFitness3D.png" % (imagePath), transparent=False)

