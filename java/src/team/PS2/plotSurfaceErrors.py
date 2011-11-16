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
Z[ 0, 0] =  68
Z[ 0, 1] =  62
Z[ 0, 2] =  77
Z[ 0, 3] =  65
Z[ 1, 0] =  44
Z[ 1, 1] =  72
Z[ 1, 2] =  73
Z[ 1, 3] =  52
Z[ 2, 0] =  72
Z[ 2, 1] =  2 
Z[ 2, 2] =  58
Z[ 2, 3] =  52
Z[ 3, 0] =  27
Z[ 3, 1] =  78
Z[ 3, 2] =  63
Z[ 3, 3] =  66


surf = ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap=cm.jet,
                       linewidth=0, antialiased=True)


# cset = ax.contour(X, Y, Z, zdir='z', offset=ax.get_zlim()[0], alpha=0.5)
# cset = ax.contour(X, Y, Z, zdir='x', offset=1, alpha=0.5)
# cset = ax.contour(X, Y, Z, zdir='y', offset=1, alpha=0.5)

# Set our labels
ax.set_xlabel('Normalized Landmark Sensor Noise')
ax.set_ylabel('Normalized Odometry Noise')
ax.set_zlabel('Number of Errors')

ax.set_title("Number of Data Association Errors versus Noise")

# This seems nice
ax.view_init(elev=54, azim=37)

fig.colorbar(surf, shrink=0.5, aspect=5)

savefig("partB.png")

