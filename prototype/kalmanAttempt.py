""" Kalman filter equations/methods for PS3 Task 1.
See the __main__ part at the bottom.
"""

from pylab import *
import numpy as np
# import matplotlib
from matplotlib.collections import PatchCollection
import matplotlib.patches as mpatches
import pdb



def mod2pi_pos(vin):
    """ Ensure that v is between [-pi,pi]
    Only good for + numbers
    """
    q = vin*(0.5/pi) + 0.5
    qi = int(q)

    return vin - qi*2*pi

def mod2pi(vin):
    """ Ensure that v is between [-pi,pi]
    """
    if vin < 0:
        return -mod2pi_pos(-vin)
    else:
        return mod2pi_pos(vin)


def predictedFeaturePosXY(posObs, rTheta):
    """ Get the predicted feature observation.
    This is just a direct application of our observation model from r theta to xy

    returns length 2 vector
    """
    return array([posObs[0] + rTheta[0]*cos(rTheta[1] + posObs[2]), posObs[1] + rTheta[0]*sin(rTheta[1] + posObs[2]) ])


def predictedFeaturePosRT(posXYT, featXY):
    """ Get the expected feature observation from our state vector prediciton
    and sensor model.

    This is just a direct application of our observation model from xy theta to rt

    returns length 2 vector
    """
    x = posXYT[0]
    y = posXYT[1]
    t = posXYT[2]

    fx = featXY[0]
    fy = featXY[1]

    r = sqrt(pow(fx-x,2) + pow(fy-y,2))
    theta = mod2pi(arctan2(fy-y,fx-x) - t)

    return array([r, theta])


def stateTransitionJacobian(size,rowIndex,colIndex, obs, currTheta):
    """ A combination of a state transition matrix and Jacobian
    for our new feature. This should only be called when we are adding to
    our "state" vector.

    returns (n+2)xn matrix
    """

    if size[1] > size[0]:
        print "HUGE LOGIC ERROR"

    if size[1] == size[0]:
        return identity(size[0])


    G = zeros((size[0], size[1]))

    G[0:(size[1]),0:(size[1]+1)] = identity(size[1])

    # Fill in new feature "transition Jacobian"
    G[rowIndex,colIndex]     = 1
    G[rowIndex,colIndex+1]   = 0
    G[rowIndex,colIndex+2]   = -obs[0]*sin(currTheta + obs[1])
    G[rowIndex+1,colIndex]   = 0
    G[rowIndex+1,colIndex+1] = 1
    G[rowIndex+1,colIndex+2] = obs[0]*cos(currTheta + obs[1])


    return G


def drivingNoiseJacobian(size,position, obs, currTheta):
    """ Relates the noise in r theta to our state vector.

    returns nx2 matrix
    """

    R = zeros((size,2))

    R[position,0] = cos(obs[1] + currTheta)
    R[position,1] = -obs[0]*sin(obs[1] + currTheta)
    R[position+1,0] = sin(obs[1] + currTheta)
    R[position+1,1] = obs[0]*cos(obs[1] + currTheta)

    return R


def kalmanUpdateJacobian(size, position, obsXY, location):
    """ Jacobian of our new observation with respect to the state vector.
    We're assuming the position of the robot where it observed the new
    feature starts at column 0.

    returns 2xn matrix
    """

    J = zeros((2,size))

    l_x = obsXY[0]
    l_y = obsXY[1]
    x0 = location[0]
    y0 = location[1]

    J[0,0] = -(2.0*l_x - 2*x0)/(2*sqrt(pow(l_x - x0,2) + pow(l_y - y0,2)))
    J[0,1] = -(2.0*l_y - 2*y0)/(2*sqrt(pow(l_x - x0,2) + pow(l_y - y0,2)))
    J[0,2] = 0.0
    J[1,0] = (l_y - y0)/(pow(l_x - x0,2)*(pow(l_y - y0,2)/pow(l_x - x0,2) + 1.0))
    J[1,1] = -1/((l_x - x0)*(pow(l_y - y0,2)/pow(l_x - x0,2) + 1.0))
    J[1,2] = -1.0

    J[0,position]   = (2.0*l_x - 2*x0)/(2*sqrt(pow(l_x - x0,2) + pow(l_y - y0,2)));
    J[1,position]   = (2.0*l_y - 2*y0)/(2*sqrt(pow(l_x - x0,2) + pow(l_y - y0,2)));
    J[0,position+1] = -(l_y - y0)/(pow(l_x - x0,2)*(pow(l_y - y0,2)/pow(l_x - x0,2) + 1.0));
    J[1,position+1] = 1/((l_x - x0)*(pow(l_y - y0,2)/pow(l_x - x0,2) + 1.0));

    return J



def kalmanUpdate(x,cov,obs,obsIndex):
    """ Computes full EKF update.
    obsIndex specifies what index in the current state vector that the
    observation represents. It will be -1 if it is a new feature.

    returns newX, newCov

    """

    xPredict = x.copy()

    # Our observation noise is given in the problem statement
    sigmaObs = diag([3,1])

    newFeature = False
    # If it's a new feature... need to initialize a new kalman filter
    if obsIndex == -1:
        # Extend the state vector in the best way we know how
        xPredict = concatenate((xPredict,predictedFeaturePosXY(x[0:3], obs)))

        # Tell the rest of this method where to place things
        obsIndex = x.size
        newFeature = True



    # We need a non-trivial "state transition" matrix here
    G = stateTransitionJacobian(array([xPredict.size,x.size]),
                                x.size,
                                0,
                                obs,
                                x[2])

    # Our driving noise Jacobian to bring RT uncentainty into current sigma
    drivingR = drivingNoiseJacobian(xPredict.shape[0],
                                    obsIndex,
                                    obs,
                                    x[2])

    # Jacobian relating RT to the state vector. Used in the big 3 equations
    J = kalmanUpdateJacobian(xPredict.shape[0],
                             obsIndex,
                             predictedFeaturePosXY(x[0:3], obs),
                             x[0:3])


    # How much does our predicted RT differ from what we observed?
    residual = obs - predictedFeaturePosRT(x[0:3], xPredict[obsIndex:obsIndex+2])



    # This is the GSigmaG^T component
    covPredict = dot(G,dot(cov,G.transpose()))

    # And then...Add the driving noise to get the full G*Sigma*G^T + R*Sigma_rt*R^T
    covPredict = covPredict + dot(drivingR,dot(sigmaObs,drivingR.transpose()))


    #####################
    # FINALLY - the big 3
    #####################

    # Compute Kalman Gain
    S = dot(J, dot(covPredict, J.transpose())) + sigmaObs
    K = dot(covPredict, dot(J.transpose(), inv(S)))

    # Compute new mean vector
    newX = xPredict + dot(K, residual)

    # Compute new covariance
    newCov = covPredict - dot(K, dot(J, covPredict))

    return newX, newCov



if __name__ == "__main__":
    """
    Runs the two Kalman updates that are specified in PS3 task 1
    """

    # Make printing a more enjoyable experience
    np.set_printoptions(suppress=True, precision=3)

    # Let's visualize this
    font = "sans-serif"
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_ylim(-10, 18)
    ax.set_xlim(-10, 18)
    grid(True)



    # Given state vector
    x = array([3,2, pi, 12, 15]);

    # Given covariance
    cov = vstack([array([4, 1, 2, 1, 1]),
                  array([1, 6, 3, 1, 2]),
                  array([2, 3, 4, 1, 2]),
                  array([1, 1, 1, 8, 1]),
                  array([1, 2, 2, 1, 10])])

    arrowLength = 0.5
    ax.arrow(x[0],x[1], arrowLength*cos(x[2]),arrowLength*sin(x[2]), width=0.1)
    hold(True)
    u0 = ax.scatter(12,15,s=80,marker=(5,1),edgecolor='g', color='g')

    print "Original state vector\n", x, "\n\n"
    print "Original covariance\n", cov, "\n\n"

    # The two observations
    z1 = array([10, 0.5*pi])
    z2 = array([11, 0.6*pi])

    ###########
    # FIRST UPDATE
    ###########

    x2,cov2 = kalmanUpdate(x,cov,z1,-1)

    print "Updated state vector\n", x2, "\n\n"
    print "Updated covariance\n", cov2, "\n\n"

    ax.arrow(x2[0],x2[1], arrowLength*cos(x2[2]),arrowLength*sin(x2[2]), width=0.1, color='r')
    scatter(x2[3],x2[4],s=80,marker=(5,1),edgecolor='r', color='r')
    u1 = ax.scatter(x2[5],x2[6],s=80,marker=(5,1),edgecolor='r', color='r')
    ###########
    # SECOND UPDATE
    ###########

    # We are manually specifying here that we know where the feature is in the state vector.
    # In the actual assignment, we'll have to do this data association in the kalmanUpdate
    # function.
    x3,cov3 = kalmanUpdate(x2,cov2,z2,5)


    print "Updated state vector\n", x3, "\n\n"
    print "Updated covariance\n", cov3, "\n\n"

    ax.arrow(x3[0],x3[1], arrowLength*cos(x3[2]),arrowLength*sin(x3[2]), width=0.1, color='b')
    scatter(x3[3],x3[4],s=80,marker=(5,1),edgecolor='b', color='b')
    u2 = ax.scatter(x3[5],x3[6],s=80,marker=(5,1),edgecolor='b', color='b')

    ax.legend((u0,u1,u2),('Initial State', 'First Update', 'Second Update'), loc='upper left')
    text(0, 7, 'Initial state hidden underneath first update', fontsize=12)
    ax.set_title('PS3 Task 1 Results')

