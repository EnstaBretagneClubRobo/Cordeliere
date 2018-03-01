from vibes import vibes
import pyibex
from pyibex import *
import numpy as np
import time


def f(X, u):
    """kinematic model -- Xn+1 = Xn + dt*f(Xn, u)"""
    xDot = X[3,0]*np.cos(X[2,0])
    yDot = X[3,0]*np.sin(X[2,0])
    vDot = u[0,0]
    thetaDot = u[1,0]
    return np.array([[xDot,yDot,thetaDot, vDot]]).T


if __name__ == '__main__':

    X = np.array([[0, 0, 0, 1]], dtype=np.float64).T  # [x, y, theta, v].T

    dv = 0.5
    dtheta = 0.01

    t0 = 0
    tmax = 60
    dt = 0.1  # step of the simulation
    T = np.arange(t0, tmax, dt)
    
    tm = 1  # period between two SIVIAs
    i = 0  # number of SIVIAs already computed
    
    P = IntervalVector(2, [-20,20])  # SIVIA init box
    vibes.beginDrawing()

    vibes.newFigure("prediction")
    time.sleep(2)

    for t in T:

        print(t)

        """measurement step"""
        # velocity
        v_noise = dv * (2*np.random.rand() - 1)
        v_measured = X[3,0] + v_noise
        v = Interval(v_measured - dv, v_measured + dv)

        # heading
        theta_noise = dtheta * (2*np.random.rand() - 1)
        theta_measured = X[2,0] + theta_noise
        theta = Interval(theta_measured - dtheta, theta_measured + dtheta)

        u = np.array([[0,0.3]]).T  # command
        X += dt*f(X, u)  # Euler's scheme
        
        """localization step"""
        if t > tm*i:
            if i == 0:  # first SIVIA
                fPred = Function("x","y","({0}-x)^2 + ({1}-y)^2".format(X[0,0], X[1,0]))
                ctc = CtcFwdBwd(fPred, Interval(0,1))
            else:
                # translation of the previous contractor because the robot has moved
                fPred = Function('x', 'y', '(x - {0}*{1}*cos({2}), y - {0}*{1}*sin({2}))'.format(tm, v, theta))
                ctc = CtcInverse(ctc, fPred) 

            pySIVIA(P, ctc, 0.5)
            vibes.drawCircle(*X[:2,0].flatten().tolist(),0.4,"[red]")  # draw robot
            
            i+=1
        
        time.sleep(dt)

    vibes.endDrawing()