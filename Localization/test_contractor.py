from vibes import vibes
import pyibex
from pyibex import sqr
import numpy as np
import time

import os
import psutil




def f(X, u):
    """kinematic model -- Xn+1 = Xn + dt*f(Xn, u)"""
    xDot = X[3,0]*np.cos(X[2,0])
    yDot = X[3,0]*np.sin(X[2,0])
    vDot = u[0,0]
    thetaDot = u[1,0]
    return np.array([[xDot,yDot,thetaDot, vDot]]).T


class myCtc(pyibex.CtcFwdBwd):
    def __init__(self, fPred, itv):
        super().__init__(fPred, itv)

    def contract(self, itv):
        print("coucou")
        super().contract(itv)


if __name__ == '__main__':

    ### generate the map ###

    X = np.array([[0, 0, 0, 1]], dtype=np.float64).T  # [x, y, theta, v].T


    dh = 0.5
    dv = 1
    dtheta = 0.01

    t0 = 0
    tmax = 60
    dt = 0.1  # step of the simulation
    T = np.arange(t0, tmax, dt)
    
    tm = 1  # period between two SIVIAs
    i = 0  # number of SIVIAs already computed
    
    P = pyibex.IntervalVector(2, [-10,10])  # SIVIA init box
    vibes.beginDrawing()

    for t in T:

        print(t)

        """measurement step"""
        # velocity
        v_noise = dv * (2*np.random.rand() - 1)
        v_measured = X[3,0] + v_noise
        v = pyibex.Interval(v_measured - dv, v_measured + dv)

        # heading
        theta_noise = dtheta * (2*np.random.rand() - 1)
        theta_measured = X[2,0] + theta_noise
        theta = pyibex.Interval(theta_measured - dtheta, theta_measured + dtheta)

        u = np.array([[0,0.3]]).T  # command
        X += dt*f(X, u)  # Euler's scheme
        
        """localization step"""
        if t > tm*i:
            if i == 0:  # first SIVIA
                fPred = pyibex.Function("x","y","({0}-x)^2 + ({1}-y)^2".format(2, 2))
                ctc = myCtc(fPred, pyibex.Interval(0,1))
            else:
                # translation of the previous contractor because the robot has moved
                fPred = pyibex.Function('x', 'y', '(x - {0}*{1}*cos({2}), y - {0}*{1}*sin({2}))'.format(tm, v, theta))
                ctc = pyibex.CtcInverse(ctc, fPred) 

            process = psutil.Process(os.getpid())
            print(process.memory_info().rss)
            pyibex.pySIVIA(P, ctc, 1)
            vibes.drawCircle(*X[:2,0].flatten().tolist(),25,"red")  # draw robot
            
            i+=1
        
        time.sleep(dt*2)

    vibes.endDrawing()