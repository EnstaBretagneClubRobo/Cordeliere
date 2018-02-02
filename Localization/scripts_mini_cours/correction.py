###############################################

# First test of a interval state observer in
# a scenario where the robot is performing a circle
# at a constant speed

###############################################


from vibes import vibes
import pyibex
from pyibex import sqr
import numpy as np
import time


def f(X, u):
    """kinematic model -- Xn+1 = Xn + dt*f(Xn, u)"""
    xDot = X[3,0]*np.cos(X[2,0])
    yDot = X[3,0]*np.sin(X[2,0])
    vDot = u[0,0]
    thetaDot = u[1,0]
    return np.array([[xDot,yDot,thetaDot, vDot]]).T


def getDistanceFromLandmark(X):
    norm = np.linalg.norm(a - X[:2,:])
    noise = 0.2
    return pyibex.Interval(norm - noise, norm + noise)


if __name__ == '__main__':
    
    X = np.array([[0, 0, 0, 0.5]]).T  # [x, y, theta, v].T
    a = np.array([[-5, -5]]).T  # landmark

    t0 = 0
    tmax = 60
    dt = 0.1  # step of the simulation
    T = np.arange(t0, tmax, dt)
    
    tm = 0.3  # period between two SIVIAs
    i = 0  # number of SIVIAs already computed
    
    P = pyibex.IntervalVector(2, [-20,20])  # SIVIA init box
    vibes.beginDrawing()

    vibes.newFigure("correction")
    time.sleep(2)

    for t in T:
        print(t)
        
        u = np.array([[0,0.3]]).T  # command
        X += dt*f(X, u)  # Euler's scheme
        if t > tm*(i+1):  # SIVIA
            d = getDistanceFromLandmark(X)
            fCorr = pyibex.Function("x","y","({0}-x)^2 + ({1}-y)^2".format(a[0,0], a[1,0]))
            if i == 0:  # first SIVIA
                ctc = pyibex.CtcFwdBwd(fCorr, sqr(d))
            else:
                fPred = pyibex.Function('x', 'y', '(x - {0}*{1}*cos({2}), y - {0}*{1}*sin({2}))'.format(tm, X[3,0], X[2,0]))
                ctc_pred = pyibex.CtcInverse(ctc, fPred)
                ctc = ctc_pred
                # ctc_corr = pyibex.CtcFwdBwd(fCorr, sqr(d))
                # ctc = ctc_pred & ctc_corr
                
            pyibex.pySIVIA(P, ctc, 0.5)
            vibes.drawCircle(*X[:2,0].flatten().tolist(),0.4,"[red]")  # draw robot
            vibes.drawCircle(*a[:2,0].flatten().tolist(),0.4,"[black]")  # draw landmark
            
            i+=1
        
        time.sleep(dt)
    






