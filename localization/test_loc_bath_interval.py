from vibes import vibes
import pyibex
from pyibex import sqr
from pyibex.image import *
import numpy as np
import time

def getDepth(X):
    x, y = X[0,0], X[1,0]
    h = 10*np.exp(-((x/500)**2 + (y/500)**2)) + 6*np.exp(-((x+600)/300)**2 - ((y-400)/300)**2) - 20
    return h

def f(X, u):
    """kinematic model -- Xn+1 = Xn + dt*f(Xn, u)"""
    xDot = X[3,0]*np.cos(X[2,0])
    yDot = X[3,0]*np.sin(X[2,0])
    vDot = u[0,0]
    thetaDot = u[1,0]
    return np.array([[xDot,yDot,thetaDot, vDot]]).T


def getCtc(bathy, h0, dh):
    ### generate the slice image for an altitude h0 ###
    hBin = ((bathy < h0 + dh) * (bathy > h0 - dh)).astype(int)

    ### create the contractor associated with the slice
    hBin = np.flipud(hBin)  # flip the slice (the origin for an image is the top-left corner)
    hOut = hBin.cumsum(0).cumsum(1) # compute the integral image
    ctc = CtcRaster(hOut.T, -1000, 1000, 4, -4)

    return ctc



if __name__ == '__main__':

    ### generate the map ###
    x = np.linspace(-1000, 1000, 500)
    y = np.linspace(-1000, 1000, 500)
    x, y = np.meshgrid(x, y)
    bathy = 10*np.exp(-((x/500)**2 + (y/500)**2)) + 6*np.exp(-((x+600)/300)**2 - ((y-400)/300)**2) - 20
    

    X = np.array([[0, 0, 0, 50]], dtype=np.float64).T  # [x, y, theta, v].T

    dh = 0.5

    t0 = 0
    tmax = 60
    dt = 0.1  # step of the simulation
    T = np.arange(t0, tmax, dt)
    
    tm = 0.3  # period between two SIVIAs
    i = 0  # number of SIVIAs already computed
    
    P = pyibex.IntervalVector(2, [-1000,1000])  # SIVIA init box
    vibes.beginDrawing()

    for t in T:

        print(t)

        u = np.array([[0,0.3]]).T  # command
        X += dt*f(X, u)  # Euler's scheme
        if t > tm*i:  # SIVIA
            h = getDepth(X)
            if i == 0:  # first SIVIA
                ctc = getCtc(bathy, h, dh)
            else:
                fPred = pyibex.Function('x', 'y', '(x - {0}*{1}*cos({2}), y - {0}*{1}*sin({2}))'.format(tm, X[3,0], X[2,0]))
                ctc_pred = pyibex.CtcInverse(ctc, fPred)
                ctc_corr = getCtc(bathy, h, dh)
                ctc = ctc_pred & ctc_corr
                
            pyibex.pySIVIA(P, ctc, 30)
            vibes.drawCircle(*X[:2,0].flatten().tolist(),25,"[red]")  # draw robot
            
            i+=1
        
        time.sleep(dt)

    vibes.endDrawing()