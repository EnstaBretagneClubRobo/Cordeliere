##########################

# First test of the computation 
# of a contractor associated with
# a slice of the map for an given
# altitude

##########################


import numpy as np
import matplotlib.pyplot as plt
from pyibex import *
from pyibex.image import *
from vibes import vibes

def getCtc(h, h0, dh):
    ### generate the slice image for an altitude h0 ###
    hBin = ((h < h0 + dh) * (h > h0 - dh)).astype(int)

    ### create the contractor associated with the slice
    hBin = np.flipud(hBin)  # flip the slice (the origin for an image is the top-left corner)
    hOut = hBin.cumsum(0).cumsum(1) # compute the integral image
    ctc = CtcRaster(hOut.T, -1000, 1000, 4, -4)

    return ctc

### generate the map ###
x = np.linspace(-1000, 1000, 500)
y = np.linspace(-1000, 1000, 500)
x, y = np.meshgrid(x, y)
h = 10*np.exp(-((x/500)**2 + (y/500)**2)) + 6*np.exp(-((x+600)/300)**2 - ((y-400)/300)**2) - 20

plt.subplot(121)
cont = plt.contour(x, y, h, 10)
plt.clabel(cont, inline=1, fontsize=8)
plt.axis('equal')

h0 = -17
dh = 0.3
### generate the slice image for an altitude h0 ###
hBin = ((h < h0 + dh) * (h > h0 - dh)).astype(int)


plt.subplot(122)
plt.imshow(hBin, extent=[-1000, 1000, -1000, 1000], origin='lower')
plt.title("h0 = " + str(h0) + ", dh = " + str(dh))
plt.show()

### create the contractor associated with the slice
hBin = np.flipud(hBin)  # flip the slice (the origin for an image is the top-left corner)
hOut = hBin.cumsum(0).cumsum(1) # compute the integral image
ctc = CtcRaster(hOut.T, -1000, 1000, 4, -4)

### results on Vibes ###
vibes.beginDrawing()
vibes.newFigure('CtcImage')
P = IntervalVector(2, [-1000, 1000])
pySIVIA(P, ctc, 30)
vibes.axisEqual()
