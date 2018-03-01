import sys
import os
import numpy as np

# load the map
if len(sys.argv) != 2:
    raise IOError("input : filename of the map")

mapFileName = sys.argv[1]
bathy_map = np.loadtxt(open(mapFileName, "rb"),
                       delimiter=",")

# flatten the map and save
mapFlattenFileName = mapFileName.split('.')[0] + "_flatten.csv"
np.savetxt(mapFlattenFileName, -bathy_map.flatten(), fmt='%.2f')
