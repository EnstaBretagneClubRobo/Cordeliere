import numpy as np
import matplotlib.pyplot as plt

bathy_map = np.loadtxt(open("MNT_ATL100m_HOMONIM_GEO_refNM_ZNEG254487_kriging_Zhat.csv", "rb"),
                       delimiter=",", skiprows=1)

plt.matshow(bathy_map, cmap=plt.get_cmap('jet'))
plt.colorbar()
plt.show()