import os
import sys
import math
import operator
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib import colors

import cloudComPy as cc                                                # import the CloudComPy module
cc.initCC()                                                            # to do once before dealing with plugins

#model = cc.loadPointCloud(r"C:\Users\Stormholt\Desktop\ring-table-test-setupv6.ply")
model = cc.loadPointCloud(r"C:\Users\Stormholt\Desktop\notmiwire.ply")
scene = cc.loadPointCloud("normals000.ply")#,cc.CC_SHIFT_MODE.XYZ)

#scene = cc.loadPointCloud(r"C:\Users\Stormholt\Desktop\ring-table-test-setupv6.ply")#,cc.CC_SHIFT_MODE.XYZ)

model_octree = model.computeOctree()
scene_octree = scene.computeOctree()


#print(cc.DistanceComputationTools.computeApproxCloud2CloudDistance(scene, model, 6))


# Cloud-to-cloud “Hausdorff” distance computation
params = cc.Cloud2CloudDistancesComputationParams() # parameters only for hausdorff
params.maxThreadCount=4
#params.octreeLevel=6
print(cc.DistanceComputationTools.computeCloud2CloudDistances(scene, model, params))

matplotlib.use('agg') # png images

sf=scene.getScalarField(0)
asf= sf.toNpArray()

(n, bins, patches) = plt.hist(asf, bins=256, density=1) # histogram for matplotlib
fracs = bins / bins.max()
norm = colors.Normalize(fracs.min(), fracs.max())
for thisfrac, thispatch in zip(fracs, patches):
    color = plt.cm.rainbow(norm(thisfrac))
    thispatch.set_facecolor(color)

plt.savefig("histogram.png")

cc.SaveEntities([scene, model], "ring-scene0.bin")