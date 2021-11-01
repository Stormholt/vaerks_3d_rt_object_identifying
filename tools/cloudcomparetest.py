import os
import sys
import math
import operator
import cloudComPy as cc                                                # import the CloudComPy module
cc.initCC()                                                            # to do once before dealing with plugins

model = cc.loadPointCloud(r"converted\sbt_pcd.ply")
scene = cc.loadPointCloud(r"pointclouds\10.pcd")#,cc.CC_SHIFT_MODE.XYZ)
print(scene)
model_zero_coord = model.getPoint(0)
scene_zero_coord = scene.getPoint(0)

scene.translate(tuple(map(operator.sub, model_zero_coord, scene_zero_coord)))

model_octree = model.computeOctree()
scene_octree = scene.computeOctree()

model_boundingbox =  model_octree.getBoundingBox()
scene_boundingbox = scene_octree.getBoundingBox()
""" model_boundingbox =  cc.ReferenceCloud(model).getBoundingBox()
scene_boundingbox = cc.ReferenceCloud(scene).getBoundingBox() """

scale = (model_boundingbox[1][0] )#-scene_boundingbox[1][0])# model_boundingbox[0][0]) #- (scene_boundingbox[1][0] - scene_boundingbox[0][0])
scene.scale(scale,scale,scale)
model_zero_coord = model.getPoint(0)
scene_zero_coord = scene.getPoint(0)

print("number of scalar fields:")
print(model.getNumberOfScalarFields() )
print(scene.getNumberOfScalarFields() )

scene.translate(tuple(map(operator.sub, model_zero_coord, scene_zero_coord)))
#scene.translate(tuple(map(operator.sub, model_boundingbox[0],scene_boundingbox[0])))
#cc.SaveEntities([scene, model], "scene2model.bin")

"""cloud = cc.loadPointCloud(r"pointclouds\0.pcd") 
 mesh = cc.loadMesh(r"referenceModels\component_1.stl")

params = cc.Cloud2MeshDistancesComputationParams()
params.maxThreadCount=12
params.octreeLevel=6
cc.DistanceComputationTools.computeCloud2MeshDistances(cloud, mesh, params)

# --- save distances histogram, png file

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib import colors
matplotlib.use('agg') # png images

sf=cloud.getScalarField(0)
asf= sf.toNpArray()

(n, bins, patches) = plt.hist(asf, bins=256, density=1) # histogram for matplotlib
fracs = bins / bins.max()
norm = colors.Normalize(fracs.min(), fracs.max())
for thisfrac, thispatch in zip(fracs, patches):
    color = plt.cm.rainbow(norm(thisfrac))
    thispatch.set_facecolor(color)

plt.savefig("histogram.png")

# --- save distances histogram, csv file

cc.SaveEntities([cloud, mesh], "cloudMesh.bin") """
