
import open3d as o3d
from vaerksValidator import *
import numpy as np
import copy

app = App(r"C:\Users\Stormholt\Documents\Thesis", False)
model ="comp-test-model.ply"
table = "table_under_origo_dense.ply"
scene= "matrox-comp-test-scene0_t.ply"
print("1")
app.model  = o3d.io.read_point_cloud(app.pcd_path+ model)
print("2")
app.scene.pcd =o3d.io.read_point_cloud(app.pcd_path + scene)
print("3")
app.table = o3d.io.read_point_cloud(app.pcd_path + table)
print("4")
app.scene.process_point_cloud(app.model,app.table)
print("5")
app.saveScenePointcloud(app.Filetype.PLY,"comp-test-scene0_p")

#model_color = copy.deepcopy(app.model)
##model_color.paint_uniform_color([1., 0.706, 0.])
#o3d.visualization.draw_geometries([app.scene.pcd + model_color])
#app.compareScene2Model()