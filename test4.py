from vaerksValidator import *
import copy
import pcloud

main = App("C:/Users/Stormholt/Documents/Thesis", False)
model ="comp-test-model-big.ply"
table = "table_under_origo_dense.ply"
main.model = o3d.io.read_point_cloud(main.pcd_path + model )
main.table = o3d.io.read_point_cloud(main.pcd_path + table)
model_color = copy.deepcopy(main.model)
model_color.paint_uniform_color([1., 0.706, 0.])
#main.scene.pcd = o3d.io.read_point_cloud(main.pcd_path+ "goodfit-matrox-processed_0.ply")
main.scene.pcd = o3d.io.read_point_cloud(main.pcd_path+ "goodfit-processed_0.ply")
o3d.visualization.draw_geometries([main.scene.pcd + model_color])
#main.scene.filterBackground()
#main.scene.filterPcloud(main.model, pcloud.TABLE_FILTER_THRESHOLD)
#main.scene.filterModelbbox(main.model)
#main.saveScenePointcloud(main.Filetype.PLY, "goodfit-matrox-processed")
#main.scene.processMatroxPcloud(main.model,main.table)
#main.scene.filterBackground()
#main.scene.filterModelbbox(main.model)
#main.saveScenePointcloud(main.Filetype.PLY, "matrox-comp-test-scene0_r")
#o3d.visualization.draw_geometries([main.scene.pcd + model_color])
main.compareScene2Model()
main.compareScene2ModelBbox()


