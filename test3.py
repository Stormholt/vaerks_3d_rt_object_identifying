from vaerksValidator import *
import copy

main = App("C:/Users/Stormholt/Documents/Thesis", False)
model ="comp-test-model.ply"
table = "table_under_origo_dense.ply"
main.model = o3d.io.read_point_cloud(main.pcd_path + model )
main.table = o3d.io.read_point_cloud(main.pcd_path + table)
model_color = copy.deepcopy(main.model)
model_color.paint_uniform_color([1., 0.706, 0.])
main.scene.pcd = o3d.io.read_point_cloud(main.pcd_path+ "goodfit-processed_0.ply")

o3d.visualization.draw_geometries([main.scene.pcd + model_color])
#main.scene.filterBackground()
#main.scene.filterPcloud(main.scene.pcd,main.model)
#main.scene.filterModelbbox(main.scene.pcd,main.model)
#main.saveScenePointcloud(main.Filetype.PLY,"goodfit-processed")
#ain.scene.processPcloud(main.model,main.table)
#o3d.visualization.draw_geometries([main.scene.pcd + model_color])
#main.compareScene2Model()
main.compareScene2ModelBbox()


