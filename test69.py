#Author: Ajs R. Stormholt
import cv2

import open3d as o3d
import vaerksValidator
import copy
    
main = vaerksValidator.App(r"C:\Users\Stormholt\Documents\Thesis")
model ="notmiwire_pos10_10_0.ply"
table = "table_under_origo_dense.ply"

#main.loadPointClouds(15)
try:
    main.model = o3d.io.read_point_cloud(main.pcd_path + model )
    main.table = o3d.io.read_point_cloud(main.pcd_path + table)
    while True:
        main.camera.stream()
        key = cv2.waitKey(1)
        if key == 27:
            break
        #if key == 32:
           # main.camera.snapshot()
            #main.camera.generatePLY()
            #main.camera.generateOpen3DPLY()
            #main.generatePointCloud(main.Filetype.PLY,"notmiwire_scene")
finally:
    #main.genTransMothership()
    scene = o3d.io.read_point_cloud(main.pcd_path +"notmiwire_scene_2.ply")
    
    model_color = copy.deepcopy(main.model)
    model_color.paint_uniform_color([1., 0.706, 0.])
    o3d.visualization.draw_geometries([scene ])#+ model_color])
    main.compareScene2Model()
    main.camera.release()




 
