#Author: Ajs R. Stormholt
import cv2

import open3d as o3d
import vaerksValidator
import copy
    
main = vaerksValidator.App(r"C:\Users\Stormholt\Documents\Thesis", True)
model ="comp-test-model.ply"
table = "table_under_origo_dense.ply"

try:
    main.model = o3d.io.read_point_cloud(main.pcd_path + model)
    main.table = o3d.io.read_point_cloud(main.pcd_path + table)
    
    main.camera.x= 125# - 17.5
    main.camera.y = 82
    main.camera.z = -24 #+ main.camera.depth_startpoint_offset #265
    
    while True:
        main.camera.stream(True, False)
        key = cv2.waitKey(1)
        if key == 27:
            break
        if key == 32:
            print("hello")
            main.captureScenePointcloud()
            print("hello 0")
            print(main.plys_generated)
            main.camera.y += 4.0
finally:
    main.saveScenePointcloud(main.Filetype.PLY,"09_01_2021_test_inverted")
    model_color = copy.deepcopy(main.model)
    model_color.paint_uniform_color([1., 0.706, 0.])
    o3d.visualization.draw_geometries([main.scene.pcd + model_color])
    main.compareScene2Model()
    main.compareScene2ModelBbox()
    main.camera.release()




 
