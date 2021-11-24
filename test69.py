#Author: Ajs R. Stormholt
import cv2

import open3d as o3d
import vaerksValidator
import copy
    
main = vaerksValidator.App(r"C:\Users\Stormholt\Documents\Thesis")
model ="vice-maho-v2.ply"
table = "table_under_origo_dense.ply"

#main.loadPointClouds(15)
try:
    main.model = o3d.io.read_point_cloud(main.pcd_path + model )
    main.table = o3d.io.read_point_cloud(main.pcd_path + table)
###################Test position v1    
    #main.camera.updatePosition(170,110,25.5)

################### Test position v2:
    #Bottomplate : 0, 0 , 30
    #Vice : 
    #Camera : 170 , 100, 415
    #main.camera.updatePosition(170, 100, 41.5)
################### Test position v3:
    #Bottomplate : 0, 0 , 30
    #Vice : 
    #Camera : 170 , 60, 415
    #main.camera.updatePosition(170, 60, 41.5)
  ################### Test position v4:
     #Bottomplate : 0, 0 , 30
    #Vice : 
    #Camera : 170 , 140, 415
    #main.camera.updatePosition(170, 140, 41.5)

  ################### Test position v5:
     #Bottomplate : 0, 0 , 30
    #Vice : 
    #Camera : 210 , 140, 415
    #main.camera.updatePosition(210, 140, 41.5)
    ################### Test position v6:
     #Bottomplate : 0, 0 , 30
    #Vice : 
    #Camera : 170 , 140, 395
   # main.camera.updatePosition(170, 140, 39.5)
    while True:
        main.camera.stream()
        key = cv2.waitKey(1)
        if key == 27:
            break
        if key == 32:
            #main.captureScenePointcloud()
            #main.camera.y += -4.0
           # main.camera.snapshot()
            #main.camera.generatePLY()
            #main.camera.generateOpen3DPLY()
           main.generatePointCloud(main.Filetype.PLY,"perspective-test")
finally:
    #main.genTransMothership()
    #scene = o3d.io.read_point_cloud(main.pcd_path +"notmiwire_scene_2.ply")
   # main.saveScenePointcloud(main.Filetype.PLY,"scene-vice-maho")
    model_color = copy.deepcopy(main.model)
    model_color.paint_uniform_color([1., 0.706, 0.])
    o3d.visualization.draw_geometries([main.scene.pcd])# + model_color])
    #main.compareScene2Model()
    main.camera.release()




 
