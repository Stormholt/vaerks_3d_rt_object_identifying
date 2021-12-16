
import open3d as o3d
import vaerksValidator
import copy

main = vaerksValidator.App(r"C:\Users\Stormholt\Documents\Thesis")
model ="vice-maho-v3.ply"
table = "table_under_origo_dense.ply"

try:
    main.model = o3d.io.read_point_cloud(main.pcd_path + model )
    main.table = o3d.io.read_point_cloud(main.pcd_path + table)

finally:
    #main.genTransMothership()
    #scene = o3d.io.read_point_cloud(main.pcd_path +"notmiwire_scene_2.ply")
    print("ONE")
    main.scene.pcd = o3d.io.read_point_cloud(main.pcd_path +"vice-scene-setupv2_0.ply")
    main.scene.tvector = [170, 100*0.7, 60.5]
    main.scene.process_point_cloud(main.model, main.table)
    main.saveScenePointcloud(main.Filetype.PLY,"scene-vice-maho-v2")
    model_color = copy.deepcopy(main.model)
    model_color.paint_uniform_color([1., 0.706, 0.])
    o3d.visualization.draw_geometries([main.scene.pcd + model_color])
    main.compareScene2Model()
    
    print("TWO")
    main.scene.pcd = o3d.io.read_point_cloud(main.pcd_path +"vice-scene-setupv3_0.ply")
    main.scene.tvector = [170, 60, 60.5]
    main.scene.process_point_cloud(main.model, main.table)
    main.saveScenePointcloud(main.Filetype.PLY,"scene-vice-maho-v3")
    o3d.visualization.draw_geometries([main.scene.pcd + model_color])
    main.compareScene2Model()
    
    print("THREE")
    main.scene.pcd = o3d.io.read_point_cloud(main.pcd_path +"vice-scene-setupv4_0.ply")
    main.scene.tvector = [170, 140, 41.5]
    main.scene.process_point_cloud(main.model, main.table)
    main.saveScenePointcloud(main.Filetype.PLY,"scene-vice-maho-v4")
    o3d.visualization.draw_geometries([main.scene.pcd + model_color])
    main.compareScene2Model()

    print("FOUR")
    main.scene.pcd = o3d.io.read_point_cloud(main.pcd_path +"vice-scene-setupv5_0.ply")
    main.scene.tvector = [210, 140, 41.5]
    main.scene.process_point_cloud(main.model, main.table)
    main.saveScenePointcloud(main.Filetype.PLY,"scene-vice-maho-v5")
    o3d.visualization.draw_geometries([main.scene.pcd + model_color])
    main.compareScene2Model()

    print("FIVE")
    main.scene.pcd = o3d.io.read_point_cloud(main.pcd_path +"vice-scene-setupv6_0.ply")
    main.scene.tvector = [170, 140, 39.5]
    main.scene.process_point_cloud(main.model, main.table)
    main.saveScenePointcloud(main.Filetype.PLY,"scene-vice-maho-v6")
    o3d.visualization.draw_geometries([main.scene.pcd + model_color])
    main.compareScene2Model()

    main.camera.release()