Write-Host "Congratulations! Your first script executed successfully"


"""     def generatePCD(self):
        wtf = Pcloud(self.x,self.y,self.z)
        wtf.pcd = o3d.geometry.PointCloud.create_from_depth_image(self.camera.depth_frame_open3d,self.camera.intrinsic) # Creating pointcloud from depth 
        wtf.process_point_cloud(voxel_size, depth_threshold)
        wtf.updateTvector()
        wtf.pcd_down.translate(wtf.tvector)
        self.pcloud_array.append(wtf)

        self.savetoPCD(pcd_path, str(self.pcds_generated), wtf)
        self.pcds_generated += 1

    def generatePLY(self):
        filename = str(self.plys_generated) + ".ply"
        wtf = Pcloud(0,0,0)
        wtf.pcd = o3d.geometry.PointCloud.create_from_depth_image(self.depth_frame_open3d,self.intrinsic) # Creating pointcloud from depth 
        wtf.process_point_cloud(voxel_size, depth_threshold, self.model, self.table)
        o3d.io.write_point_cloud(filename,wtf.pcd)
        self.plys_generated += 1 """