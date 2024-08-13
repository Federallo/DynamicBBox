from _blensor_analysis import dataset_loader
import open3d as o3d

# generating pointcloud
# for now pointclouds will be generated filtered and without noise
def generatePointClouds(pointClouds, nSensors, i):

    pointclouds = dataset_loader.loadscan(nSensors, i, 0)
    
    pcds = []
    for pointcloud in pointclouds:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pointclouds)
        pcds.append(pcd)
    
    # combining pointcluds and removing duplicate coordinates
    merged_pcd = o3d.geometry.PointCloud()

    for pcd in pcds:
        for pt in pcd.points:
            if pt not in merged_pcd.points:
                merged_pcd.points.append(point)

    # returning filetered pointcloud
    filtered_pcd = merged_pcd.select_by_index([i for i in range(len(merged_pcd.points)) if merged_pcd.points[i][2] > 0.3])
    return filtered_pcd.voxel_down_sampe(voxel_size = 0.25)
