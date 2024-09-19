from _blensor_analysis import pointcloud as pc
from _blensor_analysis import dataset_loader
import open3d as o3d
import numpy as np

# generating pointcloud
# for now pointclouds will be generated filtered and without noise
# NOTE: pointclouds can be generated in those ways: no noise|filtered
def generatePointClouds(nSensors, i):

    pointclouds = dataset_loader.loadscan(nSensors, i, 0)

    
    pcds = []
    
    for pointcloud in pointclouds:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pointcloud)
        pcds.append(pcd)
    
    # combining pointcluds and removing duplicate coordinates
    merged_pcd = o3d.geometry.PointCloud()

    
    for pcd in pcds:
        for pt in pcd.points:
            if pt not in merged_pcd.points:
                merged_pcd.points.append(pt)

    numPoints = len(merged_pcd.points)
    blackColors = np.zeros((numPoints, 3))
    merged_pcd.colors = o3d.utility.Vector3dVector(blackColors)
    
    # returning filetered pointcloud
    filtered_pcd = merged_pcd.select_by_index([i for i in range(len(merged_pcd.points)) if merged_pcd.points[i][2] > 0.3])
    return filtered_pcd.voxel_down_sample(voxel_size = 0.25)


def generateBoundingBoxes(nSensors, i):
    pointCloud = []
    pointCloud.append(generatePointClouds(nSensors, i))
    bounding_box = pc.clusterData(pointCloud)
    bounding_box = bounding_box[0] # because the blensor_analysis function requires a list of pointclouds and not only one and it returns a list of bounding boxes
    return bounding_box
