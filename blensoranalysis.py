from _blensor_analysis import pointcloud as pc
import pointcloud

def generateBoundingBoxes(nSensors, i):
    pointCloud = []
    pointCloud.append(pointcloud.generatePointClouds(nSensors, i))
    bounding_box = pc.clusterData(pointCloud, True)
    bounding_box = bounding_box[0] # because the blensor_analysis function requires a list of pointclouds and not only one and, because of that, it returns a list of bounding boxes
    return bounding_box

def generateBB(pointcloud):
    pointCloud = []
    pointCloud.append(pointcloud)
    bounding_box = pc.clusterData(pointCloud, False)
    return bounding_box[0]
    
