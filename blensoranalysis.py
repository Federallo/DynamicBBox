from _blensor_analysis import pointcloud as pc
import pointcloud

def generateBoundingBoxes(nSensors, i):
    pointCloud = []
    pointCloud.append(pointcloud.generatePointClouds(nSensors, i))
    bounding_box = pc.clusterData(pointCloud)
    bounding_box = bounding_box[0] # because we will display the bounding boxes of only one scan, and blensor_analysis returns a list of bounding boxes
    return bounding_box
