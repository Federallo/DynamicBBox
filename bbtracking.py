import pointcloud as pc
import customdbscan
import open3d as o3d
import numpy as np
import blensoranalysis

# NOTE: the bounding boxes of the custom cluster must be OrientedBoudingBox type, the same as the ones in blensory_analysis repo

def findPointsOutsideBB(pointcloud, clusters):
    
    # creating a mask to remove all the existing clusters from pointcloud
    set1 = set(tuple(point) for point in pointcloud.points)
    if clusters:
        set2 = set(tuple(point) for cluster in clusters for point in cluster.points)
        points = set1 - set2
        
        remainingPoints = np.array([np.array(point) for point in points])
        if remainingPoints.any():
            pointcloud = o3d.geometry.PointCloud()
            pointcloud.points = o3d.utility.Vector3dVector(remainingPoints)

        return pointcloud
        


def updateBB(bbs, nSensor, i):

    # loading the next scan
    pointcloud = pc.generatePointClouds(nSensor, i+1)

    # updating every single bounding box
    newBB = []
    newClusters = []
    for bb in bbs:

        boxes, clusters = customdbscan.customDBSCAN(pointcloud, bb, 1.5, 5)#FIXME bounding box not displayed correctly

        if boxes:
            for box in boxes:
                newBB.append(box)
            for cluster in clusters:
                newClusters.append(cluster)
    #TODO
    '''
    remainingPoints = findPointsOutsideBB(pointcloud, newClusters)
    discoveredBoxes = []
    if len(remainingPoints.points) > 10:
        discoveredBoxes = blensoranalysis.generateBB(remainingPoints)
        for discovederedBox in discoveredBoxes:
            newBB.append(discovederedBox)

    return newBB, newClusters # pointcloud
    '''

def displayBoundingBoxes(bbs, pointcloud):
    o3d.visualization.draw([*pointcloud, *bbs], show_skybox=False) #TODO remove *
