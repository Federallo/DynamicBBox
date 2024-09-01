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

def removeOverlappingBoxes(bbs):
    
    for box1 in bbs:
        boxCorners1 = box1.get_box_points()
        for box2 in bbs:
            boxCorners2 = box2.get_box_points()
            xmin_overlap = max(np.min(np.asarray(boxCorners1)[:,0]), np.min(np.asarray(boxCorners2)[:,0]))
            xmax_overlap = min(np.max(np.asarray(boxCorners1)[:,0]), np.max(np.asarray(boxCorners2)[:,0]))
            ymin_overlap = max(np.min(np.asarray(boxCorners1)[:,1]), np.min(np.asarray(boxCorners2)[:,1]))
            ymax_overlap = min(np.max(np.asarray(boxCorners1)[:,1]), np.max(np.asarray(boxCorners2)[:,1]))


            # checking if there i overlap between the two boxes
            if xmin_overlap < xmax_overlap and ymin_overlap < ymax_overlap and box1 != box2:
                if box1.volume() > box2.volume():
                    bbs.remove(box2)
                else:
                    bbs.remove(box1)
    
        


def updateBB(bbs, nSensor, i):

    # loading the next scan
    pointcloud = pc.generatePointClouds(nSensor, i+1)

    # updating every single bounding box
    newBB = []
    newClusters = []
    for bb in bbs:

        boxes, clusters = customdbscan.customDBSCAN(pointcloud, bb, 0.6, 1.5, 5)

        if boxes:
            for box in boxes:
                newBB.append(box)
            for cluster in clusters:
                newClusters.append(cluster)
    
    remainingPoints = findPointsOutsideBB(pointcloud, newClusters)
    discoveredBoxes = []

    if len(remainingPoints.points) > 10:
        discoveredBoxes = blensoranalysis.generateBB(remainingPoints)
        for discovederedBox in discoveredBoxes:
            newBB.append(discovederedBox)

    removeOverlappingBoxes(newBB)

    return newBB, pointcloud

def displayBoundingBoxes(bbs, pointcloud):
    o3d.visualization.draw([pointcloud, *bbs], show_skybox=False) 
