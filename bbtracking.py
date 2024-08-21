import pointcloud as pc
import customdbscan
import open3d as o3d
import numpy as np

# NOTE: the bounding boxes of the custom cluster must be OrientedBoudingBox type, the same as the ones in blensory_analysis repo

def updateBB(bbs, nSensor, i):

    # loading the next scan
    pointcloud = pc.generatePointClouds(nSensor, i+1)

    # updating every single bounding box
    new_bb = []
    new_clusters = []
    for bb in bbs:

        boxes, clusters = customdbscan.customDBSCAN(pointcloud, bb, 0.47, 6)

        if boxes:
            for box in boxes:
                new_bb.append(box)
            for cluster in clusters:
                new_clusters.append(cluster)

    return new_bb, new_clusters

def displayBoundingBoxes(bbs, clusters):
    o3d.visualization.draw([*clusters, *bbs], show_skybox=False)
