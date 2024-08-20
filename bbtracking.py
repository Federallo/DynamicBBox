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
    for bb in bbs:

        boxes = customdbscan.customDBSCAN(pointcloud, bb, 0.47, 6)

        if boxes:
            for box in boxes:
                new_bb.append(box)

    return new_bb

def displayBoundingBoxes(bbs):
    o3d.visualization.draw([*bbs], show_skybox=False)
