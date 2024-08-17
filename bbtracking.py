import pointcloud as pc
import customdbscan
import open3d as o3d
import numpy as np

# important note: the bounding boxes of the custom cluster must be OrientedBoudingBox type, the same as the ones in blensory_analysis repo

#global bb_history = np.array([])

def updateBB(bbs, nSensor, i):

    '''
    #it will be executed only the first time will be added a bounding box 
    if bb_history.size == 0 and bbs:
        bb_history = [None] * (i-1)
        bb_history[i].append(bb)
    '''
    # loading the next scan
    pointcloud = pc.generatePointClouds(nSensor, i+1)

    # getting the points that are within the bounding box
    # updating every single bounding box
    new_bb = []
    for box in bbs:
        '''
        for pointcloud in pointclouds:
            bb = customdbscan.customDBSCAN(pointcloud, box, 0.25, 5)
            if bb:
                new_bb.append(bb)
                #bb_history[i+1].append(bb)
        '''
        bb = customdbscan.customDBSCAN(pointcloud, box, 0.35, 5)
        if bb:
            new_bb.append(bb)
    return new_bb

def displayBoundingBoxes(bbs):

    for bb in bbs:# FIXME it is a list of lists
        o3d.visualization.draw(*bb, show_skybox=False)
