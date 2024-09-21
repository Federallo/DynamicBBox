import blensoranalysis as pc
import dbscan_expand_bbox as dbscan1
import dbscan_sphere_bound as dbscan2
import open3d as o3d
import numpy as np
import blensoranalysis

# NOTE: the bounding boxes of the custom cluster must be OrientedBoudingBox type, the same as the ones in blensory_analysis repo

def findPointsOutsideBB(pointcloud, bbs):
    
    indices = set()

    # creating a mask to remove all the existing clusters from pointcloud
    for bb in bbs:
        indices.update(bb.get_point_indices_within_bounding_box(pointcloud.points))

    # creating a mask to remove all the existing bounding boxes from pointcloud
    mask = np.ones(len(pointcloud.points), dtype=bool)
    mask[list(indices)] = False

    return np.asarray(pointcloud.points)[mask]


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
    
        


def updateBB(bbs, nSensor, i, nMethod):

    # loading the next scan
    pointcloud = pc.generatePointClouds(nSensor, i+1)

    # displaying pointcloud of the next scan and the bounding boxes
    o3d.visualization.draw([pointcloud, *bbs], show_skybox = False)

    # Mapping between nMethod and the corresponding DBSCAN method
    dbscanMethod = { 1: (dbscan1.customDBSCAN, (0.6, 1.5, 5)), 2: (dbscan2.customDBSCAN, (1.5, 5, 1.3)) }

    if nMethod not in dbscanMethod:
        print("Invalid method")
        return None, None

    dbscanFunction, params = dbscanMethod[nMethod]

    # updating every single bounding box
    newBB = []
    newClusters = []
    expandedBoxes = []
    for bb in bbs:
        
        result = dbscanFunction(pointcloud, bb, *params)

        if nMethod == 1:
            boxes, clusters, expandedBox = result
            expandedBoxes.append(expandedBox)
        else:
            boxes, clusters, expandedSphere = result
            expandedBoxes.append(expandedSphere)
        
        if boxes:
            newBB.append(boxes)
        
    o3d.visualization.draw([pointcloud, *expandedBoxes], show_skybox = False)
    
    
    remainingPoints = findPointsOutsideBB(pointcloud, newBB)
    discoveredBoxes = []
    
    #FIXME the last pointcloud is not considered entirely because, at first, it finds a little part of the object, put it gets deleted when it is discovered
           #an overlapping box that deletes the previous one
    
    if len(remainingPoints) > 4:
    
        remainingPointCloud = o3d.geometry.PointCloud()
        remainingPointCloud.points = o3d.utility.Vector3dVector(remainingPoints)


        labels = np.array(remainingPointCloud.cluster_dbscan(eps = 2, min_points = 5, print_progress = False))
        clusters = []
        for label in np.unique(labels):
            if label != -1:
                cluster_mask = labels == label
                clusters.append(remainingPointCloud.select_by_index(np.where(cluster_mask)[0]))
        for cluster in clusters:
            if len(cluster.points) > 4:
                bb = cluster.get_oriented_bounding_box()
                bb.color = [0, 1, 0]
                discoveredBoxes.append(bb)

        o3d.visualization.draw([pointcloud, *discoveredBoxes, *newBB], show_skybox = False)

        for discovederedBox in discoveredBoxes:
            #print("discovered boxes", discoveredBoxes)
            newBB.append(discovederedBox)
    
    removeOverlappingBoxes(newBB)

    return newBB, pointcloud

def displayBoundingBoxes(bbs, pointcloud): # to display only the clustered points "pointcloud" must be changed in "*pointcloud"
    o3d.visualization.draw([pointcloud, *bbs], show_skybox=False) 
