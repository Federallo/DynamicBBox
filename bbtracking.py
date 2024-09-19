import blensoranalysis as pc
import dbscan_expand_bbox as dbscan1
import dbscan_sphere_bound as dbscan2
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
            '''
            print("printing new pointcloud")
            for pts in pointcloud.points:
                print(pts)
            print("end")
            '''

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
            newClusters.append(clusters)
        
    o3d.visualization.draw([pointcloud, *expandedBoxes], show_skybox = False)
    
    
    #print("pointcloud", pointcloud)
    #print("used points", newClusters)
    remainingPoints = findPointsOutsideBB(pointcloud, newClusters)
    discoveredBoxes = []
    
    #FIXME the last pointcloud is not considered entirely because, at first, it finds a little part of the object, put it gets deleted when it is discovered
           #an overlapping box that deletes the previous one
    
    if len(remainingPoints.points) > 4:

        #print("remaining points", remainingPoints)

        labels = np.array(remainingPoints.cluster_dbscan(eps = 2, min_points = 5, print_progress = False))
        clusters = []
        for label in np.unique(labels):
            cluster_mask = labels == label
            clusters.append(remainingPoints.select_by_index(np.where(cluster_mask)[0]))
        for cluster in clusters:
            if len(cluster.points) > 4:
                bb = cluster.get_oriented_bounding_box()
                bb.color = [0, 1, 0]
                discoveredBoxes.append(bb)

        for discovederedBox in discoveredBoxes:
            #print("discovered boxes", discoveredBoxes)
            newBB.append(discovederedBox)
    
    removeOverlappingBoxes(newBB)

    return newBB, pointcloud

def displayBoundingBoxes(bbs, pointcloud): # to display only the clustered points "pointcloud" must be changed in "*pointcloud"
    o3d.visualization.draw([pointcloud, *bbs], show_skybox=False) 
