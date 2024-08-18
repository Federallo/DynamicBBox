import numpy as np
import open3d as o3d


def pointsInBB(pointcloud, bounding_box):

    #FIXME points must be returned as vector3dvector array
    
    # getting boints of point cloud
    #print("point cloud", pointcloud)
    #print("points", pointcloud.points)
    #print("bounding box", bounding_box)
    points = np.asarray(pointcloud.points)

    # getting the lower left and upper right corners of the bounding box
    min_point = bounding_box.center - bounding_box.extent / 2
    max_point = bounding_box.center + bounding_box.extent / 2

    # getting indices of points within the bounding box (if there are any)
    indices = [i for i, point in enumerate(points) if min_point[0] <= point[0] <= max_point[0] and min_point[1] <= point[1] <= max_point[1] and min_point[2] <= point[2] <= max_point[2]]

    points_in_box = points[indices]
    
    return points_in_box

def customDBSCAN(pointcloud, bounding_box, eps, minPts):

    points = pointsInBB(pointcloud, bounding_box) # getting the points that are within the bounding box   
    if points.any(): # checking if there are any points in the bounding box

        labels = np.zeros(len(points)) -1 # initializing all points as noise (-1)
        nCluster = 0 # initializing the cluster label to assign to the points (so for example the first cluster will have label 0, the second 1, and so on. Each of them will have a group of points)

        for i in range(len(points)):
            if labels[i] == -1:
                
                neighbours = [j for j, point in enumerate(points) if np.linalg.norm(point - points[i]) < eps] # getting the neighbours of points[i] that are within eps distance
                #print(neighbours)
                if len(neighbours) < minPts:
                    labels[i] = -1 # labeling point as noise because it doesn't have enough neighbours
                else:
                    nCluster += 1
                    labels = expandCluster(points, labels, i, neighbours, nCluster, eps, minPts) #TODO check if the function actually saves changes to the labels variable
                #print(labels)
        #returning new bounding box (/bounding boxes in case  with the newly discovered clusters
        #print(labels)
        return createBoundingBoxes(labels, points)
    else:
        return None


def expandCluster(points, labels, i, neighbours, nCluster, eps, minPts):

    # assing the current point to a cluster
    labels[i] = nCluster
    #print(labels)
    for j in neighbours:
        #checking if the current point is labeled as noise
        if labels[j] == -1:
            labels[j] = nCluster
            #seraching for new neighbours to add to "neighbours" variable
            newNeighbours = [k for k, point in enumerate(points) if np.linalg.norm(point - points[j]) < eps]
            if len(newNeighbours) >= minPts:
                neighbours += newNeighbours
    return labels

def createBoundingBoxes(labels, points):
    boundingBoxes = []
    uniqueClusterLabels = np.unique(labels) # getting the unique cluster labels
    for clusterLabel in uniqueClusterLabels:
        if clusterLabel != -1: # we are ignoring noise points. TODO but maybe we should use blensor analysis repo to analyze them

            clusterPoints = o3d.geometry.PointCloud()
            clusterPoints.points = o3d.utility.Vector3dVector(points[labels == clusterLabel])

            boundingBox = clusterPoints.get_oriented_bounding_box()
            boundingBox.color = [1, 0, 0]
            boundingBoxes.append(boundingBox)
    return boundingBoxes
