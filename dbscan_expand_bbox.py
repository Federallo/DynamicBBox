import numpy as np
import open3d as o3d

def expandBoundingBox(boundingBox, expansionFactor):
    newExtents = boundingBox.extent * (1 + expansionFactor)
    return o3d.geometry.OrientedBoundingBox(boundingBox.center, boundingBox.R, newExtents)

def pointsInBB(pointcloud, bounding_box):

    # getting points of point cloud
    indices = bounding_box.get_point_indices_within_bounding_box(pointcloud.points)
    pointsInBox = np.asarray(pointcloud.points)[indices]

    return pointsInBox

def customDBSCAN(pointcloud, boundingBox, bbExpansionFactor, eps, minPts):
    
    # expanding the bounding box with a certain value in case the object is moving and it needs to be tracked
    # creating a new function because bounding_box is readonly
    boundingBox = expandBoundingBox(boundingBox, bbExpansionFactor)

    # getting the points that are within the bounding box  
    points = pointsInBB(pointcloud, boundingBox) 

    if points.any(): # checking if there are any points in the bounding box

        labels = np.zeros(len(points)) -1 # initializing all points as noise (-1)
        nCluster = 0 # initializing the cluster label to assign to the points (so for example the first cluster will have label 0, the second 1, and so on. Each of them will have a group of points)

        for i in range(len(points)):
            if labels[i] == -1:
                
                neighbours = [j for j, point in enumerate(points) if np.linalg.norm(point - points[i]) < eps] # getting the neighbours of points[i] that are within eps distance

                if len(neighbours) < minPts:
                    labels[i] = -1 # labeling point as noise because it doesn't have enough neighbours
                else:
                    nCluster += 1
                    labels = expandCluster(points, labels, i, neighbours, nCluster, eps, minPts)

        #returning new bounding box (/bounding boxes in case with the newly discovered clusters)
        return createBoundingBoxes(labels, points, boundingBox)
    else:
        return None, None, None

def expandCluster(points, labels, i, neighbours, nCluster, eps, minPts):

    # assing the current point to a cluster
    labels[i] = nCluster

    for j in neighbours:
        #checking if the current point is labeled as noise
        if labels[j] == -1:
            labels[j] = nCluster
            #seraching for new neighbours to add to "neighbours" variable
            newNeighbours = [k for k, point in enumerate(points) if np.linalg.norm(point - points[j]) < eps]
            if len(newNeighbours) >= minPts:
                neighbours += newNeighbours
    return labels

# considering only the bigger cluster
def createBoundingBoxes(labels, points, expandedBox):

    # generating bounding box and cluster that
    boundingBoxes = []
    clusters = []

    # excluding noise 
    effectiveLabels = labels[labels != -1]

    if len(effectiveLabels) != 0:

        uniqueClusterLabels, countClusterLabels = np.unique(labels, return_counts=True) # getting the unique cluster labels and their counts
        mostFrequentClusterLabel = uniqueClusterLabels[np.argmax(countClusterLabels)] # getting the most frequent cluster label

        clusterPoints = o3d.geometry.PointCloud()
        clusterPoints.points = o3d.utility.Vector3dVector(points[labels == mostFrequentClusterLabel])
            
        #returning clusters
        clusters.append(clusterPoints)
            
        # creating bounding boxes for each cluster
        boundingBox = clusterPoints.get_oriented_bounding_box()
        boundingBox.color = [1, 0, 0]
        boundingBoxes.append(boundingBox)

    return boundingBoxes, clusters, expandedBox
