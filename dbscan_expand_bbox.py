import numpy as np
import open3d as o3d

def expandBoundingBox(boundingBox, expansionFactor):
    newExtents = boundingBox.extent * (1 + expansionFactor)
    extendedBoundingBox = o3d.geometry.OrientedBoundingBox(boundingBox.center, boundingBox.R, newExtents)
    extendedBoundingBox.color = [1, 0, 0]
    return extendedBoundingBox

def pointsInBB(pointcloud, boundingBox):

    # getting points of point cloud
    indices = boundingBox.get_point_indices_within_bounding_box(pointcloud.points)
    pointsInBox = np.asarray(pointcloud.points)[indices]

    return pointsInBox, indices

def generatePointsAndLabels(pointcloud, boundingBox, bbExpanseFactor):

    # getting the indices of the points within the bounding box
    corePoints = boundingBox.get_point_indices_within_bounding_box(pointcloud.points)

    # expanding the bounding box with a certain value in case the object is moving and it needs to be tracked
    # creating a new function because bounding_box is readonly
    boundingBox = expandBoundingBox(boundingBox, bbExpanseFactor)

    # getting the points that are within the bounding box  
    points, indices = pointsInBB(pointcloud, boundingBox) 

    # initializing the labels of the points
    labels = np.zeros(len(points)) -1

    for i, indexValue in enumerate(indices):
        if indexValue in corePoints:
            labels[i] = 1 # setting custom label for core points within the first bounding box

    return points, labels, boundingBox


def customDBSCAN(pointcloud, boundingBox, bbExpansionFactor, eps, minPts):

    points, labels, boundingBox = generatePointsAndLabels(pointcloud, boundingBox, bbExpansionFactor)

    if points.any(): # checking if there are any points in the bounding box

        for i in range(len(points)):
            if labels[i] == 1:
                
                neighbours = [j for j, point in enumerate(points) if np.linalg.norm(point - points[i]) < eps] # getting the neighbours of points[i] that are within eps distance

                if len(neighbours) > minPts:
                    labels = expandCluster(points, labels, neighbours, eps, minPts)

        #returning new bounding box (/bounding boxes in case with the newly discovered clusters)
        return createBoundingBoxes(labels, points, boundingBox)
    else:
        return None, None, None

def expandCluster(points, labels, neighbours, eps, minPts):

    for j in neighbours:
        #checking if the current point is labeled as noise
        if labels[j] == -1:
            labels[j] = 1
            #seraching for new neighbours to add to "neighbours" variable
            newNeighbours = [k for k, point in enumerate(points) if np.linalg.norm(point - points[j]) < eps]
            if len(newNeighbours) >= minPts:
                neighbours += newNeighbours
    return labels

# considering only the bigger cluster
def createBoundingBoxes(labels, points, expandedBox):

    # excluding noise 
    effectiveLabels = labels[labels != -1]

    if len(effectiveLabels) > 4: # because open3d requires at least 5 points to create a bounding box

        clusterPoints = o3d.geometry.PointCloud()
        clusterPoints.points = o3d.utility.Vector3dVector(points[labels == 1])
            
        # creating bounding boxes for each cluster
        boundingBox = clusterPoints.get_oriented_bounding_box()
        boundingBox.color = [1, 0, 0]

        return boundingBox, clusterPoints, expandedBox
    else:
        return None, None, None
