import numpy as np
import open3d as o3d

def pointsInBB(pointcloud, bounding_box):

    # getting boints of point cloud
    points = np.asarray(pointcloud.points)

    # getting the lower left and upper right corners of the bounding box
    min_point = bounding_box.center - bounding_box.extent / 2
    max_point = bounding_box.center + bounding_box.extent / 2

    # getting indices of points within the bounding box (if there are any)
    indices = [i for i, point in enumerate(points) if min_point[0] <= point[0] <= max_point[0] and min_point[1] <= point[1] <= max_point[1] and min_point[2] <= point[2] <= max_point[2]]

    points_in_box = points[indices]
    
    return points_in_box


def customDBSCAN(pointcloud, bounding_box, eps, minPts, expanseFactor):

    points = pointsInBB(pointcloud, bounding_box) # getting the points that are within the bounding box   

    pointcloud = np.asarray(pointcloud.points)

    if points.any(): # checking if there are any points in the bounding box

        labels = np.zeros(len(points)) -1 # initializing all points as noise (-1)
        nCluster = 0 # initializing the cluster label to assign to the points (so for example the first cluster will have label 0, the second 1, and so on. Each of them will have a group of points)

        for i in range(len(points)):
            if labels[i] == -1:

                for j in range(len(pointcloud)):

                    if pointcloud[j] not in points:  # Check if point is outside the bounding box

                        # adding new pointcloud in case is close to the points inside the bounding box
                        if np.linalg.norm(pointcloud[j]-points[i]) < expanseFactor: #(pointcloud[j] - nearestPoint(pointcloud[j], points))
                            labels = np.concatenate((labels, [labels[i]])) 
                            points = np.concatenate((points, [pointcloud[j]]))

                neighbours = [j for j, point in enumerate(points) if np.linalg.norm(point - points[i]) < eps] # getting the neighbours of points[i] that are within eps distance

                if len(neighbours) < minPts:
                    labels[i] = -1 # labeling point as noise because it doesn't have enough neighbours
                else:
                    nCluster += 1
                    labels, points = expandCluster(points, labels, i, neighbours, nCluster, eps, minPts, pointcloud, expanseFactor)

        #returning new bounding box (/bounding boxes in case with the newly discovered clusters)
        return createBoundingBoxes(labels, points)
    else:
        return None, None

def expandCluster(points, labels, i, neighbours, nCluster, eps, minPts, pointcloud, expanseFactor):

    # assing the current point to a cluster
    labels[i] = nCluster

    for j in neighbours:
        #checking if the current point is labeled as noise
        if labels[j] == -1:
            labels[j] = nCluster
        
            for l in range(len(pointcloud)):

                if pointcloud[l] not in points:  # Check if point is outside the bounding box

                        # adding new pointcloud in case is close to the points inside the bounding box
                        if np.linalg.norm(pointcloud[l]-points[j]) < expanseFactor: #(pointcloud[j] - nearestPoint(pointcloud[j], points))
                            labels = np.concatenate((labels, [labels[j]])) 
                            points = np.concatenate((points, [pointcloud[l]]))


            #seraching for new neighbours to add to "neighbours" variable
            newNeighbours = [k for k, point in enumerate(points) if np.linalg.norm(point - points[j]) < eps]
            if len(newNeighbours) >= minPts:
                neighbours += newNeighbours

    return labels, points

# considering only the bigger cluster
def createBoundingBoxes(labels, points):

    # generating bounding box and cluster that
    boundingBoxes = []
    clusters = []

    # excluding noise 
    effectiveLabels = labels[labels != -1]

    if len(effectiveLabels) != 0:

        uniqueClusterLabels, countClusterLabels = np.unique(effectiveLabels, return_counts=True) # getting the unique cluster labels and their counts
        mostFrequentClusterLabel = uniqueClusterLabels[np.argmax(countClusterLabels)] # getting the most frequent cluster label

        clusterPoints = o3d.geometry.PointCloud()
        clusterPoints.points = o3d.utility.Vector3dVector(points[labels == mostFrequentClusterLabel])
            
        #returning clusters
        clusters.append(clusterPoints)
            
        # creating bounding boxes for each cluster
        boundingBox = clusterPoints.get_oriented_bounding_box()
        boundingBox.color = [1, 0, 0]
        boundingBoxes.append(boundingBox)

    return boundingBoxes, clusters
