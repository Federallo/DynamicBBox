import numpy as np
import open3d as o3d

def pointsInBB(pointcloud, boundingBox):

    # getting points of point cloud
    indices = boundingBox.get_point_indices_within_bounding_box(pointcloud.points)
    pointsInBox = np.asarray(pointcloud.points)[indices]

    return pointsInBox

def customDBSCAN(pointcloud, boundingBox, eps, minPts, expanseFactor):

    points = pointsInBB(pointcloud, boundingBox) # getting the points that are within the bounding box

    if points.any(): # checking if there are any points in the bounding box

        labels = np.zeros(len(points)) -1 # initializing all points as noise (-1)
        nCluster = 0 # initializing the cluster label to assign to the points (so for example the first cluster will have label 0, the second 1, and so on. Each of them will have a group of points)

        for i in range(len(points)):
            if labels[i] == -1:

                for j in range(len(pointcloud.points)):
                    

                    # Checking if point is outside the bounding box and adding new pointcloud in case is close to the points inside the bounding box
                    if not pointcloud.points[j] in points and np.linalg.norm(pointcloud.points[j]-points[i]) < expanseFactor:  
                        labels = np.concatenate((labels, [labels[i]])) 
                        pointcloud.colors[j] = [0,1,0] 
                        points = np.concatenate((points, [pointcloud.points[j]]))
                            
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
        
            for l in range(len(pointcloud.points)):

                # Checking if point is outside the bounding box and adding new pointcloud in case is close to the points inside the bounding box
                if not pointcloud.points[l] in points and np.linalg.norm(pointcloud.points[l]-points[j]) < expanseFactor:  
                    labels = np.concatenate((labels, [labels[j]])) 
                    pointcloud.colors[l] = [0,1,0]                           
                    points = np.concatenate((points, [pointcloud.points[l]]))

            #seraching for new neighbours to add to "neighbours" variable
            newNeighbours = [k for k, point in enumerate(points) if np.linalg.norm(point - points[j]) < eps]
            if len(newNeighbours) >= minPts:
                neighbours += newNeighbours

    return labels, points

# considering only the bigger cluster
def createBoundingBoxes(labels, points):

    # excluding noise 
    effectiveLabels = labels[labels != -1]

    if len(effectiveLabels) != 0:

        uniqueClusterLabels, countClusterLabels = np.unique(effectiveLabels, return_counts=True) # getting the unique cluster labels and their counts
        mostFrequentClusterLabel = uniqueClusterLabels[np.argmax(countClusterLabels)] # getting the most frequent cluster label

        clusterPoints = o3d.geometry.PointCloud()
        clusterPoints.points = o3d.utility.Vector3dVector(points[labels == mostFrequentClusterLabel])
            
        # creating bounding boxes for each cluster
        boundingBox = clusterPoints.get_oriented_bounding_box()
        boundingBox.color = [1, 0, 0]

        return boundingBox, clusterPoints
    else:
        return None, None
