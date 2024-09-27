import numpy as np
import open3d as o3d

def generatePointsAndLabels(pointcloud, boundingBox, expanseFactor):

    # getting the indices of the points within the bounding box
    corePoints = boundingBox.get_point_indices_within_bounding_box(pointcloud.points)
    
    # getting data for sphere creation
    bbCenter = boundingBox.get_center()
    bbExtent = boundingBox.extent
    sphereRadius = np.max(bbExtent) * expanseFactor / 2.0

    # getting the indices of points that are within the sphere
    indices = np.where(np.linalg.norm(pointcloud.points - bbCenter, axis=1) < sphereRadius)[0]

    # creating the mesh for the sphere centered at the obb center
    sphereMesh = o3d.geometry.TriangleMesh.create_sphere(radius=sphereRadius)
    sphereMesh.translate(bbCenter)
    wireFrame = o3d.geometry.LineSet.create_from_triangle_mesh(sphereMesh)
    wireFrame.paint_uniform_color([1, 0, 0])

    labels = np.zeros(len(indices)) -1

    for i, indexValue in enumerate(indices):
        if indexValue in corePoints:
            labels[i] = 1

    return np.asarray(pointcloud.points)[indices], labels, wireFrame


def customDBSCAN(pointcloud, boundingBox, eps, minPts, expanseFactor):

    points, labels, sphere = generatePointsAndLabels(pointcloud, boundingBox, expanseFactor)

    if points.any(): # checking if there are any points in the bounding box

        for i in range(len(points)):
            if labels[i] == 1:

                            
                neighbours = [j for j, point in enumerate(points) if np.linalg.norm(point - points[i]) < eps] # getting the neighbours of points[i] that are within eps distance

                if len(neighbours) < minPts:
                    labels[i] = -1 # labeling point as noise because it doesn't have enough neighbours
                else:
                    #nCluster += 1
                    labels, points = expandCluster(points, labels, neighbours, eps, minPts, pointcloud, expanseFactor)

        #returning new bounding box (/bounding boxes in case with the newly discovered clusters)
        return createBoundingBoxes(labels, points, sphere)
    else:
        return None, None, None

def expandCluster(points, labels, neighbours, eps, minPts, pointcloud, expanseFactor):

    # assing the current point to a cluster
    #labels[i] = nCluster

    for j in neighbours:
        #checking if the current point is labeled as noise
        if labels[j] == -1:
            labels[j] = 1 #nCluster

            #seraching for new neighbours to add to "neighbours" variable
            newNeighbours = [k for k, point in enumerate(points) if np.linalg.norm(point - points[j]) < eps]
            if len(newNeighbours) >= minPts:
                neighbours += newNeighbours

    return labels, points

# considering only the bigger cluster
def createBoundingBoxes(labels, points, sphere):

    # excluding noise 
    effectiveLabels = labels[labels != -1]

    if len(effectiveLabels) > 4:

        clusterPoints = o3d.geometry.PointCloud()
        clusterPoints.points = o3d.utility.Vector3dVector(points[labels == 1])
            
        # creating bounding boxes for each cluster
        boundingBox = clusterPoints.get_oriented_bounding_box()
        boundingBox.color = [1, 0, 0]

        return boundingBox, clusterPoints, sphere
    else:
        return None, None, None
