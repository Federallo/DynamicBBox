import numpy as np
import matplotlib as plt
import open3d as o3d

def generateGraphics(pointclouds,filtered):

  pcds=[]
  for pointcloud in pointclouds:
    pcd= o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pointcloud)
    pcds.append(pcd)
  
  combined_pcd = o3d.geometry.PointCloud()

  for pcd in pcds:
    for point in pcd.points:
      if point not in combined_pcd.points:
        combined_pcd.points.append(point)
  
  if(filtered==1):
    filtered_pcd = combined_pcd.select_by_index([i for i in range(len(combined_pcd.points)) if combined_pcd.points[i][2] > 0.3])
    filtered_downsampled_pcd=filtered_pcd.voxel_down_sample(voxel_size=0.25)
    return filtered_downsampled_pcd
  
  else:
    combined_downsampled_pcd=combined_pcd.voxel_down_sample(voxel_size=0.25)
    return combined_downsampled_pcd
  

def clusterData(pcds):
  global_bboxes=[]
  
  for pcd in pcds:

    labels = np.array(pcd.cluster_dbscan(eps=1.5, min_points=5, print_progress=True))

    clusters = []
    for label in np.unique(labels):
        cluster_mask = (labels == label)
        clusters.append(pcd.select_by_index(np.where(cluster_mask)[0]))

    bounding_boxes = []
    for cluster in clusters:
        aabb = cluster.get_oriented_bounding_box()
        aabb.color = [0,1,0]
        bounding_boxes.append(aabb)
        #global_bboxes.append(aabb)
    global_bboxes.append(bounding_boxes)

    colors = plt.cm.tab10(labels / (labels.max() if labels.max() > 0 else 1))
    pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    
    o3d.visualization.draw([pcd,*bounding_boxes],show_skybox=False)
    
  return global_bboxes

def visualizeGraphics(pcds):
  for pcd in pcds:
    o3d.visualization.draw([pcd],show_skybox=False)
    
  
