import bbtracking
import blensoranalysis

tracked_bb = []
tracked_clusters = []

# NOTE: pointclouds can be generated in those ways: no noise|filtered

#Generating bounding boxes without noise and filetered for the moment. TODO
bounding_box = blensoranalysis.generateBoundingBoxes(1, 20)

# TODO implement multiple scans
# NOTE: currently is used only one scan, but in the future will be used multiple scans
for i in range(20, 30):#scans+

    bounding_box, pointcloud = bbtracking.updateBB(bounding_box, 1, i) # it analyses the next point cloud
    bbtracking.displayBoundingBoxes(bounding_box, pointcloud)
