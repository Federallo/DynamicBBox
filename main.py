import bbtracking
import blensoranalysis

tracked_bb = []
tracked_clusters = []

# NOTE: pointclouds can be generated in those ways: no noise|filtered

boundingBox = blensoranalysis.generateBoundingBoxes(1, 20)

# NOTE: currently is used only one scan, but in the future will be used multiple scans
for i in range(20, 30):#scans+

    boundingBox, pointcloud = bbtracking.updateBB(boundingBox, 1, i, 2) # it analyses the next point cloud
    bbtracking.displayBoundingBoxes(boundingBox, pointcloud)
