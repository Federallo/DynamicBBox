import bbtracking
import blensoranalysis

tracked_bb = []

#TODO implement multiple scans
#for i in range(20, scans+20):

#Generating bounding boxes without noise and filetered for the moment. TODO
bounding_box = blensoranalysis.generateBoundingBoxes(1, 20)

#testing only one bounding box for the moment

#tracked_bb.append(bounding_box)


tracked_bb = bbtracking.updateBB(bounding_box, 1, 20)
bbtracking.displayBoundingBoxes(tracked_bb)

'''
print(pointCloud)

pc.visualizeGraphics(pointCloud)
'''


#pointcloud.displayPointCloud(pointCloud)
