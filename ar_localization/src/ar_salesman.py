import itertools
import math
from datetime import datetime
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
import rospy

#array of all tags
tags = {1:(0,0), 2:(10,10), 3:(4,5), 4:(6,3),
        5:(3,9), 6:(8,1), 7:(1,3), 8:(6,2),9:(7,4),}# 0:(3,3)}

#all permutations of tag IDs
perms = list(itertools.permutations(tags))

#define distance formula
def dist(a, b, dict): #given two permutations:
    return math.sqrt((dict[a][0]-dict[b][0])**2+(dict[a][1]-dict[b][1])**2)

class ARSalesman:

    graphed = False

	def __init__(self):
		rospy.loginfo("ARSalesman Started!")
		self.ar_pose_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_pose_cb)

	def ar_pose_cb(self, msg):
		if len(msg.messages)<3 or self.graphed:
            return
		#compute total distance of each permutation
		best_dist = 1000000 #really big number, doesn't matter what
		best_perm = None

		for perm in perms:
		    total_dist = 0
		    for i, e in enumerate(perm):
			if i==len(perm)-1: #if at end, continue to next permutation
			    break
			else:
			    current_dist = dist(e,perm[i+1],tags)
			    total_dist += current_dist
			    #print(current_dist)
			    #print("Total distance for "+str(perm)+" is "+str(total_dist))
		    if total_dist<best_dist: #if the new distance is shorter
			best_dist = total_dist
			best_perm = perm
			#print("Best distance: "+str(best_dist)) #print the new least distance
			#print("Best permutation: "+str(best_perm))
		print(best_perm)
		print(best_dist)

if __name__ == '__main__':
	rospy.init_node('ar_salesman')
	a = ARSalesman()

	rospy.spin()
