import itertools
import math
from datetime import datetime
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
import rospy

#all permutations of tag IDs


#define distance formula
def dist(a, b, dict): #given two permutations:
    return math.sqrt((dict[a][0]-dict[b][0])**2+(dict[a][1]-dict[b][1])**2)

class ARSalesman:

	#array of all tags

	graphed = False

	def __init__(self):
		rospy.loginfo("ARSalesman Started!")
		self.ar_pose_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_pose_cb)

	def ar_pose_cb(self, msg):
		if len(msg.markers)<1 or self.graphed:
			return

		marker_data = {}		
		for marker in msg.markers:
			marker_data[marker.id]=(marker.pose.pose.position.x,marker.pose.pose.position.y)

		perms = list(itertools.permutations(marker_data))

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
		graphed=True
		print(marker_data)

if __name__ == '__main__':
	rospy.init_node('ar_salesman')
	a = ARSalesman()

	rospy.spin()
