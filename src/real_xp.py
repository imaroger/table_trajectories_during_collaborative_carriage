#!/usr/bin/env python
import rospy
import numpy as np
from prediction_table.msg import TrajPairMsg
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import json

def getTrajs():
	# Load the choosen human trajectory according to the parameters (name and subject)
	traj = rospy.get_param('traj_name')
	pair = rospy.get_param('pair_name')
	if traj[-1] == "1":
		leader = rospy.get_param('leader_name')
	else:
		leader = "Sujet 1 et 2"
	path = '/local/imaroger/catkin_ws/src/trajectory_with_table/src/Data/Human/' + traj + '.json'
	file = open(path)
	data = json.load(file)['Trajectoires_Individuelles']

	length = len(data["Table"])
	ind = -1
	for i in range(length):
		if data["Table"][i]["Binome"] == pair and data["Table"][i]["Leader"] == leader:
			ind = i
	assert(ind >= 0)

	x_1,y_1 = data["Sujet 1"][ind]["x"],data["Sujet 1"][ind]["y"]
	theta_1 = data["Sujet 1"][ind]["Orientation_Globale"]
	x_2,y_2 = data["Sujet 2"][ind]["x"],data["Sujet 2"][ind]["y"]
	theta_2 = data["Sujet 2"][ind]["Orientation_Globale"]	
	x_t,y_t = data["Table"][ind]["x"],data["Table"][ind]["y"]
	theta_t = data["Table"][ind]["Orientation_Globale"]

	return x_1,y_1,theta_1,x_2,y_2,theta_2,x_t,y_t,theta_t

def setMarkerList(marker_ns,marker_id,pos,q,scale,color):
	marker = Marker()
	marker.header.frame_id = "world" 
	marker.header.stamp = rospy.Time.now()

	# Set the namespace and id for this marker.  This serves to create a unique ID
	# Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = marker_ns
	marker.id = marker_id 

	# Set the marker type. 
	marker.type = marker.SPHERE_LIST

	# Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker.action = marker.ADD 

	# Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.points = pos

	# Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = scale 
	marker.scale.y = scale 
	marker.scale.z = scale 

	# Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = color[0] 
	marker.color.g = color[1]
	marker.color.b = color[2] 
	marker.color.a = color[3] 

	return marker

def addTraj(x,y,name):
	# Display the whole trajectory of the human CoM
	pos_list = []
	for k in range(len(x)):
		pos = Point()
		pos.x,pos.y,pos.z = x[k],y[k],0.94
		pos_list.append(pos)
	marker = setMarkerList("HumanTraj_"+name,0,pos_list,[0,0,0,1],0.015,[0,1,0,1])
	marker.lifetime = rospy.Duration()
	return marker

def sendTraj():
	# Send the travelled trajectory on the topic /pair_trajectory
	x_1,y_1,theta_1,x_2,y_2,theta_2,x_t,y_t,theta_t = getTrajs()

	i = 0
	N_0 = rospy.get_param('N_0')
	r = rospy.get_param('rate')
	display_sub1 = rospy.get_param('display_sub1')
	display_sub2 = rospy.get_param('display_sub2')	

	pub = rospy.Publisher("pair_trajectory", TrajPairMsg, queue_size=10)
	traj = TrajPairMsg()
	rate = rospy.Rate(r) # Hz	

	data = rospy.get_param('data_recording')

	while not rospy.is_shutdown() and i <= len(x_1):
		# print(i)

		if i < 10:
			pub_traj = rospy.Publisher("visualization_marker", Marker, queue_size=10)
			if display_sub1:
				marker_traj_1 = addTraj(x_1, y_1,"sub1")
				pub_traj.publish(marker_traj_1)
			if display_sub2:
				marker_traj_2 = addTraj(x_2, y_2,"sub2")
				pub_traj.publish(marker_traj_2)
			marker_traj_t = addTraj(x_t, y_t,"table")														
			pub_traj.publish(marker_traj_t)	

		if i <= N_0-1:
			print("Start",i)
			if display_sub1:
				traj.x_traj_1, traj.y_traj_1, traj.theta_traj_1 =\
				x_1[0:(i+1)],y_1[0:(i+1)],theta_1[0:(i+1)]
			if display_sub2:
				traj.x_traj_2, traj.y_traj_2, traj.theta_traj_2 =\
				x_2[0:(i+1)],y_2[0:(i+1)],theta_2[0:(i+1)]
			traj.x_traj_table, traj.y_traj_table, traj.theta_traj_table =\
			x_t[0:(i+1)],y_t[0:(i+1)],theta_t[0:(i+1)]			
			if i == N_0-1:
				rospy.set_param('human_status', "Walk")	
		else:
			print("Walk",i)
			if display_sub1:
				traj.x_traj_1, traj.y_traj_1, traj.theta_traj_1 =\
				x_1[i-N_0+1:(i+1)],y_1[i-N_0+1:(i+1)],theta_1[i-N_0+1:(i+1)]
			if display_sub2:
				traj.x_traj_2, traj.y_traj_2, traj.theta_traj_2 =\
				x_2[i-N_0+1:(i+1)],y_2[i-N_0+1:(i+1)],theta_2[i-N_0+1:(i+1)]
			traj.x_traj_table, traj.y_traj_table, traj.theta_traj_table =\
			x_t[i-N_0+1:(i+1)],y_t[i-N_0+1:(i+1)],theta_t[i-N_0+1:(i+1)]	

		pub.publish(traj)
		# print(len(traj.x_traj),"--- x ---",traj.x_traj)		
		# print(len(traj.y_traj),"--- y ---",traj.y_traj)	
		i += 1
		rate.sleep()
	rospy.set_param('human_status', "Stop")


	while not rospy.is_shutdown():
		print("Stop")
		if display_sub1:
			traj.x_traj_1, traj.y_traj_1, traj.theta_traj_1 =\
			[x_1[-1]],[y_1[-1]],[theta_1[-1]]
		if display_sub2:
			traj.x_traj_2, traj.y_traj_2, traj.theta_traj_2 =\
			[x_2[-1]],[y_2[-1]],[theta_2[-1]]
		traj.x_traj_table, traj.y_traj_table, traj.theta_traj_table =\
		[x_t[-1]],[y_t[-1]],[theta_t[-1]]		
		pub.publish(traj)
		rate.sleep()

		if data:
			rospy.signal_shutdown("Real XP shutdown")


if __name__ == '__main__': 
    rospy.init_node('HumanTrajectory', anonymous = True)
    sendTraj() 
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("HumanTrajectory Shutting down")
