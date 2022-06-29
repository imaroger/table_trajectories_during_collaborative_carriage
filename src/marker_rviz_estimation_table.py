#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from prediction_table.msg import TrajPairMsg, NmpcMsg, TrajMsg
from geometry_msgs.msg import Point, TransformStamped
from tf2_msgs.msg import TFMessage
import numpy as np
from math import cos,sin,pi
import message_filters
# import tf.transformations as tt

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

def setMarker(marker,pos,q,scale,color):
	# Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x = pos[0] 
	marker.pose.position.y = pos[1]  
	marker.pose.position.z = pos[2]  
	marker.pose.orientation.x = q[0] 
	marker.pose.orientation.y = q[1] 
	marker.pose.orientation.z = q[2] 
	marker.pose.orientation.w = q[3] 

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

def declareMarker(marker_ns,marker_id,path):
	marker = Marker()
	marker.header.frame_id = "world" 
	marker.header.stamp = rospy.Time.now()

	# Set the namespace and id for this marker.  This serves to create a unique ID
	# Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = marker_ns
	marker.id = marker_id 

	# Set the marker type. 
	if path != "none":
		marker.type = marker.MESH_RESOURCE
		marker.mesh_resource = path
	else:
		marker.type = marker.SPHERE

	# Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker.action = marker.ADD 

	return marker

def addMesh(x,y,theta,name):
	if name[0] == "s":
		mesh = declareMarker(name, 0, "package://estimation/mesh/Human_slim.stl")
		scale = 0.001
		color = [1,1,1,0.5]
		th,z = theta+pi/2,0.08
	else:
		mesh = declareMarker("Table", 0, "package://to_table/mesh/Table.stl")
		scale = 1
		color = [1,1,1,1]
		th,z = theta,0.75+0.1
	
	current_pos = [-0.04*sin(th)+x,0.04*cos(th)+y,z]
	current_q = [0,0,sin(th/2),cos(th/2)]

	mesh = setMarker(mesh,current_pos,current_q,scale,color)
	mesh.lifetime = rospy.Duration()

	return mesh

def addMarker(x,y,z,name,color,i):
	# Send the current position of the CoM and record it
	marker = declareMarker(name, i, "none")
	marker = setMarker(marker,[x,y,z],[0,0,0,1],0.02,color)
	marker.lifetime = rospy.Duration()
	return marker

def addCurrentTraj(x,y,name):
	# Display the trajectory of the human CoM which is sent to the estimation process
	pos_list = []
	for k in range(len(x)):
		pos = Point()
		pos.x,pos.y,pos.z = x[k],y[k],0.94
		pos_list.append(pos)	
	marker = setMarkerList("CoM_"+name,0,pos_list,[0,0,0,1],0.04,[1,1,0,1])
	marker.lifetime = rospy.Duration(5)
	return marker

def addCurrentEst(x,y):
	# Display the trajectory of the table CoM which is sent by the estimation process
	pos_list = []
	for k in range(len(x)):
		pos = Point()
		pos.x,pos.y,pos.z = x[k],y[k],0.9
		pos_list.append(pos)	
	marker = setMarkerList("CoM_est",0,pos_list,[0,0,0,1],0.04,[1,0,1,1])
	marker.lifetime = rospy.Duration(5)
	return marker


def setFrame(x,y,th,dist,ind):
	frame = TransformStamped()
	frame.header.frame_id = "world" 
	frame.header.stamp = rospy.Time.now()
	frame.child_frame_id = "frame_"+str(ind)

	frame.transform.translation.z = 0.94
	frame.transform.rotation.x = 0
	frame.transform.rotation.y = 0

	if ind == 1:
		frame.transform.translation.x = dist*(cos(th))+x
		frame.transform.translation.y = dist*(sin(th))+y

		frame.transform.rotation.z = sin((th+pi)/2)
		frame.transform.rotation.w = cos((th+pi)/2)
	elif ind == 2:
		frame.transform.translation.x = -dist*(cos(th))+x
		frame.transform.translation.y = -dist*(sin(th))+y

		frame.transform.rotation.z = sin(th/2)
		frame.transform.rotation.w = cos(th/2)
	else:
		frame.transform.translation.x = x
		frame.transform.translation.y = y

		frame.transform.rotation.z = sin(th/2)
		frame.transform.rotation.w = cos(th/2)		
	return frame

def addCurrentEstRobot(x,y,th_0,dist,N_0,ind):
	# Display the trajectory of the robot CoM which is sent to the wpg
	pos_list = []
	if len(x) > N_0:
		for k in range(N_0,len(x)):
			pos = Point()
			if ind == 1:
				pos.x,pos.y,pos.z =  dist*(cos(th_0))+x[k],dist*(sin(th_0))+y[k],0.9
			elif ind == 2:
				pos.x,pos.y,pos.z = -dist*(cos(th_0))+x[k],-dist*(sin(th_0))+y[k],0.9				
			pos_list.append(pos)	
	marker = setMarkerList("CoM_est_"+str(ind),0,pos_list,[0,0,0,1],0.04,[1,0,1,1])
	marker.lifetime = rospy.Duration(5)
	return marker


class marker_pub:

	def __init__(self):
		human_sub = message_filters.Subscriber('pair_trajectory', TrajPairMsg)
		estimation_sub = message_filters.Subscriber('estimated_trajectory', TrajMsg)

		ts = message_filters.ApproximateTimeSynchronizer([human_sub, estimation_sub],\
			100, 0.1, allow_headerless=True)
		ts.registerCallback(self.callback)

		self.pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
		self.pub_tf = rospy.Publisher("tf", TFMessage, queue_size=10)		
		self.count = 0

		self.N_0 = rospy.get_param('N_0')		

	def callback(self, traj,est):
		x_1, y_1, theta_1 = traj.x_traj_1, traj.y_traj_1, traj.theta_traj_1
		x_2, y_2, theta_2 = traj.x_traj_2, traj.y_traj_2, traj.theta_traj_2
		x_t, y_t, theta_t = traj.x_traj_table, traj.y_traj_table, traj.theta_traj_table		
		
		x_est, y_est, theta_est = est.x_traj, est.y_traj, est.theta_traj
		
		# marker_traj_1 = addCurrentTraj(x_1, y_1,"sub1")
		# marker_traj_2 = addCurrentTraj(x_2, y_2,"sub2")
		marker_traj_table = addCurrentTraj(x_t, y_t,"table")

		marker_est = addCurrentEst(x_est, y_est)
		self.pub.publish(marker_est)

		# self.pub.publish(marker_traj_1)
		# self.pub.publish(marker_traj_2)
		self.pub.publish(marker_traj_table)	

		if len(x_1) != 0:
			mesh_sub1 = addMesh(x_1[-1], y_1[-1], theta_1[-1], "sub1")
			self.pub.publish(mesh_sub1)
		if len(x_2) != 0:
			mesh_sub2 = addMesh(x_2[-1], y_2[-1], theta_2[-1], "sub2")
			self.pub.publish(mesh_sub2)
		mesh_table = addMesh(x_t[-1], y_t[-1], theta_t[-1], "table")	
		self.pub.publish(mesh_table)

		if len(x_1) == 0 or len(x_2) == 0:
			frame_msg = TFMessage()
			# pos_table = setFrame(x_est[ind], y_eind[ind], theta_est[ind],0)
			# frame_msg.transforms.append(pos_table)
			status = rospy.get_param('human_status')

			if status == "Walk": 
				ind = self.N_0
			else:
				ind = -1
			dist = 0.95

			if len(x_1) == 0:
				pos1_wrt_table = setFrame(x_est[ind], y_est[ind], theta_est[ind],dist,1)
				frame_msg.transforms.append(pos1_wrt_table)
				marker_est1 = addCurrentEstRobot(x_est,y_est,theta_est[ind],dist,self.N_0,1)
				self.pub.publish(marker_est1)

			if len(x_2) == 0:
				pos2_wrt_table = setFrame(x_est[ind], y_est[ind], theta_est[ind],dist,2)
				frame_msg.transforms.append(pos2_wrt_table)
				marker_est2 = addCurrentEstRobot(x_est,y_est,theta_est[ind],dist,self.N_0,2)
				self.pub.publish(marker_est2)				
						
			self.pub_tf.publish(frame_msg)

		self.count += 1


if __name__ == '__main__':
	try:
		rospy.init_node('RVizMarkers', anonymous=True)
		marker_pub()
		while not rospy.is_shutdown():
			rospy.spin()
	except rospy.ROSInterruptException:
	    print("RVizMarkers Shutting down")
