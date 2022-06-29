#!/usr/bin/python

import numpy as np
import rospy
import crocoddyl
from math import pi, floor, sqrt, cos, sin, atan2
from scipy.optimize import minimize
from prediction_table.msg import TrajMsg,TrajPairMsg
from std_msgs.msg import Bool
import time


#########################################################################
########################## FUNCTION DEFINITION	#########################
#########################################################################

def translate(xs,x0):
	x,y,th = [],[],[]
	for state in xs:
		x.append(x0[0] + state[0])
		y.append(x0[1] + state[1])
		th.append(state[2])
	traj = [x,y,th]		
	return (traj)

def solveEstimation(x,y,theta,N_OC,previous_est,new_initial_pos):
	# print("--- human traj x ---",x)
	# print("--- human traj y ---",y)
	# print("--- human traj th ---",theta)	
	init_model_list = []
	for i in range (0,len(x)):
		init_model = crocoddyl.ActionInitModelEstimation()
		init_data  = init_model.createData()
		init_model.costWeights = np.matrix([3.71803398e+00, 5.80814113e+00,\
			6.06995099e+00, 1.42647176e-01,10,10]).T
		init_model.currentState = np.matrix([x[i]-x[0],y[i]-y[0],theta[i]]).T
		init_model_list.append(init_model)

	model = crocoddyl.ActionRunningModelEstimation()
	data  = model.createData()
	model.costWeights = np.matrix([3.71803398e+00, 5.80814113e+00, 6.06995099e+00,\
		1.42647176e-01]).T

	# print("len avt !!!",len(init_model_list))
	# print("len apres!!!",N_OC-len(x)+1)

	model_list = init_model_list + [model]*(N_OC-len(x)+1)

	if (len(previous_est[0]) == 0):
		pos_i = [x[0],y[0],theta[0]]		
		init_state = np.matrix([ 0, 0,pos_i[2] , 0, 0, 0]).T
		problem = crocoddyl.ShootingProblem(init_state, model_list[:-1] ,model_list[-1]) 
		ddp = crocoddyl.SolverDDP(problem)	
		done = ddp.solve()
	else:
		pos_i = new_initial_pos
		init_state = np.matrix([ 0, 0,pos_i[2] , 0, 0, 0]).T 
		# !!! verifier s il faut une vitesse non nulle ici !!! 
		problem = crocoddyl.ShootingProblem(init_state, model_list[:-1] ,model_list[-1]) 
		#  !!! verifier s il y a besoin d un terminal model ici !!! 

		# initial_guess = previous_est[1:len(previous_est)]
		# initial_guess.append(np.array([1,1,1,1,1,1]))
		ddp = crocoddyl.SolverDDP(problem)
		# done = ddp.solve(initial_guess)
		done = ddp.solve()
	print(done)
		# print("--- sol ddp ---",ddp.xs)

	traj = translate(ddp.xs, pos_i)
	# print("--- x ---",len(traj[0]))
	# print("--- y ---",traj[1])
	# print("--- th[:20] ---",traj[2][:20])	
	# print("--- th[20:] ---",traj[2][20:])
	return traj,ddp.xs,done

########################################################################
################################## MAIN ################################
########################################################################

class estimation_pub:

	def __init__(self,path):
		self.sub = rospy.Subscriber("pair_trajectory", TrajPairMsg, self.callback)
		self.pub = rospy.Publisher("estimated_trajectory", TrajMsg, queue_size=10)
		self.pub_ddp_solver_cv = rospy.Publisher("ddp_solver_cv", Bool, queue_size=10)

		# OC model parameters
		self.N_OC = rospy.get_param('N_OC')
		self.previous_sol = [[],[],[]]
		self.new_initial_pos = [0,0,0]

		self.data = rospy.get_param('data_recording')
		self.f_handle = open(path, 'a')

	def callback(self, traj):
		status = rospy.get_param('human_status')
		x, y, theta = traj.x_traj_table, traj.y_traj_table, traj.theta_traj_table
		estimated_traj = TrajMsg()

		if status == "Start":
			print("Start")
			est_traj = [[x[0]],[y[0]],[theta[0]]]

		if status == "Walk": 
			print("Walk",len(x))
			# print("--- old_pos_i ---",self.new_initial_pos)
			est_traj, sol, done = solveEstimation(x,y,theta,self.N_OC,\
				self.previous_sol,[x[0],y[0],theta[0]])#self.new_initial_pos)
			# self.new_initial_pos = [(est_traj[0][1]+est_traj[0][2])/2,\
			# (est_traj[1][1]+est_traj[1][2])/2,(est_traj[2][1]+est_traj[2][2])/2]
			self.new_initial_pos = [est_traj[0][2],est_traj[1][2],est_traj[2][2]]			
			self.previous_sol = sol
			self.pub_ddp_solver_cv.publish(done)

			if self.data:
				N = len(est_traj[0])
				array_to_save = np.zeros((1,3*N))
				array_to_save[0][:N] = est_traj[0]
				array_to_save[0][N:2*N] = est_traj[1]
				array_to_save[0][2*N:] = est_traj[2]

				np.savetxt(self.f_handle, array_to_save)

		if status == "Stop": #revoir ici, ajouter initial guess?
			print("Stop")
			# pos_i = self.new_initial_pos
			# pos_f = [x[0],y[0],theta[0]]
			# est_traj = solveDdp(pos_i,pos_f)
			# self.new_initial_pos = [est_traj[0][2],est_traj[1][2],est_traj[2][2]]
			est_traj = [x, y, theta]
			# rospy.signal_shutdown("Stop !")
			if self.data:
				rospy.signal_shutdown("Estimation shutdown")


		estimated_traj.x_traj = est_traj[0]
		estimated_traj.y_traj = est_traj[1]
		estimated_traj.theta_traj = est_traj[2]
		# print(len(estimated_traj.x_traj),"--- x ---",estimated_traj.x_traj)
		# print(len(estimated_traj.y_traj),"--- y ---",estimated_traj.y_traj)	
		# print("--- new_pos_i ---",self.new_initial_pos)			
		self.pub.publish(estimated_traj)
		



if __name__ == '__main__':
	pair_to_num = {'Sujet1_Aurélie&Sujet2_Stanislas':'1',
		'Sujet1_Rémy&Sujet2_Margaux':'2',
		'Sujet1_Zaki&Sujet2_Yanis':'3', 
		'Sujet1_Thanh&Sujet2_Diane':'4', 
		'Sujet1_Sabrina&Sujet2_Quentin':'5',
		'Sujet1_Aniss&Sujet2_Louise':'6', 
		'Sujet1_Hugo&Sujet2_Alexandre':'7', 
		'Sujet1_Alexia&Sujet2_Bénédicte':'8', 
		'Sujet1_Adénikè&Sujet2_Médéric':'9', 
		'Sujet1_Anaïs&Sujet2_Mariem':'10', 
		'Sujet1_Stéphane&Sujet2_Angélique':'11', 
		'Sujet1_Fanny&Sujet2_William':'12',
		'Sujet1_Romane&Sujet2_Corentin':'13', 
		'Sujet1_Paul&Sujet2_Mathieu':'14', 
		'Sujet1_Marine&Sujet2_Hélène':'15', 
		'Sujet1_Sébastien&Sujet2_Nils':'16', 
		'Sujet1_Antoine&Sujet2_Médéric_LAAS':'17', 
		'Sujet1_Amaury&Sujet2_Jason':'18', 
		'Sujet1_Guilhem&Sujet2_César':'19', 
		'Sujet1_Alexis&Sujet2_Thibaud':'20'}

	try:
		traj_name = rospy.get_param('traj_name')
		pair_name = rospy.get_param('pair_name')
		num_pair = pair_to_num[pair_name]
		N_0 = rospy.get_param('N_0')
		N_OC = rospy.get_param('N_OC')

		path = "/local/imaroger/catkin_ws/src/prediction_table/src/data/prediction/"+\
			traj_name + "_" + num_pair + "_" + str(N_0) + "_" + str(N_OC) + ".dat"
		open(path, "w")
		rospy.init_node('EstimationOC', anonymous=True)
		estimation_pub(path)
		while not rospy.is_shutdown():
			rospy.spin()
	except rospy.ROSInterruptException:
	    print("EstimationOC Shutting down")

