import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import splprep, splev
from scipy.stats import pearsonr
from math import sqrt,pi
from scipy import stats
import json
##################################################################################
#### FUNCTION DEFINITION #########################################################
##################################################################################

def lastInd(x,y,x_h,y_h):
	x_h_f,y_h_f = x_h[-1],y_h[-1]
	x_f, y_f = x[-1],y[-1]
	if abs(x[-1]-x[0]) > abs(y[-1]-y[0]): # the greater delta is on x
		if x[-1] > x[0]:
			if x_f < x_h_f : # the measured trajectory goes farther than the prediction
				ind = np.where(x_h > x_f)
				i, i_h = len(x)-1, ind[0][0]
			else: # the prediction goes farther than the measured trajectory
				ind = np.where(x > x_h_f)	
				i, i_h = ind[0][0], len(x_h)-1
		else:
			if x_f > x_h_f : # the measured trajectory goes farther than the prediction
				ind = np.where(x_h < x_f)
				i, i_h = len(x)-1, ind[0][0]
			else: # the prediction goes farther than the measured trajectory
				ind = np.where(x < x_h_f)	
				i, i_h = ind[0][0], len(x_h)-1			
	else : # the greater delta is on y	
		if y[-1] > y[0]:
			if y_f < y_h_f : # the measured trajectory goes farther than the prediction
				ind = np.where(y_h > y_f)
				i, i_h = len(y)-1, ind[0][0]
			else: # the prediction goes farther than the measured trajectory
				ind = np.where(y > y_h_f)
				i, i_h = ind[0][0], len(y_h)-1	
		else:
			if y_f > y_h_f : # the measured trajectory goes farther than the prediction
				ind = np.where(y_h < y_f)
				i, i_h = len(y)-1, ind[0][0]
			else: # the prediction goes farther than the measured trajectory
				ind = np.where(y < y_h_f)
				i, i_h = ind[0][0], len(y_h)-1					
	
	# print(x[i],x_h[i_h],y[i],y_h[i_h])	
	return i, i_h

def linearDistance(x_human,y_human,x_oc,y_oc):
	length = len(x_human)
	okay2 = np.where(np.abs(np.diff(x_oc)) + np.abs(np.diff(y_oc)) > 0)
	x_oc,y_oc = np.array(x_oc)[okay2],np.array(y_oc)[okay2]	

	tck1, u1 = splprep([x_human, y_human], s = 0)
	tck2, u2 = splprep([x_oc, y_oc], s = 0)
	xnew = np.linspace(0, 1, length)
	x_human, y_human = splev(xnew, tck1)
	x_oc, y_oc = splev(xnew, tck2)

	dist = 0
	for i in range(length):
		dist += sqrt((x_human[i]-x_oc[i])**2+(y_human[i]-y_oc[i])**2)
		# if display:
		# 	if i%25 == 0:
		# 		plt.plot([x_human[i],x_oc[i]], [y_human[i],y_oc[i]], color = 'red', linewidth = 0.5)
	# if dist/length > 0.2:
	# 	print(dist/length)
	# 	plt.plot(x_human,y_human)
	# 	plt.plot(x_oc,y_oc)
	# 	plt.show()	
	return dist/length

def angularDistance(th_human,th_oc):
	length = len(th_human)	
	th_oc2 = np.interp(np.arange(0,length,1),np.linspace(0,length,len(th_oc)),th_oc)

	epsilon = 1
	if abs(th_human[0] - (th_oc2[0]+2*pi)) < epsilon:
		th_oc2 += 2*pi
	elif abs(th_human[0]+2*pi - th_oc2[0]) < epsilon:
		th_human = np.array(th_human)+2*pi
	dist = 0
	for i in range(length):
		dist += abs(th_human[i]-th_oc2[i])

	# if dist/length > 2:
	# 	print(dist/length)
	# 	print(length)
	# 	print(th_human[0],th_oc[0],th_oc[0]+2*pi,th_oc2[0])
	# 	time2 = np.linspace(1,100,length)
	# 	time = np.linspace(1,100,len(th_oc))
	# 	plt.plot(time2,th_human)
	# 	plt.plot(time,th_oc)		
	# 	plt.plot(time2,th_oc2)
	# 	plt.show()
	return dist/length

def distance(ind,data_x,data_y,data_th,pred_x,pred_y,pred_th,N_0):

	display = False

	last_ind_pred, last_ind_data = lastInd(pred_x,pred_y,data_x,data_y)

	# print(last_ind_pred, last_ind_data)
	pred_x_short = pred_x[N_0:last_ind_pred+1]
	pred_y_short = pred_y[N_0:last_ind_pred+1]
	pred_th_short = pred_th[N_0:last_ind_pred+1]
	data_x_short = data_x[ind+N_0:last_ind_data+1]
	data_y_short = data_y[ind+N_0:last_ind_data+1]
	data_th_short = data_th[ind+N_0:last_ind_data+1]
	if display:
		# print(lin_dist,ang_dist,pred_dist)
		plt.subplot(1,2,1)
		plt.plot(data_x,data_y)
		plt.plot(data_x_short,data_y_short)	
		plt.plot(pred_x[:N_0+1],pred_y[:N_0+1])	
		plt.plot(pred_x_short,pred_y_short)		
		plt.subplot(1,2,2)	
		plt.plot(np.arange(0,len(data_th),1),data_th)	
		plt.plot(np.arange(0,len(data_th),1)[ind+N_0:last_ind_data+1],data_th_short)	
		plt.plot(np.arange(ind,len(pred_th)+ind,1)[:N_0+1],pred_th[:N_0+1])
		plt.plot(np.arange(ind,len(pred_th)+ind,1)[N_0:last_ind_pred+1],pred_th_short)	
		plt.show()
	if (len(data_x_short) > 15 and len(pred_x_short) > 15):
		lin_dist = linearDistance(data_x_short, data_y_short, pred_x_short, pred_y_short)
		ang_dist = angularDistance(data_th_short, pred_th_short)
		pred_dist = sqrt((data_x_short[-1]-data_x_short[0])**2+\
			(data_y_short[-1]-data_y_short[0])**2)
		return lin_dist,ang_dist,pred_dist
	else:
		return 0,0,0
		

##################################################################################
#### COMPUTE DISTANCES ###########################################################
##################################################################################

# list_traj = ['d1_p4_jaune_1', 'd1_p4_gris_1', 'd1_p5_jaune_1', 'd1_p5_gris_1', 'd1_p6_jaune_1', 'd1_p6_gris_1', 'd2_p7_jaune_1', 'd2_p7_gris_1', 'd3_p7_gris_1',
# 				'd1_p4_jaune_2', 'd1_p4_gris_2', 'd1_p5_jaune_2', 'd1_p5_gris_2', 'd1_p6_jaune_2', 'd1_p6_gris_2', 'd2_p7_jaune_2', 'd2_p7_gris_2', 'd3_p7_gris_2']

# pair_to_num = {'Sujet1_Aurélie&Sujet2_Stanislas':'1',
# 	'Sujet1_Rémy&Sujet2_Margaux':'2',
# 	'Sujet1_Zaki&Sujet2_Yanis':'3', 
# 	'Sujet1_Thanh&Sujet2_Diane':'4', 
# 	'Sujet1_Sabrina&Sujet2_Quentin':'5',
# 	'Sujet1_Aniss&Sujet2_Louise':'6', 
# 	'Sujet1_Hugo&Sujet2_Alexandre':'7', 
# 	'Sujet1_Alexia&Sujet2_Bénédicte':'8', 
# 	'Sujet1_Adénikè&Sujet2_Médéric':'9', 
# 	'Sujet1_Anaïs&Sujet2_Mariem':'10', 
# 	'Sujet1_Stéphane&Sujet2_Angélique':'11', 
# 	'Sujet1_Fanny&Sujet2_William':'12',
# 	'Sujet1_Romane&Sujet2_Corentin':'13', 
# 	'Sujet1_Paul&Sujet2_Mathieu':'14', 
# 	'Sujet1_Marine&Sujet2_Hélène':'15', 
# 	'Sujet1_Sébastien&Sujet2_Nils':'16', 
# 	'Sujet1_Antoine&Sujet2_Médéric_LAAS':'17', 
# 	'Sujet1_Amaury&Sujet2_Jason':'18', 
# 	'Sujet1_Guilhem&Sujet2_César':'19', 
# 	'Sujet1_Alexis&Sujet2_Thibaud':'20'}

# dist = {}
# dist["linear"] = {}
# dist["angular"] = {}
# dist["predicted"] = {}
# for traj in list_traj:
# 	dist["linear"][traj] = []
# 	dist["angular"][traj] = []	
# 	dist["predicted"][traj] = []

# # traj_name = 'd1_p5_gris_1'
# # pair_name = 'Sujet1_Amaury&Sujet2_Jason'
# # pair_num = pair_to_num[pair_name]
# N_0 = 75 
# N_OC = 100
# leader = 'Sujet 1 et 2'

# for traj_name in list_traj:

# 	for pair_name in pair_to_num:
# 		pair_num = pair_to_num[pair_name]
# 		print(traj_name,pair_name)
# 		lin_dist,ang_dist,pred_dist = [],[],[]

# 		path_pred = "/local/imaroger/catkin_ws/src/prediction_table/src/data/prediction/"+\
# 			str(N_0) + "_" + str(N_OC) + "/" +\
# 			traj_name + "_" + pair_num + "_" + str(N_0) + "_" + str(N_OC) + ".dat"
# 		pred = np.loadtxt(path_pred)

# 		path_traj = "/local/imaroger/catkin_ws/src/trajectory_with_table/src/Data/Human/"+\
# 			traj_name + ".json"
# 		f_traj = open(path_traj)
# 		data = json.load(f_traj)['Trajectoires_Individuelles']['Table']

# 		for i in range(len(data)):
# 			if data[i]["Leader"] == leader and data[i]["Binome"] == pair_name:
# 				data_x,data_y = data[i]["x"],data[i]["y"]
# 				data_th = data[i]["Orientation_Globale"]

# 		for i in range(len(pred)):
# 			pred_x,pred_y,pred_th = pred[i][:N_OC+1],pred[i][N_OC+1:2*N_OC+2],pred[i][2*N_OC+2:3*N_OC+3]
# 			# if i%50 == 0:
# 			lind,angd,predd = distance(i,data_x, data_y, data_th, pred_x, pred_y, pred_th, N_0)
# 				# print(i,lind,angd,predd)
# 			if lin_dist != 0:
# 				lin_dist.append(lind)
# 				ang_dist.append(angd)
# 				pred_dist.append(predd)
# 				if angd > 2:
# 					print(i,angd)

# 		# print(np.mean(lin_dist),np.mean(ang_dist),np.mean(pred_dist))
# 		dist["linear"][traj_name].append(np.mean(lin_dist))
# 		dist["angular"][traj_name].append(np.mean(ang_dist))				
# 		dist["predicted"][traj_name].append(np.mean(pred_dist))

# 	# print(len(dist["linear"][traj_name]),len(dist["predicted"][traj_name]))

# file_name = "/local/imaroger/catkin_ws/src/prediction_table/src/data/prediction/"+\
# 'distance_prediction_'+ str(N_0) + "_" + str(N_OC) + '.json'
# json_file = json.dumps(dist, indent = 4, ensure_ascii = False)
# with open(file_name, 'w') as outfile:
# 	outfile.write(json_file)

##################################################################################
#### DISPLAY DISTANCES ###########################################################
##################################################################################

list_traj = ['d1_p4_jaune_1', 'd1_p4_gris_1', 'd1_p5_jaune_1', 'd1_p5_gris_1', 'd1_p6_jaune_1', 'd1_p6_gris_1', 'd2_p7_jaune_1', 'd2_p7_gris_1', 'd3_p7_gris_1',
				'd1_p4_jaune_2', 'd1_p4_gris_2', 'd1_p5_jaune_2', 'd1_p5_gris_2', 'd1_p6_jaune_2', 'd1_p6_gris_2', 'd2_p7_jaune_2', 'd2_p7_gris_2', 'd3_p7_gris_2']

list_coord_end_go = [(-3.1,-2.25)]*2+[(-1.5,-2.25)]*2+[(-3,-4.5)]*2+[(0,-4.5),(0,-4.7),(0,-4.5)]
dist_to_target = []
for init in list_coord_end_go:
	dist_to_target.append(sqrt(init[0]**2+init[1]**2))
dist_to_target *= 2 
# print(dist_to_target)

dist_per_dist_to_target = {}
dist_per_dist_to_target["linear"] = {}
dist_per_dist_to_target["angular"] = {}
dist_per_dist_to_target["predicted"] = {}
for d in list(set(dist_to_target)):
	dist_per_dist_to_target["linear"][d] = []
	dist_per_dist_to_target["angular"][d] = []
	dist_per_dist_to_target["predicted"][d] = []	

# print(dist_per_dist_to_target)

N_0 = 50 
N_OC = 100

path = "/local/imaroger/catkin_ws/src/prediction_table/src/data/prediction/"+\
	'distance_prediction_'+ str(N_0) + "_" + str(N_OC) + '.json'
f_dist = open(path)
dist = json.load(f_dist)

for i in range(len(list_traj)):
	traj = list_traj[i]
	d_to_target = dist_to_target[i]
	dist_per_dist_to_target["linear"][d_to_target] += dist["linear"][traj]
	dist_per_dist_to_target["angular"][d_to_target] += dist["angular"][traj]	
	dist_per_dist_to_target["predicted"][d_to_target] += dist["predicted"][traj]


plt.subplot(4,3,4)
data = [n for n in dist_per_dist_to_target["linear"].values()]
labels = [str(int(n*100)/100) for n in dist_per_dist_to_target["linear"].keys()]
plt.boxplot(data)
plt.xticks(np.arange(1,len(labels)+1,1),labels)
plt.title("Linear Distance (N_0 = "+str(N_0)+", N_OC = "+str(N_OC)+")")
plt.xlabel("Global distance (m)")
plt.ylabel("Linear distance (m)")
plt.subplot(4,3,5)
data = [n for n in dist_per_dist_to_target["angular"].values()]
labels = [str(int(n*100)/100) for n in dist_per_dist_to_target["angular"].keys()]
plt.boxplot(data)
plt.xticks(np.arange(1,len(labels)+1,1),labels)
plt.title("Angular Distance (N_0 = "+str(N_0)+", N_OC = "+str(N_OC)+")")
plt.xlabel("Global distance (m)")
plt.ylabel("Angular distance (rad)")
plt.subplot(4,3,6)
data = [n for n in dist_per_dist_to_target["predicted"].values()]
labels = [str(int(n*100)/100) for n in dist_per_dist_to_target["predicted"].keys()]
plt.boxplot(data)
plt.xticks(np.arange(1,len(labels)+1,1),labels)
plt.title("Predicted Distance (N_0 = "+str(N_0)+", N_OC = "+str(N_OC)+")")
plt.xlabel("Global distance (m)")
plt.ylabel("Predicted distance (m)")

dist_lin = [n for v in dist_per_dist_to_target["linear"].values() for n in v]
dist_ang = [n for v in dist_per_dist_to_target["angular"].values() for n in v]
dist_pred = [n for v in dist_per_dist_to_target["predicted"].values() for n in v]
print("Results for N_0=50 and N_OC=100")
print("linear distance : ",np.mean(dist_lin)," +- ",np.std(dist_lin))
print("angular distance : ",np.mean(dist_ang)," +- ",np.std(dist_ang))
print("predicted distance : ",np.mean(dist_pred)," +- ",np.std(dist_pred))

N_0 = 50 
N_OC = 200

path = "/local/imaroger/catkin_ws/src/prediction_table/src/data/prediction/"+\
	'distance_prediction_'+ str(N_0) + "_" + str(N_OC) + '.json'
f_dist = open(path)
dist = json.load(f_dist)

for i in range(len(list_traj)):
	traj = list_traj[i]
	d_to_target = dist_to_target[i]
	dist_per_dist_to_target["linear"][d_to_target] += dist["linear"][traj]
	dist_per_dist_to_target["angular"][d_to_target] += dist["angular"][traj]	
	dist_per_dist_to_target["predicted"][d_to_target] += dist["predicted"][traj]


plt.subplot(4,3,10)
data = [n for n in dist_per_dist_to_target["linear"].values()]
labels = [str(int(n*100)/100) for n in dist_per_dist_to_target["linear"].keys()]
plt.boxplot(data)
plt.xticks(np.arange(1,len(labels)+1,1),labels)
plt.title("Linear Distance (N_0 = "+str(N_0)+", N_OC = "+str(N_OC)+")")
plt.xlabel("Global distance (m)")
plt.ylabel("Linear distance (m)")
plt.subplot(4,3,11)
data = [n for n in dist_per_dist_to_target["angular"].values()]
labels = [str(int(n*100)/100) for n in dist_per_dist_to_target["angular"].keys()]
plt.boxplot(data)
plt.xticks(np.arange(1,len(labels)+1,1),labels)
plt.title("Angular Distance (N_0 = "+str(N_0)+", N_OC = "+str(N_OC)+")")
plt.xlabel("Global distance (m)")
plt.ylabel("Angular distance (rad)")
plt.subplot(4,3,12)
data = [n for n in dist_per_dist_to_target["predicted"].values()]
labels = [str(int(n*100)/100) for n in dist_per_dist_to_target["predicted"].keys()]
plt.boxplot(data)
plt.xticks(np.arange(1,len(labels)+1,1),labels)
plt.title("Predicted Distance (N_0 = "+str(N_0)+", N_OC = "+str(N_OC)+")")
plt.xlabel("Global distance (m)")
plt.ylabel("Predicted distance (m)")

dist_lin = [n for v in dist_per_dist_to_target["linear"].values() for n in v]
dist_ang = [n for v in dist_per_dist_to_target["angular"].values() for n in v]
dist_pred = [n for v in dist_per_dist_to_target["predicted"].values() for n in v]
print("Results for N_0=50 and N_OC=200")
print("linear distance : ",np.mean(dist_lin)," +- ",np.std(dist_lin))
print("angular distance : ",np.mean(dist_ang)," +- ",np.std(dist_ang))
print("predicted distance : ",np.mean(dist_pred)," +- ",np.std(dist_pred))

N_0 = 25 
N_OC = 100

path = "/local/imaroger/catkin_ws/src/prediction_table/src/data/prediction/"+\
	'distance_prediction_'+ str(N_0) + "_" + str(N_OC) + '.json'
f_dist = open(path)
dist = json.load(f_dist)

for i in range(len(list_traj)):
	traj = list_traj[i]
	d_to_target = dist_to_target[i]
	dist_per_dist_to_target["linear"][d_to_target] += dist["linear"][traj]
	dist_per_dist_to_target["angular"][d_to_target] += dist["angular"][traj]	
	dist_per_dist_to_target["predicted"][d_to_target] += dist["predicted"][traj]


plt.subplot(4,3,1)
data = [n for n in dist_per_dist_to_target["linear"].values()]
labels = [str(int(n*100)/100) for n in dist_per_dist_to_target["linear"].keys()]
plt.boxplot(data)
plt.xticks(np.arange(1,len(labels)+1,1),labels)
plt.title("Linear Distance (N_0 = "+str(N_0)+", N_OC = "+str(N_OC)+")")
plt.xlabel("Global distance (m)")
plt.ylabel("Linear distance (m)")
plt.subplot(4,3,2)
data = [n for n in dist_per_dist_to_target["angular"].values()]
labels = [str(int(n*100)/100) for n in dist_per_dist_to_target["angular"].keys()]
plt.boxplot(data)
plt.xticks(np.arange(1,len(labels)+1,1),labels)
plt.title("Angular Distance (N_0 = "+str(N_0)+", N_OC = "+str(N_OC)+")")
plt.xlabel("Global distance (m)")
plt.ylabel("Angular distance (rad)")
plt.subplot(4,3,3)
data = [n for n in dist_per_dist_to_target["predicted"].values()]
labels = [str(int(n*100)/100) for n in dist_per_dist_to_target["predicted"].keys()]
plt.boxplot(data)
plt.xticks(np.arange(1,len(labels)+1,1),labels)
plt.title("Predicted Distance (N_0 = "+str(N_0)+", N_OC = "+str(N_OC)+")")
plt.xlabel("Global distance (m)")
plt.ylabel("Predicted distance (m)")

dist_lin = [n for v in dist_per_dist_to_target["linear"].values() for n in v]
dist_ang = [n for v in dist_per_dist_to_target["angular"].values() for n in v]
dist_pred = [n for v in dist_per_dist_to_target["predicted"].values() for n in v]
print("Results for N_0=25 and N_OC=100")
print("linear distance : ",np.mean(dist_lin)," +- ",np.std(dist_lin))
print("angular distance : ",np.mean(dist_ang)," +- ",np.std(dist_ang))
print("predicted distance : ",np.mean(dist_pred)," +- ",np.std(dist_pred))

N_0 = 75 
N_OC = 100

path = "/local/imaroger/catkin_ws/src/prediction_table/src/data/prediction/"+\
	'distance_prediction_'+ str(N_0) + "_" + str(N_OC) + '.json'
f_dist = open(path)
dist = json.load(f_dist)

for i in range(len(list_traj)):
	traj = list_traj[i]
	d_to_target = dist_to_target[i]
	dist_per_dist_to_target["linear"][d_to_target] += dist["linear"][traj]
	dist_per_dist_to_target["angular"][d_to_target] += dist["angular"][traj]	
	dist_per_dist_to_target["predicted"][d_to_target] += dist["predicted"][traj]


plt.subplot(4,3,7)
data = [n for n in dist_per_dist_to_target["linear"].values()]
labels = [str(int(n*100)/100) for n in dist_per_dist_to_target["linear"].keys()]
plt.boxplot(data)
plt.xticks(np.arange(1,len(labels)+1,1),labels)
plt.title("Linear Distance (N_0 = "+str(N_0)+", N_OC = "+str(N_OC)+")")
plt.xlabel("Global distance (m)")
plt.ylabel("Linear distance (m)")
plt.subplot(4,3,8)
data = [n for n in dist_per_dist_to_target["angular"].values()]
labels = [str(int(n*100)/100) for n in dist_per_dist_to_target["angular"].keys()]
plt.boxplot(data)
plt.xticks(np.arange(1,len(labels)+1,1),labels)
plt.title("Angular Distance (N_0 = "+str(N_0)+", N_OC = "+str(N_OC)+")")
plt.xlabel("Global distance (m)")
plt.ylabel("Angular distance (rad)")
plt.subplot(4,3,9)
data = [n for n in dist_per_dist_to_target["predicted"].values()]
labels = [str(int(n*100)/100) for n in dist_per_dist_to_target["predicted"].keys()]
plt.boxplot(data)
plt.xticks(np.arange(1,len(labels)+1,1),labels)
plt.title("Predicted Distance (N_0 = "+str(N_0)+", N_OC = "+str(N_OC)+")")
plt.xlabel("Global distance (m)")
plt.ylabel("Predicted distance (m)")


dist_lin = [n for v in dist_per_dist_to_target["linear"].values() for n in v]
dist_ang = [n for v in dist_per_dist_to_target["angular"].values() for n in v]
dist_pred = [n for v in dist_per_dist_to_target["predicted"].values() for n in v]
print("Results for N_0=75 and N_OC=100")
print("linear distance : ",np.mean(dist_lin)," +- ",np.std(dist_lin))
print("angular distance : ",np.mean(dist_ang)," +- ",np.std(dist_ang))
print("predicted distance : ",np.mean(dist_pred)," +- ",np.std(dist_pred))
plt.show()