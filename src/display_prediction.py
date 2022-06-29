import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import splprep, splev
from scipy.stats import pearsonr
from math import sqrt,pi
from scipy import stats
import json

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

traj_name = 'd1_p5_gris_1'
pair_name = 'Sujet1_Amaury&Sujet2_Jason'
pair_num = pair_to_num[pair_name]
N_0 = 50 
N_OC = 100
leader = 'Sujet 1 et 2'

path_pred = "/local/imaroger/catkin_ws/src/prediction_table/src/data/prediction/"+\
	str(N_0) + "_" + str(N_OC) + "/" +\
	traj_name + "_" + pair_num + "_" + str(N_0) + "_" + str(N_OC) + ".dat"
pred = np.loadtxt(path_pred)

path_traj = "/local/imaroger/catkin_ws/src/trajectory_with_table/src/Data/Human/"+\
	traj_name + ".json"
f_traj = open(path_traj)
data = json.load(f_traj)['Trajectoires_Individuelles']['Table']

for i in range(len(data)):
	if data[i]["Leader"] == leader and data[i]["Binome"] == pair_name:
		data_x,data_y = data[i]["x"],data[i]["y"]
		data_th = data[i]["Orientation_Globale"]


for i in range(len(pred)):
	pred_x,pred_y,pred_th = pred[i][:N_OC+1],pred[i][N_OC+1:2*N_OC+2],pred[i][2*N_OC+2:3*N_OC+3]
	plt.plot(pred_x,pred_y,linewidth = 0.5)
plt.plot(data_x,data_y,color = "black")

plt.show()



