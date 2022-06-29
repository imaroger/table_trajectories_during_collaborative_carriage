from __future__ import division
import numpy as np
import matplotlib.pyplot as plt
from math import cos,sin,sqrt
import pinocchio
from pinocchio import SE3, Quaternion
from pinocchio.rpy import rpyToMatrix
from mpl_toolkits.mplot3d import Axes3D

data = np.transpose(np.loadtxt("data/nmpc_traj_online.csv"))

# time = np.linspace(0,100, len(data[0]))

# plt.plot(time,data[16],label = 'foot L x')
# plt.plot(time,data[12],label = 'foot R x')
# plt.plot(time,data[17],label = 'foot L y')
# plt.plot(time,data[13],label = 'foot R y')
# plt.plot(time,data[18],label = 'foot R z')
# plt.plot(time,data[14],label = 'foot L z')
# plt.plot(time,data[0],label= 'com x')
# plt.plot(time,data[3],label= 'com y')
# plt.plot(time,data[9],label= 'zmp x')
# plt.plot(time,data[10],label= 'zmp y')
# legend = plt.legend()
# plt.show()

# plt.show()

# plt.plot(time_interp, traj[14])
# plt.plot(time_foot, footL[2])

# # plt.show()

com_file = open("data/interpolated/com.dat", "w")
am_file = open("data/interpolated/am.dat", "w")
phase_file = open("data/interpolated/phases.dat", "w")
footR_file = open("data/interpolated/rightFoot.dat", "w")
footL_file = open("data/interpolated/leftFoot.dat", "w")
before_start = True
for i in range (len(data[0])):
	if before_start and data[0][i] == -3.16e-3 and data[3][i] == 1.237384291203724555e-03:
		print("Write nothing")
	else :
		before_start = False
		com_file.write(str(data[0][i]) + "  " + str(data[3][i]) + "  8.786810585901939641e-01   " +\
			str(data[1][i]) + "  " + str(data[4][i])+ "  0.0  " +\
			str(data[2][i]) + "  " + str(data[5][i])+ "  0.0\n")

		am_file.write(str(data[6][i]) + "  " + str(data[7][i])+ "  " + str(data[8][i]) + "\n")

		rotR = rpyToMatrix(0,0,data[15][i])

		footR_file.write(str(data[12][i]) + "  " + str(data[13][i])+ "  " +\
			str(data[14][i]) + "  " + str(rotR[0][0]) + "  " +\
			str(rotR[0][1]) + "  " + str(rotR[0][2]) + "  " + str(rotR[1][0]) +\
			"  " + str(rotR[1][1]) + "  " + str(rotR[1][2])+ "  " +\
			str(rotR[2][0]) + "  " + str(rotR[2][1])+ "  " + str(rotR[2][2])+"\n")
			

		rotL = rpyToMatrix(0,0,data[19][i])

		footL_file.write(str(data[16][i]) + "  " + str(data[17][i])+ "  " +\
			str(data[18][i]) + "  " + str(rotL[0][0]) + "  " +\
			str(rotL[0][1]) + "  " + str(rotL[0][2]) + "  " + str(rotL[1][0]) +\
			"  " + str(rotL[1][1]) + "  " + str(rotL[1][2])+ "  " +\
			str(rotL[2][0]) + "  " + str(rotL[2][1])+ "  " + str(rotL[2][2])+"\n")
			

		if data[14][i] == 0.105 and data[18][i] == 0.105: # DS Phase
			# print(0)
			phase_file.write("0\n")
		elif data[14][i] != 0.105: # SS Phase : Left = Support foot
			# print(1)
			phase_file.write("1\n")	
		else :  # SS Phase : Right = Support foot
			# print(-1)			
			phase_file.write("-1\n")	

for k in range(6):
	com_file.write(str(data[0][-1]) + "  " + str(data[3][-1]) + "  8.786810585901939641e-01   " +\
		str(data[1][-1]) + "  " + str(data[4][-1])+ "  0.0  " +\
		str(data[2][-1]) + "  " + str(data[5][-1])+ "  0.0\n")

	rotR = rpyToMatrix(0,0,data[15][-1])

	footR_file.write(str(data[12][-1]) + "  " + str(data[13][-1])+ "  " +\
		str(data[14][-1]) + "  " + str(rotR[0][0]) + "  " +\
		str(rotR[0][1]) + "  " + str(rotR[0][2]) + "  " + str(rotR[1][0]) +\
		"  " + str(rotR[1][1]) + "  " + str(rotR[1][2])+ "  " +\
		str(rotR[2][0]) + "  " + str(rotR[2][1])+ "  " + str(rotR[2][2])+"\n")

	rotL = rpyToMatrix(0,0,data[19][-1])

	footL_file.write(str(data[16][-1]) + "  " + str(data[17][-1])+ "  " +\
		str(data[18][-1]) + "  " + str(rotL[0][0]) + "  " +\
		str(rotL[0][1]) + "  " + str(rotL[0][2]) + "  " + str(rotL[1][0]) +\
		"  " + str(rotL[1][1]) + "  " + str(rotL[1][2])+ "  " +\
		str(rotL[2][0]) + "  " + str(rotL[2][1])+ "  " + str(rotL[2][2])+"\n")	

	phase_file.write("0\n")


com_file.close()
am_file.close()
phase_file.close()
footR_file.close()
footL_file.close()

com = np.transpose(np.loadtxt("data/interpolated/com.dat"))
footR = np.transpose(np.loadtxt("data/interpolated/rightFoot.dat"))
footL = np.transpose(np.loadtxt("data/interpolated/leftFoot.dat"))
phase = np.transpose(np.loadtxt("data/interpolated/phases.dat"))
am = np.transpose(np.loadtxt("data/interpolated/am.dat"))
time = np.arange(0,len(com[0])*0.001,0.001)

print("t_tot = ",len(com[0])*0.001," s")
d = sqrt((com[0][-1]-com[0][0])**2+(com[1][-1]-com[1][0])**2)
print("d_tot = ",d," m")
ind = np.where(phase == 0)[0]
dL,dR = [],[]
for i in range(len(ind)-1):
	dL.append(sqrt((footL[0][ind[i+1]]-footL[0][ind[i]])**2+(footL[1][ind[i+1]]-footL[1][ind[i]])**2))
	dR.append(sqrt((footR[0][ind[i+1]]-footR[0][ind[i]])**2+(footR[1][ind[i+1]]-footR[1][ind[i]])**2))

all_d = np.array(dL+dR)
print("biggest step (m) = ",np.max(all_d))
print("average step (m) = ",np.mean(all_d[np.where(all_d != 0)[0]]))
print("max speed (cm.s-1) = ",np.max(all_d)/(0.3*7)*100)

plt.plot(com[0],com[1],color = 'blue',label='CoM')
plt.plot(footL[0],footL[1],color = 'green',label='Left Foot')
plt.plot(footR[0],footR[1],color = 'red',label='Right Foot')
legend = plt.legend()
plt.show()


# velL = []
# velR = []
# for i in range(len(phase)-1):
# 	if(phase[i] == -1):
# 		d = sqrt((footL[0][i+1]-footL[0][i])**2+(footL[1][i+1]-footL[1][i])**2)
# 		velL.append(d/0.001)
# 		velR.append(0)
# 	elif(phase[i] == 1): 
# 		d = sqrt((footR[0][i+1]-footR[0][i])**2+(footR[1][i+1]-footR[1][i])**2)
# 		velR.append(d/0.001)	
# 		velL.append(0)
# 	else:
# 		velR.append(0)
# 		velL.append(0)

# vel_ref = np.transpose(np.loadtxt("/local/imaroger/catkin_ws/src/prediction_table/src/data/velocity.dat"))
# time_ref = np.linspace(0,len(com[0])*0.001,len(vel_ref))

# plt.plot(time[:-1],velR,label = 'v right foot')
# plt.plot(time[:-1],velL,label = 'v left foot')
# plt.plot(time_ref,vel_ref,label = 'v ref')
# plt.legend()
# plt.show()

# plt.plot(time,footR[2],label = 'foot R z')
# plt.plot(time,footL[2],label = 'foot L z')
# plt.plot(time,phase,label = 'phase')
# legend = plt.legend()
# plt.show()

# plt.plot(time,footL[0],label = 'foot L x')
# plt.plot(time,footR[0],label = 'foot R x')
# plt.plot(time,footL[1],label = 'foot L y')
# plt.plot(time,footR[1],label = 'foot R y')
# plt.plot(time,footR[2],label = 'foot R z')
# plt.plot(time,footL[2],label = 'foot L z')
# plt.plot(time,com[0],label= 'com x')
# plt.plot(time,com[1],label= 'com y')
# plt.plot(time,com[0]-com[2]/9.81*com[6],label= 'zmp x')
# plt.plot(time,com[1]-com[2]/9.81*com[7],label= 'zmp y')
# plt.plot(time,phase,label = 'phase', color='black', lw=0.5)
# legend = plt.legend()
# plt.show()

# plt.plot(time,np.arccos(footR[3]))
# plt.plot(time,np.arccos(footL[3]))
# plt.show()

# # time_reduced = np.arange(0,len(footR[0]),100)
# # arrow_len = 0.1

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.plot3D(com[0],com[1],com[2],color = 'red',label='CoM')
# ax.plot3D(com[0]-com[2]/9.81*com[6],com[1]-com[2]/9.81*com[7],[0]*len(com[0]),color = 'blue',label='ZMP')
# ax.plot3D(footL[0],footL[1],footL[2],color = 'green',label='Left Foot')
# ax.plot3D(footR[0],footR[1],footR[2],color = 'orange',label='Right Foot')
# legend = plt.legend()

# # ax.quiver(np.array(footR[0])[time_reduced],np.array(footR[1])[time_reduced],\
# # 	np.array(footR[2])[time_reduced],np.array(footR[3])[time_reduced],\
# # 	np.array(footR[6])[time_reduced],0, length=arrow_len, lw = 2)
# # ax.quiver(np.array(footL[0])[time_reduced],np.array(footL[1])[time_reduced],\
# # 	np.array(footL[2])[time_reduced],np.array(footL[3])[time_reduced],\
# # 	np.array(footL[6])[time_reduced],0, length=arrow_len, lw = 2)

# # ax.quiver(np.array(com[0])[time_reduced],np.array(com[1])[time_reduced],\
# # 	np.array(com[2])[time_reduced],np.array(np.cos(am[0]))[time_reduced],\
# # 	np.array(np.sin(am[0]))[time_reduced],0, length=arrow_len, lw = 2)


# plt.show()

