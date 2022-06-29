#!/usr/bin/python

import os, sys
import time
import numpy as np
# np.set_printoptions(threshold=np.nan, linewidth =np.nan)
from walking_generator.visualization_traj import PlotterTraj
from walking_generator.combinedqp_traj import NMPCGeneratorTraj
from walking_generator.interpolation_traj import Interpolation

from math import sqrt,floor
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev

import rospy
from estimation.msg import TrajMsg, NmpcMsg
from math import cos,sin,pi,sqrt
from std_msgs.msg import Bool, Float64

# def resizeTraj(x, y, theta, velocity_ref):
#     traj_length = len(x)
#     # print("lenx",traj_length)

#     okay = np.where(np.abs(np.diff(x)) + np.abs(np.diff(y)) > 0)
#     x,y = x[okay],y[okay]
#     # print(x,y)
#     tck, u = splprep([x, y], s=0)
#     unew = np.linspace(0,1,traj_length)
#     data = splev(unew, tck)
#     x,y = data[0],data[1]

#     ind = np.where(np.abs(np.diff(theta))>0.2)

#     max_delta_ori = np.max(np.abs(np.diff(theta)))
#     if max_delta_ori < 0.8:
#         velocity_low = 0.05
#     elif max_delta_ori < 1.7:
#         velocity_low = 0.001
#     elif max_delta_ori < 2.8:
#         velocity_low = 0.0005
#     else:
#         velocity_low = 0.0001

#     # print("vel_low",velocity_low)

#     ind_partition, d  = [[0]], []
#     i,previous = 0,"ref"
#     while i < traj_length-1:
#         if np.sum(np.isin(ind,i)) == 0:
#             if previous == "low":     
#                 ind_partition.append([])           
#                 ind_partition[-1].append(i)
#             ind_partition[-1].append(i+1)
#             previous = "ref"
#             i+=1
#         else:
#             if previous == "ref":
#                 ind_partition.append([])              
#                 ind_partition[-1].append(i)
#             ind_partition[-1].append(i+1)                           
#             previous = "low"
#             i+=1

#     # print("ind_part",ind_partition)

#     new_length_list = []

#     for k in range(len(ind_partition)):
#         d.append(0)
#         for i in ind_partition[k][:-1]:
#             d[-1] += sqrt((x[i+1]-x[i])**2 + (y[i+1]-y[i])**2)
#         if k%2 == 0:
#             t = (d[-1])/velocity_ref
#         else:
#             t = d[-1]/velocity_low
#         # print(t)
#         new_length_list.append(int((floor(t/0.2))))   
#     # print("d",d[-1]/velocity_ref,(d[-1]+delta_d)/velocity_ref)
#     # print("new_len",new_length_list)     
    
#     new_x,new_y,new_theta = np.array([]),np.array([]),np.array([])
#     i = 0

#     if np.sum(new_length_list) > 16:
#         for length in new_length_list:
#             if length != 0:
#                 ind = np.array(ind_partition[i])
#                 current_x,current_y,current_theta = x[ind],y[ind],theta[ind]

#                 new_time = np.linspace(0,1,length)
#                 old_time = np.linspace(0,1,len(ind))
#                 current_x = np.interp(new_time,old_time,current_x) 
#                 current_y = np.interp(new_time,old_time,current_y)              
#                 current_theta = np.interp(new_time,old_time,current_theta)  

#                 new_x = np.concatenate((new_x,current_x))
#                 new_y = np.concatenate((new_y,current_y))
#                 new_theta = np.concatenate((new_theta,current_theta))
#             i += 1
#     else:
#         new_time = np.linspace(0,1,16)
#         old_time = np.linspace(0,1,len(x))
#         new_x = np.interp(new_time,old_time,x) 
#         new_y = np.interp(new_time,old_time,y)              
#         new_theta = np.interp(new_time,old_time,theta)  

#     new_traj = np.zeros((3,len(new_x)), dtype=float)
#     new_traj[0],new_traj[1],new_traj[2] = new_x,new_y,new_theta
#     return new_traj

def resizeTraj(x,y,theta,velocity_ref):
    traj_length = len(x)
    N, T = 16,0.3
    okay = np.where(np.abs(np.diff(x)) + np.abs(np.diff(y)) > 0)
    x,y = x[okay],y[okay]
    tck, u = splprep([x, y], s=0)
    unew = np.linspace(0,1,traj_length)
    data = splev(unew, tck)
    x,y = data[0],data[1]

    d = sqrt((x[1]-x[0])**2+(y[1]-y[0])**2)
    # print(d)

    d_2steps = N*T*velocity_ref
    # print(d_2steps)

    # print(d_2steps/d)

    new_length = round(traj_length*N/(d_2steps/d))
    print(new_length)

    # okay = np.where(np.abs(np.diff(x)) + np.abs(np.diff(y)) > 0)
    # new_x,new_y = x[okay],y[okay]
    # tck, u = splprep([new_x, new_y], s=0)
    # unew = np.linspace(0,1,new_length)
    # data = splev(unew, tck)
    # new_x,new_y = data[0],data[1]  

    # print(sqrt((new_x[1]-new_x[0])**2+(new_y[1]-new_y[0])**2)*16)

    new_time = np.linspace(0,1,new_length)
    old_time = np.linspace(0,1,traj_length)
    new_x = np.interp(new_time,old_time,x)
    new_y = np.interp(new_time,old_time,y)    

    # plt.plot(x,y,linestyle=':', marker='o')   
    # plt.plot(new_x,new_y,linestyle=':', marker='o')  
    # plt.show()

    new_time = np.linspace(0,1,new_length)
    old_time = np.linspace(0,1,traj_length)
    new_theta = np.interp(new_time,old_time,theta)

    # plt.plot(old_time,theta,linestyle=':', marker='o')
    # plt.plot(new_time[:-1],np.diff(new_theta))
    # plt.plot(old_time[:-1],np.diff(theta))    
    # plt.show()

    new_traj = np.zeros((3,len(new_x)), dtype=float)
    new_traj[0],new_traj[1],new_traj[2] = new_x,new_y,new_theta
    return new_traj


def initToZero(x_0, y_0, theta_0, x, y, theta):
    x_local,y_local = (x-x_0)*np.cos(theta_0) + (y-y_0)\
        *np.sin(theta_0), -(x-x_0)*np.sin(theta_0) + \
        (y-y_0)*np.cos(theta_0)
    theta_local = theta - theta_0 
    return x_local, y_local, theta_local

def zeroToInit(x_0, y_0, theta_0, x_local, y_local, theta_local):
    x,y = x_0 + x_local*cos(theta_0) - y_local*sin(theta_0),\
        y_0 + x_local*sin(theta_0) + y_local*cos(theta_0)
    theta = theta_0+theta_local
    return x,y,theta

def nmpcResults2Msg(comx,comy,comz,comq,footx,footy,footq,foot,future_footx,\
        future_footy,future_footq):
    msg = NmpcMsg()

    msg.com_pose.position.x = comx
    msg.com_pose.position.y = comy
    msg.com_pose.position.z = comz

    msg.com_pose.orientation.x = 0
    msg.com_pose.orientation.y = 0
    msg.com_pose.orientation.z = sin(comq/2)     
    msg.com_pose.orientation.w = cos(comq/2) 

    msg.foot_pose.position.x = footx
    msg.foot_pose.position.y = footy
    msg.foot_pose.position.z = 0   

    msg.foot_pose.orientation.x = 0
    msg.foot_pose.orientation.y = 0
    msg.foot_pose.orientation.z = sin(footq/2)      
    msg.foot_pose.orientation.w = cos(footq/2)  

    msg.future_foot_pose.position.x = future_footx
    msg.future_foot_pose.position.y = future_footy
    msg.future_foot_pose.position.z = 0   

    msg.future_foot_pose.orientation.x = 0
    msg.future_foot_pose.orientation.y = 0
    msg.future_foot_pose.orientation.z = sin(future_footq/2)      
    msg.future_foot_pose.orientation.w = cos(future_footq/2)  

    msg.foot = foot

    return msg

def findGoodInd(x,y,N_OC,N_0,d):
    start = []
    if d < 0:
        for i in range(N_0,-1,-1):
            d_i = sqrt((x[N_0]-x[i])**2+(y[N_0]-y[i])**2)
            if d_i >= abs(d) :
                start.append(i)
        if len(start) == 0:
            return 0
        else:
            return start[0]
    else :
        ind_f = N_0+int((N_OC-N_0)/2)
        for i in range(N_0,ind_f+1,1):
            d_i = sqrt((x[N_0]-x[i])**2+(y[N_0]-y[i])**2)
            if d_i >= d:
                start.append(i)
        if len(start) == 0:
            return ind_f
        else:
            return start[0]    

def translateTraj(x, y, th, N_0,d,ind):
    if ind == 1:
        x,y =  d*(cos(th[N_0]))+np.array(x),d*(sin(th[N_0]))+np.array(y)
        th = np.array(th)+pi
    if ind == 2:
        x,y =  -d*(cos(th[N_0]))+np.array(x),-d*(sin(th[N_0]))+np.array(y)
    return x,y,th

########################################################################
################################## MAIN ################################
########################################################################

class estimation_pub:

    def __init__(self,path):
        self.sub = rospy.Subscriber("estimated_trajectory", TrajMsg, self.callback)
        self.pub = rospy.Publisher("nmpc_generator", NmpcMsg, queue_size=10)
        self.pub_qp_solver_cv = rospy.Publisher("qp_solver_cv", Bool, queue_size=10)
        self.pub_vel = rospy.Publisher("human_vel",Float64, queue_size=10)
        self.pub_dist = rospy.Publisher("dist_human_robot",Float64, queue_size=10)
        self.r = rospy.get_param('rate')

        # instantiate pattern generator
        self.N = 16
        self.T = 0.3
        T_step= self.N*self.T/2
        self.nmpc = NMPCGeneratorTraj(N=self.N, T=self.T, T_step=T_step,fsm_state='D')
        self.nmpc.set_security_margin(0.085, 0.03)

        # set initial values 
        comx = [-3.16e-3, 0.0, 0.0]
        comy = [1.237384291203724555e-03,0.0, 0.0]
        comz = 8.786810585901939641e-01 
        footx = 1.86e-4
        footy = 0.085
        footq = 0.0
        # self.foot='left'
        # self.comq = [0.0,0.0, 0.0]
        self.nmpc.set_initial_values(comx, comy, comz, \
            footx, footy, footq, 'left')

        self.interp_nmpc = Interpolation(0.001,self.nmpc)

        self.x0,self.y0,self.th0 = 0,0,0

        self.distance = rospy.get_param('distance')
        self.N_0 = rospy.get_param('N_0')
        self.N_OC = rospy.get_param('N_OC')
        self.sub1 = not(rospy.get_param('display_sub1')) #True if the robot is subject 1, else False
        self.dist_to_table = rospy.get_param('dist_to_table')

        self.ind_start = 0
        self.ind_current = 0
        self.count = 0
        self.count_end = 0        
        self.n_end = int(self.N/2)+1
        self.old_traj_ref = []

        self.f_handle = open(path, 'a')       

    def callback(self, traj):
        done = True
        d = self.distance
        N_0 = self.N_0
        N_OC = self.N_OC
        status = rospy.get_param('human_status')

        x, y, theta = traj.x_traj, traj.y_traj, traj.theta_traj

        print("---",status, len(x))

        if status == 'Start':
            if self.sub1:
                x, y, theta = translateTraj(x, y, theta, 0,self.dist_to_table,1)
            else:
                x, y, theta = translateTraj(x, y, theta, 0,self.dist_to_table,2)
            trajectory_reference = np.zeros((3,self.N), dtype=float)
            self.x0,self.y0,self.th0 = x[0],y[0],theta[0] 
            # trajectory_reference[0],trajectory_reference[1],trajectory_reference[2] = [x[0]]*self.N,[y[0]]*self.N,[theta[0]]*self.N
        elif status == 'Walk':
            # print(x[N_0],y[N_0])

            if self.sub1:
                x, y, theta = translateTraj(x, y, theta, N_0,self.dist_to_table,1)
            else:
                x, y, theta = translateTraj(x, y, theta, N_0,self.dist_to_table,2)

            dist_from_start = sqrt((x[N_0]-x[0])**2+(y[N_0]-y[0])**2)
            dist_dpos = sqrt((x[N_0]-x[N_0+int((N_OC-N_0)/2)])**2+(y[N_0]-y[N_0+int((N_OC-N_0)/2)])**2)

            # print("dist_avt : ",d,dist_from_start,dist_dpos)
            # print(self.count_N0)

            if (d == 0 or (d < 0 and abs(d) <= dist_from_start) or (d > 0 and d <= dist_dpos)):
                if self.x0 == 0 and self.y0 == 0 and self.th0 == 0:
                    if d == 0:
                        self.ind_start = N_0
                        self.ind_current = N_0
                    else :
                        ind = findGoodInd(x,y,N_OC,N_0,d)
                        self.ind_start,self.ind_current = ind,ind
                        # print("----",self.count_N0,"----")

                    self.x0,self.y0,self.th0 = x[self.ind_start],y[self.ind_start],theta[self.ind_start] 
                
                else :
                    if d != 0:
                        self.ind_current = findGoodInd(x,y,N_OC,N_0,d)             

                # x0, y0, th0 = x[self.ind_start],y[self.ind_start],theta[self.ind_start]     
                velocity_ref = (sqrt((x[N_0]-x[0])**2+(y[N_0]-y[0])**2))*self.r/(N_0+1)
                # print("vel : ",sqrt((x[N_0]-x[0])**2+(y[N_0]-y[0])**2)*self.r/(N_0+1))
                vel_msg = Float64()
                vel_msg.data = (sqrt((x[N_0]-x[0])**2+(y[N_0]-y[0])**2))*self.r/(N_0+1)
                self.pub_vel.publish(vel_msg) 

                print(velocity_ref)
                np.savetxt(self.f_handle, [velocity_ref])

                x, y, theta = initToZero(self.x0, self.y0, self.th0, \
                    np.array(x[self.ind_current:]), np.array(y[self.ind_current:]), np.array(theta[self.ind_current:]))
                resized_traj = resizeTraj(x, y, theta, velocity_ref)

                trajectory_reference = resized_traj[:,0:self.N]
                self.old_traj_ref = trajectory_reference
        else: 
            if self.count_end < self.n_end:     
                if self.sub1:
                    x, y, theta = translateTraj(x, y, theta, 0,self.dist_to_table,1)
                else:
                    x, y, theta = translateTraj(x, y, theta, 0,self.dist_to_table,2) 

                trajectory_reference = np.zeros((3,self.N), dtype=float)
                x_old_ref,y_old_ref,th_old_ref = self.old_traj_ref[0][0],self.old_traj_ref[1][0],self.old_traj_ref[2][0]

                trajectory_reference[0],trajectory_reference[1],trajectory_reference[2]=\
                [x_old_ref]*2*self.N,[y_old_ref]*2*self.N,[th_old_ref]*2*self.N
                self.count_end += 1
            else:
                rospy.signal_shutdown("Walk shutdown")       

        self.nmpc.   set_trajectory_reference(trajectory_reference)

        # solve QP
        nb_failures = self.nmpc.   solve()
        self.nmpc.   simulate()
        self.interp_nmpc.interpolate(self.count*self.T)

        # initial value embedding by internal states and simulation
        comx, comy, comz, footx, footy, footq, foot, comq, future_footx,\
            future_footy, future_footq = \
        self.nmpc.update()
        self.nmpc.set_initial_values(comx, comy, comz, \
            footx, footy, footq, foot, comq)

        if nb_failures != 0:
            done = False      

        # print(done)
        self.pub_qp_solver_cv.publish(done)

        if nb_failures <= 2:

            # self.comx = comx
            # self.comy = comy
            # self.comz = comz
            # self.comq = comq
            # self.footx = footx
            # self.footy = footy
            # self.footq = footq
            # self.foot = foot
            
            comx, comy, comq = zeroToInit(self.x0, self.y0, self.th0, comx[0], comy[0], comq[0])
            footx, footy, footq = zeroToInit(self.x0, self.y0, self.th0, footx, footy, footq)
            future_footx, future_footy, future_footq = zeroToInit(self.x0, self.y0, self.th0, future_footx, future_footy, future_footq)     

            # dist = sqrt((traj.x_traj[N_0]-comx)**2+(traj.y_traj[N_0]-comy)**2)

            # # if d > 0:
            # #     delta_d = d - dist
            # # elif d < 0:
            # #     delta_d = d + dist                                          

            # # print("dist : ",dist)  
            # dist_msg = Float64()
            # dist_msg.data = dist    
            # self.pub_dist.publish(dist_msg)       

            nmpc_msg = nmpcResults2Msg(comx,comy,comz,comq,footx,footy,footq,\
                foot,future_footx, future_footy, future_footq)
            self.pub.publish(nmpc_msg)
            self.count += 1
        else:
            print("*** QP failed ***")
            nmpc_msg = nmpcResults2Msg(0,0,0,0,0,0,0,"none",0,0,0)
            self.pub.publish(nmpc_msg)  


    def save_data(self):
        print("")
        self.interp_nmpc.save_to_file("/local/imaroger/catkin_ws/src/prediction_table/src/data/nmpc_traj_online.csv")
        

if __name__ == '__main__':
    try:
        rospy.init_node('NmpcOnline', anonymous=True)
        path = "/local/imaroger/catkin_ws/src/prediction_table/src/data/velocity.dat"
        open(path,"w")
        estimator = estimation_pub(path)
        
        while not rospy.is_shutdown():
            rospy.spin()
            estimator.save_data()
    except rospy.ROSInterruptException:
        print("EstimationOC Shutting down")

