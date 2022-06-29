#!/usr/bin/python

import os
import rospy
import time
import roslaunch
import subprocess
import numpy as np
import sys

def oneLoop(traj_name,pair_name,N_0,N_OC):  
    # Start estimation
    rospy.init_node('Estimation', anonymous=True)

    pkg_name='prediction_table'

    # print("Enter to set the parameters ...")
    # input() 

    display_sub1, display_sub2 = False, False
    leader = 'Sujet 1 et 2'

    rospy.set_param('traj_name', traj_name) 
    rospy.set_param('pair_name', pair_name) 
    rospy.set_param('leader_name', leader)
    rospy.set_param('display_sub1',display_sub1)
    rospy.set_param('display_sub2',display_sub2)
    rospy.set_param('data_recording',True)

    rate = 50
    rospy.set_param('rate',rate)  

    # Choose the parameters for the estimation process
    rospy.set_param('N_0',N_0)
    rospy.set_param('N_OC',N_OC) 
      
    # Define if the human starts walking (t<T_0), walks (T_0<t<T_f) or stops (t>T_f)
    rospy.set_param('human_status', "Start")   

    time.sleep(2)

    executable='real_xp.py'
    node_name='XP'
    xp_node = roslaunch.core.Node(pkg_name, executable,name=node_name)

    launch_xp =roslaunch.scriptapi.ROSLaunch()
    launch_xp.start()

    launch_xp.launch(xp_node)

    executable='estimation_table.py'
    node_name='estimation_table'
    estimation_node = roslaunch.core.Node(pkg_name, executable,name=node_name)

    launch_estimation =roslaunch.scriptapi.ROSLaunch()
    launch_estimation.start()

    launch_estimation.launch(estimation_node)

    print("... done")

    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        # print(rospy.get_param('human_status')) 
        r.sleep()      
        if rospy.get_param('human_status') == "Stop":
            print("Stop all the process")
            rospy.signal_shutdown("Stop") 
        # rospy.spin()


if __name__ == "__main__":
    print(sys.argv[1],sys.argv[2],sys.argv[3],sys.argv[4])

    process = subprocess.Popen('roscore')

    traj_name = sys.argv[1]
    pair_name = sys.argv[2]
    N_0,N_OC = int(sys.argv[3]),int(sys.argv[4])

    oneLoop(traj_name,pair_name,N_0,N_OC)
    print('Finish one traj ! Subject :',pair_name)
    # time.sleep(5)





