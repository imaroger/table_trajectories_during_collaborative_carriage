#!/usr/bin/python

import os
import rospy
import time
import roslaunch
import subprocess

if __name__ == "__main__":
    roscore = subprocess.Popen('roscore')
    time.sleep(1)

    # Start RViz 
    print("Starting RViz")
    RViz_node = roslaunch.core.Node('rviz', 'rviz',name='rviz')

    RViz_launch=roslaunch.scriptapi.ROSLaunch()
    RViz_launch.start()

    RViz = RViz_launch.launch(RViz_node)
    time.sleep(2)

    # Start estimation
    rospy.init_node('WalkWithTable', anonymous=True)

    pkg_name='prediction_table'

    # print("Enter to set the parameters ...")
    # input() 

    ## Choose the human trajectory to consider
    ## human_traj is in ['d1_p4_jaune_1', 'd1_p4_gris_1', 'd1_p5_jaune_1', 
    # 'd1_p5_gris_1', 'd1_p6_jaune_1', 'd1_p6_gris_1', 'd2_p7_jaune_1', 
    # 'd2_p7_gris_1', 'd3_p7_gris_1', 'd1_p4_jaune_2', 'd1_p4_gris_2',
    # 'd1_p5_jaune_2', 'd1_p5_gris_2', 'd1_p6_jaune_2', 'd1_p6_gris_2', 
    # 'd2_p7_jaune_2', 'd2_p7_gris_2', 'd3_p7_gris_2']
    ## pair is in ['Sujet1_Aurélie&Sujet2_Stanislas', 'Sujet1_Rémy&Sujet2_Margaux', 
    # 'Sujet1_Zaki&Sujet2_Yanis', 'Sujet1_Thanh&Sujet2_Diane', 
    # 'Sujet1_Sabrina&Sujet2_Quentin', 'Sujet1_Aniss&Sujet2_Louise', 
    # 'Sujet1_Hugo&Sujet2_Alexandre', 'Sujet1_Alexia&Sujet2_Bénédicte', 
    # 'Sujet1_Adénikè&Sujet2_Médéric', 'Sujet1_Anaïs&Sujet2_Mariem', 
    # 'Sujet1_Stéphane&Sujet2_Angélique', 'Sujet1_Fanny&Sujet2_William', 
    # 'Sujet1_Romane&Sujet2_Corentin', 'Sujet1_Paul&Sujet2_Mathieu', 
    # 'Sujet1_Marine&Sujet2_Hélène', 'Sujet1_Sébastien&Sujet2_Nils', 
    # 'Sujet1_Antoine&Sujet2_Médéric_LAAS', 'Sujet1_Amaury&Sujet2_Jason', 
    # 'Sujet1_Guilhem&Sujet2_César', 'Sujet1_Alexis&Sujet2_Thibaud']
    ## leader is in ['Sujet 1','Sujet 2','Sujet 1 et 2']

    # 'Sujet1_Amaury&Sujet2_Jason' 'd1_p5_gris_1' "Sujet 1 et 2" sub1 : OK sub2:OK
    # 'Sujet1_Amaury&Sujet2_Jason' 'd1_p5_gris_1' "Sujet 1" sub1 : OK sub2:OK    
    # 'Sujet1_Amaury&Sujet2_Jason' 'd1_p5_jaune_1' "Sujet 1 et 2" sub1 : OK sub2:OK      
    # 'Sujet1_Amaury&Sujet2_Jason' 'd2_p7_jaune_1' "Sujet 1 et 2" sub1 : OK     
    # 'Sujet1_Amaury&Sujet2_Jason' 'd2_p7_gris_1' "Sujet 1 et 2" sub1 : OK 
    rospy.set_param('traj_name', 'd1_p5_jaune_1') 
    rospy.set_param('pair_name', 'Sujet1_Amaury&Sujet2_Jason') 
    rospy.set_param('leader_name', 'Sujet 1 et 2')
    rospy.set_param('display_sub1',False) 
    rospy.set_param('display_sub2',True)
    rospy.set_param('data_recording',False)

    # Choose the parameters for the estimation process
    rospy.set_param('rate',20) #30
    rospy.set_param('N_0',50) #25
    rospy.set_param('N_OC',200) #150
    rospy.set_param('distance',0.) 
    rospy.set_param('dist_to_table',0.95)   

    # Define if the human starts walking (t<T_0), walks (T_0<t<T_f) or stops (t>T_f)
    rospy.set_param('human_status', "Start")   

    print("... done")

    executable='marker_rviz_walk_table.py'
    node_name='RVizMarkers'
    marker_RViz_node = roslaunch.core.Node(pkg_name, executable,name=node_name)

    launch_marker_RViz =roslaunch.scriptapi.ROSLaunch()
    launch_marker_RViz.start()

    MarkerRviz = launch_marker_RViz.launch(marker_RViz_node)

    print("Enter to start the prediction process ...")
    input()

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

    executable='walk_table.py'
    node_name='walk_table'
    walk_node = roslaunch.core.Node(pkg_name, executable,name=node_name)

    launch_walk =roslaunch.scriptapi.ROSLaunch()
    launch_walk.start()

    launch_walk.launch(walk_node)

    print("... done")

    # r = rospy.Rate(1)
    while not rospy.is_shutdown():
        print("Running")
        if not RViz.is_alive():
            print("Stopping RViz")
            rospy.signal_shutdown("RViz stop running")
            # Terminate the roscore subprocess
            print("Stop roscore")
            roscore.terminate()
        rospy.spin()

