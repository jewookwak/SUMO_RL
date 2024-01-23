#!/usr/bin/env python
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
# Copyright (C) 2008-2023 German Aerospace Center (DLR) and others.
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# https://www.eclipse.org/legal/epl-2.0/
# This Source Code may also be made available under the following Secondary
# Licenses when the conditions for such availability set forth in the Eclipse
# Public License 2.0 are satisfied: GNU General Public License, version 2
# or later which is available at
# https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
# SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later

# @file    runner.py
# @author  Michael Behrisch
# @date    2012-12-09

from __future__ import absolute_import
from __future__ import print_function


import os
import subprocess
import sys
import shutil
import time

import sumolib
import optparse
from sumolib import checkBinary  # noqa
import traci  # noqa
import traci.constants as tc
import pandas as pd
import numpy as np
import random
import matplotlib.pyplot as plt
import pickle
import joblib
from enum import Enum
import math
sys.path.append(
    os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', "tools"))
sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
    os.path.dirname(__file__), "..", "..", "..")), "tools"))
from sumolib import checkBinary  # noqa
import sumolib
class State(Enum):
    Cruise = 0
    Rear_end_collision_avoidance=1

#svc_RBF = joblib.load('svc_RBF.pkl')  #충돌 경계 값 svc
svc_RBF_3d = joblib.load('3d_svc_RBF2.pkl')
ego=[]
leftrear0=[]
leftrear1=[]
rear=[]
rightrear0=[]
rightrear1=[]
leftforward0=[]
leftforward1=[]
leftforward2=[]
forward=[]
rightforward0=[]
rightforward1=[]
rightforward2=[]

collision =[]
truck0_TTC = []
truck1_TTC = []
input = []
# vehicles_tau=[random.uniform(0.3,1),random.uniform(0.3,1),random.uniform(0.3,1),random.uniform(0.3,1),random.uniform(0.3,1),random.uniform(0.3,1),random.uniform(0.3,1),random.uniform(0.3,1),random.uniform(0.3,1),random.uniform(0.3,1),random.uniform(0.3,1)]
vehicles_tau=[random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27)]
print("leftrear0's tau : ",vehicles_tau[0])
print("leftrear1's tau : ",vehicles_tau[1])
print("rightrear0's tau : ",vehicles_tau[2])
print("rightrear1's tau : ",vehicles_tau[3])
print("leftforward0's tau : ",vehicles_tau[4])
print("leftforward1's tau : ",vehicles_tau[5])
print("leftforward2's tau : ",vehicles_tau[6])
print("forward's tau : ",vehicles_tau[7])
print("rightforward0's tau : ",vehicles_tau[8])
print("rightforward1's tau : ",vehicles_tau[9])
print("rightforward2's tau : ",vehicles_tau[10])

global lane_buffer_ego
global ego_LC_success

car0_mode=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0] # cruise =0, Anti-RVC =1
car1_mode=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
car2_mode=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
car3_mode=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
car0_anti_RVC_mode=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0] #  cruise =0, LC=1, LK=2,LK_r=3,LK_w=4
car1_anti_RVC_mode=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
car2_anti_RVC_mode=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
car3_anti_RVC_mode=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
lane_buffer_car0 = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
lane_buffer_car1 = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
lane_buffer_car2 = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
lane_buffer_car3 = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
back_buffer_car0 = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
back_buffer_car1 = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
back_buffer_car2 = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
back_buffer_car3 = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

crossroads_end = 1100
step_length = 0.01
truck_length = 8
highway_speed_limit=22.22  #m/s




random_speed=[]
random_speed2=[]
random_clearance=[]
random_jerk=[]
random_max_a_x=[]
random_min_a_x=[]
random_min_a=[]
speed_gap_between_leader=[]
params = '/home/mds/Desktop/highway_episodic/log_data/parameters.csv'
parameters =pd.read_csv(params,index_col=0)
random_speed = parameters['random_speed'].values
random_speed2 = parameters['random_speed2'].values
random_clearance = parameters['random_clearance'].values
random_jerk = parameters['random_jerk'].values
random_max_a_x = parameters['random_max_a_x'].values
random_min_a_x = parameters['random_min_a_x'].values
random_min_a = parameters['random_min_a'].values
speed_gap_between_leader = parameters['speed_gap_between_leader'].values

global avoiding_LC
avoiding_LC =0
global finish_truck
finish_truck =0
def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("-N","--episodenum", 
                        default=20, help="numer of episode to run qlenv")
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options

def generate_routefile():
    #collisionAvoidanceOverride="50"
    ## """+"\""+str(num)+"\""+"""

    num = 28
    rand=[0,0,0,0,0,0,0,0,0,0]
    rand_shift_left =random.uniform(-15,11.5)
    rand_shift_right =random.uniform(-15,11.5)
    for i in range(10):
        # rand[i] = random.uniform(-3.5,3.5)
        rand[i] = random.uniform(0,3.5)
        print(rand[i])
    with open("highway_episodic.rou.xml", "w") as routes:
        print("""<routes>
        <vType accel="3" decel="4.5" departSpeed = "22" id="car" length="5" maxSpeed="22.22" minGap="0.0" sigma="1" lcSpeedGain ="0"/>
        <vType accel="2.5" decel="4.5" departSpeed = "33" emergencyDecel="9.0" id="truck" length="12" maxSpeed="33.3333" minGap="0.0" sigma="1" collisionMinGapFactor="0" tau="0.0001" lcSpeedGain ="0" vClass="truck"/>
        <vType accel="2.5" decel="4.5" departSpeed = "33" id="accel_truck" length="12" maxSpeed="33.3333" minGap="0.0" sigma="1" collisionMinGapFactor="0" tau="0.0001" lcSpeedGain ="0" vClass="truck"/>
        <route id="route0" color="1,1,0" edges="E5"/>
        
        <vehicle id="ego" type="car" route="route0" depart="0.00" departSpeed="avg" departPos="95" departLane="1" color="0,1,0"/>   
                
        <vehicle id="car.leftrear0" type="car" route="route0" depart="0.00" departSpeed="avg" departPos="""+"\""+str(40+rand[0]+rand_shift_left)+"\""+""" departLane="2" color="1,1,0"/>
        <vehicle id="car.leftrear1" type="car" route="route0" depart="0.00" departSpeed="avg" departPos="""+"\""+str(70+rand[1]+rand_shift_left)+"\""+""" departLane="2" color="1,1,0"/>
        <vehicle id="accel.rear" type="accel_truck" route="route0" depart="0.00" departSpeed="max" departPos="0" departLane="1" color="1,0,0"/>
        <vehicle id="car.rightrear0" type="car" route="route0" depart="0.00" departSpeed="avg" departPos="""+"\""+str(40+rand[2]+rand_shift_right)+"\""+""" departLane="0" color="1,1,0"/> 
        <vehicle id="car.rightrear1" type="car" route="route0" depart="0.00" departSpeed="avg" departPos="""+"\""+str(70+rand[3]+rand_shift_right)+"\""+""" departLane="0" color="1,1,0"/> 
        <vehicle id="car.leftforward0" type="car" route="route0" depart="0.00" departSpeed="avg" departPos="""+"\""+str(110+rand[4]+rand_shift_left)+"\""+""" departLane="2" color="1,1,0"/>
        <vehicle id="car.leftforward1" type="car" route="route0" depart="0.00" departSpeed="avg" departPos="""+"\""+str(145+rand[5]+rand_shift_left)+"\""+""" departLane="2" color="1,1,0"/>
        <vehicle id="car.leftforward2" type="car" route="route0" depart="0.00" departSpeed="avg" departPos="""+"\""+str(175+rand[6]+rand_shift_left)+"\""+""" departLane="2" color="1,1,0"/>
        <vehicle id="car.forward" type="car" route="route0" depart="0.00" departSpeed="avg" departPos="195" departLane="1" color="1,1,0"/>
        <vehicle id="car.rightforward0" type="car" route="route0" depart="0.00" departSpeed="avg" departPos="""+"\""+str(110+rand[7]+rand_shift_right)+"\""+""" departLane="0" color="1,1,0"/> 
        <vehicle id="car.rightforward1" type="car" route="route0" depart="0.00" departSpeed="avg" departPos="""+"\""+str(145+rand[8]+rand_shift_right)+"\""+""" departLane="0" color="1,1,0"/>
        <vehicle id="car.rightforward2" type="car" route="route0" depart="0.00" departSpeed="avg" departPos="""+"\""+str(175+rand[9]+rand_shift_right)+"\""+""" departLane="0" color="1,1,0"/> 
        
                    
        """, file=routes)
        print("</routes>", file=routes)

def save_csv():
    col = ['time','vehicle_id','x','y','v_x','v_y','a_x','a_y','heading_angle','x_for_id','x_for','rel_v_for','rel_a_for','front_SD_for_LKtoPreLC','front_SD_for_PreLCtoLC','x_bac_id','x_bac','rel_v_bac','rel_a_bac','ETTC_bac','left_leader','left_leader_rel_x','left_leader_rel_v','left_leader_SD','right_leader','right_leader_rel_x','right_leader_rel_v','right_leader_SD','left_follower','left_follower_rel_x','left_follower_rel_v','left_follower_SD','right_follower','right_follower_rel_x','right_follower_rel_v','right_follower_SD']
    col2 = ['time','vehicle_id','x','y','v_x','v_y','a_x','a_y','heading_angle','x_for_id','x_for','rel_v_for','rel_a_for','front_SD_for_LKtoPreLC','front_SD_for_PreLCtoLC','x_bac_id','x_bac','rel_v_bac','rel_a_bac','ETTC_bac','left_leader','left_leader_rel_x','left_leader_rel_v','left_leader_SD','right_leader','right_leader_rel_x','right_leader_rel_v','right_leader_SD','left_follower','left_follower_rel_x','left_follower_rel_v','left_follower_SD','right_follower','right_follower_rel_x','right_follower_rel_v','right_follower_SD','mode']
    col3 = ['time','vehicle_id','x','y','v_x','v_y','a_x','a_y','heading_angle']
    df_ego = pd.DataFrame(ego,columns=col3)
    df_leftrear0 = pd.DataFrame(leftrear0,columns=col3)
    df_leftrear1 = pd.DataFrame(leftrear1,columns=col3)
    df_rear = pd.DataFrame(rear,columns=col3)
    df_rightrear0 = pd.DataFrame(rightrear0,columns=col3)
    df_rightrear1 = pd.DataFrame(rightrear1,columns=col3)
    df_leftforward0 = pd.DataFrame(leftforward0,columns=col3)
    df_leftforward1 = pd.DataFrame(leftforward1,columns=col3)
    df_leftforward2 = pd.DataFrame(leftforward2,columns=col3)
    df_forward = pd.DataFrame(forward,columns=col3)
    df_rightforward0 = pd.DataFrame(rightforward0,columns=col3)
    df_rightforward1 = pd.DataFrame(rightforward1,columns=col3)
    df_rightforward2 = pd.DataFrame(rightforward2,columns=col3)

    # df_accel= pd.DataFrame(accel0[0],columns=col)
    # for i in range(1):
    #     df = pd.DataFrame(accel0[i+1],columns=col)
    #     df_accel=pd.concat([df_accel,df],ignore_index = True)
    # for i in range(2):
    #     df2 = pd.DataFrame(accel1[i],columns=col)
    #     df_accel=pd.concat([df_accel,df2],ignore_index = True)
    # df_truck= pd.DataFrame(truck0[0],columns=col)
    # for i in range(1):
    #     df3 = pd.DataFrame(truck0[i+1],columns=col)
    #     df_truck=pd.concat([df_truck,df3],ignore_index = True)
    # for i in range(2):
    #     df4 = pd.DataFrame(truck1[i],columns=col)
    #     df_truck=pd.concat([df_truck,df4],ignore_index = True)
    # df_car= pd.DataFrame(car0[0],columns=col2)  
    # for i in range(4):
    #     df5 = pd.DataFrame(car0[i+1],columns=col2)
    #     df_car=pd.concat([df_car,df5],ignore_index = True)
    # for i in range(5):
    #     df6 =pd.DataFrame(car1[i],columns=col2)
    #     df_car=pd.concat([df_car,df6],ignore_index = True)
    # for i in range(5):
    #     df7 = pd.DataFrame(car2[i],columns=col2)
    #     df_car=pd.concat([df_car,df7],ignore_index = True)
    # for i in range(5):
    #     df8 = pd.DataFrame(car3[i],columns=col2)
    #     df_car=pd.concat([df_car,df8],ignore_index = True)
    col_c = ['victim','collider']
    df_collision= pd.DataFrame(collision,columns=col_c)
    col_u = ['vehicle_id','time','input acceleration','desire acceleration']
    df_input = pd.DataFrame(input,columns=col_u)
    
    # df1=pd.DataFrame({'time':accel0[0][0][0],'vehicle_id':accel0[0][0][1],'x':accel0[0][0][2],'y':accel0[0][0][3],'v_x':accel0[0][0][4],'v_y':accel0[0][0][5],'a_x':accel0[0][0][6],'a_y':accel0[0][0][7],'theta_h':accel0[0][0][8], 'x_for_id':accel0[0][0][9],'x_for':accel0[0][0][10],'v_for':accel0[0][0][11],'x_bac_id':accel0[0][0][12],'x_bac':accel0[0][0][13],'v_bac':accel0[0][0][14]})   
    # for i in range(len(accel0[0])-1):
    #     pd.concat([df1,accel0[0][i]],ignore_index = True)      
    # 
    # 
    # params = pd.DataFrame()
    # params['random_speed']=random_speed
    # params['random_speed2']=random_speed2
    # params['random_clearance']=random_clearance
    # params['random_jerk']=random_jerk
    # params['random_max_a_x']=random_max_a_x
    # params['random_min_a_x']=random_min_a_x
    # params['random_min_a']=random_min_a
    # params['speed_gap_between_leader']=speed_gap_between_leader
    
    os.chdir('/home/mds/Desktop/highway_episodic/log_data')
    df_ego.to_csv('log_data_ego.csv')
    df_leftrear0.to_csv('log_data_leftrear0.csv')
    df_leftrear1.to_csv('log_data_leftrear1.csv')
    df_rear.to_csv('log_data_rear.csv')
    df_rightrear0.to_csv('log_data_rightrear0.csv')
    df_rightrear1.to_csv('log_data_rightrear1.csv')
    df_leftforward0.to_csv('log_data_leftforward0.csv')
    df_leftforward1.to_csv('log_data_leftforward1.csv')
    df_leftforward2.to_csv('log_data_leftforward2.csv')
    df_forward.to_csv('log_data_forward.csv')
    df_rightforward0.to_csv('log_data_rightforward0.csv')
    df_rightforward1.to_csv('log_data_rightforward1.csv')
    df_rightforward2.to_csv('log_data_rightforward2.csv')

    df_collision.to_csv('log_data_collision.csv')
    df_input.to_csv('log_data_input.csv')
    # params.to_csv('parameters.csv')
#40km/h -> 11.1111m/s, 70km/h -> 19.4444m/s, 
def K1(v): #optimal gain (input v : velocity output K1 :optimal gain upper bound)
    if(v>=0 and v<=11.1111):
        return 0.35
    elif(v>11.1111 and v<=19.4444):
        return -0.018*v+0.55
    else:
        return 0.2

def K2(v):#optimal gain (input v : velocity output K2 :optimal gain lower bound)
    if(v>=0 and v<=11.1111):
        return -1.22
    elif(v>11.1111 and v<=19.4444):
        return 0.0444*v-1.7133
    else:
        return -0.85
def a_max(v):
    if(v>=0 and v<=11.1111):
        return 1.5
    elif(v>11.1111 and v<=19.4444):
        return -0.06*v+2.1667
    else:
        return 1
def a_min(v):
    if(v>=0 and v<=11.1111):
        return -2
    elif(v>11.1111 and v<=19.4444):
        return 0.12*v-3.3333
    else:
        return -1
    
def a_desire(v_c,v_p,c_d,c): # desire acceleration by ACC, v_c : ego_vehicle's velocity, v_p : preceding_vehicle's velocity, c_d : desire clearance, c = actual clearance
    a_d = -K1(v_c)*(c_d-c) - K2(v_c)*(v_p-v_c)
    if(a_d > a_max(v_c)):
        return a_max(v_c)
    elif(a_d <= a_max(v_c) and a_d >=a_min(v_c)):
        return a_d
    elif(a_d<a_min(v_c)):
        return a_min(v_c)

def a_desire_with_speed_limit(v_c,v_p,c_d,c,speed_limit):
    v_define = min(speed_limit,v_p) # 앞차 속도와 규정 속도 중 작은 것을 따름.
    
    if(v_c > speed_limit):
        a_d = - K2(v_c)*(speed_limit-v_c)
    else:
        a_d = -K1(v_c)*(c_d-c) - K2(v_c)*(v_define-v_c)
    if(a_d > a_max(v_c)):
        return a_max(v_c)
    elif(a_d <= a_max(v_c) and a_d >=a_min(v_c)):
        return a_d
    elif(a_d<a_min(v_c)):
        return a_min(v_c)


# def set_a(id,a_des): #return 값은 jerk 값을 고려한 현실적인 가속도 값을 반환한다.
#     jerk = 1
#     dt = step_length
#     a_cur = traci.vehicle.getAcceleration(id)
#     if(a_cur+jerk*dt<a_des):
#         traci.vehicle.setAcceleration(id,a_cur+jerk*dt,1)
#         return a_cur+jerk*dt
#     elif(a_cur-jerk*dt>a_des):
#         traci.vehicle.setAcceleration(id,a_cur-jerk*dt,1)
#         return a_cur-jerk*dt
#     else:
#         traci.vehicle.setAcceleration(id,a_des,1)
#         return a_des

# def set_a(id,a_des): #return 값은 jerk 값을 고려한 현실적인 가속도 값을 반환한다.
#     jerk = 3
#     dt = step_length
#     k=jerk*dt+0.01 #jerk*dt 보다 커야함.
#     a_cur = traci.vehicle.getAcceleration(id)
#     # if(a_cur<a_des-k):
#     if(a_des-a_cur > k):
#         traci.vehicle.setAcceleration(id,a_cur+jerk*dt,1)
#         return a_cur+jerk*dt
#     # elif(a_cur>a_des+k):
#     elif(a_des-a_cur < -k):
#         traci.vehicle.setAcceleration(id,a_cur-jerk*dt,1)
#         return a_cur-jerk*dt
#     else:
#         traci.vehicle.setAcceleration(id,a_des,1)
#         return a_des



# def set_a(id,a_des):
#     jerk = 3
#     dt = step_length
#     a_cur = traci.vehicle.getAcceleration(id)
#     if(a_des -a_cur<= jerk*dt):
#         a= a_des
#         traci.vehicle.setAcceleration(id,a,1)
#     else:
#         a = a_cur +jerk*dt
#         traci.vehicle.setAcceleration(id,a,1)
    

# def set_v(id,v_des):
#     dt = step_length
#     v_cur = traci.vehicle.getSpeed(id)
#     if v_des >= v_cur:
#         a_des = a_max(v_cur)
#     else:
#         a_des = a_min(v_cur)
#     a = set_a(id,a_des)
    
#     k=abs(a*dt)+0.01 #a*dt 보다 커야함.
    
#     if(v_cur<v_des-k):
#         traci.vehicle.setSpeed(id,v_cur+a*dt)
#     elif(v_cur>v_des+k):
#         traci.vehicle.setSpeed(id,v_cur-a*dt)
#     else:
#         traci.vehicle.setSpeed(id,v_des)
# def set_v(id,v_des):
#     dt = step_length
#     v_cur = traci.vehicle.getSpeed(id)
#     if v_des > v_cur:
#         a_des = a_max(v_cur)
#     else:
#         a_des = a_min(v_cur)
#     a = set_a(id,a_des)
#     traci.vehicle.setSpeed(id,v_cur+a*dt)   

def set_v(id,v_des): # v_des에 따라 a_des을 결정하고 jerk 값이 고려된  속도를 시스템에 넣음.
    dt = step_length
    Kp =0.9 # p gain
    v_cur = traci.vehicle.getSpeed(id)
    if (v_des - v_cur)> a_max(v_cur):
        a_des = a_max(v_cur)
    elif (v_des - v_cur) <= a_max(v_cur) and (v_des - v_cur)>a_min(v_cur): #a_max(20) = 1,a_min(20) = -1
        a_des = (v_des - v_cur)*Kp
    else:
        a_des = a_min(v_cur)
    # a = set_a(id,a_des)
    traci.vehicle.setAcceleration(id,a_des,20) #입력 가속도
    a = a_des # 입력가속도.

    # traci.vehicle.setSpeed(id,v_cur+a*dt) 
    # v = v_cur+a*dt
    control_input = [a,a_des]
    return control_input


    

# def set_v_max(id,a_des,v_max):
#     dt = step_length
#     v_cur = traci.vehicle.getSpeed(id)
#     v_next = v_cur+a_des*dt
#     if(v_next<v_max):
#         set_v(id,v_next)
#     else:
#         set_v(id,v_max)    
        
def setvehiclestate(id, goal_v_x,min_a_x, max_a_x, goal_j_x):
    dt = step_length
    last_v_x = rear[-1][4]
    last_a_x = rear[-1][6]
        
    # elif id[0] == 't':
    #     if id[5] == '0':
    #         last_v_x = truck0_buffer[int(id[-1])][0][4]
    #         last_a_x = truck0_buffer[int(id[-1])][0][6]
    #     elif id[5] == '1':
    #         last_v_x = truck1_buffer[int(id[-1])][0][4]
    #         last_a_x = truck1_buffer[int(id[-1])][0][6]
    # elif id[0] == 'c':
    #     if id[3] == '0':
    #         last_v_x = car0_buffer[int(id[-1])][0][4]
    #         last_a_x = car0_buffer[int(id[-1])][0][6]
    #     elif id[3] == '1':
    #         last_v_x = car1_buffer[int(id[-1])][0][4]
    #         last_a_x = car1_buffer[int(id[-1])][0][6]
    #     elif id[3] == '2':
    #         last_v_x = car2_buffer[int(id[-1])][0][4]
    #         last_a_x = car2_buffer[int(id[-1])][0][6]
    #     elif id[3] == '3':
    #         last_v_x = car3_buffer[int(id[-1])][0][4]
    #         last_a_x = car3_buffer[int(id[-1])][0][6]
    
    if(last_v_x < goal_v_x-1):
        next_accel = goal_j_x*dt + last_a_x
        if next_accel <=max_a_x:
            next_velocity = next_accel*dt + last_v_x
        else:
            next_velocity = max_a_x*dt + last_v_x
    elif(last_v_x >= goal_v_x-1 and last_v_x <= goal_v_x+1):
        next_accel = last_a_x
        next_velocity = last_v_x
    elif(last_v_x > goal_v_x+1):
        next_accel = -goal_j_x*dt + last_a_x
        if next_accel >= min_a_x:
            next_velocity = next_accel*dt + last_v_x
        else:
            next_velocity = min_a_x*dt+last_v_x
    # set_v(id,next_velocity)
    traci.vehicle.setSpeed(id,next_velocity)
    # if(next_accel>=0 and next_accel<=2):
    if(next_accel>=0):
        # set_a(id,next_accel)
        traci.vehicle.setAccel(id,next_accel)
    elif(next_accel<0 and next_accel>= -5):
        # set_a(id,-next_accel)
        traci.vehicle.setDecel(id,-next_accel)


def find_followers(ego_id, vehicle_id,LEFT_FOLLOWER,RIGHT_FOLLOWER):
    if(traci.vehicle.getPosition(vehicle_id)[0] <traci.vehicle.getPosition(ego_id)[0]):
        if(int(traci.vehicle.getLaneIndex(vehicle_id)) == int(traci.vehicle.getLaneIndex(ego_id))+1):
            LEFT_FOLLOWER.append((traci.vehicle.getPosition(ego_id)[0]-traci.vehicle.getPosition(vehicle_id)[0],vehicle_id))
        elif(int(traci.vehicle.getLaneIndex(vehicle_id)) == int(traci.vehicle.getLaneIndex(ego_id))-1):
            RIGHT_FOLLOWER.append((traci.vehicle.getPosition(ego_id)[0]-traci.vehicle.getPosition(vehicle_id)[0],vehicle_id))
    if LEFT_FOLLOWER:
        left_follower = sorted(LEFT_FOLLOWER)[0][1]
    else:
        left_follower = None
    if RIGHT_FOLLOWER:
        right_follower = sorted(RIGHT_FOLLOWER)[0][1]
    else:
        right_follower = None
    return left_follower,right_follower

def avoiding_motion_plan(id,no_road_dir):
    #no_road_dir -> left or right or none
    rel_v_with_front =0 
    c_LK = 2 #min clearance for lane keeping
    time_gap_LK = 1.36  #95% percentile time_gap (most safe) // 1.36s -> 중간값   0.74s -> 5% persentile 가장 위험.
    time_gap_LC_prepare = 0.74 
    time_gap_LC_1 =1
    time_gap_LC_2 =0.5 
    c_LC =12 # min clearance for lane change
    ego_v_x = traci.vehicle.getSpeed(id)
    ego_x_pos = traci.vehicle.getPosition(id)[0]
    if traci.vehicle.getLeader(id) is not None:
        Leader_id,x_forward = traci.vehicle.getLeader(id)
        rel_v_with_front = traci.vehicle.getSpeed(Leader_id) - ego_v_x

    if type(traci.vehicle.getFollower(id)) is not None and traci.vehicle.getFollower(id)[0] != '':
        follower_id, x_back = traci.vehicle.getFollower(id)      
    if(no_road_dir =='none'):  # 왼쪽 오른쪽에 차로가 있는 경우 
        if traci.vehicle.getLeftLeaders(id) is not None and len(traci.vehicle.getLeftLeaders(id)) !=0:
            Left_leader_id= traci.vehicle.getLeftLeaders(id)
        if traci.vehicle.getLeftFollowers(id) is not None and len(traci.vehicle.getLeftFollowers(id)) !=0:
            Left_follower_id= traci.vehicle.getLeftFollowers(id)
        if traci.vehicle.getRightLeaders(id) is not None and len(traci.vehicle.getRightLeaders(id)) !=0:
            Right_leader_id= traci.vehicle.getRightLeaders(id)
        if traci.vehicle.getRightFollowers(id) is not None and len(traci.vehicle.getRightFollowers(id)) !=0:
            Right_follower_id= traci.vehicle.getRightFollowers(id)
        if traci.vehicle.getLeader(id) is not None and type(traci.vehicle.getFollower(id)) is not None and traci.vehicle.getFollower(id)[0] != '': #LK mode
            v_p = traci.vehicle.getSpeed(Leader_id)
            des_clearance = c_LK + time_gap_LK*v_p #앞차 속도에 따른 앞차와의 ACC 안전거리 (safe)
            des_clearance2 = c_LK + time_gap_LC_prepare*v_p #앞차 속도에 따른 앞차와의 ACC 안전거리 (aggressive)
            if(des_clearance<= x_forward):    #Lk 모드
                if(id[3] == '0'):
                    car0_anti_RVC_mode[int(id[5:])] = 2
                elif(id[3] == '1'):
                    car1_anti_RVC_mode[int(id[5:])] = 2
                elif(id[3] == '2'):
                    car2_anti_RVC_mode[int(id[5:])] = 2
                elif(id[3] == '3'):
                    car3_anti_RVC_mode[int(id[5:])] = 2  

                # traci.vehicle.setSpeedMode(id,0b011110)
                traci.vehicle.setMaxSpeed(id,35)
                traci.vehicle.setLaneChangeMode(id,0b000000000000) # 마음대로 차선 변경 x 
                if type(traci.vehicle.getFollower(id)) is not None and traci.vehicle.getFollower(id)[0] != '':
                    # traci.vehicle.setAccel(id,3) ## 가속도 높임
                    # if traci.vehicle.getSpeed(follower_id)>=traci.vehicle.getSpeed(id):
                    #     # traci.vehicle.setSpeed(id,traci.vehicle.getSpeed(follower_id))
                    #     set_v(id,traci.vehicle.getSpeed(follower_id),3)
                    # else:## 충돌 위험이 없는 경우 일반 주행 속도로 설정
                    #     # print(id,' LK free speed') 
                    #     traci.vehicle.setSpeedMode(id,0b011111)

                    # set_v(id,traci.vehicle.getSpeed(follower_id),3)
                    set_a(id,3)
                    set_v(id,traci.vehicle.getSpeed(follower_id))

            else:
                if id:                    
                    LEFT_FOLLOWER =[]
                    RIGHT_FOLLOWER=[]
                    traci.vehicle.subscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 30.0, [tc.VAR_POSITION])
                    # print(id,' subscribeContext')
                    for v in traci.vehicle.getContextSubscriptionResults(str(id)):
                        left_follower_id = find_followers(id, v,LEFT_FOLLOWER,RIGHT_FOLLOWER)[0]
                        right_follower_id = find_followers(id, v,LEFT_FOLLOWER,RIGHT_FOLLOWER)[1]
                    traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 30.0)
                if(des_clearance2<=x_forward): #LK_r 모드
                    if(id[3] == '0'):
                        car0_anti_RVC_mode[int(id[5:])] = 3
                    elif(id[3] == '1'):
                        car1_anti_RVC_mode[int(id[5:])] = 3
                    elif(id[3] == '2'):
                        car2_anti_RVC_mode[int(id[5:])] = 3
                    elif(id[3] == '3'):
                        car3_anti_RVC_mode[int(id[5:])] = 3 
                    # print(id,' back speed LK')
                    # traci.vehicle.setSpeedMode(id,0b011110)
                    traci.vehicle.setMaxSpeed(id,35)
                    traci.vehicle.setLaneChangeMode(id,0b000000000000) # 마음대로 차선 변경 x 
                    if type(traci.vehicle.getFollower(id)) is not None and traci.vehicle.getFollower(id)[0] != '':
                        # traci.vehicle.setAccel(id,3) ## 가속도 높임
                        # traci.vehicle.setSpeed(id,traci.vehicle.getSpeed(follower_id))
                        
                        
                        
                        # set_v(id,traci.vehicle.getSpeed(follower_id),3)
                        set_a(id,3)
                        set_v(id,traci.vehicle.getSpeed(follower_id))


                    # if type(traci.vehicle.getFollower(id)) is not None and traci.vehicle.getFollower(id)[0] != '':
                        
                    #     if(traci.vehicle.getDecel(follower_id) >= ): #뒤차가 앞차보다 속도가 낮다면 cruise mode로 변경
                    #         print(follower_id," decel : ",traci.vehicle.getDecel(follower_id))
                    #         if(id[3]==0):
                    #             car0_mode[int(id[5:])] = 0
                    #         elif(id[3]==1):
                    #             car1_mode[int(id[5:])] = 0
                    #         elif(id[3]==2):
                    #             car2_mode[int(id[5:])] = 0
                    #         elif(id[3]==3):
                    #             car3_mode[int(id[5:])] = 0
                    #     else:
                    #         traci.vehicle.setAccel(id,3) ## 가속도 높임
                    #         traci.vehicle.setSpeed(id,traci.vehicle.getSpeed(follower_id))
                else: #LK_w 모드
                    if(id[3] == '0'):
                        car0_anti_RVC_mode[int(id[5:])] = 4
                    elif(id[3] == '1'):
                        car1_anti_RVC_mode[int(id[5:])] = 4
                    elif(id[3] == '2'):
                        car2_anti_RVC_mode[int(id[5:])] = 4
                    elif(id[3] == '3'):
                        car3_anti_RVC_mode[int(id[5:])] = 4 
                    # print(id,' weighted speed LK')
                    # traci.vehicle.setSpeedMode(id,0b011110)
                    # traci.vehicle.setMaxSpeed(id,35)
                    traci.vehicle.setLaneChangeMode(id,0b000000000000) # 마음대로 차선 변경 x 
                    # traci.vehicle.setDecel(id,5) ## 감속도
                    # front, back clearance가 작아서 (x_back-c_Lk)항 과, x_forward -c_LK*2)항이 0이 0인경우는 단순히 x_forward : 2*x_back으로 가중치 부여한 속도로 결정.(if else 없애고 else 아래 수식으로만 사용해도 될것 같음.)
                    speed = (traci.vehicle.getSpeed(follower_id)*(x_forward) + traci.vehicle.getSpeed(Leader_id)*(2*x_back))/(x_forward+2*x_back)
                    # traci.vehicle.setSpeed(id,speed)
                    if ((speed-set_a(id,3))> traci.vehicle.getSpeed(id)):
                        # set_v(id,speed,3)
                        set_a(id,3)
                        set_v(id,speed)
                        
                    elif((speed+abs(set_a(id,-4.5))) < traci.vehicle.getSpeed(id)):
                        # set_v(id,speed,-4.5)
                        set_a(id,-4.5)
                        set_v(id,speed)
                    else:
                        # traci.vehicle.setSpeed(id,speed)
                        set_v(id,speed)
                    print(id,"'s v_w_desire : ",speed)
                # traci.vehicle.setLaneChangeMode(id,0b000000001001)

                # LC mode 체크
                if len(traci.vehicle.getLeftLeaders(id)) !=0 and left_follower_id is not None:
                    front_relative_velocity_term = max( [traci.vehicle.getSpeed(id) - traci.vehicle.getSpeed(Left_leader_id[0][0]) ,0] )*time_gap_LC_1
                    front_minimum_clearance_trem = max([traci.vehicle.getSpeed(id)*time_gap_LC_2,c_LC])
                    front_SD_LC = front_relative_velocity_term +front_minimum_clearance_trem
                    back_relative_velocity_term = max( [traci.vehicle.getSpeed(left_follower_id)-traci.vehicle.getSpeed(id),0] )*time_gap_LC_1
                    back__minimum_clearance_trem = max([traci.vehicle.getSpeed(left_follower_id)*time_gap_LC_2,c_LC])
                    back_SD_LC = back_relative_velocity_term + back__minimum_clearance_trem

                    if((traci.vehicle.getPosition(Left_leader_id[0][0])[0]-ego_x_pos) >= front_SD_LC  and (ego_x_pos -traci.vehicle.getPosition(left_follower_id)[0])>= back_SD_LC):
                        # print(id,' LC left')
                        traci.vehicle.setLaneChangeMode(id,0b000000001001)
                        traci.vehicle.changeLaneRelative(id,1,5)
                        
                    else:
                        if len(traci.vehicle.getRightLeaders(id))!=0 and right_follower_id is not None:
                            front_relative_velocity_term = max( [traci.vehicle.getSpeed(id) - traci.vehicle.getSpeed(Right_leader_id[0][0]) ,0] )*time_gap_LC_1
                            front_minimum_clearance_trem = max([traci.vehicle.getSpeed(id)*time_gap_LC_2,c_LC])
                            front_SD_LC = front_relative_velocity_term +front_minimum_clearance_trem
                            back_relative_velocity_term = max( [traci.vehicle.getSpeed(right_follower_id)-traci.vehicle.getSpeed(id),0] )*time_gap_LC_1
                            back__minimum_clearance_trem = max([traci.vehicle.getSpeed(right_follower_id)*time_gap_LC_2,c_LC])
                            back_SD_LC = back_relative_velocity_term + back__minimum_clearance_trem
                            if((traci.vehicle.getPosition(Right_leader_id[0][0])[0]-ego_x_pos) >=front_SD_LC  and (ego_x_pos -traci.vehicle.getPosition(right_follower_id)[0])>=back_SD_LC):
                                # print(id,' LC right')
                                traci.vehicle.setLaneChangeMode(id,0b000000001001)
                                traci.vehicle.changeLaneRelative(id,-1,5)
                        elif len(traci.vehicle.getRightLeaders(id)) !=0 and right_follower_id is None:
                            front_relative_velocity_term = max( [traci.vehicle.getSpeed(id) - traci.vehicle.getSpeed(Right_leader_id[0][0]) ,0] )*time_gap_LC_1
                            front_minimum_clearance_trem = max([traci.vehicle.getSpeed(id)*time_gap_LC_2,c_LC])
                            front_SD_LC = front_relative_velocity_term +front_minimum_clearance_trem
                            if(traci.vehicle.getPosition(Right_leader_id[0][0])[0]-ego_x_pos) >=front_SD_LC:
                                # print(id,' LC right')
                                traci.vehicle.setLaneChangeMode(id,0b000000001001)
                                traci.vehicle.changeLaneRelative(id,-1,5)
                        elif len(traci.vehicle.getRightLeaders(id))==0 and right_follower_id is not None:
                            back_relative_velocity_term = max( [traci.vehicle.getSpeed(right_follower_id)-traci.vehicle.getSpeed(id),0] )*time_gap_LC_1
                            back__minimum_clearance_trem = max([traci.vehicle.getSpeed(right_follower_id)*time_gap_LC_2,c_LC])
                            back_SD_LC = back_relative_velocity_term + back__minimum_clearance_trem
                            if (ego_x_pos -traci.vehicle.getPosition(right_follower_id)[0])>=back_SD_LC:
                                # print(id,' LC right')
                                traci.vehicle.setLaneChangeMode(id,0b000000001001)
                                traci.vehicle.changeLaneRelative(id,-1,5)
                        elif len(traci.vehicle.getRightLeaders(id))==0 and right_follower_id is None:
                            # print(id,' LC right')
                            traci.vehicle.setLaneChangeMode(id,0b000000001001)
                            traci.vehicle.changeLaneRelative(id,-1,5)
                elif len(traci.vehicle.getLeftLeaders(id)) !=0 and left_follower_id is None:
                    front_relative_velocity_term = max( [traci.vehicle.getSpeed(id) - traci.vehicle.getSpeed(Left_leader_id[0][0]) ,0] )*time_gap_LC_1
                    front_minimum_clearance_trem = max([traci.vehicle.getSpeed(id)*time_gap_LC_2,c_LC])
                    front_SD_LC = front_relative_velocity_term +front_minimum_clearance_trem
                    if(traci.vehicle.getPosition(Left_leader_id[0][0])[0]-ego_x_pos) >= front_SD_LC:
                        # print(id,' LC left')
                        traci.vehicle.setLaneChangeMode(id,0b000000001001)
                        traci.vehicle.changeLaneRelative(id,1,5)
                    else:
                        if len(traci.vehicle.getRightLeaders(id))!=0 and right_follower_id is not None:
                            front_relative_velocity_term = max( [traci.vehicle.getSpeed(id) - traci.vehicle.getSpeed(Right_leader_id[0][0]) ,0] )*time_gap_LC_1
                            front_minimum_clearance_trem = max([traci.vehicle.getSpeed(id)*time_gap_LC_2,c_LC])
                            front_SD_LC = front_relative_velocity_term +front_minimum_clearance_trem
                            back_relative_velocity_term = max( [traci.vehicle.getSpeed(right_follower_id)-traci.vehicle.getSpeed(id),0] )*time_gap_LC_1
                            back__minimum_clearance_trem = max([traci.vehicle.getSpeed(right_follower_id)*time_gap_LC_2,c_LC])
                            back_SD_LC = back_relative_velocity_term + back__minimum_clearance_trem
                            if((traci.vehicle.getPosition(Right_leader_id[0][0])[0]-ego_x_pos) >=front_SD_LC  and (ego_x_pos -traci.vehicle.getPosition(right_follower_id)[0])>=back_SD_LC):
                                # print(id,' LC right')
                                traci.vehicle.setLaneChangeMode(id,0b000000001001)
                                traci.vehicle.changeLaneRelative(id,-1,5)
                        elif len(traci.vehicle.getRightLeaders(id)) !=0 and right_follower_id is None:
                            front_relative_velocity_term = max( [traci.vehicle.getSpeed(id) - traci.vehicle.getSpeed(Right_leader_id[0][0]) ,0] )*time_gap_LC_1
                            front_minimum_clearance_trem = max([traci.vehicle.getSpeed(id)*time_gap_LC_2,c_LC])
                            front_SD_LC = front_relative_velocity_term +front_minimum_clearance_trem
                            if(traci.vehicle.getPosition(Right_leader_id[0][0])[0]-ego_x_pos) >=front_SD_LC:
                                # print(id,' LC right')
                                traci.vehicle.setLaneChangeMode(id,0b000000001001)
                                traci.vehicle.changeLaneRelative(id,-1,5)
                        elif len(traci.vehicle.getRightLeaders(id))==0 and right_follower_id is not None:
                            back_relative_velocity_term = max( [traci.vehicle.getSpeed(right_follower_id)-traci.vehicle.getSpeed(id),0] )*time_gap_LC_1
                            back__minimum_clearance_trem = max([traci.vehicle.getSpeed(right_follower_id)*time_gap_LC_2,c_LC])
                            back_SD_LC = back_relative_velocity_term + back__minimum_clearance_trem
                            if (ego_x_pos -traci.vehicle.getPosition(right_follower_id)[0])>=back_SD_LC:
                                # print(id,' LC right')
                                traci.vehicle.setLaneChangeMode(id,0b000000001001)
                                traci.vehicle.changeLaneRelative(id,-1,5)
                        elif len(traci.vehicle.getRightLeaders(id))==0 and right_follower_id is None:
                            # print(id,' LC right')
                            traci.vehicle.setLaneChangeMode(id,0b000000001001)
                            traci.vehicle.changeLaneRelative(id,-1,5)
                elif len(traci.vehicle.getLeftLeaders(id)) ==0 and left_follower_id is not None:
                    back_relative_velocity_term = max( [traci.vehicle.getSpeed(left_follower_id)-traci.vehicle.getSpeed(id),0] )*time_gap_LC_1
                    back__minimum_clearance_trem = max([traci.vehicle.getSpeed(left_follower_id)*time_gap_LC_2,c_LC])
                    back_SD_LC = back_relative_velocity_term + back__minimum_clearance_trem
                    if (ego_x_pos -traci.vehicle.getPosition(left_follower_id)[0])>=back_SD_LC:
                        # print(id,' LC left')
                        traci.vehicle.setLaneChangeMode(id,0b000000001001)
                        traci.vehicle.changeLaneRelative(id,1,5)
                    else:
                        if len(traci.vehicle.getRightLeaders(id))!=0 and right_follower_id is not None:
                            front_relative_velocity_term = max( [traci.vehicle.getSpeed(id) - traci.vehicle.getSpeed(Right_leader_id[0][0]) ,0] )*time_gap_LC_1
                            front_minimum_clearance_trem = max([traci.vehicle.getSpeed(id)*time_gap_LC_2,c_LC])
                            front_SD_LC = front_relative_velocity_term +front_minimum_clearance_trem
                            back_relative_velocity_term = max( [traci.vehicle.getSpeed(right_follower_id)-traci.vehicle.getSpeed(id),0] )*time_gap_LC_1
                            back__minimum_clearance_trem = max([traci.vehicle.getSpeed(right_follower_id)*time_gap_LC_2,c_LC])
                            back_SD_LC = back_relative_velocity_term + back__minimum_clearance_trem
                            if((traci.vehicle.getPosition(Right_leader_id[0][0])[0]-ego_x_pos) >=front_SD_LC  and (ego_x_pos -traci.vehicle.getPosition(right_follower_id)[0])>=back_SD_LC):
                                # print(id,' LC right')
                                traci.vehicle.setLaneChangeMode(id,0b000000001001)
                                traci.vehicle.changeLaneRelative(id,-1,5)
                        elif len(traci.vehicle.getRightLeaders(id)) !=0 and right_follower_id is None:
                            front_relative_velocity_term = max( [traci.vehicle.getSpeed(id) - traci.vehicle.getSpeed(Right_leader_id[0][0]) ,0] )*time_gap_LC_1
                            front_minimum_clearance_trem = max([traci.vehicle.getSpeed(id)*time_gap_LC_2,c_LC])
                            front_SD_LC = front_relative_velocity_term +front_minimum_clearance_trem
                            if(traci.vehicle.getPosition(Right_leader_id[0][0])[0]-ego_x_pos) >=front_SD_LC:
                                # print(id,' LC right')
                                traci.vehicle.setLaneChangeMode(id,0b000000001001)
                                traci.vehicle.changeLaneRelative(id,-1,5)
                        elif len(traci.vehicle.getRightLeaders(id))==0 and right_follower_id is not None:
                            back_relative_velocity_term = max( [traci.vehicle.getSpeed(right_follower_id)-traci.vehicle.getSpeed(id),0] )*time_gap_LC_1
                            back__minimum_clearance_trem = max([traci.vehicle.getSpeed(right_follower_id)*time_gap_LC_2,c_LC])
                            back_SD_LC = back_relative_velocity_term + back__minimum_clearance_trem
                            if (ego_x_pos -traci.vehicle.getPosition(right_follower_id)[0])>=back_SD_LC:
                                # print(id,' LC right')
                                traci.vehicle.setLaneChangeMode(id,0b000000001001)
                                traci.vehicle.changeLaneRelative(id,-1,5)
                        elif len(traci.vehicle.getRightLeaders(id))==0 and right_follower_id is None:
                            # print(id,' LC right')
                            traci.vehicle.setLaneChangeMode(id,0b000000001001)
                            traci.vehicle.changeLaneRelative(id,-1,5)
                elif len(traci.vehicle.getLeftLeaders(id)) ==0 and left_follower_id is None:
                    # print(id,' LC left')
                    traci.vehicle.setLaneChangeMode(id,0b000000001001)
                    traci.vehicle.changeLaneRelative(id,1,5)           
        else: #LK mode 앞 차가 없을 때
            if(id[3] == '0'):
                car0_anti_RVC_mode[int(id[5:])] = 2
            elif(id[3] == '1'):
                car1_anti_RVC_mode[int(id[5:])] = 2
            elif(id[3] == '2'):
                car2_anti_RVC_mode[int(id[5:])] = 2
            elif(id[3] == '3'):
                car3_anti_RVC_mode[int(id[5:])] = 2 
            # traci.vehicle.setSpeedMode(id,0b011110)
            traci.vehicle.setMaxSpeed(id,35)
            traci.vehicle.setLaneChangeMode(id,0b000000000000) # 마음대로 차선 변경 x 
            if type(traci.vehicle.getFollower(id)) is not None and traci.vehicle.getFollower(id)[0] != '':
                # traci.vehicle.setAccel(id,3) ## 가속도 높임
                # traci.vehicle.setSpeed(id,traci.vehicle.getSpeed(follower_id))
                # set_v(id,traci.vehicle.getSpeed(follower_id),3)
                set_a(id,3)
                set_v(id,traci.vehicle.getSpeed(follower_id))

    elif(no_road_dir =='left'):  # 왼쪽 차로는 없고 오른쪽에 차로가 있는 경우
        if len(traci.vehicle.getRightLeaders(id)) !=0:
            Right_leader_id = traci.vehicle.getRightLeaders(id)
        if len(traci.vehicle.getRightFollowers(id)) !=0:
            Right_follower_id = traci.vehicle.getRightFollowers(id)
        if traci.vehicle.getLeader(id) is not None:
            Leader_id,x_forward = traci.vehicle.getLeader(id)
            rel_v_with_front = traci.vehicle.getSpeed(Leader_id) - ego_v_x
        if type(traci.vehicle.getFollower(id)) is not None and traci.vehicle.getFollower(id)[0] != '':
            follower_id, x_back = traci.vehicle.getFollower(id)      
        
        if traci.vehicle.getLeader(id) is not None and type(traci.vehicle.getFollower(id)) is not None and traci.vehicle.getFollower(id)[0] != '': #LK mode
            v_p = traci.vehicle.getSpeed(Leader_id)
            des_clearance = c_LK + time_gap_LK*v_p #앞차 속도에 따른 앞차와의 ACC 안전거리 (safe)
            des_clearance2 = c_LK + time_gap_LC_prepare*v_p #앞차 속도에 따른 앞차와의 ACC 안전거리 (aggressive)
            if(des_clearance<= x_forward):    #Lk 모드
                if(id[3] == '0'):
                    car0_anti_RVC_mode[int(id[5:])] = 2
                elif(id[3] == '1'):
                    car1_anti_RVC_mode[int(id[5:])] = 2
                elif(id[3] == '2'):
                    car2_anti_RVC_mode[int(id[5:])] = 2
                elif(id[3] == '3'):
                    car3_anti_RVC_mode[int(id[5:])] = 2 
                # traci.vehicle.setSpeedMode(id,0b011110)
                traci.vehicle.setMaxSpeed(id,35)
                traci.vehicle.setLaneChangeMode(id,0b000000000000) # 마음대로 차선 변경 x 
                if type(traci.vehicle.getFollower(id)) is not None and traci.vehicle.getFollower(id)[0] != '':
                    # traci.vehicle.setAccel(id,3) ## 가속도 높임
                    # if traci.vehicle.getSpeed(follower_id)>=traci.vehicle.getSpeed(id):
                        # traci.vehicle.setSpeed(id,traci.vehicle.getSpeed(follower_id))
                    # set_v(id,traci.vehicle.getSpeed(follower_id),3)

                    set_a(id,3)
                    set_v(id,traci.vehicle.getSpeed(follower_id))

                    # else:## 충돌 위험이 없는 경우 일반 주행 속도로 설정
                    #     # print(id,' LK free speed') 
                    #     traci.vehicle.setSpeedMode(id,0b011111)
            else: 
                if id:                    
                    LEFT_FOLLOWER =[]
                    RIGHT_FOLLOWER=[]
                    traci.vehicle.subscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 30.0, [tc.VAR_POSITION])
                    # print(id,' subscribeContext')
                    for v in traci.vehicle.getContextSubscriptionResults(str(id)):
                        # left_follower_id = find_followers(id, v,LEFT_FOLLOWER,RIGHT_FOLLOWER)[0]
                        right_follower_id = find_followers(id, v,LEFT_FOLLOWER,RIGHT_FOLLOWER)[1]
                    traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 30.0)
                if(des_clearance2<=x_forward): #LK_r 모드
                    if(id[3] == '0'):
                        car0_anti_RVC_mode[int(id[5:])] = 3
                    elif(id[3] == '1'):
                        car1_anti_RVC_mode[int(id[5:])] = 3
                    elif(id[3] == '2'):
                        car2_anti_RVC_mode[int(id[5:])] = 3
                    elif(id[3] == '3'):
                        car3_anti_RVC_mode[int(id[5:])] = 3 
                    # traci.vehicle.setSpeedMode(id,0b011110)
                    traci.vehicle.setMaxSpeed(id,35)
                    traci.vehicle.setLaneChangeMode(id,0b000000000000) # 마음대로 차선 변경 x 
                    if type(traci.vehicle.getFollower(id)) is not None and traci.vehicle.getFollower(id)[0] != '':
                        # traci.vehicle.setAccel(id,3) ## 가속도 높임
                        # traci.vehicle.setSpeed(id,traci.vehicle.getSpeed(follower_id))
                        # set_v(id,traci.vehicle.getSpeed(follower_id),3)
                        set_a(id,3)
                        set_v(id,traci.vehicle.getSpeed(follower_id))

                else: #LK_w 모드
                    if(id[3] == '0'):
                        car0_anti_RVC_mode[int(id[5:])] = 4
                    elif(id[3] == '1'):
                        car1_anti_RVC_mode[int(id[5:])] = 4
                    elif(id[3] == '2'):
                        car2_anti_RVC_mode[int(id[5:])] = 4
                    elif(id[3] == '3'):
                        car3_anti_RVC_mode[int(id[5:])] = 4 
                    # traci.vehicle.setSpeedMode(id,0b011110)
                    traci.vehicle.setMaxSpeed(id,35)
                    traci.vehicle.setLaneChangeMode(id,0b000000000000) # 마음대로 차선 변경 x 
                    # traci.vehicle.setDecel(id,4.5) ## 감속도
                    # front, back clearance가 작아서 (x_back-c_Lk)항 과, x_forward -c_LK*2)항이 0이 0인경우는 단순히 x_forward : 2*x_back으로 가중치 부여한 속도로 결정.(if else 없애고 else 아래 수식으로만 사용해도 될것 같음.)
                    speed = (traci.vehicle.getSpeed(follower_id)*(x_forward) + traci.vehicle.getSpeed(Leader_id)*(2*x_back))/(x_forward+2*x_back)
                    # traci.vehicle.setSpeed(id,speed)
                    if ((speed-set_a(id,3))> traci.vehicle.getSpeed(id)):
                        # set_v(id,speed,3)
                        set_a(id,3)
                        set_v(id,speed)
                    elif((speed+abs(set_a(id,-4.5))) < traci.vehicle.getSpeed(id)):
                        # set_v(id,speed,-4.5)
                        set_a(id,-4.5)
                        set_v(id,speed)
                    else:
                        # traci.vehicle.setSpeed(id,speed)
                        set_v(id,speed)
                        
                    print(id,"'s v_w_desire : ",speed)
                # traci.vehicle.setLaneChangeMode(id,0b000000001001)
                if len(traci.vehicle.getRightLeaders(id))!=0 and right_follower_id is not None:
                    front_relative_velocity_term = max( [traci.vehicle.getSpeed(id) - traci.vehicle.getSpeed(Right_leader_id[0][0]) ,0] )*time_gap_LC_1
                    front_minimum_clearance_trem = max([traci.vehicle.getSpeed(id)*time_gap_LC_2,c_LC])
                    front_SD_LC = front_relative_velocity_term +front_minimum_clearance_trem
                    back_relative_velocity_term = max( [traci.vehicle.getSpeed(right_follower_id)-traci.vehicle.getSpeed(id),0] )*time_gap_LC_1
                    back__minimum_clearance_trem = max([traci.vehicle.getSpeed(right_follower_id)*time_gap_LC_2,c_LC])
                    back_SD_LC = back_relative_velocity_term + back__minimum_clearance_trem
                    if((traci.vehicle.getPosition(Right_leader_id[0][0])[0]-ego_x_pos) >=front_SD_LC  and (ego_x_pos -traci.vehicle.getPosition(right_follower_id)[0])>=back_SD_LC):
                        traci.vehicle.setLaneChangeMode(id,0b000000001001)
                        traci.vehicle.changeLaneRelative(id,-1,5)
                elif len(traci.vehicle.getRightLeaders(id)) !=0 and right_follower_id is None:
                    front_relative_velocity_term = max( [traci.vehicle.getSpeed(id) - traci.vehicle.getSpeed(Right_leader_id[0][0]) ,0] )*time_gap_LC_1
                    front_minimum_clearance_trem = max([traci.vehicle.getSpeed(id)*time_gap_LC_2,c_LC])
                    front_SD_LC = front_relative_velocity_term +front_minimum_clearance_trem
                    if(traci.vehicle.getPosition(Right_leader_id[0][0])[0]-ego_x_pos) >=front_SD_LC:
                        traci.vehicle.setLaneChangeMode(id,0b000000001001)
                        traci.vehicle.changeLaneRelative(id,-1,5)
                elif len(traci.vehicle.getRightLeaders(id))==0 and right_follower_id is not None:
                    back_relative_velocity_term = max( [traci.vehicle.getSpeed(right_follower_id)-traci.vehicle.getSpeed(id),0] )*time_gap_LC_1
                    back__minimum_clearance_trem = max([traci.vehicle.getSpeed(right_follower_id)*time_gap_LC_2,c_LC])
                    back_SD_LC = back_relative_velocity_term + back__minimum_clearance_trem
                    if (ego_x_pos -traci.vehicle.getPosition(right_follower_id)[0])>=back_SD_LC:
                        traci.vehicle.setLaneChangeMode(id,0b000000001001)
                        traci.vehicle.changeLaneRelative(id,-1,5)
                elif len(traci.vehicle.getRightLeaders(id))==0 and right_follower_id is None:
                    traci.vehicle.setLaneChangeMode(id,0b000000001001)
                    traci.vehicle.changeLaneRelative(id,-1,5)
                
        else: #LK 모드 (앞차가 없을 때)
            if(id[3] == '0'):
                car0_anti_RVC_mode[int(id[5:])] = 2
            elif(id[3] == '1'):
                car1_anti_RVC_mode[int(id[5:])] = 2
            elif(id[3] == '2'):
                car2_anti_RVC_mode[int(id[5:])] = 2
            elif(id[3] == '3'):
                car3_anti_RVC_mode[int(id[5:])] = 2 
            # traci.vehicle.setSpeedMode(id,0b011110)
            traci.vehicle.setMaxSpeed(id,35)
            traci.vehicle.setLaneChangeMode(id,0b000000000000) # 마음대로 차선 변경 x 
            if type(traci.vehicle.getFollower(id)) is not None and traci.vehicle.getFollower(id)[0] != '':
                # traci.vehicle.setAccel(id,3) ## 가속도 높임
                # traci.vehicle.setSpeed(id,traci.vehicle.getSpeed(follower_id))
                # set_v(id,traci.vehicle.getSpeed(follower_id),3)
                set_a(id,3)
                set_v(id,traci.vehicle.getSpeed(follower_id))


    elif(no_road_dir =='right'):  # 오른쪽 차로는 없고 왼쪽 차로가 있는 경우
        if len(traci.vehicle.getLeftLeaders(id)) !=0:
            Left_leader_id = traci.vehicle.getLeftLeaders(id)
        if len(traci.vehicle.getLeftFollowers(id)) !=0:
            Left_follower_id = traci.vehicle.getLeftFollowers(id)
        if traci.vehicle.getLeader(id) is not None:
            Leader_id,x_forward = traci.vehicle.getLeader(id)
            rel_v_with_front = traci.vehicle.getSpeed(Leader_id) - ego_v_x
        if type(traci.vehicle.getFollower(id)) is not None and traci.vehicle.getFollower(id)[0] != '':
            follower_id, x_back = traci.vehicle.getFollower(id)  

        if traci.vehicle.getLeader(id) is not None and type(traci.vehicle.getFollower(id)) is not None and traci.vehicle.getFollower(id)[0] != '': #LK mode
            v_p = traci.vehicle.getSpeed(Leader_id)
            des_clearance = c_LK + time_gap_LK*v_p #앞차 속도에 따른 앞차와의 ACC 안전거리 (safe)
            des_clearance2 = c_LK + time_gap_LC_prepare*v_p #앞차 속도에 따른 앞차와의 ACC 안전거리 (aggressive)
            if(des_clearance<= x_forward):    #Lk 모드
                if(id[3] == '0'):
                    car0_anti_RVC_mode[int(id[5:])] = 2
                elif(id[3] == '1'):
                    car1_anti_RVC_mode[int(id[5:])] = 2
                elif(id[3] == '2'):
                    car2_anti_RVC_mode[int(id[5:])] = 2
                elif(id[3] == '3'):
                    car3_anti_RVC_mode[int(id[5:])] = 2 
                # traci.vehicle.setSpeedMode(id,0b011110)
                traci.vehicle.setMaxSpeed(id,35)
                traci.vehicle.setLaneChangeMode(id,0b000000000000) # 마음대로 차선 변경 x 
                if type(traci.vehicle.getFollower(id)) is not None and traci.vehicle.getFollower(id)[0] != '':
                    # traci.vehicle.setAccel(id,3) ## 가속도 높임
                    # if traci.vehicle.getSpeed(follower_id)>=traci.vehicle.getSpeed(id):
                        # traci.vehicle.setSpeed(id,traci.vehicle.getSpeed(follower_id))
                    # set_v(id,traci.vehicle.getSpeed(follower_id),3)
                    set_a(id,3)
                    set_v(id,traci.vehicle.getSpeed(follower_id))
                    # else:## 충돌 위험이 없는 경우 일반 주행 속도로 설정
                    #     # print(id,' LK free speed') 
                    #     traci.vehicle.setSpeedMode(id,0b011111)
            else:
                if id:
                    LEFT_FOLLOWER =[]
                    RIGHT_FOLLOWER=[]
                    traci.vehicle.subscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 30.0, [tc.VAR_POSITION])
                    # print(id,' subscribeContext')
                    for v in traci.vehicle.getContextSubscriptionResults(str(id)):
                        left_follower_id = find_followers(id, v,LEFT_FOLLOWER,RIGHT_FOLLOWER)[0]
                        # right_follower_id = find_followers(id, v,LEFT_FOLLOWER,RIGHT_FOLLOWER)[1]
                    traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 30.0)
                if(des_clearance2<=x_forward): #LK_r 모드
                    if(id[3] == '0'):
                        car0_anti_RVC_mode[int(id[5:])] = 3
                    elif(id[3] == '1'):
                        car1_anti_RVC_mode[int(id[5:])] = 3
                    elif(id[3] == '2'):
                        car2_anti_RVC_mode[int(id[5:])] = 3
                    elif(id[3] == '3'):
                        car3_anti_RVC_mode[int(id[5:])] = 3 
                    # traci.vehicle.setSpeedMode(id,0b011110)
                    traci.vehicle.setMaxSpeed(id,35)
                    traci.vehicle.setLaneChangeMode(id,0b000000000000) # 마음대로 차선 변경 x 
                    if type(traci.vehicle.getFollower(id)) is not None and traci.vehicle.getFollower(id)[0] != '':
                        # traci.vehicle.setAccel(id,3) ## 가속도 높임
                        # traci.vehicle.setSpeed(id,traci.vehicle.getSpeed(follower_id))
                        # set_v(id,traci.vehicle.getSpeed(follower_id),3)
                        set_a(id,3)
                        set_v(id,traci.vehicle.getSpeed(follower_id))

                else: # LK_w 모드
                    if(id[3] == '0'):
                        car0_anti_RVC_mode[int(id[5:])] = 4
                    elif(id[3] == '1'):
                        car1_anti_RVC_mode[int(id[5:])] = 4
                    elif(id[3] == '2'):
                        car2_anti_RVC_mode[int(id[5:])] = 4
                    elif(id[3] == '3'):
                        car3_anti_RVC_mode[int(id[5:])] = 4 
                    # traci.vehicle.setSpeedMode(id,0b011110)
                    traci.vehicle.setMaxSpeed(id,35)
                    traci.vehicle.setLaneChangeMode(id,0b000000000000) # 마음대로 차선 변경 x 
                    # traci.vehicle.setDecel(id,4.5) ## 감속도
                    # front, back clearance가 작아서 (x_back-c_Lk)항 과, x_forward -c_LK*2)항이 0이 0인경우는 단순히 x_forward : 2*x_back으로 가중치 부여한 속도로 결정.(if else 없애고 else 아래 수식으로만 사용해도 될것 같음.)
                    speed = (traci.vehicle.getSpeed(follower_id)*(x_forward) + traci.vehicle.getSpeed(Leader_id)*(2*x_back))/(x_forward+2*x_back)
                    # traci.vehicle.setSpeed(id,speed)
                    if ((speed-set_a(id,3))> traci.vehicle.getSpeed(id)):
                        # set_v(id,speed,3)
                        set_a(id,3)
                        set_v(id,speed)
                    elif((speed+abs(set_a(id,-4.5))) < traci.vehicle.getSpeed(id)):
                        # set_v(id,speed,-4.5)
                        set_a(id,-4.5)
                        set_v(id,speed)
                    else:
                        # traci.vehicle.setSpeed(id,speed)
                        set_v(id,speed)
                    print(id,"'s v_w_desire : ",speed)
                # traci.vehicle.setLaneChangeMode(id,0b000000001001)
                if len(traci.vehicle.getLeftLeaders(id)) !=0 and left_follower_id is not None:
                    front_relative_velocity_term = max( [traci.vehicle.getSpeed(id) - traci.vehicle.getSpeed(Left_leader_id[0][0]) ,0] )*time_gap_LC_1
                    front_minimum_clearance_trem = max([traci.vehicle.getSpeed(id)*time_gap_LC_2,c_LC])
                    front_SD_LC = front_relative_velocity_term +front_minimum_clearance_trem
                    back_relative_velocity_term = max( [traci.vehicle.getSpeed(left_follower_id)-traci.vehicle.getSpeed(id),0] )*time_gap_LC_1
                    back__minimum_clearance_trem = max([traci.vehicle.getSpeed(left_follower_id)*time_gap_LC_2,c_LC])
                    back_SD_LC = back_relative_velocity_term + back__minimum_clearance_trem
                    traci.vehicle.setLaneChangeMode(id,0b000000000000)
                    # print(id,' : left following neighbor',traci.vehicle.getNeighbors(id,0b000))
                    # print(id,' : left follower',traci.vehicle.getLeftFollowers(id,blockingOnly=True),"is wrong info") ## wrong 
                    # print(id,' : left follower is ',left_follower_id,"-> is true info")
                    if(((traci.vehicle.getPosition(Left_leader_id[0][0])[0]-ego_x_pos) >= front_SD_LC)  and ((ego_x_pos -traci.vehicle.getPosition(left_follower_id)[0])>= back_SD_LC)):
                        # print(id,'LC')
                        # print('ego_x_pos : ',ego_x_pos)
                        # print('left follower pos : ',traci.vehicle.getPosition(Left_follower_id[0][0])[0])
                        # print('rel_x_left_back : ', (ego_x_pos -traci.vehicle.getPosition(Left_follower_id[0][0])[0]))
                        # print('back_SD_LC : ',back_SD_LC)

                        traci.vehicle.setLaneChangeMode(id,0b000000001001)
                        traci.vehicle.changeLaneRelative(id,1,5)
                elif len(traci.vehicle.getLeftLeaders(id)) !=0 and left_follower_id is None:
                    front_relative_velocity_term = max( [traci.vehicle.getSpeed(id) - traci.vehicle.getSpeed(Left_leader_id[0][0]) ,0] )*time_gap_LC_1
                    front_minimum_clearance_trem = max([traci.vehicle.getSpeed(id)*time_gap_LC_2,c_LC])
                    front_SD_LC = front_relative_velocity_term +front_minimum_clearance_trem
                    traci.vehicle.setLaneChangeMode(id,0b000000000000)
                    if(traci.vehicle.getPosition(Left_leader_id[0][0])[0]-ego_x_pos) >= front_SD_LC:
                        traci.vehicle.setLaneChangeMode(id,0b000000001001)
                        traci.vehicle.changeLaneRelative(id,1,5)
                elif len(traci.vehicle.getLeftLeaders(id)) ==0 and left_follower_id is not None:
                    back_relative_velocity_term = max( [traci.vehicle.getSpeed(left_follower_id)-traci.vehicle.getSpeed(id),0] )*time_gap_LC_1
                    back__minimum_clearance_trem = max([traci.vehicle.getSpeed(left_follower_id)*time_gap_LC_2,c_LC])
                    back_SD_LC = back_relative_velocity_term + back__minimum_clearance_trem
                    traci.vehicle.setLaneChangeMode(id,0b000000000000)
                    if (ego_x_pos -traci.vehicle.getPosition(left_follower_id)[0])>=back_SD_LC:
                        traci.vehicle.setLaneChangeMode(id,0b000000001001)
                        traci.vehicle.changeLaneRelative(id,1,5)
                elif len(traci.vehicle.getLeftLeaders(id)) ==0 and left_follower_id is None:
                    traci.vehicle.setLaneChangeMode(id,0b000000001001)
                    traci.vehicle.changeLaneRelative(id,1,5)
        else: # LK 모드 (앞 차량이 없을 때)
            if(id[3] == '0'):
                car0_anti_RVC_mode[int(id[5:])] = 2
            elif(id[3] == '1'):
                car1_anti_RVC_mode[int(id[5:])] = 2
            elif(id[3] == '2'):
                car2_anti_RVC_mode[int(id[5:])] = 2
            elif(id[3] == '3'):
                car3_anti_RVC_mode[int(id[5:])] = 2 
            # traci.vehicle.setSpeedMode(id,0b011110)
            traci.vehicle.setMaxSpeed(id,35)
            traci.vehicle.setLaneChangeMode(id,0b000000000000) # 마음대로 차선 변경 x 
            if type(traci.vehicle.getFollower(id)) is not None and traci.vehicle.getFollower(id)[0] != '':
                # traci.vehicle.setAccel(id,3) ## 가속도 높임
                # traci.vehicle.setSpeed(id,traci.vehicle.getSpeed(follower_id))
                # set_v(id,traci.vehicle.getSpeed(follower_id),3)
                set_a(id,3)
                set_v(id,traci.vehicle.getSpeed(follower_id))
def ego_vehicle_LC_success():
    global lane_buffer_ego
    cur_lane = traci.vehicle.getLaneIndex('ego')
    # print('lane_buffer_ego : ',lane_buffer_ego)
    # print('cur_lane : ',cur_lane)
    last_lane = lane_buffer_ego
    if(last_lane != cur_lane):
        # traci.vehicle.setSpeedMode(id,0b011000)
        # traci.vehicle.setLaneChangeMode(id,0b101010101010)
        traci.vehicle.setLaneChangeMode('ego',0b100010101010) #-> 이거 대신해서
        # ACC 모드로 변경해야함.
        return True
    else:
        return False
def Lane_has_changed(id):
    cur_lane = traci.vehicle.getLaneIndex(id)
    if id[3] =='0':
        last_lane =lane_buffer_car0[int(id[5:])]
    elif id[3] =='1':
        last_lane =lane_buffer_car1[int(id[5:])]  
    elif id[3] =='2':
        last_lane =lane_buffer_car2[int(id[5:])] 
    elif id[3] =='3': 
        last_lane =lane_buffer_car3[int(id[5:])] 
    if(last_lane != cur_lane):
        # traci.vehicle.setSpeedMode(id,0b011000)
        # traci.vehicle.setLaneChangeMode(id,0b101010101010)
        traci.vehicle.setLaneChangeMode(id,0b100010101010)
        return True
    else:
        return False
def Back_Car_has_changed(id):
    if type(traci.vehicle.getFollower(id)) is not None and traci.vehicle.getFollower(id)[0] != '':
        cur_back_car_id = traci.vehicle.getFollower(id)[0]
    if id[3] =='0':
        last_back_car_id =back_buffer_car0[int(id[5:])]
    elif id[3] =='1':
        last_back_car_id =back_buffer_car1[int(id[5:])]
    elif id[3] =='2':
        last_back_car_id =back_buffer_car2[int(id[5:])] 
    elif id[3] =='3': 
        last_back_car_id =back_buffer_car3[int(id[5:])] 

    if(last_back_car_id == cur_back_car_id):
        return False 
    else:
        # traci.vehicle.setSpeedMode(id,0b011111)
        traci.vehicle.setLaneChangeMode(id,0b101010101010)
        return True
def lane_change_to_right(id): ### left, right follower 수정 필요
    time_gap_LC_prepare = 0.74 
    time_gap_LC_1 =1
    time_gap_LC_2 =0.5 
    c_LC =12 # min clearance for lane change
    ego_x_pos = traci.vehicle.getPosition(id)[0]
    if traci.vehicle.getLeftLeaders(id) is not None and len(traci.vehicle.getLeftLeaders(id)) !=0:
        Left_leader_id= traci.vehicle.getLeftLeaders(id)
    if traci.vehicle.getLeftFollowers(id) is not None and len(traci.vehicle.getLeftFollowers(id)) !=0:
        Left_follower_id= traci.vehicle.getLeftFollowers(id)
    if traci.vehicle.getRightLeaders(id) is not None and len(traci.vehicle.getRightLeaders(id)) !=0:
        Right_leader_id= traci.vehicle.getRightLeaders(id)
    if traci.vehicle.getRightFollowers(id) is not None and len(traci.vehicle.getRightFollowers(id)) !=0:
        Right_follower_id= traci.vehicle.getRightFollowers(id)

    if len(traci.vehicle.getRightLeaders(id))!=0 and len(traci.vehicle.getRightFollowers(id)) !=0:
        front_relative_velocity_term = max( [traci.vehicle.getSpeed(id) - traci.vehicle.getSpeed(Right_leader_id[0][0]) ,0] )*time_gap_LC_1
        front_minimum_clearance_trem = max([traci.vehicle.getSpeed(id)*time_gap_LC_2,c_LC])
        front_SD_LC = front_relative_velocity_term +front_minimum_clearance_trem
        back_relative_velocity_term = max( [traci.vehicle.getSpeed(Right_follower_id[0][0])-traci.vehicle.getSpeed(id),0] )*time_gap_LC_1
        back__minimum_clearance_trem = max([traci.vehicle.getSpeed(Right_follower_id[0][0])*time_gap_LC_2,c_LC])
        back_SD_LC = back_relative_velocity_term + back__minimum_clearance_trem
        if((traci.vehicle.getPosition(Right_leader_id[0][0])[0]-ego_x_pos) >=front_SD_LC  and (ego_x_pos -traci.vehicle.getPosition(Right_follower_id[0][0])[0])>=back_SD_LC):   
            traci.vehicle.setLaneChangeMode(id,0b000000001001)
            traci.vehicle.changeLaneRelative(id,-1,5)
    elif len(traci.vehicle.getRightLeaders(id)) !=0 and len(traci.vehicle.getRightFollowers(id))==0:
        front_relative_velocity_term = max( [traci.vehicle.getSpeed(id) - traci.vehicle.getSpeed(Right_leader_id[0][0]) ,0] )*time_gap_LC_1
        front_minimum_clearance_trem = max([traci.vehicle.getSpeed(id)*time_gap_LC_2,c_LC])
        front_SD_LC = front_relative_velocity_term +front_minimum_clearance_trem
        if(traci.vehicle.getPosition(Right_leader_id[0][0])[0]-ego_x_pos) >=front_SD_LC:
            traci.vehicle.setLaneChangeMode(id,0b000000001001)
            traci.vehicle.changeLaneRelative(id,-1,5)
    elif len(traci.vehicle.getRightLeaders(id))==0 and len(traci.vehicle.getRightFollowers(id)) !=0:
        back_relative_velocity_term = max( [traci.vehicle.getSpeed(Right_follower_id[0][0])-traci.vehicle.getSpeed(id),0] )*time_gap_LC_1
        back__minimum_clearance_trem = max([traci.vehicle.getSpeed(Right_follower_id[0][0])*time_gap_LC_2,c_LC])
        back_SD_LC = back_relative_velocity_term + back__minimum_clearance_trem
        if (ego_x_pos -traci.vehicle.getPosition(Right_follower_id[0][0])[0])>=back_SD_LC:
            traci.vehicle.setLaneChangeMode(id,0b000000001001)
            traci.vehicle.changeLaneRelative(id,-1,5)
    elif len(traci.vehicle.getRightLeaders(id))==0 and len(traci.vehicle.getRightFollowers(id)) ==0:
        traci.vehicle.setLaneChangeMode(id,0b000000001001)
        traci.vehicle.changeLaneRelative(id,-1,5)

def lane_change_to_left(id): ### left, right follower 수정 필요
    time_gap_LC_prepare = 0.74 
    time_gap_LC_1 =1
    time_gap_LC_2 =0.5 
    c_LC =12 # min clearance for lane change
    ego_x_pos = traci.vehicle.getPosition(id)[0]
    if traci.vehicle.getLeftLeaders(id) is not None and len(traci.vehicle.getLeftLeaders(id)) !=0:
        Left_leader_id= traci.vehicle.getLeftLeaders(id)
    if traci.vehicle.getLeftFollowers(id) is not None and len(traci.vehicle.getLeftFollowers(id)) !=0:
        Left_follower_id= traci.vehicle.getLeftFollowers(id)
    if traci.vehicle.getRightLeaders(id) is not None and len(traci.vehicle.getRightLeaders(id)) !=0:
        Right_leader_id= traci.vehicle.getRightLeaders(id)
    if traci.vehicle.getRightFollowers(id) is not None and len(traci.vehicle.getRightFollowers(id)) !=0:
        Right_follower_id= traci.vehicle.getRightFollowers(id)

    # traci.vehicle.setLaneChangeMode(id,0b000000001001)
    if len(traci.vehicle.getLeftLeaders(id)) !=0 and len(traci.vehicle.getLeftFollowers(id)) !=0:
        front_relative_velocity_term = max( [traci.vehicle.getSpeed(id) - traci.vehicle.getSpeed(Left_leader_id[0][0]) ,0] )*time_gap_LC_1
        front_minimum_clearance_trem = max([traci.vehicle.getSpeed(id)*time_gap_LC_2,c_LC])
        front_SD_LC = front_relative_velocity_term +front_minimum_clearance_trem
        back_relative_velocity_term = max( [traci.vehicle.getSpeed(Left_follower_id[0][0])-traci.vehicle.getSpeed(id),0] )*time_gap_LC_1
        back__minimum_clearance_trem = max([traci.vehicle.getSpeed(Left_follower_id[0][0])*time_gap_LC_2,c_LC])
        back_SD_LC = back_relative_velocity_term + back__minimum_clearance_trem

        if((traci.vehicle.getPosition(Left_leader_id[0][0])[0]-ego_x_pos) >= front_SD_LC  and (ego_x_pos -traci.vehicle.getPosition(Left_follower_id[0][0])[0])>= back_SD_LC):
            traci.vehicle.setLaneChangeMode(id,0b000000001001)
            traci.vehicle.changeLaneRelative(id,1,5)
        elif len(traci.vehicle.getLeftLeaders(id)) !=0 and len(traci.vehicle.getLeftFollowers(id)) ==0:
            front_relative_velocity_term = max( [traci.vehicle.getSpeed(id) - traci.vehicle.getSpeed(Left_leader_id[0][0]) ,0] )*time_gap_LC_1
            front_minimum_clearance_trem = max([traci.vehicle.getSpeed(id)*time_gap_LC_2,c_LC])
            front_SD_LC = front_relative_velocity_term +front_minimum_clearance_trem
            if(traci.vehicle.getPosition(Left_leader_id[0][0])[0]-ego_x_pos) >= front_SD_LC:
                traci.vehicle.setLaneChangeMode(id,0b000000001001)
                traci.vehicle.changeLaneRelative(id,1,5)
        elif len(traci.vehicle.getLeftLeaders(id)) ==0 and len(traci.vehicle.getLeftFollowers(id)) !=0:
            back_relative_velocity_term = max( [traci.vehicle.getSpeed(Left_follower_id[0][0])-traci.vehicle.getSpeed(id),0] )*time_gap_LC_1
            back__minimum_clearance_trem = max([traci.vehicle.getSpeed(Left_follower_id[0][0])*time_gap_LC_2,c_LC])
            back_SD_LC = back_relative_velocity_term + back__minimum_clearance_trem
            if (ego_x_pos -traci.vehicle.getPosition(Left_follower_id[0][0])[0])>=back_SD_LC:
                traci.vehicle.setLaneChangeMode(id,0b000000001001)
                traci.vehicle.changeLaneRelative(id,1,5)
        elif len(traci.vehicle.getLeftLeaders(id)) ==0 and len(traci.vehicle.getLeftFollowers(id)) ==0:
            traci.vehicle.setLaneChangeMode(id,0b000000001001)
            traci.vehicle.changeLaneRelative(id,1,5)
  


def run():  
    """execute the TraCI control loop"""
    # step = 0
    
    
    # answer =svc_RBF.predict([[1,15]]) 
    # print(answer[0])
    
    # r_seed = random.randint(20,80)
    r_seed =0
    global avoiding_LC
    global finish_truck
    global lane_buffer_ego
    global ego_LC_success
    ego_LC_success =False
    lane_buffer_ego=0
    end_flag =False
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        vehs = traci.vehicle.getIDList()
        for veh_id in vehs:
            traci.vehicle.setMinGap(veh_id,'0')
            traci.vehicle.setSpeedMode(veh_id,0b000000)
            traci.vehicle.setLaneChangeMode(veh_id,0b000000000000) # 마음대로 차선 변경 x 
        # print(traci.simulation.getCollisions())
        # log = traci.simulation.getCollisions()
        # if(len(log) is not 0):
        #     print(str(log[0]).split(','))
        #     print(str(log[0]).split(',')[1]== ' victim=ego')
        ## ego_vehicle 처음 차로 번호    
        

        if len(traci.simulation.getCollidingVehiclesIDList()) !=0:
            # print(traci.simulation.getCollidingVehiclesIDList())
            # print(len(traci.simulation.getCollidingVehiclesIDList()))
            # print(traci.simulation.getCollidingVehiclesIDList()[0])
            if len(traci.simulation.getCollidingVehiclesIDList()) ==2:
                collision.append(traci.simulation.getCollidingVehiclesIDList())
            elif len(traci.simulation.getCollidingVehiclesIDList()) ==4:
                # print(traci.simulation.getCollidingVehiclesIDList()[:2])
                # print(traci.simulation.getCollidingVehiclesIDList()[2:4])
                collision.append(traci.simulation.getCollidingVehiclesIDList()[:2])
                collision.append(traci.simulation.getCollidingVehiclesIDList()[2:4])
            log = traci.simulation.getCollisions()
            if str(log[0]).split(',')[1]== ' victim=ego' or str(log[0]).split(',')[0]== 'Collision(collider=ego' :
                print('close')
                traci.close()
                sys.stdout.flush()
                end_flag = True
                break
        
        for veh_id in vehs:
            vehicle_state = []
            if traci.vehicle.getRoadID(veh_id) == 'E5': 
                vehicle_state.append(traci.simulation.getTime())
                # print("cur_time : ",traci.simulation.getTime())
                vehicle_state.append(veh_id)
                # print("veh_id : ",veh_id)

                x1,y1 = traci.vehicle.getPosition(veh_id)
                vehicle_state.append(x1)
                vehicle_state.append(y1)
                # print("p_x, p_y : " ,traci.vehicle.getPosition(veh_id))

                vehicle_state.append(traci.vehicle.getSpeed(veh_id))
                # print("v_x : ", traci.vehicle.getSpeed(veh_id))
                
                vehicle_state.append(traci.vehicle.getLateralSpeed(veh_id))
                # print("v_y : ",traci.vehicle.getLateralSpeed(veh_id))
                
                ID = veh_id.split('.')
                acceleration_x =0
                
                
                acceleration_x= traci.vehicle.getAcceleration(veh_id)            
                vehicle_state.append(acceleration_x)
                acceleration_y=0
                if veh_id== 'ego' and not (len(ego)==0):
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - ego[-1][5])/step_length
                if veh_id== 'car.leftrear0' and not (len(leftrear0))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - leftrear0[-1][5])/step_length
                if veh_id== 'car.leftrear1' and not (len(leftrear1))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - leftrear1[-1][5])/step_length
                if veh_id== 'accel.rear' and not (len(rear))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rear[-1][5])/step_length
                if veh_id== 'car.rightrear0' and not (len(rightrear0))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rightrear0[-1][5])/step_length
                if veh_id== 'car.rightrear1' and not (len(rightrear1))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rightrear1[-1][5])/step_length
                if veh_id== 'car.leftforward0' and not (len(leftforward0))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - leftforward0[-1][5])/step_length
                if veh_id== 'car.leftforward1' and not (len(leftforward1))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - leftforward1[-1][5])/step_length
                if veh_id== 'car.leftforward2' and not (len(leftforward2))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - leftforward2[-1][5])/step_length
                if veh_id== 'car.forward' and not (len(forward))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - forward[-1][5])/step_length
                if veh_id== 'car.rightforward0' and not (len(rightforward0))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rightforward0[-1][5])/step_length
                if veh_id== 'car.rightforward1' and not (len(rightforward1))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rightforward1[-1][5])/step_length
                if veh_id== 'car.rightforward2' and not (len(rightforward2))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rightforward2[-1][5])/step_length

                vehicle_state.append(acceleration_y)
                
                if(traci.simulation.getTime()>4 and traci.simulation.getTime()<4.1):
                    traci.vehicle.changeLaneRelative('ego',1,5)
                vehicle_state.append(traci.vehicle.getAngle(veh_id))
               
                # c_LK = 2 #min clearance for lane keeping
                # time_gap_LK = 1.36  #95% percentile time_gap (most safe) // 1.36s -> 중간값   0.74s -> 5% persentile 가장 위험.
                # time_gap_LC_prepare = 0.74 
                
                # if traci.vehicle.getLeader(veh_id) is not None:
                    
                #     Leader_id,x_forward = traci.vehicle.getLeader(veh_id)
                #     vehicle_state.append(Leader_id)
                #     vehicle_state.append(x_forward)
                #     v_forward = traci.vehicle.getSpeed(traci.vehicle.getLeader(veh_id)[0]) -traci.vehicle.getSpeed(veh_id) #(relative speed(v_for) = front_vehicle speed - ego_vehicle)
                #     # print("v_for : ",v_for)
                #     vehicle_state.append(v_forward)
                    
                #     a_forward = None
                #     ID = veh_id.split('.')
                #     if ID[0][0] == 'a' and ID[0][-1]=='0' and not (len(accel0[int(ID[1])])==0) and accel0[int(ID[1])][-1][11] is not None:
                #         a_forward = (v_forward - accel0[int(ID[1])][-1][11])/step_length
                #     if ID[0][0] == 'a' and ID[0][-1]=='1'and not (len(accel1[int(ID[1])])==0) and accel1[int(ID[1])][-1][11] is not None:
                #         a_forward = (v_forward  - accel1[int(ID[1])][-1][11])/step_length
                #     if ID[0][0] == 't' and ID[0][-1]=='0'and not (len(truck0[int(ID[1])])==0) and truck0[int(ID[1])][-1][11] is not None:
                #         a_forward = (v_forward  - truck0[int(ID[1])][-1][11])/step_length
                #     if ID[0][0] == 't' and ID[0][-1]=='1'and not (len(truck1[int(ID[1])])==0) and truck1[int(ID[1])][-1][11] is not None:
                #         a_forward = (v_forward  - truck1[int(ID[1])][-1][11])/step_length
                #     if ID[0][0] == 'c' and ID[0][-1]=='0'and not (len(car0[int(ID[1])])==0) and car0[int(ID[1])][-1][11] is not None:
                #         a_forward = (v_forward  - car0[int(ID[1])][-1][11])/step_length
                #     if ID[0][0] == 'c' and ID[0][-1]=='1'and not (len(car1[int(ID[1])])==0) and car1[int(ID[1])][-1][11] is not None:
                #         a_forward = (v_forward  - car1[int(ID[1])][-1][11])/step_length
                #     if ID[0][0] == 'c' and ID[0][-1]=='2'and not (len(car2[int(ID[1])])==0) and car2[int(ID[1])][-1][11] is not None:
                #         a_forward = (v_forward  - car2[int(ID[1])][-1][11])/step_length
                #     if ID[0][0] == 'c' and ID[0][-1]=='3'and not (len(car3[int(ID[1])])==0) and car3[int(ID[1])][-1][11] is not None:
                #         a_forward = (v_forward  - car3[int(ID[1])][-1][11])/step_length

                #     vehicle_state.append(a_forward)
                #     v_p = traci.vehicle.getSpeed(Leader_id)
                #     des_clearance = c_LK + time_gap_LK*v_p # 앞차와의 상대 속도에 따른 ACC 안전거리 (safe)
                #     des_clearance2 = c_LK + time_gap_LC_prepare*v_p # 앞차와의 상대 속도에 따른 ACC 안전거리 (aggressive)
                #     vehicle_state.append(des_clearance)
                #     vehicle_state.append(des_clearance2)
                # else:
                #     vehicle_state.append(None)
                #     vehicle_state.append(None)
                #     # print("v_for : ",None)
                #     vehicle_state.append(None)
                #     vehicle_state.append(None)
                #     vehicle_state.append(None)
                #     vehicle_state.append(None)
                    
                # # print("Follower_id, x_bac : ",traci.vehicle.getFollower(veh_id) )
                # Follower_id,x_backward = traci.vehicle.getFollower(veh_id)
                         
                # if type(traci.vehicle.getFollower(veh_id)) is not None and traci.vehicle.getFollower(veh_id)[0] != '':
                #     v_backward= traci.vehicle.getSpeed(traci.vehicle.getFollower(veh_id)[0]) -traci.vehicle.getSpeed(veh_id) #(relative speed(v_bac) = back_vehicle speed - ego_vehicle)
                #     vehicle_state.append(Follower_id)
                #     vehicle_state.append(x_backward)
                #     # print("v_bac : ",v_bac)
                #     vehicle_state.append(v_backward)
                #     a_backward = None
                #     ID = veh_id.split('.')
                #     if ID[0][0] == 'a' and ID[0][-1]=='0' and not (len(accel0[int(ID[1])])==0) and accel0[int(ID[1])][-1][17] is not None:
                #         a_backward = (v_backward - accel0[int(ID[1])][-1][17])/step_length
                #     if ID[0][0] == 'a' and ID[0][-1]=='1'and not (len(accel1[int(ID[1])])==0) and accel1[int(ID[1])][-1][17] is not None:
                #         a_backward = (v_backward  - accel1[int(ID[1])][-1][17])/step_length
                #     if ID[0][0] == 't' and ID[0][-1]=='0'and not (len(truck0[int(ID[1])])==0) and truck0[int(ID[1])][-1][17] is not None:
                #         a_backward = (v_backward  - truck0[int(ID[1])][-1][17])/step_length
                #     if ID[0][0] == 't' and ID[0][-1]=='1'and not (len(truck1[int(ID[1])])==0) and truck1[int(ID[1])][-1][17] is not None:
                #         a_backward = (v_backward  - truck1[int(ID[1])][-1][17])/step_length
                #     if ID[0][0] == 'c' and ID[0][-1]=='0'and not (len(car0[int(ID[1])])==0) and car0[int(ID[1])][-1][17] is not None:
                #         a_backward = (v_backward  - car0[int(ID[1])][-1][17])/step_length
                #     if ID[0][0] == 'c' and ID[0][-1]=='1'and not (len(car1[int(ID[1])])==0) and car1[int(ID[1])][-1][17] is not None:
                #         a_backward = (v_backward  - car1[int(ID[1])][-1][17])/step_length
                #     if ID[0][0] == 'c' and ID[0][-1]=='2'and not (len(car2[int(ID[1])])==0) and car2[int(ID[1])][-1][17] is not None:
                #         a_backward = (v_backward  - car2[int(ID[1])][-1][17])/step_length
                #     if ID[0][0] == 'c' and ID[0][-1]=='3'and not (len(car3[int(ID[1])])==0) and car3[int(ID[1])][-1][17] is not None:
                #         a_backward = (v_backward  - car3[int(ID[1])][-1][17])/step_length

                #     vehicle_state.append(a_backward)
                #     if(x_backward is not None and v_backward is not None and a_backward is not None):
                #         if(a_backward ==0):
                #             a_backward = 0.000001
                #         if(math.pow(v_backward,2)+2*a_backward*x_backward>=0):
                #             ETTC1 = (-v_backward +math.sqrt(math.pow(v_backward,2)+2*a_backward*x_backward))/a_backward
                #             ETTC2 = (-v_backward -math.sqrt(math.pow(v_backward,2)+2*a_backward*x_backward))/a_backward
                #             if(ETTC1>=0 and ETTC2>=0):
                #                 if(ETTC1>=ETTC2):
                #                     vehicle_state.append(ETTC2)
                #                 else:
                #                     vehicle_state.append(ETTC1)
                #             elif(ETTC1>=0 and ETTC2<0):
                #                 vehicle_state.append(ETTC1)
                #             elif(ETTC1<0 and ETTC2>=0):
                #                 vehicle_state.append(ETTC2)
                #             else:
                #                 vehicle_state.append(None)
                #         else:
                #             vehicle_state.append(None)
                #     else:
                #         vehicle_state.append(None)
                # else:
                #     # print("v_bac : ",None)
                #     vehicle_state.append(None)
                #     vehicle_state.append(None)
                #     vehicle_state.append(None)
                #     vehicle_state.append(None)
                #     vehicle_state.append(None)
               

                # # print("left_follower :" ,traci.vehicle.getLeftFollowers(veh_id))
                # # print("right_follower :" ,traci.vehicle.getRightFollowers(veh_id))
                # # print("left_leader :" ,traci.vehicle.getLeftLeaders(veh_id))
                # # print(len(traci.vehicle.getLeftLeaders(veh_id)))
                # # print("right_leader :" ,traci.vehicle.getRightLeaders(veh_id))
                # time_gap_LC_1 =1
                # time_gap_LC_2 =0.5 
                # c_LC =12 # min clearance for lane change
                # if traci.vehicle.getLeftLeaders(veh_id) is not None and len(traci.vehicle.getLeftLeaders(veh_id)) !=0:
                #     Left_leader_id = traci.vehicle.getLeftLeaders(veh_id)
                #     Left_leader_rel_x= traci.vehicle.getPosition(Left_leader_id[0][0])[0] - traci.vehicle.getPosition(veh_id)[0] # ego-vehicle을 기준으로 옆차량의 속도를 비교(상대차량이 더 빠르면 양수값)
                #     Left_leader_rel_v= traci.vehicle.getSpeed(Left_leader_id[0][0]) - traci.vehicle.getSpeed(veh_id) # ego-vehicle을 기준으로 옆차량의 속도를 비교(상대차량이 더 빠르면 양수값)
                    
                    
                #     front_relative_velocity_term = max( [traci.vehicle.getSpeed(veh_id) - traci.vehicle.getSpeed(Left_leader_id[0][0]) ,0] )*time_gap_LC_1 #왼쪽 ego-vehicle이 왼쪽 앞차량 보다 빠를 때 우선적 고려
                #     front_minimum_clearance_trem = max([traci.vehicle.getSpeed(veh_id)*time_gap_LC_2,c_LC])
                #     Left_leader_SD = front_relative_velocity_term +front_minimum_clearance_trem
                    
                #     vehicle_state.append(Left_leader_id[0][0])
                #     vehicle_state.append(Left_leader_rel_x)
                #     vehicle_state.append(Left_leader_rel_v)
                #     vehicle_state.append(Left_leader_SD)
                # else:
                #     vehicle_state.append(None)
                #     vehicle_state.append(None)
                #     vehicle_state.append(None)
                #     vehicle_state.append(None)

                
                
                # if traci.vehicle.getRightLeaders(veh_id) is not None and len(traci.vehicle.getRightLeaders(veh_id)) !=0:
                #     Right_leader_id = traci.vehicle.getRightLeaders(veh_id)
                #     Right_leader_rel_x = traci.vehicle.getPosition(Right_leader_id[0][0])[0] - traci.vehicle.getPosition(veh_id)[0] # ego-vehicle을 기준으로 옆차량의 속도를 비교(상대차량이 더 빠르면 양수값)
                #     Right_leader_rel_v = traci.vehicle.getSpeed(Right_leader_id[0][0]) - traci.vehicle.getSpeed(veh_id) # ego-vehicle을 기준으로 옆차량의 속도를 비교(상대차량이 더 빠르면 양수값)

                #     front_relative_velocity_term = max( [traci.vehicle.getSpeed(veh_id) - traci.vehicle.getSpeed(Right_leader_id[0][0]) ,0] )*time_gap_LC_1 #왼쪽 ego-vehicle이 왼쪽 앞차량 보다 빠를 때 우선적 고려
                #     front_minimum_clearance_trem = max([traci.vehicle.getSpeed(veh_id)*time_gap_LC_2,c_LC])
                #     Right_leader_SD = front_relative_velocity_term +front_minimum_clearance_trem

                #     vehicle_state.append(Right_leader_id[0][0])
                #     vehicle_state.append(Right_leader_rel_x)
                #     vehicle_state.append(Right_leader_rel_v)
                #     vehicle_state.append(Right_leader_SD)
                    
                # else:
                #     vehicle_state.append(None) 
                #     vehicle_state.append(None)
                #     vehicle_state.append(None)
                #     vehicle_state.append(None)
                


                # LEFT_FOLLOWER =[]
                # RIGHT_FOLLOWER=[]
                # traci.vehicle.subscribeContext(str(veh_id), tc.CMD_GET_VEHICLE_VARIABLE, 30.0, [tc.VAR_POSITION])
                # for v in traci.vehicle.getContextSubscriptionResults(str(veh_id)):
                #     left_follower_id = find_followers(veh_id, v,LEFT_FOLLOWER,RIGHT_FOLLOWER)[0]
                #     right_follower_id = find_followers(veh_id, v,LEFT_FOLLOWER,RIGHT_FOLLOWER)[1]
                # if left_follower_id is not None:
                #     Left_follower_id = left_follower_id
                #     Left_follower_rel_x = traci.vehicle.getPosition(left_follower_id)[0] - traci.vehicle.getPosition(veh_id)[0] # ego-vehicle을 기준으로 옆차량의 속도를 비교(상대차량이 더 빠르면 양수값)
                #     Left_follower_rel_v = traci.vehicle.getSpeed(left_follower_id) - traci.vehicle.getSpeed(veh_id) # ego-vehicle을 기준으로 옆차량의 속도를 비교(상대차량이 더 빠르면 양수값)
                    

                #     back_relative_velocity_term = max( [traci.vehicle.getSpeed(left_follower_id)-traci.vehicle.getSpeed(veh_id),0] )*time_gap_LC_1 #왼쪽 뒷차량이 ego-vehicle 보다 빠를 때 우선적 고려
                #     back__minimum_clearance_trem = max([traci.vehicle.getSpeed(left_follower_id)*time_gap_LC_2,c_LC]) 
                #     Left_follower_SD = back_relative_velocity_term + back__minimum_clearance_trem
                    
                #     vehicle_state.append(Left_follower_id)
                #     vehicle_state.append(Left_follower_rel_x)
                #     vehicle_state.append(Left_follower_rel_v)
                #     vehicle_state.append(Left_follower_SD)
                # else:
                #     vehicle_state.append(None)
                #     vehicle_state.append(None)
                #     vehicle_state.append(None)
                #     vehicle_state.append(None)
                # if right_follower_id is not None:
                #     Right_follower_id = right_follower_id
                #     Right_follower_rel_x = traci.vehicle.getPosition(right_follower_id)[0] - traci.vehicle.getPosition(veh_id)[0] # ego-vehicle을 기준으로 옆차량의 속도를 비교(상대차량이 더 빠르면 양수값)
                #     Right_follower_rel_v = traci.vehicle.getSpeed(right_follower_id) - traci.vehicle.getSpeed(veh_id) # ego-vehicle을 기준으로 옆차량의 속도를 비교(상대차량이 더 빠르면 양수값)

                #     back_relative_velocity_term = max( [traci.vehicle.getSpeed(right_follower_id)-traci.vehicle.getSpeed(veh_id),0] )*time_gap_LC_1 #왼쪽 뒷차량이 ego-vehicle 보다 빠를 때 우선적 고려
                #     back__minimum_clearance_trem = max([traci.vehicle.getSpeed(right_follower_id)*time_gap_LC_2,c_LC]) 
                #     Right_follower_SD = back_relative_velocity_term + back__minimum_clearance_trem
                    
                #     vehicle_state.append(Right_follower_id)
                #     vehicle_state.append(Right_follower_rel_x)
                #     vehicle_state.append(Right_follower_rel_v)
                #     vehicle_state.append(Right_follower_SD)
                # else:
                #     vehicle_state.append(None)
                #     vehicle_state.append(None)
                #     vehicle_state.append(None)
                #     vehicle_state.append(None)
                # traci.vehicle.unsubscribeContext(str(veh_id), tc.CMD_GET_VEHICLE_VARIABLE, 30.0)

                # if(veh_id[3] == '0'):
                #     if(car0_mode[int(veh_id[5:])]==1):
                #         vehicle_state.append(car0_anti_RVC_mode[int(veh_id[5:])])
                #     else:
                #         vehicle_state.append(0)
                # elif(veh_id[3] == '1'):
                #     if(car1_mode[int(veh_id[5:])]==1):
                #         vehicle_state.append(car1_anti_RVC_mode[int(veh_id[5:])])
                #     else:   
                #         vehicle_state.append(0)
                # elif(veh_id[3] == '2'):
                #     if(car2_mode[int(veh_id[5:])]==1):
                #         vehicle_state.append(car2_anti_RVC_mode[int(veh_id[5:])])
                #     else:
                #         vehicle_state.append(0)
                # elif(veh_id[3] == '3'):
                #     if(car3_mode[int(veh_id[5:])]==1):
                #         vehicle_state.append(car3_anti_RVC_mode[int(veh_id[5:])])
                #     else:
                #         vehicle_state.append(0)
                if(veh_id == 'ego'):
                    ego.append(vehicle_state)
                if(veh_id =='car.leftrear0'):
                    leftrear0.append(vehicle_state)
                if(veh_id=='car.leftrear1'):
                    leftrear1.append(vehicle_state)
                if(veh_id=='accel.rear'):
                    rear.append(vehicle_state)
                if(veh_id=='car.rightrear0'):
                    rightrear0.append(vehicle_state)
                if(veh_id=='car.rightrear1'):
                    rightrear1.append(vehicle_state)
                if(veh_id=='car.leftforward0'):
                    leftforward0.append(vehicle_state)
                if(veh_id=='car.leftforward1'):
                    leftforward1.append(vehicle_state)
                if(veh_id=='car.leftforward2'):
                    leftforward2.append(vehicle_state)
                if(veh_id=='car.forward'):
                    forward.append(vehicle_state)
                if(veh_id=='car.rightforward0'):
                    rightforward0.append(vehicle_state)
                if(veh_id=='car.rightforward1'):
                    rightforward1.append(vehicle_state)
                if(veh_id=='car.rightforward2'):
                    rightforward2.append(vehicle_state)
            ### ego_vehicle LC -> episode ends.
            
            # print('ego_vehicle_LC_success : ',ego_vehicle_LC_success())
            if veh_id == 'ego':
                # print("success? : ",ego_LC_success)
                if ego_vehicle_LC_success() and traci.simulation.getTime()>=1:
                    print('Lane change success')
                    ego_LC_success =True
                    # print('angle : ',traci.vehicle.getAngle('ego'))
                if ego_LC_success:
                    if traci.vehicle.getAngle('ego') >= 89.9 and traci.vehicle.getAngle('ego') <= 90.1:
                    # if traci.vehicle.getAngle('ego') == 90:
                        print('close')
                        ego_LC_success=False
                        end_flag = True
                        traci.close()
                        sys.stdout.flush()
                        break

                if(veh_id =='ego'):
                    lane_buffer_ego = traci.vehicle.getLaneIndex('ego') # ego_car lane buffer    
                

            if traci.vehicle.getRoadID(veh_id) == 'E5' and veh_id[0] == 'a':
                traci.vehicle.setAccel(veh_id, '2.5')
                traci.vehicle.setDecel(veh_id, '0.00001')
                # traci.vehicle.setSpeedMode(veh_id,'0')
                setvehiclestate('accel.rear',random_speed[4],random_min_a[4],random_max_a_x[4],random_jerk[4])
            
            if (traci.vehicle.getRoadID(veh_id) == 'E5' and veh_id[0] == 't'):
                ID = veh_id.split('.')
                if traci.vehicle.getLeader(veh_id) is not None:
                    Leader_id,x_forward = traci.vehicle.getLeader(veh_id)
                    if(int(ID[0][-1])==0):
                        if(x_forward>=random_clearance[int(veh_id[7:])+10]):
                            # traci.vehicle.setSpeedMode(veh_id,'0')
                            # traci.vehicle.setSpeedMode(veh_id,0b011110)  # Regard safe speed -> 0ff
                            setvehiclestate(veh_id,random_speed[int(veh_id[7:])+10],random_min_a_x[int(veh_id[7:])+10],random_max_a_x[int(veh_id[7:])+10],random_jerk[int(veh_id[7:])+10])
                        elif(x_forward<random_clearance[int(veh_id[7:])+10]): #급 제동 (랜덤한 가속도로 제동)
                            # random_min_a_x[int(veh_id[7:])+10] /x_forward
                            setvehiclestate(veh_id,traci.vehicle.getSpeed(Leader_id)-speed_gap_between_leader[int(veh_id[7:])+10],random_min_a_x[int(veh_id[7:])+10],random_max_a_x[int(veh_id[7:])+10],random_jerk[int(veh_id[7:])+10]) #jerk value must be positive value.              
                    if(int(ID[0][-1])==1):
                        if(x_forward>=random_clearance[int(veh_id[7:])+15]):
                            # traci.vehicle.setSpeedMode(veh_id,'0')
                            # traci.vehicle.setSpeedMode(veh_id,0b011110)  # Regard safe speed -> 0ff
                            setvehiclestate(veh_id,random_speed[int(veh_id[7:])+15],random_min_a_x[int(veh_id[7:])+15],random_max_a_x[int(veh_id[7:])+15],random_jerk[int(veh_id[7:])+15])
                        elif(x_forward<random_clearance[int(veh_id[7:])+15]): #급 제동 (랜덤한 가속도로 제동)
                            setvehiclestate(veh_id,traci.vehicle.getSpeed(Leader_id)-speed_gap_between_leader[int(veh_id[7:])+15],random_min_a_x[int(veh_id[7:])+15],random_max_a_x[int(veh_id[7:])+15],random_jerk[int(veh_id[7:])+15]) #jerk value must be positive value.
                
                
                
            control_input = []
            if (traci.vehicle.getRoadID(veh_id) == 'E5' and veh_id[0] == 'c'):
                c0 = 1.98
                # tau = 1.36
                
                if veh_id == 'car.leftrear0':
                    tau = vehicles_tau[0]                    
                elif veh_id =='car.leftrear1':
                    tau = vehicles_tau[1]                    
                elif veh_id =='car.rightrear0':
                    tau = vehicles_tau[2]                    
                elif veh_id =='car.rightrear1':
                    tau = vehicles_tau[3]                    
                elif veh_id =='car.leftforward0':
                    tau = vehicles_tau[4]               
                elif veh_id =='car.leftforward1':
                    tau = vehicles_tau[5]                    
                elif veh_id =='car.leftforward2':
                    tau = vehicles_tau[6]
                elif veh_id =='car.forward':
                    tau = vehicles_tau[7]                    
                elif veh_id =='car.rightforward0':
                    tau = vehicles_tau[8]                    
                elif veh_id =='car.rightforward1':
                    tau = vehicles_tau[9]                    
                elif veh_id =='car.rightforward2':
                    tau = vehicles_tau[10]
                else:
                    tau = 1.36
                
                
                v_controled = traci.vehicle.getSpeed(veh_id)

                ### control input graph ###
                control_input.append(veh_id)
                control_input.append(traci.simulation.getTime())
                ###########################
                if traci.vehicle.getLeader(veh_id) is not None: # 선행 차량이 있을 때 ACC
                    Leader_id,c_front = traci.vehicle.getLeader(veh_id)
                    v_preceding= traci.vehicle.getSpeed(Leader_id)
                    c_desire = c0+tau*v_preceding
                    
                    ##### ACC with speed limit ########
                    speed_limit = 23
                    
                    # a = set_a(veh_id,a_desire_with_speed_limit(v_controled,v_preceding,c_desire,c_front,speed_limit))
                    traci.vehicle.setAcceleration(veh_id,a_desire_with_speed_limit(v_controled,v_preceding,c_desire,c_front,speed_limit),20) #입력 가속도
                    a = a_desire_with_speed_limit(v_controled,v_preceding,c_desire,c_front,speed_limit) #입력 가속도


                    # set_a(veh_id,a_desire(v_controled,v_preceding,c_desire,c_front))
                    
                    ### control input graph ###
                    control_input.append(a) #입력 가속도
                    control_input.append(a_desire_with_speed_limit(v_controled,v_preceding,c_desire,c_front,speed_limit)) #목표 가속도
                else: # 선두 차량 CC
                    
                    u = set_v(veh_id,22.22)
                    ### control input graph ###
                    control_input.append(u[0]) # 입력 가속도 
                    control_input.append(u[1]) # 목표 가속도
                input.append(control_input)             
                    
        if(end_flag):
            break
    if not end_flag:
        traci.close()
        sys.stdout.flush()



netconvertBinary = checkBinary('netconvert')
sumoBinary = checkBinary('sumo-gui')
# build/check network
retcode = subprocess.call(
    [netconvertBinary, "-c", "highway_episodic.netccfg"], stdout=sys.stdout, stderr=sys.stderr)
try:
    shutil.copy("highway_episodic.net.xml", "net.net.xml")
except IOError:
    print("Missing 'highway_episodic.net.xml'")



if __name__ == "__main__":
    
    
    options = get_options()
    #FILE = os.path
    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # generate_routefile()
    
    episodenum= int(options.episodenum)
    for episode in range(episodenum):
        
        # first, generate the route file for this simulation
          
        print('episode : ',episode)  
        # this is the normal way of using traci. sumo is started as a
        # subprocess and then the python script connects and runs
        # traci.start([sumoBinary, "-c", "hello.sumocfg"])
        

        traci.start([sumolib.checkBinary("sumo-gui"),"-c", "/home/mds/Desktop/highway_episodic/highway_episodic.sumocfg",
                '--start','true',
                '--route-files','/home/mds/Desktop/highway_episodic/route_file/highway_episodic_'+str(episode)+'.rou.xml',
                '--gui-settings-file','/home/mds/Desktop/highway_episodic/viewsettings.xml',
                '--lanechange.duration', '2',
                #  '--collision.action', 'warn',
                #  '--collision.stoptime','5',
                '--collision.action', 'remove',
                '--collision.mingap-factor', '0',
                #  '--time-to-teleport','10',
                '--collision-output','colliderSpeed',
                '--step-length', str(step_length),
                '--no-step-log',
                '--quit-on-end','true']) 
        
        run()
        save_csv()
        print("avoiding_LC count : ",avoiding_LC)
        print('finish truck count : ',finish_truck)
