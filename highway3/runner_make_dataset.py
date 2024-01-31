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
class State(Enum):
    Cruise = 0
    Rear_end_collision_avoidance=1

svc_RBF = joblib.load('svc_RBF.pkl')  #충돌 경계 값 svc

accel0 =[]
truck0 =[]
accel1 =[]
truck1 =[]
car0 =[]
car1 =[]
car2 =[]
car3 =[]
collision =[]

truck0_buffer=[]
truck1_buffer=[]
accel_truck0_buffer=[]
accel_truck1_buffer=[]
car0_buffer=[]
car1_buffer=[]
car2_buffer=[]   
car3_buffer=[]


crossroads_end = 1000
step_length = 0.01
for i in range(5):
    accel0.append([])
    truck0.append([])
    accel_truck0_buffer.append([])
    truck0_buffer.append([])
for i in range(5):
    accel1.append([])
    truck1.append([])
    accel_truck1_buffer.append([])
    truck1_buffer.append([])
for i in range(20):
    car0.append([])
    car0_buffer.append([])
    car1.append([])
    car1_buffer.append([])
    car2.append([])
    car2_buffer.append([])
    car3.append([])
    car3_buffer.append([])

def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options

def generate_routefile():
    #collisionAvoidanceOverride="50"
    ##
    with open("highway3.rou.xml", "w") as routes:
        print("""<routes>
        <vType accel="3" decel="4.5" departSpeed = "22" id="car" length="5" maxSpeed="22.22" minGap="2.0" sigma="0.5" lcSpeedGain ="0"/>
        <vType accel="2.5" decel="4.5" departSpeed = "33" emergencyDecel="9.0" id="truck" length="12" maxSpeed="33.3333" minGap="1.0" sigma="0.5" lcSpeedGain ="0"/>
        <vType accel="2.5" decel="4.5"   departSpeed = "33" id="accel_truck" length="12" maxSpeed="33.3333" minGap="0.0" sigma="1" collisionMinGapFactor="0" tau="0.0001" lcSpeedGain ="0"/>
 
        
        
        <flow id="car0" departPos="free" begin="0.00" from="E4" to="E6" end="50.00" number="20" departSpeed="max" type="car"/>
        <flow id="car1" begin="1.00" from="E4" to="E8" end="50.00" number="20" departSpeed="max" type="car"/>
        <flow id="car2" begin="3.00" from="E7" to="E6" end="50.00" number="20" departSpeed="max" type="car" color="0,1,0"/>
        <flow id="car3" begin="4.50" from="E7" to="E8" end="50.00" number="20" departSpeed="max" type="car" color="0,1,0"/>
        <flow id="truck1" departPos="free" begin="5.50" from="E7" to="E8" end="50.00" number="5" departSpeed="max" type="truck" color="0,0,1"/>
        <flow id="truck0" departPos="free" begin="12.50" from="E4" to="E6" end="50.00" number="5" departSpeed="max" type="truck" color="0,0,1"/>        
        
        <flow id="accel0"  departPos="free" begin="19.00" from="E4" to="E6" end="50.00" number="5" type="accel_truck" color="1,0,0"/>
        <flow id="accel1"  departPos="free" begin="23.50" from="E7" to="E8" end="50.00" number="5" type="accel_truck" color="1,0,0"/>""", file=routes)
        print("</routes>", file=routes)

def save_csv():
    col = ['time','vehicle_id','x','y','v_x','v_y','a_x','a_y','heading_angle','x_for_id','x_for','rel_v_for','rel_a_for','front_SD_for_LKtoPreLC','front_SD_for_PreLCtoLC','x_bac_id','x_bac','rel_v_bac','rel_a_bac','ETTC_bac','left_leader','left_leader_rel_x','left_leader_rel_v','left_leader_SD','left_follower','left_follower_rel_x','left_follower_rel_v','left_follower_SD','right_leader','right_leader_rel_x','right_leader_rel_v','right_leader_SD','right_follower','right_follower_rel_x','right_follower_rel_v','right_follower_SD']
    df_accel= pd.DataFrame(accel0[0],columns=col)
    for i in range(4):
        df = pd.DataFrame(accel0[i+1],columns=col)
        df_accel=pd.concat([df_accel,df],ignore_index = True)
    for i in range(5):
        df2 = pd.DataFrame(accel1[i],columns=col)
        df_accel=pd.concat([df_accel,df2],ignore_index = True)
    df_truck= pd.DataFrame(truck0[0],columns=col)
    for i in range(4):
        df3 = pd.DataFrame(truck0[i+1],columns=col)
        df_truck=pd.concat([df_truck,df3],ignore_index = True)
    for i in range(5):
        df4 = pd.DataFrame(truck1[i],columns=col)
        df_truck=pd.concat([df_truck,df4],ignore_index = True)
    df_car= pd.DataFrame(car0[0],columns=col)  
    for i in range(19):
        df5 = pd.DataFrame(car0[i+1],columns=col)
        df_car=pd.concat([df_car,df5],ignore_index = True)
    for i in range(20):
        df6 =pd.DataFrame(car1[i],columns=col)
        df_car=pd.concat([df_car,df6],ignore_index = True)
    for i in range(20):
        df7 = pd.DataFrame(car2[i],columns=col)
        df_car=pd.concat([df_car,df7],ignore_index = True)
    for i in range(20):
        df8 = pd.DataFrame(car3[i],columns=col)
        df_car=pd.concat([df_car,df8],ignore_index = True)
    col_c = ['victim','collider']
    df_collision= pd.DataFrame(collision,columns=col_c)

    
    # df1=pd.DataFrame({'time':accel0[0][0][0],'vehicle_id':accel0[0][0][1],'x':accel0[0][0][2],'y':accel0[0][0][3],'v_x':accel0[0][0][4],'v_y':accel0[0][0][5],'a_x':accel0[0][0][6],'a_y':accel0[0][0][7],'theta_h':accel0[0][0][8], 'x_for_id':accel0[0][0][9],'x_for':accel0[0][0][10],'v_for':accel0[0][0][11],'x_bac_id':accel0[0][0][12],'x_bac':accel0[0][0][13],'v_bac':accel0[0][0][14]})   
    # for i in range(len(accel0[0])-1):
    #     pd.concat([df1,accel0[0][i]],ignore_index = True)               
    os.chdir('C:/Users/jewoo/OneDrive/바탕 화면/SUMO/highway3/log_data/3d_svm_dataset')
    df_accel.to_csv('log_data_accel.csv')
    df_truck.to_csv('log_data_truck.csv')
    df_car.to_csv('log_data_car.csv')
    df_collision.to_csv('log_data_collision.csv')

def setvehiclestate(id, goal_v_x,min_a_x, max_a_x, goal_j_x):
    dt = step_length
    if id[0] == 'a':
        if id[5] == '0':
            last_v_x = accel_truck0_buffer[int(id[-1])][0][4]
            last_a_x = accel_truck0_buffer[int(id[-1])][0][6]
        elif id[5] == '1':
            last_v_x = accel_truck1_buffer[int(id[-1])][0][4]
            last_a_x = accel_truck1_buffer[int(id[-1])][0][6]
    elif id[0] == 't':
        if id[5] == '0':
            last_v_x = truck0_buffer[int(id[-1])][0][4]
            last_a_x = truck0_buffer[int(id[-1])][0][6]
        elif id[5] == '1':
            last_v_x = truck1_buffer[int(id[-1])][0][4]
            last_a_x = truck1_buffer[int(id[-1])][0][6]
    elif id[0] == 'c':
        if id[3] == '0':
            last_v_x = car0_buffer[int(id[-1])][0][4]
            last_a_x = car0_buffer[int(id[-1])][0][6]
        elif id[3] == '1':
            last_v_x = car1_buffer[int(id[-1])][0][4]
            last_a_x = car1_buffer[int(id[-1])][0][6]
        elif id[3] == '2':
            last_v_x = car2_buffer[int(id[-1])][0][4]
            last_a_x = car2_buffer[int(id[-1])][0][6]
        elif id[3] == '3':
            last_v_x = car3_buffer[int(id[-1])][0][4]
            last_a_x = car3_buffer[int(id[-1])][0][6]
    
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
        next_accel = -2*goal_j_x*dt + last_a_x
        if next_accel >= min_a_x:
            next_velocity = next_accel*dt + last_v_x
        else:
            next_velocity = min_a_x*dt+last_v_x
    traci.vehicle.setSpeed(id,next_velocity)
    if(next_accel>=0 and next_accel<=2):
        traci.vehicle.setAccel(id,next_accel)
    elif(next_accel<0 and next_accel>= -5):
        traci.vehicle.setDecel(id,-next_accel)


def run():  
    """execute the TraCI control loop"""
    # step = 0
    
    
    # answer =svc_RBF.predict([[1,15]]) 
    # print(answer[0])
    random_speed=[]
    random_speed2=[]
    random_clearance=[]
    random_jerk=[]
    random_max_a_x=[]
    random_min_a_x=[]
    random_min_a=[]
    speed_gap_between_leader=[]
    r_seed = random.randint(20,80)
    for i in range(5):
        random_speed.append(float(random.randint(2777, 3330))/100)
        random_speed2.append(float(random.randint(2000, 2222))/100)
        random_clearance.append(float(random.randint(400, 1200))/100)
        random_jerk.append(float(random.randint(1000, 3000))/100)
        random_max_a_x.append(float(random.randint(150, 350))/100)
        random_min_a_x.append(float(random.randint(-550, -350))/100)
        random_min_a.append(float(random.randint(-100, 0))/100)
        speed_gap_between_leader.append(float(random.randint(100, 400))/100)
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        
        vehs = traci.vehicle.getIDList()
        if len(traci.simulation.getCollidingVehiclesIDList()) !=0:
            collision.append(traci.simulation.getCollidingVehiclesIDList())
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
                acceleartion_x =0
                if ID[0][0] == 'a' and ID[0][-1]=='0' and not (len(accel0[int(ID[1])])==0):
                    acceleartion_x = (traci.vehicle.getSpeed(veh_id) - accel0[int(ID[1])][-1][4])/step_length
                if ID[0][0] == 'a' and ID[0][-1]=='1'and not (len(accel1[int(ID[1])])==0):
                    acceleartion_x = (traci.vehicle.getSpeed(veh_id) - accel1[int(ID[1])][-1][4])/step_length
                if ID[0][0] == 't' and ID[0][-1]=='0'and not (len(truck0[int(ID[1])])==0):
                    acceleartion_x = (traci.vehicle.getSpeed(veh_id) - truck0[int(ID[1])][-1][4])/step_length
                if ID[0][0] == 't' and ID[0][-1]=='1'and not (len(truck1[int(ID[1])])==0):
                    acceleartion_x = (traci.vehicle.getSpeed(veh_id) - truck1[int(ID[1])][-1][4])/step_length
                if ID[0][0] == 'c' and ID[0][-1]=='0'and not (len(car0[int(ID[1])])==0):
                    acceleartion_x = (traci.vehicle.getSpeed(veh_id) - car0[int(ID[1])][-1][4])/step_length
                if ID[0][0] == 'c' and ID[0][-1]=='1'and not (len(car1[int(ID[1])])==0):
                    acceleartion_x = (traci.vehicle.getSpeed(veh_id) - car1[int(ID[1])][-1][4])/step_length
                if ID[0][0] == 'c' and ID[0][-1]=='2'and not (len(car2[int(ID[1])])==0):
                    acceleartion_x = (traci.vehicle.getSpeed(veh_id) - car2[int(ID[1])][-1][4])/step_length
                if ID[0][0] == 'c' and ID[0][-1]=='3'and not (len(car3[int(ID[1])])==0):
                    acceleartion_x = (traci.vehicle.getSpeed(veh_id) - car3[int(ID[1])][-1][4])/step_length


                vehicle_state.append(acceleartion_x)

                acceleartion_y=0
                if ID[0][0] == 'a' and ID[0][-1]=='0'and not (len(accel0[int(ID[1])])==0):
                    acceleartion_y = (traci.vehicle.getLateralSpeed(veh_id) - accel0[int(ID[1])][-1][5])/step_length
                if ID[0][0] == 'a' and ID[0][-1]=='1'and not (len(accel1[int(ID[1])])==0):
                    acceleartion_y = (traci.vehicle.getLateralSpeed(veh_id) - accel1[int(ID[1])][-1][5])/step_length
                if ID[0][0] == 't' and ID[0][-1]=='0'and not (len(truck0[int(ID[1])])==0):
                    acceleartion_y = (traci.vehicle.getLateralSpeed(veh_id) - truck0[int(ID[1])][-1][5])/step_length
                if ID[0][0] == 't' and ID[0][-1]=='1'and not (len(truck1[int(ID[1])])==0):
                    acceleartion_y = (traci.vehicle.getLateralSpeed(veh_id) - truck1[int(ID[1])][-1][5])/step_length
                if ID[0][0] == 'c' and ID[0][-1]=='0'and not (len(car0[int(ID[1])])==0):
                    acceleartion_y = (traci.vehicle.getLateralSpeed(veh_id) - car0[int(ID[1])][-1][5])/step_length
                if ID[0][0] == 'c' and ID[0][-1]=='1'and not (len(car1[int(ID[1])])==0):
                    acceleartion_y = (traci.vehicle.getLateralSpeed(veh_id) - car1[int(ID[1])][-1][5])/step_length
                if ID[0][0] == 'c' and ID[0][-1]=='2'and not (len(car2[int(ID[1])])==0):
                    acceleartion_y = (traci.vehicle.getLateralSpeed(veh_id) - car2[int(ID[1])][-1][5])/step_length
                if ID[0][0] == 'c' and ID[0][-1]=='3'and not (len(car3[int(ID[1])])==0):
                    acceleartion_y = (traci.vehicle.getLateralSpeed(veh_id) - car3[int(ID[1])][-1][5])/step_length
                vehicle_state.append(acceleartion_y)
                # print("a_x : ",traci.vehicle.getAccel(veh_id))
                
                #print("getAcceleration : ",traci.vehicle.getAcceleration(veh_id))
                vehicle_state.append(traci.vehicle.getAngle(veh_id))
                # print("theta_h : ", traci.vehicle.getAngle(veh_id))
                
                # print("Leader_id, x_for  : ",traci.vehicle.getLeader(veh_id))

                c_LK = 4 #min clearance for lane keeping
                time_gap_LK = 1.36  #95% percentile time_gap (most safe) // 1.36s -> 중간값   0.74s -> 5% persentile 가장 위험.
                time_gap_LC_prepare = 0.74 
                
                if traci.vehicle.getLeader(veh_id) is not None:
                    
                    Leader_id,x_forward = traci.vehicle.getLeader(veh_id)
                    vehicle_state.append(Leader_id)
                    vehicle_state.append(x_forward)
                    v_forward = traci.vehicle.getSpeed(traci.vehicle.getLeader(veh_id)[0]) -traci.vehicle.getSpeed(veh_id) #(relative speed(v_for) = front_vehicle speed - ego_vehicle)
                    # print("v_for : ",v_for)
                    vehicle_state.append(v_forward)
                    
                    a_forward = None
                    ID = veh_id.split('.')
                    if ID[0][0] == 'a' and ID[0][-1]=='0' and not (len(accel0[int(ID[1])])==0) and accel0[int(ID[1])][-1][11] is not None:
                        a_forward = (v_forward - accel0[int(ID[1])][-1][11])/step_length
                    if ID[0][0] == 'a' and ID[0][-1]=='1'and not (len(accel1[int(ID[1])])==0) and accel1[int(ID[1])][-1][11] is not None:
                        a_forward = (v_forward  - accel1[int(ID[1])][-1][11])/step_length
                    if ID[0][0] == 't' and ID[0][-1]=='0'and not (len(truck0[int(ID[1])])==0) and truck0[int(ID[1])][-1][11] is not None:
                        a_forward = (v_forward  - truck0[int(ID[1])][-1][11])/step_length
                    if ID[0][0] == 't' and ID[0][-1]=='1'and not (len(truck1[int(ID[1])])==0) and truck1[int(ID[1])][-1][11] is not None:
                        a_forward = (v_forward  - truck1[int(ID[1])][-1][11])/step_length
                    if ID[0][0] == 'c' and ID[0][-1]=='0'and not (len(car0[int(ID[1])])==0) and car0[int(ID[1])][-1][11] is not None:
                        a_forward = (v_forward  - car0[int(ID[1])][-1][11])/step_length
                    if ID[0][0] == 'c' and ID[0][-1]=='1'and not (len(car1[int(ID[1])])==0) and car1[int(ID[1])][-1][11] is not None:
                        a_forward = (v_forward  - car1[int(ID[1])][-1][11])/step_length
                    if ID[0][0] == 'c' and ID[0][-1]=='2'and not (len(car2[int(ID[1])])==0) and car2[int(ID[1])][-1][11] is not None:
                        a_forward = (v_forward  - car2[int(ID[1])][-1][11])/step_length
                    if ID[0][0] == 'c' and ID[0][-1]=='3'and not (len(car3[int(ID[1])])==0) and car3[int(ID[1])][-1][11] is not None:
                        a_forward = (v_forward  - car3[int(ID[1])][-1][11])/step_length

                    vehicle_state.append(a_forward)
                    v_p = traci.vehicle.getSpeed(Leader_id)
                    des_clearance = c_LK + time_gap_LK*v_p # 앞차와의 상대 속도에 따른 ACC 안전거리 (safe)
                    des_clearance2 = c_LK + time_gap_LC_prepare*v_p # 앞차와의 상대 속도에 따른 ACC 안전거리 (aggressive)
                    vehicle_state.append(des_clearance)
                    vehicle_state.append(des_clearance2)
                else:
                    vehicle_state.append(None)
                    vehicle_state.append(None)
                    # print("v_for : ",None)
                    vehicle_state.append(None)
                    vehicle_state.append(None)
                    vehicle_state.append(None)
                    vehicle_state.append(None)
                    
                # print("Follower_id, x_bac : ",traci.vehicle.getFollower(veh_id) )
                Follower_id,x_backward = traci.vehicle.getFollower(veh_id)
                         
                if type(traci.vehicle.getFollower(veh_id)) is not None and traci.vehicle.getFollower(veh_id)[0] != '':
                    v_backward= traci.vehicle.getSpeed(traci.vehicle.getFollower(veh_id)[0]) -traci.vehicle.getSpeed(veh_id) #(relative speed(v_bac) = back_vehicle speed - ego_vehicle)
                    vehicle_state.append(Follower_id)
                    vehicle_state.append(x_backward)
                    # print("v_bac : ",v_bac)
                    vehicle_state.append(v_backward)
                    a_backward = None
                    ID = veh_id.split('.')
                    if ID[0][0] == 'a' and ID[0][-1]=='0' and not (len(accel0[int(ID[1])])==0) and accel0[int(ID[1])][-1][17] is not None:
                        a_backward = (v_backward - accel0[int(ID[1])][-1][17])/step_length
                    if ID[0][0] == 'a' and ID[0][-1]=='1'and not (len(accel1[int(ID[1])])==0) and accel1[int(ID[1])][-1][17] is not None:
                        a_backward = (v_backward  - accel1[int(ID[1])][-1][17])/step_length
                    if ID[0][0] == 't' and ID[0][-1]=='0'and not (len(truck0[int(ID[1])])==0) and truck0[int(ID[1])][-1][17] is not None:
                        a_backward = (v_backward  - truck0[int(ID[1])][-1][17])/step_length
                    if ID[0][0] == 't' and ID[0][-1]=='1'and not (len(truck1[int(ID[1])])==0) and truck1[int(ID[1])][-1][17] is not None:
                        a_backward = (v_backward  - truck1[int(ID[1])][-1][17])/step_length
                    if ID[0][0] == 'c' and ID[0][-1]=='0'and not (len(car0[int(ID[1])])==0) and car0[int(ID[1])][-1][17] is not None:
                        a_backward = (v_backward  - car0[int(ID[1])][-1][17])/step_length
                    if ID[0][0] == 'c' and ID[0][-1]=='1'and not (len(car1[int(ID[1])])==0) and car1[int(ID[1])][-1][17] is not None:
                        a_backward = (v_backward  - car1[int(ID[1])][-1][17])/step_length
                    if ID[0][0] == 'c' and ID[0][-1]=='2'and not (len(car2[int(ID[1])])==0) and car2[int(ID[1])][-1][17] is not None:
                        a_backward = (v_backward  - car2[int(ID[1])][-1][17])/step_length
                    if ID[0][0] == 'c' and ID[0][-1]=='3'and not (len(car3[int(ID[1])])==0) and car3[int(ID[1])][-1][17] is not None:
                        a_backward = (v_backward  - car3[int(ID[1])][-1][17])/step_length

                    vehicle_state.append(a_backward)
                    if(x_backward is not None and v_backward is not None and a_backward is not None):
                        if(a_backward ==0):
                            a_backward = 0.000001
                        if(math.pow(v_backward,2)+2*a_backward*x_backward>=0):
                            ETTC1 = (-v_backward +math.sqrt(math.pow(v_backward,2)+2*a_backward*x_backward))/a_backward
                            ETTC2 = (-v_backward -math.sqrt(math.pow(v_backward,2)+2*a_backward*x_backward))/a_backward
                            if(ETTC1>=0 and ETTC2>=0):
                                if(ETTC1>=ETTC2):
                                    vehicle_state.append(ETTC2)
                                else:
                                    vehicle_state.append(ETTC1)
                            elif(ETTC1>=0 and ETTC2<0):
                                vehicle_state.append(ETTC1)
                            elif(ETTC1<0 and ETTC2>=0):
                                vehicle_state.append(ETTC2)
                            else:
                                vehicle_state.append(None)
                        else:
                            vehicle_state.append(None)
                    else:
                        vehicle_state.append(None)
                else:
                    # print("v_bac : ",None)
                    vehicle_state.append(None)
                    vehicle_state.append(None)
                    vehicle_state.append(None)
                    vehicle_state.append(None)
                    vehicle_state.append(None)
               

                # print("left_follower :" ,traci.vehicle.getLeftFollowers(veh_id))
                # print("right_follower :" ,traci.vehicle.getRightFollowers(veh_id))
                # print("left_leader :" ,traci.vehicle.getLeftLeaders(veh_id))
                # print(len(traci.vehicle.getLeftLeaders(veh_id)))
                # print("right_leader :" ,traci.vehicle.getRightLeaders(veh_id))
                time_gap_LC_1 =1
                time_gap_LC_2 =0.5 
                c_LC =12 # min clearance for lane change
                if traci.vehicle.getLeftLeaders(veh_id) is not None and len(traci.vehicle.getLeftLeaders(veh_id)) !=0:
                    Left_leader_id = traci.vehicle.getLeftLeaders(veh_id)
                    Left_leader_rel_x= traci.vehicle.getPosition(Left_leader_id[0][0])[0] - traci.vehicle.getPosition(veh_id)[0] # ego-vehicle을 기준으로 옆차량의 속도를 비교(상대차량이 더 빠르면 양수값)
                    Left_leader_rel_v= traci.vehicle.getSpeed(Left_leader_id[0][0]) - traci.vehicle.getSpeed(veh_id) # ego-vehicle을 기준으로 옆차량의 속도를 비교(상대차량이 더 빠르면 양수값)
                    
                    
                    front_relative_velocity_term = max( [traci.vehicle.getSpeed(veh_id) - traci.vehicle.getSpeed(Left_leader_id[0][0]) ,0] )*time_gap_LC_1 #왼쪽 ego-vehicle이 왼쪽 앞차량 보다 빠를 때 우선적 고려
                    front_minimum_clearance_trem = max([traci.vehicle.getSpeed(veh_id)*time_gap_LC_2,c_LC])
                    Left_leader_SD = front_relative_velocity_term +front_minimum_clearance_trem
                    
                    vehicle_state.append(Left_leader_id[0][0])
                    vehicle_state.append(Left_leader_rel_x)
                    vehicle_state.append(Left_leader_rel_v)
                    vehicle_state.append(Left_leader_SD)
                else:
                    vehicle_state.append(None)
                    vehicle_state.append(None)
                    vehicle_state.append(None)
                    vehicle_state.append(None)

                if traci.vehicle.getLeftFollowers(veh_id) is not None and len(traci.vehicle.getLeftFollowers(veh_id)) !=0:
                    Left_follower_id = traci.vehicle.getLeftFollowers(veh_id)
                    Left_follower_rel_x = traci.vehicle.getPosition(Left_follower_id[0][0])[0] - traci.vehicle.getPosition(veh_id)[0] # ego-vehicle을 기준으로 옆차량의 속도를 비교(상대차량이 더 빠르면 양수값)
                    Left_follower_rel_v = traci.vehicle.getSpeed(Left_follower_id[0][0]) - traci.vehicle.getSpeed(veh_id) # ego-vehicle을 기준으로 옆차량의 속도를 비교(상대차량이 더 빠르면 양수값)
                    

                    back_relative_velocity_term = max( [traci.vehicle.getSpeed(Left_follower_id[0][0])-traci.vehicle.getSpeed(veh_id),0] )*time_gap_LC_1 #왼쪽 뒷차량이 ego-vehicle 보다 빠를 때 우선적 고려
                    back__minimum_clearance_trem = max([traci.vehicle.getSpeed(Left_follower_id[0][0])*time_gap_LC_2,c_LC]) 
                    Left_follower_SD = back_relative_velocity_term + back__minimum_clearance_trem
                    
                    vehicle_state.append(Left_follower_id[0][0])
                    vehicle_state.append(Left_follower_rel_x)
                    vehicle_state.append(Left_follower_rel_v)
                    vehicle_state.append(Left_follower_SD)
                else:
                    vehicle_state.append(None)
                    vehicle_state.append(None)
                    vehicle_state.append(None)
                    vehicle_state.append(None)
                
                if traci.vehicle.getRightLeaders(veh_id) is not None and len(traci.vehicle.getRightLeaders(veh_id)) !=0:
                    Right_leader_id = traci.vehicle.getRightLeaders(veh_id)
                    Right_leader_rel_x = traci.vehicle.getPosition(Right_leader_id[0][0])[0] - traci.vehicle.getPosition(veh_id)[0] # ego-vehicle을 기준으로 옆차량의 속도를 비교(상대차량이 더 빠르면 양수값)
                    Right_leader_rel_v = traci.vehicle.getSpeed(Right_leader_id[0][0]) - traci.vehicle.getSpeed(veh_id) # ego-vehicle을 기준으로 옆차량의 속도를 비교(상대차량이 더 빠르면 양수값)

                    front_relative_velocity_term = max( [traci.vehicle.getSpeed(veh_id) - traci.vehicle.getSpeed(Right_leader_id[0][0]) ,0] )*time_gap_LC_1 #왼쪽 ego-vehicle이 왼쪽 앞차량 보다 빠를 때 우선적 고려
                    front_minimum_clearance_trem = max([traci.vehicle.getSpeed(veh_id)*time_gap_LC_2,c_LC])
                    Right_leader_SD = front_relative_velocity_term +front_minimum_clearance_trem

                    vehicle_state.append(Right_leader_id[0][0])
                    vehicle_state.append(Right_leader_rel_x)
                    vehicle_state.append(Right_leader_rel_v)
                    vehicle_state.append(Right_leader_SD)
                    
                else:
                    vehicle_state.append(None) 
                    vehicle_state.append(None)
                    vehicle_state.append(None)
                    vehicle_state.append(None)
                
                if traci.vehicle.getRightFollowers(veh_id) is not None and len(traci.vehicle.getRightFollowers(veh_id)) !=0:
                    Right_follower_id = traci.vehicle.getRightFollowers(veh_id)
                    Right_follower_rel_x = traci.vehicle.getPosition(Right_follower_id[0][0])[0] - traci.vehicle.getPosition(veh_id)[0] # ego-vehicle을 기준으로 옆차량의 속도를 비교(상대차량이 더 빠르면 양수값)
                    Right_follower_rel_v = traci.vehicle.getSpeed(Right_follower_id[0][0]) - traci.vehicle.getSpeed(veh_id) # ego-vehicle을 기준으로 옆차량의 속도를 비교(상대차량이 더 빠르면 양수값)

                    back_relative_velocity_term = max( [traci.vehicle.getSpeed(Right_follower_id[0][0])-traci.vehicle.getSpeed(veh_id),0] )*time_gap_LC_1 #왼쪽 뒷차량이 ego-vehicle 보다 빠를 때 우선적 고려
                    back__minimum_clearance_trem = max([traci.vehicle.getSpeed(Right_follower_id[0][0])*time_gap_LC_2,c_LC]) 
                    Right_follower_SD = back_relative_velocity_term + back__minimum_clearance_trem
                    
                    vehicle_state.append(Right_follower_id[0][0])
                    vehicle_state.append(Right_follower_rel_x)
                    vehicle_state.append(Right_follower_rel_v)
                    vehicle_state.append(Right_follower_SD)
                else:
                    vehicle_state.append(None)
                    vehicle_state.append(None)
                    vehicle_state.append(None)
                    vehicle_state.append(None)

                
                
                for i in range(5):
                    if(veh_id == 'accel0.'+str(i)):
                        accel0[i].append(vehicle_state)
                        accel_truck0_buffer[i].insert(0,vehicle_state)
                    if(veh_id == 'accel1.'+str(i)):
                        accel1[i].append(vehicle_state)
                        accel_truck1_buffer[i].insert(0,vehicle_state)
                    if(veh_id == 'truck0.'+str(i)):
                        truck0[i].append(vehicle_state)
                        truck0_buffer[i].insert(0,vehicle_state)
                    if(veh_id == 'truck1.'+str(i)):
                        truck1[i].append(vehicle_state)
                        truck1_buffer[i].insert(0,vehicle_state)                   
                for i in range(20):
                    if(veh_id == 'car0.'+str(i)):
                        car0[i].append(vehicle_state)
                        car0_buffer[i].insert(0,vehicle_state)
                    if(veh_id == 'car1.'+str(i)):
                        car1[i].append(vehicle_state)
                        car1_buffer[i].insert(0,vehicle_state)
                    if(veh_id == 'car2.'+str(i)):
                        car2[i].append(vehicle_state)
                        car2_buffer[i].insert(0,vehicle_state)
                    if(veh_id == 'car3.'+str(i)):
                        car3[i].append(vehicle_state)
                        car3_buffer[i].insert(0,vehicle_state)  

            if traci.vehicle.getRoadID(veh_id) == 'E5':
                traci.vehicle.setMinGap(veh_id,'0')
            if traci.vehicle.getRoadID(veh_id) == 'E5' and veh_id[0] == 'a':
                traci.vehicle.setAccel(veh_id, '10')
                traci.vehicle.setDecel(veh_id, '0.00001')
                traci.vehicle.setSpeedMode(veh_id,'0')
                setvehiclestate(veh_id,random_speed[int(veh_id[7:])],random_min_a[int(veh_id[7:])],random_max_a_x[int(veh_id[7:])],random_jerk[int(veh_id[7:])])
            if (traci.vehicle.getRoadID(veh_id) == 'E5' and veh_id[0] == 't') or (traci.vehicle.getRoadID(veh_id) == 'E6' and veh_id[0] == 't'):
                if traci.vehicle.getLeader(veh_id) is not None:
                    Leader_id,x_forward = traci.vehicle.getLeader(veh_id)
                    if(x_forward>=random_clearance[int(veh_id[7:])]):
                        # traci.vehicle.setSpeedMode(veh_id,'0')
                        traci.vehicle.setSpeedMode(veh_id,0b011110)  # Regard safe speed -> 0ff
                        setvehiclestate(veh_id,random_speed[int(veh_id[7:])],random_min_a_x[int(veh_id[7:])],random_max_a_x[int(veh_id[7:])],random_jerk[int(veh_id[7:])])
                    elif(x_forward<random_clearance[int(veh_id[7:])]):
                        setvehiclestate(veh_id,traci.vehicle.getSpeed(Leader_id)-speed_gap_between_leader[int(veh_id[7:])],random_min_a_x[int(veh_id[7:])],random_max_a_x[int(veh_id[7:])],random_jerk[int(veh_id[7:])]) #jerk value must be positive value.              

                for i in range(5): 
                    if(veh_id == 'truck0.'+str(i)):
                        if(len(truck0_buffer[i])>1):
                            truck0_buffer[i].pop()
                    if(veh_id == 'truck1.'+str(i)):
                        if(len(truck1_buffer[i])>1):
                            truck1_buffer[i].pop()
                    if(veh_id == 'accel0.'+str(i)):
                        if(len(accel_truck0_buffer[i])>1):
                            accel_truck0_buffer[i].pop()
                    if(veh_id == 'accel1.'+str(i)):
                        if(len(accel_truck1_buffer[i])>1):
                            accel_truck1_buffer[i].pop()
                for i in range(20): 
                    if(veh_id == 'car0.'+str(i)):
                        if(len(car0_buffer[i])>1):
                            car0_buffer[i].pop()
                    if(veh_id == 'car1.'+str(i)):
                        if(len(car1_buffer[i])>1):
                            car1_buffer[i].pop()
                    if(veh_id == 'car2.'+str(i)): 
                        if(len(car2_buffer[i])>1):
                            car2_buffer[i].pop()
                    if(veh_id == 'car3.'+str(i)):
                        if(len(car3_buffer[i])>1):
                            car3_buffer[i].pop()
                
    traci.close()
    sys.stdout.flush()



netconvertBinary = checkBinary('netconvert')
sumoBinary = checkBinary('sumo-gui')
# build/check network
retcode = subprocess.call(
    [netconvertBinary, "-c", "highway3.netccfg"], stdout=sys.stdout, stderr=sys.stderr)
try:
    shutil.copy("highway3.net.xml", "net.net.xml")
except IOError:
    print("Missing 'highway3.net.xml'")



if __name__ == "__main__":
    
    options = get_options()
    #FILE = os.path
    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # first, generate the route file for this simulation
    generate_routefile()
    
    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    # traci.start([sumoBinary, "-c", "hello.sumocfg"])
    import sumolib
    traci.start([sumolib.checkBinary("sumo-gui"),"-c", "highway3.sumocfg",
             '--lanechange.duration', '1',
            #  '--collision.action', 'warn',
            #  '--collision.stoptime','5',
             '--ignore-junction-blocker','0.5',
             '--collision.action', 'remove',
             '--collision.mingap-factor', '0',
            #  '--time-to-teleport','10',
             '--collision-output','colliderSpeed',
             '--step-length', str(step_length),
             '--no-step-log']) 
    run()
    save_csv()