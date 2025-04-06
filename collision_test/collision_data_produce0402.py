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
import datetime
import subprocess
import sys
import shutil

import optparse
from sumolib import checkBinary  
import traci
import traci.constants as tc
import pandas as pd
import numpy as np
import random
import matplotlib.pyplot as plt

today = datetime.datetime.now().strftime("%Y%m%d")

sys.path.append(
    os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', "tools"))
sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
    os.path.dirname(__file__), "..", "..", "..")), "tools"))
from sumolib import checkBinary 

accel0 =[]
accel1 =[]
accel2 =[]
accel3 =[]
truck0 =[]
truck1 =[]
truck2 =[]
truck3 =[]
car0 =[]
car1 =[]
car2 =[]
car3 =[]
car4 =[]
car5 =[]
car6 =[]
car7 =[]
collision =[]

for i in range(5):
    accel0.append([])
    truck0.append([])
    accel1.append([])
    truck1.append([])
    accel2.append([])
    truck2.append([])
    accel3.append([])
    truck3.append([])
    
for i in range(10):
    car0.append([])
    car1.append([])
    car2.append([])
    car3.append([])
    car4.append([])
    car5.append([])
    car6.append([])
    car7.append([])




def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options

def generate_routefile():
    #collisionAvoidanceOverride="50"
    with open("highway.rou.xml", "w") as routes:
        print("""<routes>
        <vType accel="3" decel="4.5" departSpeed = "22" id="car" length="5" maxSpeed="27.77" minGap="0.0" sigma="1" lcSpeedGain ="0" />
        <vType accel="2.5" decel="4.5" departSpeed = "22" emergencyDecel="9.0" id="truck" length="12" maxSpeed="33.3333" minGap="0.0" sigma="1" collisionMinGapFactor="0" tau="0.0001" lcSpeedGain ="0" vClass="truck" color="0,0,1"/>
        <vType accel="2.5" decel="4.5" departSpeed = "22" id="accel_truck" length="12" maxSpeed="33.3333" minGap="0.0" sigma="1" collisionMinGapFactor="0" tau="0.0001" lcSpeedGain ="0" vClass="truck" color="1,0,0"/>
        
        <flow id="car0" departPos="free" begin="0.00" from="E4" to="E6" end="500.00" number="10" type="car"/>
        <flow id="car1" begin="5.00" from="E4" to="E8" end="500.00" number="10" type="car"/>
        <flow id="car2" begin="10.00" from="E7" to="E6" end="500.00" number="10" type="car" color="0,1,0"/>
        <flow id="car3" begin="15.00" from="E7" to="E8" end="500.00" number="10" type="car" color="0,1,0"/>
        <flow id="truck0" departPos="free" begin="20.00" from="E4" to="E6" end="500.00" number="5" type="truck" color="0,0,1"/>
        <flow id="truck1" departPos="free" begin="25.00" from="E7" to="E6" end="500.00" number="5" type="truck" color="0,0,1"/>
        <flow id="car4" departPos="free" begin="30.00" from="E4" to="E6" end="500.00" number="10" type="car"/>
        <flow id="car5" begin="35.00" from="E4" to="E8" end="500.00" number="10" type="car"/>
        <flow id="car6" begin="40.00" from="E7" to="E6" end="500.00" number="10" type="car" color="0,1,0"/>
        <flow id="car7" begin="45.00" from="E7" to="E8" end="500.00" number="10" type="car" color="0,1,0"/>
        <flow id="truck2" departPos="free" begin="50.00" from="E4" to="E6" end="500.00" number="5" type="truck" color="0,0,1"/>
        <flow id="truck3" departPos="free" begin="55.00" from="E7" to="E6" end="500.00" number="5" type="truck" color="0,0,1"/>
       """, file=routes)
        print("</routes>", file=routes)
        # \<flow id="accel0"  departPos="free" begin="40.00" from="E4" to="E6" end="500.00" number="5" type="accel_truck" color="1,0,0"/>
        #<flow id="accel1"  departPos="free" begin="50.00" from="E7" to="E6" end="500.00" number="5" type="accel_truck" color="1,0,0"/>
        
def save_csv():
    col = ['time','vehicle_id','x','y','v_x','v_y','a_x','a_y','theta_h']
        #    ,'x_for_id','x_for','v_for','x_bac_id','x_bac','v_bac','left_leader','left_follower','right_leader','right_follower']
    # df_accel= pd.DataFrame(accel0[0],columns=col)
    # for i in range(1):
    #     df = pd.DataFrame(accel0[i+1],columns=col)
    #     df_accel=pd.concat([df_accel,df],ignore_index = True)
    # for i in range(2):
    #     df2 = pd.DataFrame(accel1[i],columns=col)
    #     df_accel=pd.concat([df_accel,df2],ignore_index = True)
    df_truck= pd.DataFrame(truck0[0],columns=col)
    for i in range(4):
        df = pd.DataFrame(truck0[i+1],columns=col)
        df_truck=pd.concat([df_truck,df],ignore_index = True)
    for i in range(5):
        df1 = pd.DataFrame(truck1[i],columns=col)
        df_truck=pd.concat([df_truck,df1],ignore_index = True)
    for i in range(5):
        df2 = pd.DataFrame(truck2[i],columns=col)
        df_truck=pd.concat([df_truck,df2],ignore_index = True)
    for i in range(5):
        df3 = pd.DataFrame(truck3[i],columns=col)
        df_truck=pd.concat([df_truck,df3],ignore_index = True)
    
    
    
    df_car= pd.DataFrame(car0[0],columns=col)  
    for i in range(9):
        df4 = pd.DataFrame(car0[i+1],columns=col)
        df_car=pd.concat([df_car,df4],ignore_index = True)
    for i in range(10):
        df5 =pd.DataFrame(car1[i],columns=col)
        df_car=pd.concat([df_car,df5],ignore_index = True)
    for i in range(10):
        df6 = pd.DataFrame(car2[i],columns=col)
        df_car=pd.concat([df_car,df6],ignore_index = True)
    for i in range(10):
        df7 = pd.DataFrame(car3[i],columns=col)
        df_car=pd.concat([df_car,df7],ignore_index = True)
    for i in range(10):
        df8 = pd.DataFrame(car4[i],columns=col)
        df_car=pd.concat([df_car,df8],ignore_index = True)
    for i in range(10):
        df9 =pd.DataFrame(car5[i],columns=col)
        df_car=pd.concat([df_car,df9],ignore_index = True)
    for i in range(10):
        df10 = pd.DataFrame(car6[i],columns=col)
        df_car=pd.concat([df_car,df10],ignore_index = True)
    for i in range(10):
        df11 = pd.DataFrame(car7[i],columns=col)
        df_car=pd.concat([df_car,df11],ignore_index = True)
    col_c = ['victim','collider']

    df_collision= pd.DataFrame(collision,columns=col_c)
        
    # df1=pd.DataFrame({'time':accel0[0][0][0],'vehicle_id':accel0[0][0][1],'x':accel0[0][0][2],'y':accel0[0][0][3],'v_x':accel0[0][0][4],'v_y':accel0[0][0][5],'a_x':accel0[0][0][6],'a_y':accel0[0][0][7],'theta_h':accel0[0][0][8], 'x_for_id':accel0[0][0][9],'x_for':accel0[0][0][10],'v_for':accel0[0][0][11],'x_bac_id':accel0[0][0][12],'x_bac':accel0[0][0][13],'v_bac':accel0[0][0][14]})   
    # for i in range(len(accel0[0])-1):
    #     pd.concat([df1,accel0[0][i]],ignore_index = True)               
    # os.chdir('/home/jewoo/Desktop/collision_test/log_data/20250402')
    log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "log_data", today)
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    os.chdir(log_dir)
    # df_accel.to_csv('log_data_accel.csv')
    df_truck.to_csv('log_data_truck.csv')
    df_car.to_csv('log_data_car.csv')
    df_collision.to_csv('log_data_collision.csv')

def run():
    """execute the TraCI control loop"""
    # step = 0
    random_speed=[]
    random_acceleration=[]
    random_deceleration=[]
    random_tau=[]
    for i in range(200):
        random_speed.append(float(random.randint(2777, 3333))/100)
        random_acceleration.append(float(random.randint(100,300))/100)
        random_deceleration.append(float(random.randint(-300,0))/100)
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        if len(traci.simulation.getCollidingVehiclesIDList()) !=0:
            collision.append(traci.simulation.getCollidingVehiclesIDList())
        e5_vehicles = traci.edge.getLastStepVehicleIDs('E5')
        for veh_id in e5_vehicles:
            if veh_id[0][-1] == 't':
               ID = veh_id.split('.')
            #    traci.vehicle.setSpeedMode(veh_id,32)
               traci.vehicle.setSpeed(veh_id,random_speed[int(ID[0][-1])*10+int(ID[1])])
               traci.vehicle.setAcceleration(veh_id,random_speed[int(ID[0][-1])*10+int(ID[1])],1)
               traci.vehicle.setDecel(veh_id,random_speed[int(ID[0][-1])*10+int(ID[1])]) 
        for veh_id in e5_vehicles:
            vehicle_state = []
            vehicle_state.append(traci.simulation.getTime()/1000)
            vehicle_state.append(veh_id)
            x1,y1 = traci.vehicle.getPosition(veh_id)
            vehicle_state.append(x1)
            vehicle_state.append(y1)
            vehicle_state.append(traci.vehicle.getSpeed(veh_id))            
            vehicle_state.append(traci.vehicle.getLateralSpeed(veh_id))           
            ID = veh_id.split('.')
            acceleartion_x =0
            # if ID[0][0] == 'a' and ID[0][-1]=='0' and not (len(accel0[int(ID[1])])==0):
            #     acceleartion_x = traci.vehicle.getSpeed(veh_id) - accel0[int(ID[1])][-1][4]
            # if ID[0][0] == 'a' and ID[0][-1]=='1'and not (len(accel1[int(ID[1])])==0):
            #     acceleartion_x = traci.vehicle.getSpeed(veh_id) - accel1[int(ID[1])][-1][4]
            if ID[0][0] == 't' and ID[0][-1]=='0'and not (len(truck0[int(ID[1])])==0):
                acceleartion_x = traci.vehicle.getSpeed(veh_id) - truck0[int(ID[1])][-1][4]
            if ID[0][0] == 't' and ID[0][-1]=='1'and not (len(truck1[int(ID[1])])==0):
                acceleartion_x = traci.vehicle.getSpeed(veh_id) - truck1[int(ID[1])][-1][4]
            if ID[0][0] == 't' and ID[0][-1]=='2'and not (len(truck2[int(ID[1])])==0):
                acceleartion_x = traci.vehicle.getSpeed(veh_id) - truck2[int(ID[1])][-1][4]
            if ID[0][0] == 't' and ID[0][-1]=='3'and not (len(truck3[int(ID[1])])==0):
                acceleartion_x = traci.vehicle.getSpeed(veh_id) - truck3[int(ID[1])][-1][4]

            if ID[0][0] == 'c' and ID[0][-1]=='0'and not (len(car0[int(ID[1])])==0):
                acceleartion_x = traci.vehicle.getSpeed(veh_id) - car0[int(ID[1])][-1][4]
            if ID[0][0] == 'c' and ID[0][-1]=='0'and not (len(car0[int(ID[1])])==0):
                acceleartion_x = traci.vehicle.getSpeed(veh_id) - car0[int(ID[1])][-1][4]
            if ID[0][0] == 'c' and ID[0][-1]=='1'and not (len(car1[int(ID[1])])==0):
                acceleartion_x = traci.vehicle.getSpeed(veh_id) - car1[int(ID[1])][-1][4]
            if ID[0][0] == 'c' and ID[0][-1]=='1'and not (len(car1[int(ID[1])])==0):
                acceleartion_x = traci.vehicle.getSpeed(veh_id) - car1[int(ID[1])][-1][4]
            if ID[0][0] == 'c' and ID[0][-1]=='2'and not (len(car2[int(ID[1])])==0):
                acceleartion_x = traci.vehicle.getSpeed(veh_id) - car2[int(ID[1])][-1][4]
            if ID[0][0] == 'c' and ID[0][-1]=='2'and not (len(car2[int(ID[1])])==0):
                acceleartion_x = traci.vehicle.getSpeed(veh_id) - car2[int(ID[1])][-1][4]
            if ID[0][0] == 'c' and ID[0][-1]=='3'and not (len(car3[int(ID[1])])==0):
                acceleartion_x = traci.vehicle.getSpeed(veh_id) - car3[int(ID[1])][-1][4]
            if ID[0][0] == 'c' and ID[0][-1]=='3'and not (len(car3[int(ID[1])])==0):
                acceleartion_x = traci.vehicle.getSpeed(veh_id) - car3[int(ID[1])][-1][4]

            if ID[0][0] == 'c' and ID[0][-1]=='4'and not (len(car4[int(ID[1])])==0):
                acceleartion_x = traci.vehicle.getSpeed(veh_id) - car4[int(ID[1])][-1][4]
            if ID[0][0] == 'c' and ID[0][-1]=='4'and not (len(car4[int(ID[1])])==0):
                acceleartion_x = traci.vehicle.getSpeed(veh_id) - car4[int(ID[1])][-1][4]
            if ID[0][0] == 'c' and ID[0][-1]=='5'and not (len(car5[int(ID[1])])==0):
                acceleartion_x = traci.vehicle.getSpeed(veh_id) - car5[int(ID[1])][-1][4]
            if ID[0][0] == 'c' and ID[0][-1]=='5'and not (len(car5[int(ID[1])])==0):
                acceleartion_x = traci.vehicle.getSpeed(veh_id) - car5[int(ID[1])][-1][4]
            if ID[0][0] == 'c' and ID[0][-1]=='6'and not (len(car6[int(ID[1])])==0):
                acceleartion_x = traci.vehicle.getSpeed(veh_id) - car6[int(ID[1])][-1][4]
            if ID[0][0] == 'c' and ID[0][-1]=='6'and not (len(car6[int(ID[1])])==0):
                acceleartion_x = traci.vehicle.getSpeed(veh_id) - car6[int(ID[1])][-1][4]
            if ID[0][0] == 'c' and ID[0][-1]=='7'and not (len(car7[int(ID[1])])==0):
                acceleartion_x = traci.vehicle.getSpeed(veh_id) - car7[int(ID[1])][-1][4]
            if ID[0][0] == 'c' and ID[0][-1]=='7'and not (len(car7[int(ID[1])])==0):
                acceleartion_x = traci.vehicle.getSpeed(veh_id) - car7[int(ID[1])][-1][4]
            vehicle_state.append(acceleartion_x)

            acceleartion_y=0
            # if ID[0][0] == 'a' and ID[0][-1]=='0'and not (len(accel0[int(ID[1])])==0):
            #     acceleartion_y = traci.vehicle.getLateralSpeed(veh_id) - accel0[int(ID[1])][-1][5]
            # if ID[0][0] == 'a' and ID[0][-1]=='1'and not (len(accel1[int(ID[1])])==0):
            #     acceleartion_y = traci.vehicle.getLateralSpeed(veh_id) - accel1[int(ID[1])][-1][5]
            if ID[0][0] == 't' and ID[0][-1]=='0'and not (len(truck0[int(ID[1])])==0):
                acceleartion_y = traci.vehicle.getLateralSpeed(veh_id) - truck0[int(ID[1])][-1][5]
            if ID[0][0] == 't' and ID[0][-1]=='1'and not (len(truck1[int(ID[1])])==0):
                acceleartion_y = traci.vehicle.getLateralSpeed(veh_id) - truck1[int(ID[1])][-1][5]
            if ID[0][0] == 't' and ID[0][-1]=='2'and not (len(truck2[int(ID[1])])==0):
                acceleartion_y = traci.vehicle.getLateralSpeed(veh_id) - truck2[int(ID[1])][-1][5]
            if ID[0][0] == 't' and ID[0][-1]=='3'and not (len(truck3[int(ID[1])])==0):
                acceleartion_y = traci.vehicle.getLateralSpeed(veh_id) - truck3[int(ID[1])][-1][5]

            if ID[0][0] == 'c' and ID[0][-1]=='0'and not (len(car0[int(ID[1])])==0):
                acceleartion_y = traci.vehicle.getLateralSpeed(veh_id) - car0[int(ID[1])][-1][5]
            if ID[0][0] == 'c' and ID[0][-1]=='0'and not (len(car0[int(ID[1])])==0):
                acceleartion_y = traci.vehicle.getLateralSpeed(veh_id) - car0[int(ID[1])][-1][5]
            if ID[0][0] == 'c' and ID[0][-1]=='1'and not (len(car1[int(ID[1])])==0):
                acceleartion_y = traci.vehicle.getLateralSpeed(veh_id) - car1[int(ID[1])][-1][5]
            if ID[0][0] == 'c' and ID[0][-1]=='1'and not (len(car1[int(ID[1])])==0):
                acceleartion_y = traci.vehicle.getLateralSpeed(veh_id) - car1[int(ID[1])][-1][5]
            if ID[0][0] == 'c' and ID[0][-1]=='2'and not (len(car2[int(ID[1])])==0):
                acceleartion_y = traci.vehicle.getLateralSpeed(veh_id) - car2[int(ID[1])][-1][5]
            if ID[0][0] == 'c' and ID[0][-1]=='2'and not (len(car2[int(ID[1])])==0):
                acceleartion_y = traci.vehicle.getLateralSpeed(veh_id) - car2[int(ID[1])][-1][5]
            if ID[0][0] == 'c' and ID[0][-1]=='3'and not (len(car3[int(ID[1])])==0):
                acceleartion_y = traci.vehicle.getLateralSpeed(veh_id) - car3[int(ID[1])][-1][5]
            if ID[0][0] == 'c' and ID[0][-1]=='3'and not (len(car3[int(ID[1])])==0):
                acceleartion_y = traci.vehicle.getLateralSpeed(veh_id) - car3[int(ID[1])][-1][5]

            if ID[0][0] == 'c' and ID[0][-1]=='4'and not (len(car4[int(ID[1])])==0):
                acceleartion_y = traci.vehicle.getLateralSpeed(veh_id) - car4[int(ID[1])][-1][5]
            if ID[0][0] == 'c' and ID[0][-1]=='4'and not (len(car4[int(ID[1])])==0):
                acceleartion_y = traci.vehicle.getLateralSpeed(veh_id) - car4[int(ID[1])][-1][5]
            if ID[0][0] == 'c' and ID[0][-1]=='5'and not (len(car5[int(ID[1])])==0):
                acceleartion_y = traci.vehicle.getLateralSpeed(veh_id) - car5[int(ID[1])][-1][5]
            if ID[0][0] == 'c' and ID[0][-1]=='5'and not (len(car5[int(ID[1])])==0):
                acceleartion_y = traci.vehicle.getLateralSpeed(veh_id) - car5[int(ID[1])][-1][5]
            if ID[0][0] == 'c' and ID[0][-1]=='6'and not (len(car6[int(ID[1])])==0):
                acceleartion_y = traci.vehicle.getLateralSpeed(veh_id) - car6[int(ID[1])][-1][5]
            if ID[0][0] == 'c' and ID[0][-1]=='6'and not (len(car6[int(ID[1])])==0):
                acceleartion_y = traci.vehicle.getLateralSpeed(veh_id) - car6[int(ID[1])][-1][5]
            if ID[0][0] == 'c' and ID[0][-1]=='7'and not (len(car7[int(ID[1])])==0):
                acceleartion_y = traci.vehicle.getLateralSpeed(veh_id) - car7[int(ID[1])][-1][5]
            if ID[0][0] == 'c' and ID[0][-1]=='7'and not (len(car7[int(ID[1])])==0):
                acceleartion_y = traci.vehicle.getLateralSpeed(veh_id) - car7[int(ID[1])][-1][5]

            vehicle_state.append(acceleartion_y)
            # print("a_x : ",traci.vehicle.getAccel(veh_id))
            
            #print("getAcceleration : ",traci.vehicle.getAcceleration(veh_id))
            vehicle_state.append(traci.vehicle.getAngle(veh_id))
            # print("theta_h : ", traci.vehicle.getAngle(veh_id))
            
            # print("Leader_id, x_for  : ",traci.vehicle.getLeader(veh_id))
            # if traci.vehicle.getLeader(veh_id) is not None:
            #     Learder_id,x_forward = traci.vehicle.getLeader(veh_id)
            #     vehicle_state.append(Learder_id)
            #     vehicle_state.append(x_forward)
            #     v_forward = traci.vehicle.getSpeed(traci.vehicle.getLeader(veh_id)[0]) -traci.vehicle.getSpeed(veh_id) #(relative speed(v_for) = front_vehicle speed - ego_vehicle)
            #     # print("v_for : ",v_for)
            #     vehicle_state.append(v_forward)
            # else:
            #     vehicle_state.append('None')
            #     vehicle_state.append('None')
            #     # print("v_for : ",'None')
            #     vehicle_state.append('None')

            # # print("Follower_id, x_bac : ",traci.vehicle.getFollower(veh_id) )
            # Follower_id,x_bacward = traci.vehicle.getFollower(veh_id)
                        
            # if type(traci.vehicle.getFollower(veh_id)) is not None and traci.vehicle.getFollower(veh_id)[0] != '':
            #     v_backward= traci.vehicle.getSpeed(traci.vehicle.getFollower(veh_id)[0]) -traci.vehicle.getSpeed(veh_id) #(relative speed(v_bac) = back_vehicle speed - ego_vehicle)
            #     vehicle_state.append(Follower_id)
            #     vehicle_state.append(x_bacward)
            #     # print("v_bac : ",v_bac)
            #     vehicle_state.append(v_backward)
            # else:
            #     # print("v_bac : ",'None')
            #     vehicle_state.append("None")
            #     vehicle_state.append("None")
            #     vehicle_state.append("None")


            # # print("left_follower :" ,traci.vehicle.getLeftFollowers(veh_id))
            # # print("right_follower :" ,traci.vehicle.getRightFollowers(veh_id))
            # # print("left_leader :" ,traci.vehicle.getLeftLeaders(veh_id))
            # # print(len(traci.vehicle.getLeftLeaders(veh_id)))
            # # print("right_leader :" ,traci.vehicle.getRightLeaders(veh_id))

            # if traci.vehicle.getLeftLeaders(veh_id) is not None and len(traci.vehicle.getLeftLeaders(veh_id)) !=0:
            #     Left_leader_id = traci.vehicle.getLeftLeaders(veh_id)
            #     vehicle_state.append(Left_leader_id[0][0])
            # else:
            #     vehicle_state.append('None')
            # if traci.vehicle.getLeftFollowers(veh_id) is not None and len(traci.vehicle.getLeftFollowers(veh_id)) !=0:
            #     Left_follower_id = traci.vehicle.getLeftFollowers(veh_id)
            #     vehicle_state.append(Left_follower_id[0][0])
            # else:
            #     vehicle_state.append('None')
            
            # if traci.vehicle.getRightLeaders(veh_id) is not None and len(traci.vehicle.getRightLeaders(veh_id)) !=0:
            #     Right_reader_id = traci.vehicle.getRightLeaders(veh_id)
            #     vehicle_state.append(Right_reader_id[0][0])  
            # else:
            #     vehicle_state.append('None') 
            
            # if traci.vehicle.getRightFollowers(veh_id) is not None and len(traci.vehicle.getRightFollowers(veh_id)) !=0:
            #     Right_follower_id = traci.vehicle.getRightFollowers(veh_id)
            #     vehicle_state.append(Right_follower_id[0][0])
            # else:
            #     vehicle_state.append('None')

            
            # for i in range(2):
            #     if(veh_id == 'accel0.'+str(i)):
            #         accel0[i].append(vehicle_state)
            #     if(veh_id == 'accel1.'+str(i)):
            #         accel1[i].append(vehicle_state)
            for i in range(5): 
                if(veh_id == 'truck0.'+str(i)):
                    truck0[i].append(vehicle_state)
                if(veh_id == 'truck1.'+str(i)):
                    truck1[i].append(vehicle_state)
                if(veh_id == 'truck2.'+str(i)):
                    truck2[i].append(vehicle_state)
                if(veh_id == 'truck3.'+str(i)):
                    truck3[i].append(vehicle_state)

            for i in range(10):
                if(veh_id == 'car0.'+str(i)):
                    car0[i].append(vehicle_state)
                if(veh_id == 'car1.'+str(i)):
                    car1[i].append(vehicle_state)
                if(veh_id == 'car2.'+str(i)):
                    car2[i].append(vehicle_state)
                if(veh_id == 'car3.'+str(i)):
                    car3[i].append(vehicle_state)
                if(veh_id == 'car4.'+str(i)):
                    car4[i].append(vehicle_state)
                if(veh_id == 'car5.'+str(i)):
                    car5[i].append(vehicle_state)
                if(veh_id == 'car6.'+str(i)):
                    car6[i].append(vehicle_state)
                if(veh_id == 'car7.'+str(i)):
                    car7[i].append(vehicle_state)

            
        # if traci.vehicle.getRoadID(veh_id) == 'E5' and veh_id[0] == 'a':
        #     traci.vehicle.setSpeed(veh_id, random_speed[int(veh_id[7:])])
        #     traci.vehicle.setAccel(veh_id, '50')
        #     traci.vehicle.setDecel(veh_id, '0.00001')
        #     traci.vehicle.setSpeedMode(veh_id,'0')
        
        
    traci.close()
    sys.stdout.flush()



# netconvertBinary = checkBinary('netconvert')
sumoBinary = checkBinary('sumo-gui')
# build/check network
# retcode = subprocess.call(
#     [netconvertBinary, "-c", "highway.netgcfg"], stdout=sys.stdout, stderr=sys.stderr)
try:
    shutil.copy("highway.net.xml", "net.net.xml")
except IOError:
    print("Missing 'highway.net.xml'")



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
    traci.start([sumolib.checkBinary("sumo-gui"),"-c", "highway.sumocfg",
             '--lanechange.duration', '4',
            #  '--collision.action', 'warn',
            #  '--collision.stoptime','5',
             '--collision.action', 'remove',
             '--collision.mingap-factor', '0',
            #  '--time-to-teleport','10',
             '--collision-output','colliderSpeed',
             '--step-length', '0.01',
             '--no-step-log'])
    run()
    save_csv()