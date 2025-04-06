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
from sumolib import checkBinary  
import traci
import traci.constants as tc
import pandas as pd
import numpy as np
import random
import matplotlib.pyplot as plt

sys.path.append(
    os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', "tools"))
sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
    os.path.dirname(__file__), "..", "..", "..")), "tools"))
from sumolib import checkBinary 

class HighwaySimulation:
    def __init__(self):
        # 차량 데이터 초기화
        self.accel0 = []
        self.truck0 = []
        self.accel1 = []
        self.truck1 = []
        self.car0 = []
        self.car1 = []
        self.car2 = []
        self.car3 = []
        self.collision = []

        # 리스트 초기화
        for i in range(12):
            self.accel0.append([])
            self.truck0.append([])
        for i in range(8):
            self.accel1.append([])
            self.truck1.append([])
        for i in range(15):
            self.car0.append([])
            self.car1.append([])
            self.car2.append([])
            self.car3.append([])

    def get_options(self):
        optParser = optparse.OptionParser()
        optParser.add_option("--nogui", action="store_true",
                            default=False, help="run the commandline version of sumo")
        options, args = optParser.parse_args()
        return options

    def generate_routefile(self):
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
            
    def save_csv(self):
        col = ['time','vehicle_id','x','y','v_x','v_y','a_x','a_y','theta_h','x_for_id','x_for','v_for','x_bac_id','x_bac','v_bac','left_leader','left_follower','right_leader','right_follower']
        df_accel= pd.DataFrame(self.accel0[0],columns=col)
        for i in range(1):
            df = pd.DataFrame(self.accel0[i+1],columns=col)
            df_accel=pd.concat([df_accel,df],ignore_index = True)
        for i in range(2):
            df2 = pd.DataFrame(self.accel1[i],columns=col)
            df_accel=pd.concat([df_accel,df2],ignore_index = True)
        df_truck= pd.DataFrame(self.truck0[0],columns=col)
        for i in range(11):
            df3 = pd.DataFrame(self.truck0[i+1],columns=col)
            df_truck=pd.concat([df_truck,df3],ignore_index = True)
        for i in range(8):
            df4 = pd.DataFrame(self.truck1[i],columns=col)
            df_truck=pd.concat([df_truck,df4],ignore_index = True)
        df_car= pd.DataFrame(self.car0[0],columns=col)  
        for i in range(14):
            df5 = pd.DataFrame(self.car0[i+1],columns=col)
            df_car=pd.concat([df_car,df5],ignore_index = True)
        for i in range(15):
            df6 =pd.DataFrame(self.car1[i],columns=col)
            df_car=pd.concat([df_car,df6],ignore_index = True)
        for i in range(15):
            df7 = pd.DataFrame(self.car2[i],columns=col)
            df_car=pd.concat([df_car,df7],ignore_index = True)
        for i in range(15):
            df8 = pd.DataFrame(self.car3[i],columns=col)
            df_car=pd.concat([df_car,df8],ignore_index = True)
        col_c = ['victim','collider']

        df_collision= pd.DataFrame(self.collision,columns=col_c)
            
        # df1=pd.DataFrame({'time':accel0[0][0][0],'vehicle_id':accel0[0][0][1],'x':accel0[0][0][2],'y':accel0[0][0][3],'v_x':accel0[0][0][4],'v_y':accel0[0][0][5],'a_x':accel0[0][0][6],'a_y':accel0[0][0][7],'theta_h':accel0[0][0][8], 'x_for_id':accel0[0][0][9],'x_for':accel0[0][0][10],'v_for':accel0[0][0][11],'x_bac_id':accel0[0][0][12],'x_bac':accel0[0][0][13],'v_bac':accel0[0][0][14]})   
        # for i in range(len(accel0[0])-1):
        #     pd.concat([df1,accel0[0][i]],ignore_index = True)               
        os.chdir('/home/jewoo/Desktop/collision_test/log_data/20250328')
        df_accel.to_csv('log_data_accel.csv')
        df_truck.to_csv('log_data_truck.csv')
        df_car.to_csv('log_data_car.csv')
        df_collision.to_csv('log_data_collsion')

    def run(self):
        """execute the TraCI control loop"""
        # step = 0
        random_speed=[]
        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            for i in range(200):
                random_speed.append(float(random.randint(5000, 5550))/100)
            vehs = traci.vehicle.getIDList()
            if len(traci.simulation.getCollidingVehiclesIDList()) !=0:
                self.collision.append(traci.simulation.getCollidingVehiclesIDList())
            for veh_id in vehs:
                vehicle_state = []
                #  print(veh_id)
                #  print(veh_id[7:])
                if traci.vehicle.getRoadID(veh_id) == 'E5': 
                    vehicle_state.append(traci.simulation.getCurrentTime()/1000)
                    # print("cur_time : ",traci.simulation.getCurrentTime())
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
                    if ID[0][0] == 'a' and ID[0][-1]=='0' and not (len(self.accel0[int(ID[1])])==0):
                        acceleartion_x = traci.vehicle.getSpeed(veh_id) - self.accel0[int(ID[1])][-1][4]
                    if ID[0][0] == 'a' and ID[0][-1]=='1'and not (len(self.accel1[int(ID[1])])==0):
                        acceleartion_x = traci.vehicle.getSpeed(veh_id) - self.accel1[int(ID[1])][-1][4]
                    if ID[0][0] == 't' and ID[0][-1]=='0'and not (len(self.truck0[int(ID[1])])==0):
                        acceleartion_x = traci.vehicle.getSpeed(veh_id) - self.truck0[int(ID[1])][-1][4]
                    if ID[0][0] == 't' and ID[0][-1]=='1'and not (len(self.truck1[int(ID[1])])==0):
                        acceleartion_x = traci.vehicle.getSpeed(veh_id) - self.truck1[int(ID[1])][-1][4]
                    if ID[0][0] == 'c' and ID[0][-1]=='0'and not (len(self.car0[int(ID[1])])==0):
                        acceleartion_x = traci.vehicle.getSpeed(veh_id) - self.car0[int(ID[1])][-1][4]
                    if ID[0][0] == 'c' and ID[0][-1]=='1'and not (len(self.car0[int(ID[1])])==0):
                        acceleartion_x = traci.vehicle.getSpeed(veh_id) - self.car0[int(ID[1])][-1][4]
                    if ID[0][0] == 'c' and ID[0][-1]=='0'and not (len(self.car1[int(ID[1])])==0):
                        acceleartion_x = traci.vehicle.getSpeed(veh_id) - self.car1[int(ID[1])][-1][4]
                    if ID[0][0] == 'c' and ID[0][-1]=='1'and not (len(self.car1[int(ID[1])])==0):
                        acceleartion_x = traci.vehicle.getSpeed(veh_id) - self.car1[int(ID[1])][-1][4]
                    if ID[0][0] == 'c' and ID[0][-1]=='0'and not (len(self.car2[int(ID[1])])==0):
                        acceleartion_x = traci.vehicle.getSpeed(veh_id) - self.car2[int(ID[1])][-1][4]
                    if ID[0][0] == 'c' and ID[0][-1]=='1'and not (len(self.car2[int(ID[1])])==0):
                        acceleartion_x = traci.vehicle.getSpeed(veh_id) - self.car2[int(ID[1])][-1][4]
                    if ID[0][0] == 'c' and ID[0][-1]=='0'and not (len(self.car3[int(ID[1])])==0):
                        acceleartion_x = traci.vehicle.getSpeed(veh_id) - self.car3[int(ID[1])][-1][4]
                    if ID[0][0] == 'c' and ID[0][-1]=='1'and not (len(self.car3[int(ID[1])])==0):
                        acceleartion_x = traci.vehicle.getSpeed(veh_id) - self.car3[int(ID[1])][-1][4]
                    vehicle_state.append(acceleartion_x)

                    acceleartion_y=0
                    if ID[0][0] == 'a' and ID[0][-1]=='0'and not (len(self.accel0[int(ID[1])])==0):
                        acceleartion_y = traci.vehicle.getLateralSpeed(veh_id) - self.accel0[int(ID[1])][-1][5]
                    if ID[0][0] == 'a' and ID[0][-1]=='1'and not (len(self.accel1[int(ID[1])])==0):
                        acceleartion_y = traci.vehicle.getLateralSpeed(veh_id) - self.accel1[int(ID[1])][-1][5]
                    if ID[0][0] == 't' and ID[0][-1]=='0'and not (len(self.truck0[int(ID[1])])==0):
                        acceleartion_y = traci.vehicle.getLateralSpeed(veh_id) - self.truck0[int(ID[1])][-1][5]
                    if ID[0][0] == 't' and ID[0][-1]=='1'and not (len(self.truck1[int(ID[1])])==0):
                        acceleartion_y = traci.vehicle.getLateralSpeed(veh_id) - self.truck1[int(ID[1])][-1][5]
                    if ID[0][0] == 'c' and ID[0][-1]=='0'and not (len(self.car0[int(ID[1])])==0):
                        acceleartion_y = traci.vehicle.getLateralSpeed(veh_id) - self.car0[int(ID[1])][-1][5]
                    if ID[0][0] == 'c' and ID[0][-1]=='1'and not (len(self.car0[int(ID[1])])==0):
                        acceleartion_y = traci.vehicle.getLateralSpeed(veh_id) - self.car0[int(ID[1])][-1][5]
                    if ID[0][0] == 'c' and ID[0][-1]=='0'and not (len(self.car1[int(ID[1])])==0):
                        acceleartion_y = traci.vehicle.getLateralSpeed(veh_id) - self.car1[int(ID[1])][-1][5]
                    if ID[0][0] == 'c' and ID[0][-1]=='1'and not (len(self.car1[int(ID[1])])==0):
                        acceleartion_y = traci.vehicle.getLateralSpeed(veh_id) - self.car1[int(ID[1])][-1][5]
                    if ID[0][0] == 'c' and ID[0][-1]=='0'and not (len(self.car2[int(ID[1])])==0):
                        acceleartion_y = traci.vehicle.getLateralSpeed(veh_id) - self.car2[int(ID[1])][-1][5]
                    if ID[0][0] == 'c' and ID[0][-1]=='1'and not (len(self.car2[int(ID[1])])==0):
                        acceleartion_y = traci.vehicle.getLateralSpeed(veh_id) - self.car2[int(ID[1])][-1][5]
                    if ID[0][0] == 'c' and ID[0][-1]=='0'and not (len(self.car3[int(ID[1])])==0):
                        acceleartion_y = traci.vehicle.getLateralSpeed(veh_id) - self.car3[int(ID[1])][-1][5]
                    if ID[0][0] == 'c' and ID[0][-1]=='1'and not (len(self.car3[int(ID[1])])==0):
                        acceleartion_y = traci.vehicle.getLateralSpeed(veh_id) - self.car3[int(ID[1])][-1][5]
                    vehicle_state.append(acceleartion_y)
                    # print("a_x : ",traci.vehicle.getAccel(veh_id))
                    
                    #print("getAcceleration : ",traci.vehicle.getAcceleration(veh_id))
                    vehicle_state.append(traci.vehicle.getAngle(veh_id))
                    # print("theta_h : ", traci.vehicle.getAngle(veh_id))
                    
                    # print("Leader_id, x_for  : ",traci.vehicle.getLeader(veh_id))
                    if traci.vehicle.getLeader(veh_id) is not None:
                        Learder_id,x_forward = traci.vehicle.getLeader(veh_id)
                        vehicle_state.append(Learder_id)
                        vehicle_state.append(x_forward)
                        v_forward = traci.vehicle.getSpeed(traci.vehicle.getLeader(veh_id)[0]) -traci.vehicle.getSpeed(veh_id) #(relative speed(v_for) = front_vehicle speed - ego_vehicle)
                        # print("v_for : ",v_for)
                        vehicle_state.append(v_forward)
                    else:
                        vehicle_state.append('None')
                        vehicle_state.append('None')
                        # print("v_for : ",'None')
                        vehicle_state.append('None')

                    # print("Follower_id, x_bac : ",traci.vehicle.getFollower(veh_id) )
                    Follower_id,x_bacward = traci.vehicle.getFollower(veh_id)
                            
                    if type(traci.vehicle.getFollower(veh_id)) is not None and traci.vehicle.getFollower(veh_id)[0] != '':
                        v_backward= traci.vehicle.getSpeed(traci.vehicle.getFollower(veh_id)[0]) -traci.vehicle.getSpeed(veh_id) #(relative speed(v_bac) = back_vehicle speed - ego_vehicle)
                        vehicle_state.append(Follower_id)
                        vehicle_state.append(x_bacward)
                        # print("v_bac : ",v_bac)
                        vehicle_state.append(v_backward)
                    else:
                        # print("v_bac : ",'None')
                        vehicle_state.append("None")
                        vehicle_state.append("None")
                        vehicle_state.append("None")


                    # print("left_follower :" ,traci.vehicle.getLeftFollowers(veh_id))
                    # print("right_follower :" ,traci.vehicle.getRightFollowers(veh_id))
                    # print("left_leader :" ,traci.vehicle.getLeftLeaders(veh_id))
                    # print(len(traci.vehicle.getLeftLeaders(veh_id)))
                    # print("right_leader :" ,traci.vehicle.getRightLeaders(veh_id))

                    if traci.vehicle.getLeftLeaders(veh_id) is not None and len(traci.vehicle.getLeftLeaders(veh_id)) !=0:
                        Left_leader_id = traci.vehicle.getLeftLeaders(veh_id)
                        vehicle_state.append(Left_leader_id[0][0])
                    else:
                        vehicle_state.append('None')
                    if traci.vehicle.getLeftFollowers(veh_id) is not None and len(traci.vehicle.getLeftFollowers(veh_id)) !=0:
                        Left_follower_id = traci.vehicle.getLeftFollowers(veh_id)
                        vehicle_state.append(Left_follower_id[0][0])
                    else:
                        vehicle_state.append('None')
                    
                    if traci.vehicle.getRightLeaders(veh_id) is not None and len(traci.vehicle.getRightLeaders(veh_id)) !=0:
                        Right_reader_id = traci.vehicle.getRightLeaders(veh_id)
                        vehicle_state.append(Right_reader_id[0][0])  
                    else:
                        vehicle_state.append('None') 
                    
                    if traci.vehicle.getRightFollowers(veh_id) is not None and len(traci.vehicle.getRightFollowers(veh_id)) !=0:
                        Right_follower_id = traci.vehicle.getRightFollowers(veh_id)
                        vehicle_state.append(Right_follower_id[0][0])
                    else:
                        vehicle_state.append('None')

                    
                    for i in range(2):
                        if(veh_id == 'accel0.'+str(i)):
                            self.accel0[i].append(vehicle_state)
                        if(veh_id == 'accel1.'+str(i)):
                            self.accel1[i].append(vehicle_state)
                    for i in range(12): 
                        if(veh_id == 'truck0.'+str(i)):
                            self.truck0[i].append(vehicle_state)
                    for i in range(8):
                        if(veh_id == 'truck1.'+str(i)):
                            self.truck1[i].append(vehicle_state)
                    for i in range(15):
                        if(veh_id == 'car0.'+str(i)):
                            self.car0[i].append(vehicle_state)
                        if(veh_id == 'car1.'+str(i)):
                            self.car1[i].append(vehicle_state)
                        if(veh_id == 'car2.'+str(i)):
                            self.car2[i].append(vehicle_state)
                        if(veh_id == 'car3.'+str(i)):
                            self.car3[i].append(vehicle_state)

                if traci.vehicle.getRoadID(veh_id) == 'E5' and veh_id[0] == 'a':
                    traci.vehicle.setSpeed(veh_id, random_speed[int(veh_id[7:])])
                    traci.vehicle.setAccel(veh_id, '50')
                    traci.vehicle.setDecel(veh_id, '0.00001')
                    traci.vehicle.setSpeedMode(veh_id,'0')
        
                
        traci.close()
        sys.stdout.flush()

    def setup_and_run(self):
        """Setup the simulation environment and run it"""
        options = self.get_options()
        
        netconvertBinary = checkBinary('netconvert')
        sumoBinary = checkBinary('sumo-gui')
        if options.nogui:
            sumoBinary = checkBinary('sumo')
        
        # build/check network
        retcode = subprocess.call(
            [netconvertBinary, "-c", "highway.netgcfg"], stdout=sys.stdout, stderr=sys.stderr)
        
        try:
            shutil.copy("highway_episodic.net.xml", "net.net.xml")
        except IOError:
            print("Missing 'highway_episodic.net.xml'")
            
        # Generate route file
        self.generate_routefile()
        
        # Start SUMO with appropriate settings
        traci.start([sumoBinary, "-c", "highway.sumocfg",
             '--lanechange.duration', '4',
             '--collision.action', 'remove',
             '--collision.mingap-factor', '0',
             '--collision-output','colliderSpeed',
             '--step-length', '0.01',
             '--no-step-log'])
             
        # Run the simulation
        self.run()
        # Save collected data
        self.save_csv()

# 메인 함수 - 프로그램 실행 시작점
def main():
    sim = HighwaySimulation()
    sim.setup_and_run()
    
# 스크립트가 직접 실행될 때만 main() 함수 호출
if __name__ == "__main__":
    main()