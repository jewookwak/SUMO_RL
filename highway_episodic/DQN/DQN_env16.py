# pid test env

import os
import sys
import time
from typing import DefaultDict
import numpy as np
import random
import traci.constants as tc
import pandas as pd
from xml.etree.ElementTree import parse
from collections import defaultdict
from pid import PID_Control
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'],'tools')
    print(tools)
    sys.path.append(tools)
    try:
        import traci
    except ImportError:
        raise EnvironmentError("Declare SUMO_HOME environment")
else:
    sys.exit('Declare environment variable "SUMO_HOME"')


class rlEnv():
    #class global variable
    lane_buffer_ego = 0
    ego_LC_success = False
    ego_LC_start = False
    ego_LC_completed = False
    Space1=[]
    Space2=[]
    Space3=[]
    Space4=[]
    integral_clearance_e=[0,0,0,0,0]
    integral_velocity_e=[0,0,0,0,0]
    pre_error_clearance = [0,0,0,0,0]
    pre_error_velocity = [0,0,0,0,0]
    ego_control=[]
    ego=[]
    left0=[]
    left1=[]
    left2=[]
    left3=[]
    left4=[]
    left5=[]
    left6=[]
    left7=[]
    rear=[]
    right0=[]
    right1=[]
    forward=[]
    right2=[]
    right3=[]
    right4=[]
    right5=[]
    Target1=[]
    Target2=[]
    Target3=[]
    Target4=[]
    step_num = 0

    vehicles_tau=[random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27)]
    def __init__(self, sumoBinary, net_file: str, cfg_file: str,  veh:str, use_gui: bool = True,
            begin_time: int =0, step_length: int = 0.01):
        # class attributes
        self.sumoBinary = sumoBinary
        self.net = net_file
        self.sumocfg = cfg_file
        self.use_gui = use_gui
        self.step_length = step_length
        self.veh = veh
        self.done = False
        self.episode = 0 # # of run time 
        self.begin_time = begin_time
        self.action_space = [0, 1, 2, 3, 4] # 0: LK_const_vel, 1: LK_accel, 2: LK_decel, 3: LC_left, 4: LC_right
        #self.action_space = [LK_const_vel,LK_accel, LK_decel,LC_left_const_vel, LC_right_const_vel, LC_left_accel, LC_left_decel, LC_right_accel, LC_left_decel]
        self.n_actions = len(self.action_space)
        
        self.sumo =traci
        self.LC_succeed_num = 0
        self.collision_num = 0

        


    def start_simulation(self):

        sumo_cmd = [self.sumoBinary,
            '-c', self.sumocfg,
            '--start','true',
            '--route-files','/home/mds/Desktop/highway_episodic/highway_episodic2.rou.xml',
            '--gui-settings-file','/home/mds/Desktop/highway_episodic/viewsettings.xml',
            '--lanechange.duration', '3',
            # '--collision.action', 'warn',
            #  '--collision.stoptime','5',
            '--collision.action', 'remove',
            '--collision.mingap-factor', '0',
            '--no-warnings',
            #  '--time-to-teleport','10',
            # '--collision-output','colliderSpeed',
            '--step-length', str(self.step_length),
            '--no-step-log',
            '--quit-on-end','true']
        
        self.sumo.start(sumo_cmd)

    def reset(self):

        #initiate values of class rlEnv.
        rlEnv.lane_buffer_ego = 0
        rlEnv.ego_LC_success = False
        rlEnv.ego_LC_start = False
        rlEnv.ego_LC_completed = False
        rlEnv.Space1=[]
        rlEnv.Space2=[]
        rlEnv.Space3=[]
        rlEnv.Space4=[]
        rlEnv.integral_clearance_e=[0,0,0,0,0]
        rlEnv.integral_velocity_e=[0,0,0,0,0]
        rlEnv.pre_error_clearance = [0,0,0,0,0]
        rlEnv.pre_error_velocity = [0,0,0,0,0]
        rlEnv.ego_control=[]
        rlEnv.ego=[]
        rlEnv.left0=[]
        rlEnv.left1=[]
        rlEnv.left2=[]
        rlEnv.left3=[]
        rlEnv.left4=[]
        rlEnv.left5=[]
        rlEnv.left6=[]
        rlEnv.left7=[]
        rlEnv.rear=[]
        rlEnv.right0=[]
        rlEnv.right1=[]
        rlEnv.forward=[]
        rlEnv.right2=[]
        rlEnv.right3=[]
        rlEnv.right4=[]
        rlEnv.right5=[]
        rlEnv.Target1=[]
        rlEnv.Target2=[]
        rlEnv.Target3=[]
        rlEnv.Target4=[]
        rlEnv.step_num = 0
        rlEnv.vehicles_tau=[random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27)]


        # if self.episode!=0: 
        #     self.sumo.close()
        #     sys.stdout.flush()
        # ego_rand = random.uniform(50,115)
        ego_initial_v = random.uniform(21,23)
        forward_initial_v = random.uniform(21,23)
        rear_initial_v = random.uniform(27.77,33.33)
        left_initial_v =[random.uniform(22.5,23),random.uniform(22.5,23),random.uniform(22.5,23),random.uniform(22.5,23),random.uniform(22.5,23),random.uniform(22.5,23),random.uniform(22.5,23),random.uniform(22.5,23)]
        right_initial_v =[random.uniform(22.5,23),random.uniform(22.5,23),random.uniform(22.5,23),random.uniform(22.5,23),random.uniform(22.5,23),random.uniform(22.5,23)]
        ego_rand = random.uniform(70,100)
        forward_rand = random.uniform(150,160)
        r0 =random.uniform(0,31)
        l0 =random.uniform(0,31)
        r_delta1 = random.uniform(31,40)
        r_delta2 = random.uniform(31,40)
        r_delta3 = random.uniform(31,40)
        r_delta4 = random.uniform(31,40)
        r_delta5 = random.uniform(31,40)
        l_delta1 = random.uniform(31,40)
        l_delta2 = random.uniform(31,40)
        l_delta3 = random.uniform(31,40)
        l_delta4 = random.uniform(31,40)
        l_delta5 = random.uniform(31,40)
        l_delta6 = random.uniform(31,40)
        l_delta7 = random.uniform(31,40)
        r1 = r0 + r_delta1
        r2 = r1 + r_delta2
        r3 = r2 + r_delta3
        r4 = r3 + r_delta4
        r5 = r4 + r_delta5
        l1 = l0 + l_delta1
        l2 = l1 + l_delta2
        l3 = l2 + l_delta3
        l4 = l3 + l_delta4
        l5 = l4 + l_delta5
        l6 = l5 + l_delta6
        l7 = l6 + l_delta7
        self.done = False
        self.start_simulation()
        self.episode+=1
        print('episode : ',self.episode)
        traci.simulationStep()
        traci.vehicle.add(vehID='car.forward',routeID='route0',typeID="car",depart='now',departLane='1',departPos=str(forward_rand), departSpeed=str(forward_initial_v), arrivalLane='current', arrivalPos='max', arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0, personNumber=0)
        traci.vehicle.add(vehID='ego',routeID='route0',typeID="ego_car",depart='now',departLane='1',departPos=str(ego_rand), departSpeed=str(ego_initial_v), arrivalLane='current', arrivalPos='max', arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0, personNumber=0)
        traci.vehicle.add(vehID='accel.rear',routeID='route0',typeID="accel_truck",depart='now',departLane='1',departPos='0', departSpeed=str(rear_initial_v), arrivalLane='current', arrivalPos='max', arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0, personNumber=0)
        

        traci.vehicle.add(vehID='car.left0',routeID='route0',typeID="car",depart='now',departLane='2',departPos=str(l0), departSpeed=str(left_initial_v[0]), arrivalLane='current', arrivalPos='max', arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0, personNumber=0)
        traci.vehicle.add(vehID='car.left1',routeID='route0',typeID="car",depart='now',departLane='2',departPos=str(l1), departSpeed=str(left_initial_v[1]), arrivalLane='current', arrivalPos='max', arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0, personNumber=0)
        traci.vehicle.add(vehID='car.left2',routeID='route0',typeID="car",depart='now',departLane='2',departPos=str(l2), departSpeed=str(left_initial_v[2]), arrivalLane='current', arrivalPos='max', arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0, personNumber=0)
        traci.vehicle.add(vehID='car.left3',routeID='route0',typeID="car",depart='now',departLane='2',departPos=str(l3), departSpeed=str(left_initial_v[3]), arrivalLane='current', arrivalPos='max', arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0, personNumber=0)
        traci.vehicle.add(vehID='car.left4',routeID='route0',typeID="car",depart='now',departLane='2',departPos=str(l4), departSpeed=str(left_initial_v[4]), arrivalLane='current', arrivalPos='max', arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0, personNumber=0)
        traci.vehicle.add(vehID='car.left5',routeID='route0',typeID="car",depart='now',departLane='2',departPos=str(l5), departSpeed=str(left_initial_v[5]), arrivalLane='current', arrivalPos='max', arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0, personNumber=0)
        traci.vehicle.add(vehID='car.left6',routeID='route0',typeID="car",depart='now',departLane='2',departPos=str(l6), departSpeed=str(left_initial_v[6]), arrivalLane='current', arrivalPos='max', arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0, personNumber=0)
        traci.vehicle.add(vehID='car.left7',routeID='route0',typeID="car",depart='now',departLane='2',departPos=str(l7), departSpeed=str(left_initial_v[7]), arrivalLane='current', arrivalPos='max', arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0, personNumber=0)

        traci.vehicle.add(vehID='car.right0',routeID='route0',typeID="car",depart='now',departLane='0',departPos=str(r0), departSpeed=str(right_initial_v[0]), arrivalLane='current', arrivalPos='max', arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0, personNumber=0)
        traci.vehicle.add(vehID='car.right1',routeID='route0',typeID="car",depart='now',departLane='0',departPos=str(r1), departSpeed=str(right_initial_v[1]), arrivalLane='current', arrivalPos='max', arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0, personNumber=0)
        traci.vehicle.add(vehID='car.right2',routeID='route0',typeID="car",depart='now',departLane='0',departPos=str(r2), departSpeed=str(right_initial_v[2]), arrivalLane='current', arrivalPos='max', arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0, personNumber=0)
        traci.vehicle.add(vehID='car.right3',routeID='route0',typeID="car",depart='now',departLane='0',departPos=str(r3), departSpeed=str(right_initial_v[3]), arrivalLane='current', arrivalPos='max', arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0, personNumber=0)
        traci.vehicle.add(vehID='car.right4',routeID='route0',typeID="car",depart='now',departLane='0',departPos=str(r4), departSpeed=str(right_initial_v[4]), arrivalLane='current', arrivalPos='max', arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0, personNumber=0)
        traci.vehicle.add(vehID='car.right5',routeID='route0',typeID="car",depart='now',departLane='0',departPos=str(r5), departSpeed=str(right_initial_v[5]), arrivalLane='current', arrivalPos='max', arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0, personNumber=0)
        
        traci.simulationStep()
        return self.state('ego')

        
    def end(self):
        rlEnv.ego_LC_start=False
        rlEnv.ego_LC_completed=False
        traci.close()
        sys.stdout.flush()
    
    def save_csv(self):
         
        col=['time','vehicle_id','x','y','v_x','v_y','a_x','a_y','heading_angle']
        col_target = ['time','num','clearance','x','v_x']
        col_control = ['time','desire_a_x','desire_a_y']
        df_ego = pd.DataFrame(rlEnv.ego,columns=col)
        df_rear = pd.DataFrame(rlEnv.rear,columns=col)
        df_forward = pd.DataFrame(rlEnv.forward,columns=col)
        df_left0 = pd.DataFrame(rlEnv.left0,columns=col)
        df_left1 = pd.DataFrame(rlEnv.left1,columns=col)
        df_left2 = pd.DataFrame(rlEnv.left2,columns=col)
        df_left3 = pd.DataFrame(rlEnv.left3,columns=col)
        df_left4 = pd.DataFrame(rlEnv.left4,columns=col)
        df_left5 = pd.DataFrame(rlEnv.left5,columns=col)
        df_left6 = pd.DataFrame(rlEnv.left6,columns=col)
        df_left7 = pd.DataFrame(rlEnv.left7,columns=col)
        df_right0 = pd.DataFrame(rlEnv.right0,columns=col)
        df_right1 = pd.DataFrame(rlEnv.right1,columns=col)
        df_right2 = pd.DataFrame(rlEnv.right2,columns=col)
        df_right3 = pd.DataFrame(rlEnv.right3,columns=col)
        df_right4 = pd.DataFrame(rlEnv.right4,columns=col)
        df_right5 = pd.DataFrame(rlEnv.right5,columns=col)
        df_target1 = pd.DataFrame(rlEnv.Target1,columns=col_target)
        df_target2 = pd.DataFrame(rlEnv.Target2,columns=col_target)
        df_target3 = pd.DataFrame(rlEnv.Target3,columns=col_target)
        df_target4 = pd.DataFrame(rlEnv.Target4,columns=col_target)
        df_ego_control = pd.DataFrame(rlEnv.ego_control,columns=col_control)
        os.chdir('/home/mds/Desktop/highway_episodic/DQN/data_log/2')
        df_ego.to_csv('log_data_ego.csv')
        df_rear.to_csv('log_data_rear.csv')
        df_forward.to_csv('log_data_forward.csv')
        df_left0.to_csv('log_data_left0.csv')
        df_left1.to_csv('log_data_left1.csv')
        df_left2.to_csv('log_data_left2.csv')
        df_left3.to_csv('log_data_left3.csv')
        df_left4.to_csv('log_data_left4.csv')
        df_left5.to_csv('log_data_left5.csv')
        df_left6.to_csv('log_data_left6.csv')
        df_left7.to_csv('log_data_left7.csv')
        df_right0.to_csv('log_data_right0.csv')
        df_right1.to_csv('log_data_right1.csv')
        df_right2.to_csv('log_data_right2.csv')
        df_right3.to_csv('log_data_right3.csv')
        df_right4.to_csv('log_data_right4.csv')
        df_right5.to_csv('log_data_right5.csv')
        df_target1.to_csv('log_data_target1.csv')
        df_target2.to_csv('log_data_target2.csv')
        df_target3.to_csv('log_data_target3.csv')
        df_target4.to_csv('log_data_target4.csv')
        df_ego_control.to_csv('log_data_ego_control.csv')
    
    def ego_collision_happened(self):
        if len(traci.simulation.getCollidingVehiclesIDList()) !=0:
            # print('negative reward')
            log = traci.simulation.getCollisions()
            if str(log[0]).split(',')[1]== ' victim=ego' or str(log[0]).split(',')[0]== 'Collision(collider=ego': 
                self.done = True
                return True
            else:
                return False

    def __ego_vehicle_LC_start(self):
        vehs = traci.vehicle.getIDList()
        if 'ego' in vehs:
            cur_lane = traci.vehicle.getLaneIndex('ego')

            # print('cur_lane : ',cur_lane)
            last_lane = rlEnv.lane_buffer_ego
            if(last_lane != cur_lane):
                # traci.vehicle.setSpeedMode(id,0b011000)
                # traci.vehicle.setLaneChangeMode(id,0b101010101010)
                traci.vehicle.setLaneChangeMode('ego',0)
                # ACC 모드로 변경해야함.

                
                return True
            else:
                return False
    
    def __ego_vehicle_LC_completed(self):
        vehs = traci.vehicle.getIDList()
        if 'ego' in vehs:
            if rlEnv.ego_LC_start == True:
                if (traci.vehicle.getLaneIndex('ego')== 2 or traci.vehicle.getLaneIndex('ego')== 0) and traci.vehicle.getAngle('ego') >= 89.8 and traci.vehicle.getAngle('ego') <= 90.2 and np.abs(traci.vehicle.getLateralLanePosition('ego'))<=0.2:                    
                    rlEnv.ego_LC_completed =True
                    self.done = True
                    print('Lanechange completed')
                    if self.done:
                        print('done = True')
                    return True
                else:
                    return False
    def __get_a_x(self,id):
        dt = self.step_length
        if(len(rlEnv.ego)!=0):
            v_x_cur = (traci.vehicle.getPosition(id)[0] - rlEnv.ego[-1][2])/dt
            a_x_cur = (v_x_cur -rlEnv.ego[-1][4])/dt
        else:
            v_x_cur=0
            a_x_cur=0
        return a_x_cur
    def __get_v_x(self,id):
        dt = self.step_length
        if(len(rlEnv.ego)!=0):
            v_x_cur = (traci.vehicle.getPosition(id)[0] - rlEnv.ego[-1][2])/dt
        else:
            v_x_cur=0
        return v_x_cur
    
    def __get_a_y(self,id):
        dt = self.step_length
        if(len(rlEnv.ego)!=0):
            v_y_cur = (traci.vehicle.getPosition(id)[1] - rlEnv.ego[-1][3])/dt
            a_y_cur = (v_y_cur -rlEnv.ego[-1][5])/dt
        else:
            v_y_cur=0
            a_y_cur=0
        return a_y_cur
    def __get_v_y(self,id):
        dt = self.step_length
        if(len(rlEnv.ego)!=0):
            v_y_cur = (traci.vehicle.getPosition(id)[1] - rlEnv.ego[-1][3])/dt
        else:
            v_y_cur=0
        return v_y_cur
    def __set_a_x_possible(self,id,a_x_des): #id =='ego'
        jerk = 5
        dt = self.step_length
        
        # if(len(rlEnv.ego)!=0):            
        #     v_x_cur = (traci.vehicle.getPosition(id)[0] - rlEnv.ego[-1][2])/dt
        #     a_x_cur = (v_x_cur -rlEnv.ego[-1][4])/dt
        # else:
        #     v_x_cur= traci.vehicle.getSpeed(id)
        #     a_x_cur = traci.vehicle.getAcceleration(id)
        v_x_cur= traci.vehicle.getSpeed(id)
        a_x_cur = traci.vehicle.getAcceleration(id)
        if a_x_cur+jerk*dt < a_x_des:
            # print('a_x_cur+jerk*dt: ',a_x_cur+jerk*dt)
            traci.vehicle.setAcceleration(id,a_x_cur+jerk*dt,1)
            return a_x_cur+jerk*dt
        elif a_x_cur-jerk*dt > a_x_des:
            # print('a_x_cur-jerk*dt: ',a_x_cur-jerk*dt)
            traci.vehicle.setAcceleration(id,a_x_cur-jerk*dt,1)
            return a_x_cur-jerk*dt
        else:
            # print('a_x_des: ',a_x_des )
            traci.vehicle.setAcceleration(id,a_x_des,1)
            return a_x_des
        
    def __set_a_y_possible(self,id,a_y_des): #id =='ego'
        jerk = 5
        dt = self.step_length
        if(len(rlEnv.ego)!=0):
            v_y_cur = (traci.vehicle.getPosition(id)[1] - rlEnv.ego[-1][3])/dt
            a_y_cur = (v_y_cur -rlEnv.ego[-1][5])/dt
            # print("rlEnv.ego[-1][5]): ",rlEnv.ego[-1][5])
            # print("rlEnv.ego[-1][3]: ",rlEnv.ego[-1][3])
        else:
            v_y_cur=0
            a_y_cur=0
        #     print("rlEnv.ego[-1][5]): None")
        #     print("rlEnv.ego[-1][3]: None")
        # print("a_y_cur: ",a_y_cur)
        # print("v_y_cur: ",v_y_cur)
        
        if a_y_cur+jerk*dt < a_y_des:
            return a_y_cur+jerk*dt
        elif a_y_cur-jerk*dt > a_y_des:
            return a_y_cur-jerk*dt
        else:
            return a_y_des

    def __set_v_possible(self,id,v_des):
        jerk = 5
        dt = self.step_length
        if(len(rlEnv.ego)!=0):
            v_y_cur = (traci.vehicle.getPosition(id)[1] - rlEnv.ego[-1][3])/dt
            a_y_cur = (v_y_cur -rlEnv.ego[-1][5])/dt
        else:
            v_y_cur=0
            a_y_cur=0
        
        if(v_y_cur+a_y_cur*dt+1/2*jerk*dt**2<v_des):
            return v_y_cur+a_y_cur*dt+1/2*jerk*dt**2
        elif(v_y_cur-a_y_cur*dt-1/2*jerk*dt**2>v_des):
            return v_y_cur+a_y_cur*dt-1/2*jerk*dt**2
        else:
            return v_des

            

    def __ACC_target_id(self,id,action): #action=5 left , action=6 right #nearest forward vehicle in target lane and current lane !!! This code only considered 3 lane.
        if id:
            cur_lane = traci.vehicle.getLaneIndex(id)
            if cur_lane == 0:
                if action == 5:
                    target_vehicles = []
                    traci.vehicle.subscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 120.0, [tc.VAR_POSITION])
                    for v_id in traci.vehicle.getContextSubscriptionResults(str(id)):
                        if v_id != id and (traci.vehicle.getLaneIndex(v_id) == 1 or traci.vehicle.getLaneIndex(v_id) == 0) and traci.vehicle.getPosition(v_id)[0]>traci.vehicle.getPosition(id)[0]:
                            target_vehicles.append([traci.vehicle.getPosition(v_id)[0],v_id])
                    target_vehicles.sort()
                    traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 120.0)
                    # print(target_vehicles)
                    return target_vehicles[0][1]

                elif action == 6:
                    target_vehicles = []
                    traci.vehicle.subscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 120.0, [tc.VAR_POSITION])
                    for v_id in traci.vehicle.getContextSubscriptionResults(str(id)):
                        if v_id != id and (traci.vehicle.getLaneIndex(v_id) == 0 or traci.vehicle.getLaneIndex(v_id) == 1) and traci.vehicle.getPosition(v_id)[0]>traci.vehicle.getPosition(id)[0]:
                            target_vehicles.append([traci.vehicle.getPosition(v_id)[0],v_id])
                    target_vehicles.sort()
                    traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 120.0)
                    # print(target_vehicles)
                    return target_vehicles[0][1]
            elif cur_lane == 1:
                if action == 5:
                    target_vehicles=[]
                    traci.vehicle.subscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 120.0, [tc.VAR_POSITION])
                    for v_id in traci.vehicle.getContextSubscriptionResults(str(id)):
                        if v_id != id and (traci.vehicle.getLaneIndex(v_id) == 2 or traci.vehicle.getLaneIndex(v_id) == 1) and traci.vehicle.getPosition(v_id)[0]>traci.vehicle.getPosition(id)[0]:
                            target_vehicles.append([traci.vehicle.getPosition(v_id)[0],v_id])
                    target_vehicles.sort()
                    traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 120.0)
                    # print(target_vehicles)
                    return target_vehicles[0][1]
                
                elif action == 6:
                    target_vehicles=[]
                    traci.vehicle.subscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 120.0, [tc.VAR_POSITION])
                    for v_id in traci.vehicle.getContextSubscriptionResults(str(id)):
                        if v_id != id and (traci.vehicle.getLaneIndex(v_id) == 0 or traci.vehicle.getLaneIndex(v_id) == 1) and traci.vehicle.getPosition(v_id)[0]>traci.vehicle.getPosition(id)[0]:
                            target_vehicles.append([traci.vehicle.getPosition(v_id)[0],v_id])
                    target_vehicles.sort()
                    traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 120.0)
                    # print(target_vehicles)
                    return target_vehicles[0][1]
            elif cur_lane == 2:
                if action == 5:
                    target_vehicles = []
                    traci.vehicle.subscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 120.0, [tc.VAR_POSITION])
                    for v_id in traci.vehicle.getContextSubscriptionResults(str(id)):
                        if v_id != id and (traci.vehicle.getLaneIndex(v_id) == 2 or traci.vehicle.getLaneIndex(v_id) == 1) and traci.vehicle.getPosition(v_id)[0]>traci.vehicle.getPosition(id)[0]:
                            target_vehicles.append([traci.vehicle.getPosition(v_id)[0],v_id])
                    target_vehicles.sort()
                    traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 120.0)
                    # print(target_vehicles)
                    return target_vehicles[0][1]
                elif action == 6:
                    target_vehicles=[]
                    traci.vehicle.subscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 120.0, [tc.VAR_POSITION])
                    for v_id in traci.vehicle.getContextSubscriptionResults(str(id)):
                        if v_id != id and (traci.vehicle.getLaneIndex(v_id) == 1 or traci.vehicle.getLaneIndex(v_id) == 2) and traci.vehicle.getPosition(v_id)[0]>traci.vehicle.getPosition(id)[0]:
                            target_vehicles.append([traci.vehicle.getPosition(v_id)[0],v_id])
                    target_vehicles.sort()
                    traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 120.0)
                    # print(target_vehicles)
                    return target_vehicles[0][1]
            else:
                print("Can't find ACC Target, cur_lane, action : ",cur_lane,", ",action)
            
    def __K1(self, v): #optimal gain (input v : velocity output K1 :optimal gain upper bound)
        if(v>=0 and v<=11.1111):
            return 0.35*1
        elif(v>11.1111 and v<=19.4444):
            return (-0.018*v+0.55)*1
        else:
            return 0.2*1

    def __K2(self, v):#optimal gain (input v : velocity output K2 :optimal gain lower bound)
        if(v>=0 and v<=11.1111):
            return -1.22*1
        elif(v>11.1111 and v<=19.4444):
            return (0.0444*v-1.7133)*1
        else:
            return -0.85*1
    def __a_max(self, v):
        if(v>=0 and v<=11.1111):
            return 1.5
        elif(v>11.1111 and v<=19.4444):
            return -0.06*v+2.1667
        else:
            return 1
    def __a_min(self, v):
        if(v>=0 and v<=11.1111):
            return -2
        elif(v>11.1111 and v<=19.4444):
            return 0.12*v-3.3333
        else:
            return -1
        
    def __a_desire(self, v_c,v_p,c_d,c): # desire acceleration by ACC, v_c : ego_vehicle's velocity, v_p : preceding_vehicle's velocity, c_d : desire clearance, c = actual clearance
        a_d = -self.__K1(v_c)*(c_d-c) - self.__K2(v_c)*(v_p-v_c)
        if(a_d > self.__a_max(v_c)):
            return self.__a_max(v_c)
        elif(a_d <= self.__a_max(v_c) and a_d >= self.__a_min(v_c)):
            return a_d
        elif(a_d< self.__a_min(v_c)):
            return self.__a_min(v_c)
    def __ego_acc(self, v_c,v_p,c_d,c): # desire acceleration by ACC, v_c : ego_vehicle's velocity, v_p : preceding_vehicle's velocity, c_d : desire clearance, c = actual clearance
        a_d = -self.__K1(v_c)*(c_d-c) - self.__K2(v_c)*(v_p-v_c)
        if(a_d > 3):
            return 3
        elif(a_d <= 3 and a_d >= -5):
            return a_d
        elif(a_d< -5):
            return -5
        
    def __ego_a_desire(self, v_c,v_t,c_d,c,space_num): # desire acceleration by ACC, v_c : ego_vehicle's velocity, v_t : target velocity, c_d : desire clearance, c = actual clearance
        P=1
        P2=1
        # Ki=0.04
        # Ki2 = 0.6
        # Kd = 0.001
        # Kd2 = 0.3
        Ki=0
        Ki2 = 0
        Kd = 0.005
        Kd2 = 0.7
        rlEnv.integral_clearance_e[space_num] += Ki*(c_d -c)*self.step_length
        rlEnv.integral_velocity_e[space_num] += Ki2*(v_t-v_c)*self.step_length

        clearance_error = (c_d -c)
        velocity_error = (v_t-v_c)
        if len(rlEnv.pre_error_velocity)>0 and len(rlEnv.pre_error_velocity)>0:
            a_d = -P*self.__K1(v_c)*(c_d-c) - P2*self.__K2(v_c)*(v_t-v_c) -rlEnv.integral_clearance_e[space_num] +rlEnv.integral_velocity_e[space_num] -Kd*(clearance_error-rlEnv.pre_error_clearance[space_num])/self.step_length +Kd2*(velocity_error-rlEnv.pre_error_velocity[space_num])
        else:
            a_d = -P*self.__K1(v_c)*(c_d-c) - P2*self.__K2(v_c)*(v_t-v_c) -rlEnv.integral_clearance_e[space_num] +rlEnv.integral_velocity_e[space_num]
        # a_d = -self.__K1(v_c)*(c_d-c) - self.__K2(v_c)*(v_t-v_c)
        if(a_d >3):
            return 3
        elif(a_d <= 3 and a_d >= -5):
            return a_d
        elif(a_d< -5):
            return -5
        rlEnv.pre_error_clearance[space_num] = (c_d -c)
        rlEnv.pre_error_velocity[space_num] = (v_t-v_c)

        
    def __ego_a_desire2(self, v_c,v_t,c_d,c,space_num): # desire acceleration by ACC, v_c : ego_vehicle's velocity, v_t : target velocity, c_d : desire clearance, c = actual clearance
        # Ki=0.0021
        # Ki2 = 0.09
        # Kd = 0.0021
        # Kd2 = 0.99
        # #action4
        # Ki=0.001
        # Ki2 = 0.04
        # Kd = 0.002
        # Kd2 = 0.9
        #action3
        Ki=0.001
        Ki2 = 0.05
        Kd = 0.001
        Kd2 = 0.99
        rlEnv.integral_clearance_e[space_num] += Ki*(c_d -c)*self.step_length
        rlEnv.integral_velocity_e[space_num] += Ki2*(v_t-v_c)*self.step_length

        clearance_error = (c_d -c)
        velocity_error = (v_t-v_c)
        if len(rlEnv.pre_error_velocity)>0 and len(rlEnv.pre_error_velocity)>0:
            a_d = -self.__K1(v_c)*(c_d-c) - self.__K2(v_c)*(v_t-v_c) -rlEnv.integral_clearance_e[space_num] +rlEnv.integral_velocity_e[space_num] -Kd*(clearance_error-rlEnv.pre_error_clearance[space_num])/self.step_length +Kd2*(velocity_error-rlEnv.pre_error_velocity[space_num])
        else:
            a_d = -self.__K1(v_c)*(c_d-c) - self.__K2(v_c)*(v_t-v_c) -rlEnv.integral_clearance_e[space_num] +rlEnv.integral_velocity_e[space_num]
        # a_d = -self.__K1(v_c)*(c_d-c) - self.__K2(v_c)*(v_t-v_c)
        if(a_d >3):
            return 3
        elif(a_d <= 3 and a_d >= -5):
            return a_d
        elif(a_d< -5):
            return -5
        rlEnv.pre_error_clearance[space_num] = (c_d -c)
        rlEnv.pre_error_velocity[space_num] = (v_t-v_c)

    def __a_desire_with_speed_limit(self, v_c,v_p,c_d,c,speed_limit):
        v_define = min(speed_limit,v_p) # 앞차 속도와 규정 속도 중 작은 것을 따름.
        
        # if(v_c > speed_limit):
        #     a_d = - self.__K2(v_c)*(speed_limit-v_c)
        # else:
        a_d = -self.__K1(v_c)*(c_d-c) - self.__K2(v_c)*(v_define-v_c)
        if(a_d > self.__a_max(v_c)):
            return self.__a_max(v_c)
        elif(a_d <= self.__a_max(v_c) and a_d >= self.__a_min(v_c)):
            return a_d
        elif(a_d< self.__a_min(v_c)):
            return self.__a_min(v_c)
    def __setvehiclestate(self, id):
            
        goal_v_x = random.uniform(27.77,33.33)
        goal_j_x = random.uniform(1,3)
        max_a_x = random.uniform(5,10)
        min_a_x = random.uniform(1,3)

        dt = self.step_length
        last_v_x = rlEnv.rear[-1][4]
        last_a_x = rlEnv.rear[-1][6]  
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

    def __set_a(self,id,a_des): #return 값은 jerk 값을 고려한 현실적인 가속도 값을 반환한다. # SUMO에 가속도를 입력한다.
        jerk = 3
        dt = self.step_length
        a_cur = traci.vehicle.getAcceleration(id)
        if(a_cur+jerk*dt<a_des):
            traci.vehicle.setAcceleration(id,a_cur+jerk*dt,1)
            return a_cur+jerk*dt
        elif(a_cur-jerk*dt>a_des):
            traci.vehicle.setAcceleration(id,a_cur-jerk*dt,1)
            return a_cur-jerk*dt
        else:
            traci.vehicle.setAcceleration(id,a_des,1)
            return a_des
    def __set_ego_a(self,id,a_des): #return 값은 jerk 값을 고려한 현실적인 가속도 값을 반환한다. # SUMO에 가속도를 입력한다.
        jerk = 3
        dt = self.step_length
        a_cur = traci.vehicle.getAcceleration(id)
        # print("::::::::::a_des: ",a_des)
        # print("::::::::::a_cur: ",a_cur)
        if(a_cur+jerk*dt<a_des):
            traci.vehicle.setAcceleration(id,a_cur+jerk*dt,1)
            return a_cur+jerk*dt
        elif(a_cur-jerk*dt>a_des):
            traci.vehicle.setAcceleration(id,a_cur-jerk*dt,1)
            return a_cur-jerk*dt
        else:
            traci.vehicle.setAcceleration(id,a_des,1)
            return a_des
        
    def __set_v(self,id,v_des): # v_des에 따라 a_des을 결정하고 jerk 값이 고려된  속도를 시스템에 넣음.
        dt = self.step_length
        Kp =0.9 # p gain
        # Kp=0.01
        v_cur = traci.vehicle.getSpeed(id)
        if (v_des - v_cur)> self.__a_max(v_cur):
            a_des = self.__a_max(v_cur)
        elif (v_des - v_cur) <= self.__a_max(v_cur) and (v_des - v_cur)>self.__a_min(v_cur): #a_max(20) = 1,a_min(20) = -1
            a_des = (v_des - v_cur)*Kp
        else:
            a_des = self.__a_min(v_cur)
        # a = set_a(id,a_des)
        self.__set_a(id,a_des)
    
    def __set_ego_v(self,id,v_des): # v_des에 따라 a_des을 결정하고 jerk 값이 고려된  속도를 시스템에 넣음.
        dt = self.step_length
        Kp =1 # p gain
        a_max = 3
        a_min = -5
        # Kp=0.01
        v_cur = traci.vehicle.getSpeed(id)
        if (v_des - v_cur)>= a_max:
            a_des = a_max
        elif (v_des - v_cur) < a_max and (v_des - v_cur)>a_min: 
            a_des = (v_des - v_cur)*Kp
        else:
            a_des = a_min
        # a = set_a(id,a_des)
        self.__set_ego_a(id,a_des)
        return a_des

    def __find_followers(self,ego_id, vehicle_id,LEFT_FOLLOWER,RIGHT_FOLLOWER):
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
        
    def state(self,id):
        left_vehicles=[] #[|ego_x_pos - x_pos|, id]
        right_vehicles=[] #[|ego_x_pos - x_pos|, id]
        left_near_vehicles=[] #[x_pos]
        right_near_vehicles =[] #[x_pos]
        
        left_near_spaces=[] #[space_x_pos, space_size]
        right_near_spaces=[] #[space_x_pos, space_size]
        ego_to_left_spaces=[]
        ego_to_right_spaces=[]
        
        states = [] # ego_forward_clearance, ego_rear_clearance, ego_lane_num, ego_to_left_spaces, ego_to_right_spaces, left_space_size, right_space_size
        if id:
            traci.vehicle.subscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 120.0, [tc.VAR_POSITION])
            # print(id,' subscribeContext')
            for v_id in traci.vehicle.getContextSubscriptionResults(str(id)):
                if(traci.vehicle.getLaneIndex(v_id)==2) and not (v_id=='ego'):
                    #left_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),v_id]) -> id check
                    left_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),traci.vehicle.getPosition(v_id)[0],v_id])
                if(traci.vehicle.getLaneIndex(v_id)==0)and not (v_id=='ego') :
                    right_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),traci.vehicle.getPosition(v_id)[0],v_id])
                    
            
            
            left_vehicles.sort()
            right_vehicles.sort()
            # if len(left_vehicles) >=3:
            # print('left_near_vehicles: ',left_vehicles[:2])
            # print('right_near_vehicles: ',right_vehicles[:2])
            
            for i in range(3): # getting near 3 vehicles x_pos each left and right
                left_near_vehicles.append([left_vehicles[:3][i][1],left_vehicles[:3][i][2]])# 1: left vehicle position 2: left vehicle id
                right_near_vehicles.append([right_vehicles[:3][i][1],right_vehicles[:3][i][2]])# 1: right vehicle position 2: right vehicle id
                
            left_near_vehicles.sort()
            right_near_vehicles.sort()
            # print('left_near_vehicles: ',left_near_vehicles)
            # print('right_near_vehicles: ',right_near_vehicles)
            # print('left_near_vehicles[0][0]: ', left_near_vehicles[0][0])
            # print('left_near_vehicles[0][1]: ', left_near_vehicles[0][1])
            # print('left_near_vehicles[1][0]: ', left_near_vehicles[1][0])
            # print('left_near_vehicles[1][1]: ', left_near_vehicles[1][1])
            ego_to_left_first = left_near_vehicles[2][0] - traci.vehicle.getPosition('ego')[0] # left first: 왼쪽 차로에 있는 ego vehicle에서 가까운 3대 중 앞에서 첫번째 차량까지 거리
            ego_to_left_second = left_near_vehicles[1][0] -traci.vehicle.getPosition('ego')[0] # left second: 왼쪽 차로에 있는 ego vehicle에서 가까운 3대 중 앞에서 두번째 차량까지 거리
            ego_to_left_third = left_near_vehicles[0][0] - traci.vehicle.getPosition('ego')[0] # left third: 왼쪽 차로에 있는 ego vehicle에서 가까운 3대 중 앞에서 세번째 차량까지 거리
            ego_to_right_first = right_near_vehicles[2][0] - traci.vehicle.getPosition('ego')[0] # right third: 오른쪽 차로에 있는 ego vehicle에서 가까운 3대 중 앞에서 첫번째 차량까지 거리
            ego_to_right_second = right_near_vehicles[1][0] - traci.vehicle.getPosition('ego')[0]# right third: 오른쪽 차로에 있는 ego vehicle에서 가까운 3대 중 앞에서 두번째 차량까지 거리
            ego_to_right_third = right_near_vehicles[0][0] - traci.vehicle.getPosition('ego')[0] # right third: 오른쪽 차로에 있는 ego vehicle에서 가까운 3대 중 앞에서 세번째 차량까지 거리

            left_near_spaces.append([left_near_vehicles[1][0]+(left_near_vehicles[2][0]-left_near_vehicles[1][0])/2,left_near_vehicles[2][0]-left_near_vehicles[1][0]]) #ego vehicle 주위 두 개의 왼쪽 빈공간 중 앞에서 첫번째 빈공간의 위치. 빈공간의 크기
            left_near_spaces.append([left_near_vehicles[0][0]+(left_near_vehicles[1][0]-left_near_vehicles[0][0])/2,left_near_vehicles[1][0]-left_near_vehicles[0][0]]) #ego vehicle 주위 두 개의 왼쪽 빈공간 중 앞에서 두번째 빈공간의 위치, 빈공간의 크기
            right_near_spaces.append([right_near_vehicles[1][0]+(right_near_vehicles[2][0]-right_near_vehicles[1][0])/2, right_near_vehicles[2][0]-right_near_vehicles[1][0]]) #ego vehicle 주위 두 개의 오른쪽 빈공간 중 앞에서 첫번째 빈공간의 위치. 빈공간의 크기
            right_near_spaces.append([right_near_vehicles[0][0]+(right_near_vehicles[1][0]-right_near_vehicles[0][0])/2, right_near_vehicles[1][0]-right_near_vehicles[0][0]]) #ego vehicle 주위 두 개의 오른쪽 빈공간 중 앞에서 두번째 빈공간의 위치, 빈공간의 크기
            

            ego_to_left_spaces.append(left_near_spaces[0][0]- traci.vehicle.getPosition('ego')[0])#ego vehicle에서 주위 두 개의 왼쪽 빈공간 중 앞에서 첫번째 빈공간의 중점까지 거리
            ego_to_left_spaces.append(left_near_spaces[1][0]- traci.vehicle.getPosition('ego')[0])#ego vehicle에서 주위 두 개의 왼쪽 빈공간 중 앞에서 두번째 빈공간의 중점까지 거리
            ego_to_right_spaces.append(right_near_spaces[0][0]-traci.vehicle.getPosition('ego')[0])#ego vehicle에서 주위 두 개의 오른쪽 빈공간 중 앞에서 첫번째 빈공간의 중점까지 거리
            ego_to_right_spaces.append(right_near_spaces[1][0]-traci.vehicle.getPosition('ego')[0])#ego vehicle에서 주위 두 개의 오른쪽 빈공간 중 앞에서 두번째 빈공간의 중점까지 거리

                
            if traci.vehicle.getLeader(id) is not None:
                leader_id,forward_clearance = traci.vehicle.getLeader(id)
                if forward_clearance>47:
                    forward_clearance =47
            else:
                forward_clearance =47
            if type(traci.vehicle.getFollower(id)) is not None and traci.vehicle.getFollower(id)[0] != '':
                follower_id, rear_clearance = traci.vehicle.getFollower(id) 
                
                if rear_clearance >47:
                    rear_clearance = 47
            else:
                rear_clearance =47
            rel_v_rear = traci.vehicle.getSpeed('ego') - traci.vehicle.getSpeed('accel.rear')
            rel_v_forward = traci.vehicle.getSpeed('ego') - traci.vehicle.getSpeed('car.forward')
            rel_v_left_forward = traci.vehicle.getSpeed('ego') - traci.vehicle.getSpeed(left_near_vehicles[1][1])
            rel_v_left_rear = traci.vehicle.getSpeed('ego') - traci.vehicle.getSpeed(left_near_vehicles[0][1])
            rel_v_right_forward = traci.vehicle.getSpeed('ego') - traci.vehicle.getSpeed(right_near_vehicles[1][1])
            rel_v_right_rear = traci.vehicle.getSpeed('ego') - traci.vehicle.getSpeed(right_near_vehicles[0][1])
            ego_cur_lane = traci.vehicle.getLaneIndex('ego')
            if len(rlEnv.ego)!=0:
                ego_y = rlEnv.ego[-1][3]
            else:
                ego_y = -4.8

            states.append(forward_clearance)
            states.append(rear_clearance)
                        
            states.append(ego_to_left_spaces[0])
            states.append(ego_to_left_spaces[1])
            states.append(ego_to_right_spaces[0])
            states.append(ego_to_right_spaces[1])

            states.append(left_near_spaces[0][1])
            states.append(left_near_spaces[1][1])
            states.append(right_near_spaces[0][1])
            states.append(right_near_spaces[1][1])

            states.append(ego_to_left_first)
            states.append(ego_to_left_second)
            states.append(ego_to_left_third)
            states.append(ego_to_right_first)
            states.append(ego_to_right_second)
            states.append(ego_to_right_third)

            states.append(rel_v_rear)
            states.append(rel_v_forward)
            states.append(rel_v_left_forward)
            states.append(rel_v_left_rear)
            states.append(rel_v_right_forward)
            states.append(rel_v_right_rear)
            states.append(ego_cur_lane)
            states.append(ego_y)
            # print('left_near_vehicles : ',left_near_vehicles)
            # print('right_near_vehicles : ',right_near_vehicles)
            # print('ego_x_pos : ',traci.vehicle.getPosition('ego')[0])
            # print('accel.rear : ',traci.vehicle.getPosition('accel.rear')[0])
            # print('left_near_spaces : ',left_near_spaces)
            # print('right_near_spaces : ',right_near_spaces)
                # print(v_id)
                # print(traci.vehicle.getLaneIndex(v_id))
                # left_follower_id = self.__find_followers(id, v,LEFT_FOLLOWER,RIGHT_FOLLOWER)[0]
                # right_follower_id = self.__find_followers(id, v,LEFT_FOLLOWER,RIGHT_FOLLOWER)[1]
            # print(states)
            traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 120.0)
            states = np.array(states)
            # states.reshape(6,1)
            # print('states',states)
        return states
    
    def __toward_biggest_empty_space_reward(self,id):
        left_vehicles=[] #[|ego_x_pos - x_pos|, id]
        right_vehicles=[] #[|ego_x_pos - x_pos|, id]
        left_near_vehicles=[] #[x_pos]
        right_near_vehicles =[] #[x_pos]
        
        left_near_spaces=[] #[space_x_pos, space_size]
        right_near_spaces=[] #[space_x_pos, space_size]
        
        chosen_space_size=0  ## need to check if it is right intialization
        chosen_space_number = 0 ## need to check if it is right intialization
        
        if id:
            traci.vehicle.subscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 120.0, [tc.VAR_POSITION])
            # print(id,' subscribeContext')
            for v_id in traci.vehicle.getContextSubscriptionResults(str(id)):
                if(traci.vehicle.getLaneIndex(v_id)==2) and not (v_id=='ego'):
                    #left_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),v_id]) -> id check
                    left_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),traci.vehicle.getPosition(v_id)[0]])
                if(traci.vehicle.getLaneIndex(v_id)==0) and not (v_id=='ego'):
                    right_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),traci.vehicle.getPosition(v_id)[0]])
            
            
            left_vehicles.sort()
            right_vehicles.sort()
            traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 120.0)
        for i in range(2): # getting near 3 vehicles x_pos each left and right
            left_near_vehicles.append(left_vehicles[:3][i][1])
            right_near_vehicles.append(right_vehicles[:3][i][1])
                
        left_near_vehicles.sort()
        right_near_vehicles.sort()
        
        left_near_spaces.append([left_near_vehicles[1],left_near_vehicles[0],left_near_vehicles[1]-left_near_vehicles[0]])
        right_near_spaces.append([right_near_vehicles[1],right_near_vehicles[0], right_near_vehicles[1]-right_near_vehicles[0]])
        
        empty_space_dictionary = {"L0":left_near_spaces[0],"R0":right_near_spaces[0]}

        empty_space_dictionary_sorted_by_space = sorted(empty_space_dictionary.items(), key=lambda x: x[1][2],reverse=True)

        dist_to_biggest_empty_space = np.abs(traci.vehicle.getPosition('ego')[0]-(empty_space_dictionary_sorted_by_space[0][1][1]+(empty_space_dictionary_sorted_by_space[0][1][0]-empty_space_dictionary_sorted_by_space[0][1][1])/2))
        # if(round(np.exp(-dist_to_biggest_empty_space),2)*100 <0):
        #     print('.........................Negative.............................')
        # return round(np.exp(-dist_to_biggest_empty_space),2)

        # 1차식
        # print("toward_biggest_space reward: ",round(-dist_to_biggest_empty_space/50 + 10,2))
        # return round(-dist_to_biggest_empty_space/50 + 10,2)

        # 2차식
        # print('dist: ',dist_to_biggest_empty_space)
        # print("toward_biggest_space reward: ",round(max([-1,-(dist_to_biggest_empty_space-15)**3/75]),2))
        
        return round(max([-1,-(dist_to_biggest_empty_space-15)**3/75]),3)


    def __chosen_space_size_reward(self,id):
        left_vehicles=[] #[|ego_x_pos - x_pos|, id]
        right_vehicles=[] #[|ego_x_pos - x_pos|, id]
        left_near_vehicles=[] #[x_pos]
        right_near_vehicles =[] #[x_pos]
        
        left_near_spaces=[] #[space_x_pos, space_size]
        right_near_spaces=[] #[space_x_pos, space_size]
        
        chosen_space_size=0  ## need to check if it is right intialization
        chosen_space_number = 0 ## need to check if it is right intialization
        if id:
            traci.vehicle.subscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 120.0, [tc.VAR_POSITION])
            # print(id,' subscribeContext')
            for v_id in traci.vehicle.getContextSubscriptionResults(str(id)):
                if(traci.vehicle.getLaneIndex(v_id)==2) and not (v_id=='ego'):
                    #left_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),v_id]) -> id check
                    left_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),traci.vehicle.getPosition(v_id)[0]])
                if(traci.vehicle.getLaneIndex(v_id)==0) and not (v_id=='ego'):
                    right_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),traci.vehicle.getPosition(v_id)[0]])
            
            
            left_vehicles.sort()
            right_vehicles.sort()
            traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 120.0)
        for i in range(2): # getting near 3 vehicles x_pos each left and right
            left_near_vehicles.append(left_vehicles[:3][i][1])
            right_near_vehicles.append(right_vehicles[:3][i][1])
                
        left_near_vehicles.sort()
        right_near_vehicles.sort()
        
        left_near_spaces.append([left_near_vehicles[1],left_near_vehicles[0],left_near_vehicles[1]-left_near_vehicles[0]])
        right_near_spaces.append([right_near_vehicles[1],right_near_vehicles[0], right_near_vehicles[1]-right_near_vehicles[0]])
        
        left_near_spaces.sort()
        # print(left_near_spaces)
        right_near_spaces.sort()
        # print(right_near_spaces)
        empty_space_dictionary = {"L0":left_near_spaces[0],"R0":right_near_spaces[0]}

        empty_space_dictionary_sorted_by_space = sorted(empty_space_dictionary.items(), key=lambda x: x[1][2],reverse=True)

        for i in range(2):
            if empty_space_dictionary_sorted_by_space[i][1][0]> traci.vehicle.getPosition('ego')[0] and empty_space_dictionary_sorted_by_space[i][1][1] <traci.vehicle.getPosition('ego')[0] :

                if (traci.vehicle.getAngle('ego')>90 and empty_space_dictionary_sorted_by_space[i][0][0] == "R") or (traci.vehicle.getAngle('ego')<90 and empty_space_dictionary_sorted_by_space[i][0][0] == "L"):
                    chosen_space_size = empty_space_dictionary_sorted_by_space[i][1][2]
                    chosen_space_number = i

        ## 0 : biggest space is chosen, 1: second big space is chosen, 2 : third big space is chosen, 3: fourth(last) big space is chosen
        distance_to_center = np.abs(traci.vehicle.getPosition('ego')[0]-(empty_space_dictionary_sorted_by_space[chosen_space_number][1][1]+(empty_space_dictionary_sorted_by_space[chosen_space_number][1][0]-empty_space_dictionary_sorted_by_space[chosen_space_number][1][1])/2))
        # print('distance_to_center: ',distance_to_center)
        if chosen_space_number == 0:            
            # print('biggest space reward ')
            return 300 -distance_to_center
        elif chosen_space_number == 1:            
            # print('2nd space reward ')
            return 150-distance_to_center

    def __Forward_SD_negative_reward(self): # SD_LK 침범 안하면 reward 0, SD_LK 침범한 거리 만큼 negative reward
        id = 'ego'
        time_gap_LK = 1.36
        c_LK = 2 #min clearance for lane keeping
        ego_v_x = traci.vehicle.getSpeed(id)
        ego_x_pos = traci.vehicle.getPosition(id)[0]

        if traci.vehicle.getLeader(id) is not None:
            Leader_id,x_forward = traci.vehicle.getLeader(id)
            rel_v_with_front = traci.vehicle.getSpeed(Leader_id) - ego_v_x
            v_p = traci.vehicle.getSpeed(Leader_id)
            des_clearance = c_LK + time_gap_LK*v_p #앞차 속도에 따른 앞차와의 ACC 안전거리 (safe)
            
            if x_forward > des_clearance :
                return 0
            else:
                # print('Forward_SD_negative_reward: ',des_clearance -x_forward)
                return int(x_forward-des_clearance)
                
    def __LaneChange_SD_negative_reward(self): # SD_LC 침범 안하면 reward 0, SD_LC 침범한 거리 만큼 negative reward , 앞뒤 차량 SD_LC 침범시 reward -50
        id = 'ego'
        time_gap_LC_1 =1
        time_gap_LC_2 =0.5 
        c_LC =12 # min clearance for lane change
        ego_v_x = traci.vehicle.getSpeed(id)
        ego_x_pos = traci.vehicle.getPosition(id)[0]



        if traci.vehicle.getLeader(id) is not None:
            Leader_id,x_forward = traci.vehicle.getLeader(id)
        if type(traci.vehicle.getFollower(id)) is not None and traci.vehicle.getFollower(id)[0] != '':
            follower_id, x_back = traci.vehicle.getFollower(id) 


        if traci.vehicle.getLeader(id) is not None and type(traci.vehicle.getFollower(id)) is not None and traci.vehicle.getFollower(id)[0] != '':
            front_relative_velocity_term = max( [traci.vehicle.getSpeed(id) - traci.vehicle.getSpeed(Leader_id) ,0] )*time_gap_LC_1
            front_minimum_clearance_trem = max([traci.vehicle.getSpeed(id)*time_gap_LC_2,c_LC])
            front_SD_LC = front_relative_velocity_term +front_minimum_clearance_trem
            back_relative_velocity_term = max( [traci.vehicle.getSpeed(follower_id)-traci.vehicle.getSpeed(id),0] )*time_gap_LC_1
            back__minimum_clearance_trem = max([traci.vehicle.getSpeed(follower_id)*time_gap_LC_2,c_LC])
            back_SD_LC = back_relative_velocity_term + back__minimum_clearance_trem
            # print('back_SD_LC: ',back_SD_LC)
            # print('front_SD_LC: ',front_SD_LC)
            if((traci.vehicle.getPosition(Leader_id)[0]-ego_x_pos) >= front_SD_LC  and (ego_x_pos -traci.vehicle.getPosition(follower_id)[0])>= back_SD_LC):
                # print('LaneChange_SD_negative_reward -> None ')
                return 0
            elif((traci.vehicle.getPosition(Leader_id)[0]-ego_x_pos) < front_SD_LC  and (ego_x_pos -traci.vehicle.getPosition(follower_id)[0])>= back_SD_LC):
                # print('LaneChange_SD_negative_reward-> forward ')
                return int((traci.vehicle.getPosition(Leader_id)[0]-ego_x_pos) - front_SD_LC )
            elif((traci.vehicle.getPosition(Leader_id)[0]-ego_x_pos) >= front_SD_LC  and (ego_x_pos -traci.vehicle.getPosition(follower_id)[0]) < back_SD_LC):
                # print('LaneChange_SD_negative_reward-> rear ')
                return int((ego_x_pos -traci.vehicle.getPosition(follower_id)[0]) - back_SD_LC)
            else:
                # print('LaneChange_SD_negative_reward-> forward and rear')
                return-50           
                
        
    def __reward(self, step_num, action):
        w1= 0.5 # collision negative reward
        w2 =0.4 # SD_LC not satisfied negative reward
        w3 = 0.01 # distance to biggest empty space positive reward
        w4 = 0.01*2 # LK continue if rear accel is far away positive reward or if rear accel is closer give negative reward
        w5 = 0.5 # LC succeed positive reward
        reward = 0
        vehs = traci.vehicle.getIDList()
        ###negative rewards            
        #collision
        if len(traci.simulation.getCollidingVehiclesIDList()) !=0:
            # print('negative reward')
            log = traci.simulation.getCollisions()
            if str(log[0]).split(',')[1]== ' victim=ego': 
                
                # print('Rear vehicle collision negative reward: ',-100*w1)   
                # print('collision negative reward: ',-200*w1)
                reward -= 200*w1
            # collision caused by ego
            elif str(log[0]).split(',')[0]== 'Collision(collider=ego' :
                # print('ego cause collision negative reward: ',-200*w1)
                reward -= 500*w1
        if self.__ego_vehicle_LC_completed():
            # print('LaneChange_SD_negative_reward: ',self.__LaneChange_SD_negative_reward()*w2)
            reward += self.__LaneChange_SD_negative_reward()*w2


        if('ego' in vehs and 'accel.rear' in vehs):           
        
            if traci.vehicle.getLaneIndex('ego')==1 and 'car.forward' in vehs:
                # print('Forward_SD_negative_reward: ',self.__Forward_SD_negative_reward()*w2)
                reward += self.__Forward_SD_negative_reward()*w2

            rear_clearance = traci.vehicle.getPosition('ego')[0] - traci.vehicle.getPosition('accel.rear')[0]
            # print('rear_clearance: ',rear_clearance)
                # print('rear_clearance: ',rear_clearance) 
            if traci.vehicle.getLaneIndex('ego')==1:
                if(rear_clearance<=60): # 차로 변경을 위한 가장큰 빈공간의 중앙에 가까워질 수 록 보상을 준다.
                    # print(self.__toward_biggest_empty_space_reward('ego'))
                    # print('toward_biggest_empty_space_reward: ',self.__toward_biggest_empty_space_reward('ego')*w3)
                    reward += self.__toward_biggest_empty_space_reward('ego')*w3
                if rear_clearance>60 and (action ==0 or action ==1 or action ==2):
                    # print('LK continue reward: ', 1*w4)
                    reward +=1*w4
                if(rlEnv.ego_LC_success ==False and rear_clearance<=60): #후방 차량 충돌 판단 가능 범위
                    # print('Danger LK continue reward: ' ,-1*w4)
                    reward -= 1.1**(10-rear_clearance)*w4
        
        
            if (self.__ego_vehicle_LC_completed() and rear_clearance<=60):
                # print('LC success reward: ',self.__chosen_space_size_reward('ego')*w5 )
                reward += self.__chosen_space_size_reward('ego')*w5
            # vehs = traci.vehicle.getIDList()
            
            
            if(rear_clearance>60 and self.__ego_vehicle_LC_completed()): #후방 차량 충돌 판단 불가능 범위에서 불필요한 차로 변경
                # print('early LC negative reward: ',-20 *w5)
                reward -= 20 *w5
        return reward
    
    # def __next_state(self):
    #     next_states =[]

    #     return next_states
    
    def step(self, action): #action-> 0: LK_const_vel, 1: LK_accel, 2: LK_decel


        self.done =False
                           
        
        # vehs = traci.vehicle.getIDList()
        traci.simulationStep()
        # if 'car.forward' in vehs:
        #     traci.simulationStep()
        # else:
        #     print('forward end1')            
        #     self.done=True
        vehs = traci.vehicle.getIDList()
        for veh_id in vehs:
            traci.vehicle.setMinGap(veh_id,'0')
            traci.vehicle.setSpeedMode(veh_id,0b100000)
            traci.vehicle.setLaneChangeMode(veh_id,0b000000000000) # 마음대로 차선 변경 x    
            
        ##################### test ########################
        # if('ego' in vehs):
        #     print('ego_lane: ',traci.vehicle.getLaneIndex('ego'))
        #    self.state('ego')
           
           
           
        ## agent action ##
        #action-> 0: LK_const_vel, 1: LK_accel, 2: LK_decel, 3: LC_left, 4: LC_right

        #차로 변경 끝
        if 'ego' in vehs:    
            if(traci.vehicle.getLaneIndex('ego')==2 and traci.vehicle.getLateralLanePosition('ego')>=0)or(traci.vehicle.getLaneIndex('ego')==0 and traci.vehicle.getLateralLanePosition('ego')<=0):
                self.done = True


        if len(rlEnv.ego) !=0:
            ego_v_last = rlEnv.ego[-1][4]
        else:
            ego_v_last = traci.vehicle.getSpeed('ego')

        left_vehicles = []
        right_vehicles = []
        left_near_vehicles = []
        right_near_vehicles = []
        
        if('ego' in vehs and 'car.forward' in vehs and 'accel.rear' in vehs):  
            rear_relative_distance = traci.vehicle.getPosition('ego')[0] - traci.vehicle.getPosition('accel.rear')[0]
            rear_relative_velocity = traci.vehicle.getSpeed('accel.rear')-traci.vehicle.getSpeed('ego')
            TTC= rear_relative_distance/rear_relative_velocity
            # print('TTC: ',TTC)

            ### LK mode
            if traci.vehicle.getLeader('ego') is not None:
                Leader_id,x_forward = traci.vehicle.getLeader('ego')
            if type(traci.vehicle.getFollower('ego')) is not None and traci.vehicle.getFollower('ego')[0] != '':
                follower_id, x_back = traci.vehicle.getFollower('ego')  
            Anti_RVC_on = False
            c_LK = 2 #min clearance for lane keeping
            time_gap_LC_prepare = 0.74
            time_gap_LK = 1.36  #95% percentile time_gap (most safe) // 1.36s -> 중간값   0.74s -> 5% persentile 가장 위험.
            v_p = traci.vehicle.getSpeed(Leader_id)
            des_clearance = c_LK + time_gap_LK*v_p #앞차 속도에 따른 앞차와의 ACC 안전거리 (safe)
            des_clearance2 = c_LK + time_gap_LC_prepare*v_p #앞차 속도에 따른 앞차와의 ACC 안전거리 (aggressive)
            if rear_relative_distance<=60:
                Anti_RVC_on = True
            else:
                Anti_RVC_on = False


            traci.vehicle.subscribeContext('ego', tc.CMD_GET_VEHICLE_VARIABLE, 120.0, [tc.VAR_POSITION])
            # print(id,' subscribeContext')
            for v_id in traci.vehicle.getContextSubscriptionResults('ego'):
                if(traci.vehicle.getLaneIndex(v_id)==2) and not (v_id=='ego'):
                    #left_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),v_id]) -> id check
                    left_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),traci.vehicle.getPosition(v_id)[0],v_id])
                if(traci.vehicle.getLaneIndex(v_id)==0) and not (v_id=='ego'):
                    right_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),traci.vehicle.getPosition(v_id)[0],v_id])
            traci.vehicle.unsubscribeContext('ego', tc.CMD_GET_VEHICLE_VARIABLE, 120.0)
            if len(left_vehicles)<3:
                left_vehicles.append([100,0,'virtual_left_at_start_point'])
                left_vehicles.append([100,1000,'virtual_left_at_end_point'])
            if len(right_vehicles)<3:
                right_vehicles.append([100,0,'virtual_right_at_start_point'])
                right_vehicles.append([100,1000,'virtual_right_at_end_point'])
            left_vehicles.sort()
            right_vehicles.sort()
            
            # print('right_vehicles: ',right_vehicles)
            for i in range(3): # getting near 3 vehicles x_pos each left and right
                left_near_vehicles.append([left_vehicles[:3][i][1],left_vehicles[:3][i][2]])                
                right_near_vehicles.append([right_vehicles[:3][i][1],right_vehicles[:3][i][2]])
                
            left_near_vehicles.sort()
            right_near_vehicles.sort()

            # print('left_near_vehicles: ',left_near_vehicles)
            # print('right_near_vehicles: ',right_near_vehicles)
            s1 = (left_near_vehicles[1][0]+left_near_vehicles[2][0])/2 - traci.vehicle.getPosition('ego')[0]  #왼쪽 차로 가장 가까운 전방 빈공간의 중점 까지 x축 방향 거리 거리
            s2 = (right_near_vehicles[1][0]+right_near_vehicles[2][0])/2 - traci.vehicle.getPosition('ego')[0] #오른쪽 차로 가장 가까운 전방 빈공간의 중점 까지 x축 방향 거리 거리
            s3 = (left_near_vehicles[0][0]+left_near_vehicles[1][0])/2 - traci.vehicle.getPosition('ego')[0] #왼쪽 차로 가장 가까운 후방 빈공간의 중점 까지 x축 방향 거리 거리
            s4 = (right_near_vehicles[0][0]+right_near_vehicles[1][0])/2 - traci.vehicle.getPosition('ego')[0] #오른쪽 차로 가장 가까운 후방 빈공간의 중점 까지 x축 방향 거리 거리
            space1 = (left_near_vehicles[1][0]+left_near_vehicles[2][0])/2
            space2 = (right_near_vehicles[1][0]+right_near_vehicles[2][0])/2
            space3 = (left_near_vehicles[0][0]+left_near_vehicles[1][0])/2
            space4 = (right_near_vehicles[0][0]+right_near_vehicles[1][0])/2
            # print('s1,s2,s3,s4: ',s1,' ',s2,' ',s3,' ',s4)
            v_ego = traci.vehicle.getSpeed('ego')
            accel_weight = 1
            if action == 0: 
                if Anti_RVC_on == True: #LK_mode
                    if des_clearance2<=x_forward:
                        speed = traci.vehicle.getSpeed(follower_id)                        
                    else:
                        speed = (traci.vehicle.getSpeed(follower_id)*(x_forward) + traci.vehicle.getSpeed(Leader_id)*(2*x_back))/(x_forward+2*x_back)
                    # self.__set_ego_v('ego',speed)
                    control = [traci.simulation.getTime(),self.__set_ego_v('ego',speed),0]
                else:
                    # self.__set_ego_v('ego',ego_v_last)
                    control = [traci.simulation.getTime(),self.__set_ego_v('ego',ego_v_last),0]
            elif action == 1:
                if len(rlEnv.Space1)>=1:
                    v_t = (space1-rlEnv.Space1[-1])/self.step_length
                    # print('(space1-rlEnv.Space1[-1]): ',(space1-rlEnv.Space1[-1]))
                else:
                    v_t = 0
                rlEnv.Target1.append([traci.simulation.getTime(),1,s1,space1,v_t]) 
                # print('v_t: ',v_t)             
                # print('acceleration: ',self.__set_ego_a('ego',self.__ego_a_desire(v_ego,v_t,1,s1,1)))    
                self.__set_ego_a('ego',self.__ego_a_desire(v_ego,v_t,1,s1,1))
                control = [traci.simulation.getTime(),self.__ego_a_desire(v_ego,v_t,1,s1,1),0]

            elif action == 2:
                if len(rlEnv.Space2)>=1:
                    v_t = (space2-rlEnv.Space2[-1])/self.step_length
                    # print('(space2-rlEnv.Space2[-1]): ',(space2-rlEnv.Space2[-1]))
                else:
                    v_t = 0
                rlEnv.Target2.append([traci.simulation.getTime(),2,s2,space2,v_t])  
                # print('v_t: ',v_t)            
                # print('acceleration: ',self.__set_ego_a('ego',self.__ego_a_desire(v_ego,v_t,1,s2,1)))    
                self.__set_ego_a('ego',self.__ego_a_desire(v_ego,v_t,1,s2,1))
                control = [traci.simulation.getTime(),self.__ego_a_desire(v_ego,v_t,1,s2,1),0]

            elif action == 3: #s3 <0
                if len(rlEnv.Space3)>=1:
                    v_t = (space3-rlEnv.Space3[-1])/self.step_length
                    # print('(space3-rlEnv.Space3[-1]): ',(space3-rlEnv.Space3[-1]))
                else:
                    v_t = 0 
                rlEnv.Target3.append([traci.simulation.getTime(),3,s3,space3,v_t]) 
                # print('v_t: ',v_t)            
                # print('acceleration: ',self.__set_ego_a('ego',self.__ego_a_desire2(v_ego,v_t,1,s3,1)))    
                self.__set_ego_a('ego',self.__ego_a_desire2(v_ego,v_t,1,s3,1))
                control = [traci.simulation.getTime(),self.__ego_a_desire2(v_ego,v_t,1,s3,1),0]
            elif action == 4: #s4 <0
                if len(rlEnv.Space4)>=1:
                    v_t = (space4-rlEnv.Space4[-1])/self.step_length
                    # print('(space4-rlEnv.Space4[-1]): ',(space4-rlEnv.Space4[-1]))
                else:
                    v_t = 0 
                rlEnv.Target4.append([traci.simulation.getTime(),4,s4,space4,v_t]) 
                # print('v_t: ',v_t)            
                # print('acceleration: ',self.__set_ego_a('ego',self.__ego_a_desire2(v_ego,v_t,1,s4,1)))    
                self.__set_ego_a('ego',self.__ego_a_desire2(v_ego,v_t,1,s4,1))
                control = [traci.simulation.getTime(),self.__ego_a_desire2(v_ego,v_t,1,s4,1),0]
            rlEnv.ego_control.append(control)
            # if action == 0:
            #     self.__set_v('ego',ego_v_last)
            # elif action == 1: 
            #     self.__set_v('ego',ego_v_last +0.5)                
            # elif action == 2:                 
            #     self.__set_v('ego',ego_v_last +1)
            # elif action == 3:              
            #     self.__set_v('ego',ego_v_last -0.5)
            # elif action == 4:
            #     self.__set_v('ego',ego_v_last -1)
                
            rlEnv.Space1.append(space1)
            rlEnv.Space2.append(space2)
            rlEnv.Space3.append(space3)
            rlEnv.Space4.append(space4)
        
        if('ego' in vehs and 'car.forward' in vehs and 'accel.rear' in vehs):
            nextstate = self.state('ego')            
            reward = self.__reward(rlEnv.step_num, action) # rear vehicle collision : -10, collision caused by ego : -20, LC_succeed_with_biggest_space : +20, LC_succeed_with_smaller_space : +10, step*-0.01
        else:
            nextstate = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0] ## it might wrong code
            reward = self.__reward(rlEnv.step_num, action)
            print("else")
            self.done = True  
        
        if len(traci.simulation.getCollidingVehiclesIDList()) !=0:
            log = traci.simulation.getCollisions()
            if str(log[0]).split(',')[1]== ' victim=ego' or str(log[0]).split(',')[0]== 'Collision(collider=ego' :
                print('done')
                self.collision_num +=1
                self.done = True

          
        
        for veh_id in vehs:
            # surrounding vehicles LK for first 2secs.
            # if veh_id[0] =='c'and traci.simulation.getTime()<2:
            #     traci.vehicle.setLaneChangeMode(veh_id,0b000000000000)

 
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
                ID = veh_id.split('.')
                velocity_y=0
                if veh_id== 'ego' and not (len(rlEnv.ego)==0):
                    velocity_y = (y1 - rlEnv.ego[-1][3])/self.step_length
                if veh_id== 'car.left0' and not (len(rlEnv.left0))==0:
                    velocity_y = (y1 - rlEnv.left0[-1][3])/self.step_length
                if veh_id== 'car.left1' and not (len(rlEnv.left1))==0:
                    velocity_y = (y1 - rlEnv.left1[-1][3])/self.step_length
                if veh_id== 'accel.rear' and not (len(rlEnv.rear))==0:
                    velocity_y = (y1 - rlEnv.rear[-1][3])/self.step_length
                if veh_id== 'car.right0' and not (len(rlEnv.right0))==0:
                    velocity_y = (y1 - rlEnv.right0[-1][3])/self.step_length
                if veh_id== 'car.right1' and not (len(rlEnv.right1))==0:
                    velocity_y = (y1 - rlEnv.right1[-1][3])/self.step_length
                if veh_id== 'car.left2' and not (len(rlEnv.left2))==0:
                    velocity_y = (y1 - rlEnv.left2[-1][3])/self.step_length
                if veh_id== 'car.left3' and not (len(rlEnv.left3))==0:
                    velocity_y = (y1 - rlEnv.left3[-1][3])/self.step_length
                if veh_id== 'car.left4' and not (len(rlEnv.left4))==0:
                    velocity_y = (y1 - rlEnv.left4[-1][3])/self.step_length
                if veh_id== 'car.left5' and not (len(rlEnv.left5))==0:
                    velocity_y = (y1 - rlEnv.left5[-1][3])/self.step_length
                if veh_id== 'car.left6' and not (len(rlEnv.left6))==0:
                    velocity_y = (y1 - rlEnv.left6[-1][3])/self.step_length
                if veh_id== 'car.left7' and not (len(rlEnv.left7))==0:
                    velocity_y = (y1 - rlEnv.left7[-1][3])/self.step_length
                if veh_id== 'car.forward' and not (len(rlEnv.forward))==0:
                    velocity_y = (y1 - rlEnv.forward[-1][3])/self.step_length
                if veh_id== 'car.right2' and not (len(rlEnv.right2))==0:
                    velocity_y = (y1 - rlEnv.right2[-1][3])/self.step_length
                if veh_id== 'car.right3' and not (len(rlEnv.right3))==0:
                    velocity_y = (y1 - rlEnv.right3[-1][3])/self.step_length
                if veh_id== 'car.right4' and not (len(rlEnv.right4))==0:
                    velocity_y = (y1 - rlEnv.right4[-1][3])/self.step_length
                if veh_id== 'car.right5' and not (len(rlEnv.right5))==0:
                    velocity_y = (y1 - rlEnv.right5[-1][3])/self.step_length

                vehicle_state.append(velocity_y)
                # print("v_y : ",traci.vehicle.getLateralSpeed(veh_id))
                
                
                acceleration_x =0
                
                
                acceleration_x= traci.vehicle.getAcceleration(veh_id)            
                vehicle_state.append(acceleration_x)
                acceleration_y=0
                if veh_id== 'ego' and not (len(rlEnv.ego)==0):
                    acceleration_y = (velocity_y - rlEnv.ego[-1][5])/self.step_length
                if veh_id== 'car.left0' and not (len(rlEnv.left0))==0:
                    acceleration_y = (velocity_y- rlEnv.left0[-1][5])/self.step_length
                if veh_id== 'car.left1' and not (len(rlEnv.left1))==0:
                    acceleration_y = (velocity_y - rlEnv.left1[-1][5])/self.step_length
                if veh_id== 'accel.rear' and not (len(rlEnv.rear))==0:
                    acceleration_y = (velocity_y - rlEnv.rear[-1][5])/self.step_length
                if veh_id== 'car.right0' and not (len(rlEnv.right0))==0:
                    acceleration_y = (velocity_y - rlEnv.right0[-1][5])/self.step_length
                if veh_id== 'car.right1' and not (len(rlEnv.right1))==0:
                    acceleration_y = (velocity_y - rlEnv.right1[-1][5])/self.step_length
                if veh_id== 'car.left2' and not (len(rlEnv.left2))==0:
                    acceleration_y = (velocity_y - rlEnv.left2[-1][5])/self.step_length
                if veh_id== 'car.left3' and not (len(rlEnv.left3))==0:
                    acceleration_y = (velocity_y - rlEnv.left3[-1][5])/self.step_length
                if veh_id== 'car.left4' and not (len(rlEnv.left4))==0:
                    acceleration_y = (velocity_y - rlEnv.left4[-1][5])/self.step_length
                if veh_id== 'car.left5' and not (len(rlEnv.left5))==0:
                    acceleration_y = (velocity_y - rlEnv.left5[-1][5])/self.step_length
                if veh_id== 'car.left6' and not (len(rlEnv.left6))==0:
                    acceleration_y = (velocity_y - rlEnv.left6[-1][5])/self.step_length
                if veh_id== 'car.left7' and not (len(rlEnv.left7))==0:
                    acceleration_y = (velocity_y - rlEnv.left7[-1][5])/self.step_length
                if veh_id== 'car.forward' and not (len(rlEnv.forward))==0:
                    acceleration_y = (velocity_y - rlEnv.forward[-1][5])/self.step_length
                if veh_id== 'car.right2' and not (len(rlEnv.right2))==0:
                    acceleration_y = (velocity_y - rlEnv.right2[-1][5])/self.step_length
                if veh_id== 'car.right3' and not (len(rlEnv.right3))==0:
                    acceleration_y = (velocity_y - rlEnv.right3[-1][5])/self.step_length
                if veh_id== 'car.right4' and not (len(rlEnv.right4))==0:
                    acceleration_y = (velocity_y - rlEnv.right4[-1][5])/self.step_length
                if veh_id== 'car.right5' and not (len(rlEnv.right5))==0:
                    acceleration_y = (velocity_y - rlEnv.right5[-1][5])/self.step_length

                vehicle_state.append(acceleration_y)
                
                
                vehicle_state.append(traci.vehicle.getAngle(veh_id))
            
                
                if(veh_id == 'ego'):
                    rlEnv.ego.append(vehicle_state)
                if(veh_id =='car.left0'):
                    rlEnv.left0.append(vehicle_state)
                if(veh_id=='car.left1'):
                    rlEnv.left1.append(vehicle_state)
                if(veh_id=='accel.rear'):
                    rlEnv.rear.append(vehicle_state)
                if(veh_id=='car.right0'):
                    rlEnv.right0.append(vehicle_state)
                if(veh_id=='car.right1'):
                    rlEnv.right1.append(vehicle_state)
                if(veh_id=='car.left2'):
                    rlEnv.left2.append(vehicle_state)
                if(veh_id=='car.left3'):
                    rlEnv.left3.append(vehicle_state)
                if(veh_id=='car.left4'):
                    rlEnv.left4.append(vehicle_state)
                if(veh_id=='car.left5'):
                    rlEnv.left5.append(vehicle_state)
                if(veh_id=='car.left6'):
                    rlEnv.left6.append(vehicle_state)
                if(veh_id=='car.left7'):
                    rlEnv.left7.append(vehicle_state)
                if(veh_id=='car.forward'):
                    rlEnv.forward.append(vehicle_state)
                if(veh_id=='car.right2'):
                    rlEnv.right2.append(vehicle_state)
                if(veh_id=='car.right3'):
                    rlEnv.right3.append(vehicle_state)
                if(veh_id=='car.right4'):
                    rlEnv.right4.append(vehicle_state)    
                if(veh_id=='car.right5'):
                    rlEnv.right5.append(vehicle_state)   


            ### ego_vehicle LC -> episode ends.
        
            if veh_id == 'ego':
                if self.__ego_vehicle_LC_start() and traci.simulation.getTime()>=0.01:
                    print('Lane change start')
                    self.LC_succeed_num +=1
                    rlEnv.ego_LC_start =True
                    traci.vehicle.setLaneChangeMode('ego',0)
                self.__ego_vehicle_LC_completed() # LC completed checking
                rlEnv.lane_buffer_ego = traci.vehicle.getLaneIndex('ego') # ego_car lane buffer  
                   
                      
            
            if traci.vehicle.getRoadID(veh_id) == 'E5' and veh_id[0] == 'a':
                traci.vehicle.setAccel(veh_id, '1')
                traci.vehicle.setDecel(veh_id, '0.00001')
                # traci.vehicle.setSpeedMode(veh_id,'0')
                self.__setvehiclestate('accel.rear')

            if (traci.vehicle.getRoadID(veh_id) == 'E5' and veh_id[0] == 'c'):
                c0 = 1.98
                # ID=veh_id.split('.')
                # if ID[1][0] =='r':
                #     tau = 1.6
                # else:
                #     tau = 1.36
                
                if veh_id == 'car.left0':
                    tau = rlEnv.vehicles_tau[0]                    
                elif veh_id =='car.left1':
                    tau = rlEnv.vehicles_tau[1]                    
                elif veh_id =='car.right0':
                    tau = rlEnv.vehicles_tau[2]                    
                elif veh_id =='car.right1':
                    tau = rlEnv.vehicles_tau[3]                    
                elif veh_id =='car.left2':
                    tau = rlEnv.vehicles_tau[4]               
                elif veh_id =='car.left3':
                    tau = rlEnv.vehicles_tau[5]                    
                elif veh_id =='car.left4':
                    tau = rlEnv.vehicles_tau[6]
                elif veh_id =='car.left5':
                    tau = rlEnv.vehicles_tau[7]
                elif veh_id =='car.left6':
                    tau = rlEnv.vehicles_tau[8]
                elif veh_id =='car.left7':
                    tau = rlEnv.vehicles_tau[9]
                elif veh_id =='car.forward':
                    tau = rlEnv.vehicles_tau[10]                    
                elif veh_id =='car.right2':
                    tau = rlEnv.vehicles_tau[11]                    
                elif veh_id =='car.right3':
                    tau = rlEnv.vehicles_tau[12]                    
                elif veh_id =='car.right4':
                    tau = rlEnv.vehicles_tau[13]
                elif veh_id =='car.right5':
                    tau = rlEnv.vehicles_tau[14]
                else:
                    tau = 1.36
                
                
                v_controled = traci.vehicle.getSpeed(veh_id)

                if traci.vehicle.getLeader(veh_id) is not None: # 선행 차량이 있을 때 ACC
                    Leader_id,c_front = traci.vehicle.getLeader(veh_id)
                    v_preceding= traci.vehicle.getSpeed(Leader_id)
                    c_desire = c0+tau*v_preceding
                    
                    ##### ACC with speed limit ########
                    speed_limit = 23
                    # traci.vehicle.setAcceleration(veh_id,self.__a_desire_with_speed_limit(v_controled,v_preceding,c_desire,c_front,speed_limit),20) #입력 가속도
                    self.__set_a(veh_id,self.__a_desire_with_speed_limit(v_controled,v_preceding,c_desire,c_front,speed_limit))
                    # self.__a_desire_with_speed_limit(v_controled,v_preceding,c_desire,c_front,speed_limit) #입력 가속도

                else: # 선두 차량 CC
                    self.__set_v(veh_id,22.22)
                    # self.__set_v(veh_id,23)
        vehs = traci.vehicle.getIDList()
        if 'car.forward' not in vehs:
            print('forward end')            
            self.done=True
        # print(rlEnv.step_num)
        rlEnv.step_num +=1    
        self.ego_collision_happened()
        return nextstate, reward
    
    
    
                             # acition -> 3: LC_left, 4: LC_right
    def step3(self, action): # continue the simulation step but return finalstate and finalreward as result of Lanechange.
        self.done =False
        
        traci.simulationStep()
        vehs = traci.vehicle.getIDList()
        for veh_id in vehs:
            traci.vehicle.setMinGap(veh_id,'0')
            traci.vehicle.setSpeedMode(veh_id,0b100000)
            # traci.vehicle.setLaneChangeMode(veh_id,0b000000000000) # 마음대로 차선 변경 x 
        ## agent action ##
        #action-> 0: LK_const_vel, 1: LK_accel, 2: LK_decel, 3: LC_left, 4: LC_right
            

        if len(rlEnv.ego) !=0:
            ego_v_last = rlEnv.ego[-1][4]
        else:
            ego_v_last = traci.vehicle.getSpeed('ego')

        if('ego' in vehs):  
            tau = 0.74
            v_max = 22.22 #선두 차량 속도
            c0 = 1.98  
            LLP = traci.vehicle.getLateralLanePosition('ego') #LateralLanePosition
            if action == 5:
                traci.vehicle.setSpeedMode('ego',32)
                traci.vehicle.setLaneChangeMode('ego',0)
                # traci.vehicle.setAcceleration('ego',0,0.01)
                # self.__set_v('ego',ego_v_last)
                edgeID = 'E5'
                # if traci.vehicle.getLaneIndex('ego') ==1 or traci.vehicle.getLaneIndex('ego') ==0:
                #     traci.vehicle.changeLaneRelative('ego',1,3)
                if traci.vehicle.getLaneIndex('ego') ==0:
                    lane =1
                if traci.vehicle.getLaneIndex('ego') ==1 or traci.vehicle.getLaneIndex('ego') ==2:
                    lane =2
                if LLP>=0:
                    angle = 90-LLP*2
                elif LLP<=0:
                    angle = 90+LLP*2
                keepRoute =2 #기존 route에 제한 받지 않고 movetoxy 실행.
                matchThreshold =3.2 #차로 폭
                if(len(rlEnv.ego)!=0):
                    last_ego_x = rlEnv.ego[-1][2]
                    last_ego_y = rlEnv.ego[-1][3]
                else:
                    last_ego_x,last_ego_y = traci.vehicle.getPosition('ego')
                ego_x,ego_y = traci.vehicle.getPosition('ego')
                ego_v = traci.vehicle.getSpeed('ego')
                ego_a = traci.vehicle.getAcceleration('ego')
                ego_v_y = self.__get_v_y('ego')
                ego_a_y = self.__get_a_y('ego')
                #lane =2 a_y_desire = kp_l*LLP, kp_l = 1/1.6
                kp_l = 0.8
                kd_l = 2
                if traci.vehicle.getLaneIndex('ego') == 2:
                    e_lateral =0-LLP #LLP 음수
                    ego_a_y_des = kp_l*e_lateral-kd_l*ego_v_y
                elif traci.vehicle.getLaneIndex('ego') == 1 and LLP>=0: 
                    e_lateral = 0+(3.2-LLP) #LLP 양수
                    ego_a_y_des = kp_l*e_lateral-kd_l*ego_v_y
                elif traci.vehicle.getLaneIndex('ego') == 1 and LLP<0: 
                    e_lateral = 0-LLP #LLP 음수
                    ego_a_y_des = kp_l*e_lateral-kd_l*ego_v_y
                elif traci.vehicle.getLaneIndex('ego') == 0 and LLP>=0: 
                    e_lateral = 0+(3.2-LLP) #LLP 양수
                    ego_a_y_des = kp_l*e_lateral-kd_l*ego_v_y
                else:
                    ego_a_y_des=0
                # print('LLP,Lane: ',LLP,", ",traci.vehicle.getLaneIndex('ego'))
                ego_a_y_desire = self.__set_a_y_possible('ego',ego_a_y_des)
                # ego_a_y_desire = self.__set_a_y_possible('ego',0.5)
                ego_v_y_next = ego_v_y + ego_a_y_desire*self.step_length
                # ego_v_y_desire = self.__set_v_possible('ego',ego_v_y_desire)
                
                # acc variables
                v_preceding = traci.vehicle.getSpeed(self.__ACC_target_id('ego',action))
                c_desire = c0+tau*v_preceding
                c_front = traci.vehicle.getPosition(self.__ACC_target_id('ego',action))[0]-traci.vehicle.getPosition('ego')[0]
                ego_a_x_desire =self.__set_a_x_possible('ego',self.__ego_acc(ego_v,v_preceding,c_desire,c_front))
                # ego_a_x_desire = self.__ego_acc(ego_v,v_preceding,c_desire,c_front)
                # ego_v_x_desire = ego_v + ego_a_x_desire*self.step_length
                ego_v_x_next = ego_v +ego_a_x_desire*self.step_length
                x_next = ego_x+ego_v_x_next*self.step_length #ACC
                # print('ego_x: ',ego_x)
                # print('x_next: ',x_next)
                # print('ego_v_x: ',ego_v)
                # print('ego_v_x_next: ',ego_v_x_next)
                # print('ego_a: ',ego_a)
                # print('ego_a_x_desire: ',ego_a_x_desire)
                y_next = ego_y+ego_v_y_next*self.step_length
                # print("ego_y",ego_y)
                # print("ego_v_y_next",ego_v_y_next)
                # print("ego_a_y_desire",ego_a_y_desire)
                # print("y_next: ",y_next)
                # print("")
                control = [traci.simulation.getTime(),self.__ego_acc(ego_v,v_preceding,c_desire,c_front),ego_a_y_des]
                rlEnv.ego_control.append(control)
                theta = np.arctan2(ego_y-last_ego_y,ego_x-last_ego_x)

                if(ego_y-last_ego_y>=0):
                    angle = 90-theta*10
                else:
                    angle = 90-theta*10
                if self.done != True and self.__ego_vehicle_LC_completed() !=True :
                    if(traci.vehicle.getLaneIndex('ego')==2 and traci.vehicle.getLateralLanePosition('ego')>=0):
                        print('stop moveToXY')
                        self.done = True
                    else:
                        # print(':::::::::::::::::::::::::::::::::::movetoXY')
                        traci.vehicle.moveToXY('ego',edgeID,lane,x_next,y_next,angle,keepRoute,matchThreshold)        
                    
                # traci.vehicle.setAcceleration('ego',0,0.01)
            

        if('ego' in vehs and 'car.forward' in vehs and 'accel.rear' in vehs):
            nextstate = self.state('ego')            
            nextreward = self.__reward(rlEnv.step_num, action) # rear vehicle collision : -10, collision caused by ego : -20, LC_succeed_with_biggest_space : +20, LC_succeed_with_smaller_space : +10, step*-0.01
        else:
            nextstate = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0] ## it might wrong code
            nextreward = self.__reward(rlEnv.step_num, action)
            # print("else")
            # print(vehs)
            self.done = True  
        
        if len(traci.simulation.getCollidingVehiclesIDList()) !=0:
            log = traci.simulation.getCollisions()
            if str(log[0]).split(',')[1]== ' victim=ego' or str(log[0]).split(',')[0]== 'Collision(collider=ego' :
                print('done')
                self.collision_num +=1
                self.done = True

        for veh_id in vehs:
            # surrounding vehicles LK for first 2secs.
            # if veh_id[0] =='c'and traci.simulation.getTime()<2:
            #     traci.vehicle.setLaneChangeMode(veh_id,0b000000000000)

 
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
                ID = veh_id.split('.')
                velocity_y=0
                if veh_id== 'ego' and not (len(rlEnv.ego)==0):
                    velocity_y = (y1 - rlEnv.ego[-1][3])/self.step_length
                if veh_id== 'car.left0' and not (len(rlEnv.left0))==0:
                    velocity_y = (y1 - rlEnv.left0[-1][3])/self.step_length
                if veh_id== 'car.left1' and not (len(rlEnv.left1))==0:
                    velocity_y = (y1 - rlEnv.left1[-1][3])/self.step_length
                if veh_id== 'accel.rear' and not (len(rlEnv.rear))==0:
                    velocity_y = (y1 - rlEnv.rear[-1][3])/self.step_length
                if veh_id== 'car.right0' and not (len(rlEnv.right0))==0:
                    velocity_y = (y1 - rlEnv.right0[-1][3])/self.step_length
                if veh_id== 'car.right1' and not (len(rlEnv.right1))==0:
                    velocity_y = (y1 - rlEnv.right1[-1][3])/self.step_length
                if veh_id== 'car.left2' and not (len(rlEnv.left2))==0:
                    velocity_y = (y1 - rlEnv.left2[-1][3])/self.step_length
                if veh_id== 'car.left3' and not (len(rlEnv.left3))==0:
                    velocity_y = (y1 - rlEnv.left3[-1][3])/self.step_length
                if veh_id== 'car.left4' and not (len(rlEnv.left4))==0:
                    velocity_y = (y1 - rlEnv.left4[-1][3])/self.step_length
                if veh_id== 'car.left5' and not (len(rlEnv.left5))==0:
                    velocity_y = (y1 - rlEnv.left5[-1][3])/self.step_length
                if veh_id== 'car.left6' and not (len(rlEnv.left6))==0:
                    velocity_y = (y1 - rlEnv.left6[-1][3])/self.step_length
                if veh_id== 'car.left7' and not (len(rlEnv.left7))==0:
                    velocity_y = (y1 - rlEnv.left7[-1][3])/self.step_length
                if veh_id== 'car.forward' and not (len(rlEnv.forward))==0:
                    velocity_y = (y1 - rlEnv.forward[-1][3])/self.step_length
                if veh_id== 'car.right2' and not (len(rlEnv.right2))==0:
                    velocity_y = (y1 - rlEnv.right2[-1][3])/self.step_length
                if veh_id== 'car.right3' and not (len(rlEnv.right3))==0:
                    velocity_y = (y1 - rlEnv.right3[-1][3])/self.step_length
                if veh_id== 'car.right4' and not (len(rlEnv.right4))==0:
                    velocity_y = (y1 - rlEnv.right4[-1][3])/self.step_length
                if veh_id== 'car.right5' and not (len(rlEnv.right5))==0:
                    velocity_y = (y1 - rlEnv.right5[-1][3])/self.step_length

                vehicle_state.append(velocity_y)
                # print("v_y : ",traci.vehicle.getLateralSpeed(veh_id))
                
                
                acceleration_x =0
                
                
                acceleration_x= traci.vehicle.getAcceleration(veh_id)            
                vehicle_state.append(acceleration_x)
                acceleration_y=0
                if veh_id== 'ego' and not (len(rlEnv.ego)==0):
                    acceleration_y = (velocity_y - rlEnv.ego[-1][5])/self.step_length
                if veh_id== 'car.left0' and not (len(rlEnv.left0))==0:
                    acceleration_y = (velocity_y- rlEnv.left0[-1][5])/self.step_length
                if veh_id== 'car.left1' and not (len(rlEnv.left1))==0:
                    acceleration_y = (velocity_y - rlEnv.left1[-1][5])/self.step_length
                if veh_id== 'accel.rear' and not (len(rlEnv.rear))==0:
                    acceleration_y = (velocity_y - rlEnv.rear[-1][5])/self.step_length
                if veh_id== 'car.right0' and not (len(rlEnv.right0))==0:
                    acceleration_y = (velocity_y - rlEnv.right0[-1][5])/self.step_length
                if veh_id== 'car.right1' and not (len(rlEnv.right1))==0:
                    acceleration_y = (velocity_y - rlEnv.right1[-1][5])/self.step_length
                if veh_id== 'car.left2' and not (len(rlEnv.left2))==0:
                    acceleration_y = (velocity_y - rlEnv.left2[-1][5])/self.step_length
                if veh_id== 'car.left3' and not (len(rlEnv.left3))==0:
                    acceleration_y = (velocity_y - rlEnv.left3[-1][5])/self.step_length
                if veh_id== 'car.left4' and not (len(rlEnv.left4))==0:
                    acceleration_y = (velocity_y - rlEnv.left4[-1][5])/self.step_length
                if veh_id== 'car.left5' and not (len(rlEnv.left5))==0:
                    acceleration_y = (velocity_y - rlEnv.left5[-1][5])/self.step_length
                if veh_id== 'car.left6' and not (len(rlEnv.left6))==0:
                    acceleration_y = (velocity_y - rlEnv.left6[-1][5])/self.step_length
                if veh_id== 'car.left7' and not (len(rlEnv.left7))==0:
                    acceleration_y = (velocity_y - rlEnv.left7[-1][5])/self.step_length
                if veh_id== 'car.forward' and not (len(rlEnv.forward))==0:
                    acceleration_y = (velocity_y - rlEnv.forward[-1][5])/self.step_length
                if veh_id== 'car.right2' and not (len(rlEnv.right2))==0:
                    acceleration_y = (velocity_y - rlEnv.right2[-1][5])/self.step_length
                if veh_id== 'car.right3' and not (len(rlEnv.right3))==0:
                    acceleration_y = (velocity_y - rlEnv.right3[-1][5])/self.step_length
                if veh_id== 'car.right4' and not (len(rlEnv.right4))==0:
                    acceleration_y = (velocity_y - rlEnv.right4[-1][5])/self.step_length
                if veh_id== 'car.right5' and not (len(rlEnv.right5))==0:
                    acceleration_y = (velocity_y - rlEnv.right5[-1][5])/self.step_length

                vehicle_state.append(acceleration_y)
                
                
                vehicle_state.append(traci.vehicle.getAngle(veh_id))
            
                
                if(veh_id == 'ego'):
                    rlEnv.ego.append(vehicle_state)
                if(veh_id =='car.left0'):
                    rlEnv.left0.append(vehicle_state)
                if(veh_id=='car.left1'):
                    rlEnv.left1.append(vehicle_state)
                if(veh_id=='accel.rear'):
                    rlEnv.rear.append(vehicle_state)
                if(veh_id=='car.right0'):
                    rlEnv.right0.append(vehicle_state)
                if(veh_id=='car.right1'):
                    rlEnv.right1.append(vehicle_state)
                if(veh_id=='car.left2'):
                    rlEnv.left2.append(vehicle_state)
                if(veh_id=='car.left3'):
                    rlEnv.left3.append(vehicle_state)
                if(veh_id=='car.left4'):
                    rlEnv.left4.append(vehicle_state)
                if(veh_id=='car.left5'):
                    rlEnv.left5.append(vehicle_state)
                if(veh_id=='car.left6'):
                    rlEnv.left6.append(vehicle_state)
                if(veh_id=='car.left7'):
                    rlEnv.left7.append(vehicle_state)
                if(veh_id=='car.forward'):
                    rlEnv.forward.append(vehicle_state)
                if(veh_id=='car.right2'):
                    rlEnv.right2.append(vehicle_state)
                if(veh_id=='car.right3'):
                    rlEnv.right3.append(vehicle_state)
                if(veh_id=='car.right4'):
                    rlEnv.right4.append(vehicle_state)    
                if(veh_id=='car.right5'):
                    rlEnv.right5.append(vehicle_state)   


            ### ego_vehicle LC -> episode ends.
        
            if veh_id == 'ego':
                if self.__ego_vehicle_LC_start() and traci.simulation.getTime()>=0.01:
                    print('Lane change start')
                    self.LC_succeed_num +=1
                    rlEnv.ego_LC_start =True
                    traci.vehicle.setLaneChangeMode('ego',0)
                self.__ego_vehicle_LC_completed() # LC completed checking
                rlEnv.lane_buffer_ego = traci.vehicle.getLaneIndex('ego') # ego_car lane buffer 
                

            if traci.vehicle.getRoadID(veh_id) == 'E5' and veh_id[0] == 'a':
                traci.vehicle.setAccel(veh_id, '1')
                traci.vehicle.setDecel(veh_id, '0.00001')
                # traci.vehicle.setSpeedMode(veh_id,'0')
                self.__setvehiclestate('accel.rear')

            if (traci.vehicle.getRoadID(veh_id) == 'E5' and veh_id[0] == 'c'):
                c0 = 1.98
                # ID=veh_id.split('.')
                # if ID[1][0] =='r':
                #     tau = 1.6
                # else:
                #     tau = 1.36
                
                if veh_id == 'car.left0':
                    tau = rlEnv.vehicles_tau[0]                    
                elif veh_id =='car.left1':
                    tau = rlEnv.vehicles_tau[1]                    
                elif veh_id =='car.right0':
                    tau = rlEnv.vehicles_tau[2]                    
                elif veh_id =='car.right1':
                    tau = rlEnv.vehicles_tau[3]                    
                elif veh_id =='car.left2':
                    tau = rlEnv.vehicles_tau[4]               
                elif veh_id =='car.left3':
                    tau = rlEnv.vehicles_tau[5]                    
                elif veh_id =='car.left4':
                    tau = rlEnv.vehicles_tau[6]
                elif veh_id =='car.left5':
                    tau = rlEnv.vehicles_tau[6]
                elif veh_id =='car.left6':
                    tau = rlEnv.vehicles_tau[6]
                elif veh_id =='car.left7':
                    tau = rlEnv.vehicles_tau[6]
                elif veh_id =='car.forward':
                    tau = rlEnv.vehicles_tau[7]                    
                elif veh_id =='car.right2':
                    tau = rlEnv.vehicles_tau[8]                    
                elif veh_id =='car.right3':
                    tau = rlEnv.vehicles_tau[9]                    
                elif veh_id =='car.right4':
                    tau = rlEnv.vehicles_tau[10]
                elif veh_id =='car.right5':
                    tau = rlEnv.vehicles_tau[11]
                else:
                    tau = 1.36
                
                
                v_controled = traci.vehicle.getSpeed(veh_id)

                if traci.vehicle.getLeader(veh_id) is not None: # 선행 차량이 있을 때 ACC
                    Leader_id,c_front = traci.vehicle.getLeader(veh_id)
                    v_preceding= traci.vehicle.getSpeed(Leader_id)
                    c_desire = c0+tau*v_preceding
                    
                    ##### ACC with speed limit ########
                    speed_limit = 23
                    # traci.vehicle.setAcceleration(veh_id,self.__a_desire_with_speed_limit(v_controled,v_preceding,c_desire,c_front,speed_limit),20) #입력 가속도
                    self.__set_a(veh_id,self.__a_desire_with_speed_limit(v_controled,v_preceding,c_desire,c_front,speed_limit))
                    # self.__a_desire_with_speed_limit(v_controled,v_preceding,c_desire,c_front,speed_limit) #입력 가속도

                else: # 선두 차량 CC
                    self.__set_v(veh_id,22.22)
        vehs = traci.vehicle.getIDList()
        
        # print(rlEnv.step_num)
        rlEnv.step_num +=1
        
        self.ego_collision_happened()
        return nextstate,nextreward
    

    def step4(self, action): # continue the simulation step but return finalstate and finalreward as result of Lanechange.
        self.done =False
 
        traci.simulationStep()
        vehs = traci.vehicle.getIDList()
        for veh_id in vehs:
            traci.vehicle.setMinGap(veh_id,'0')
            traci.vehicle.setSpeedMode(veh_id,0b100000)
            # traci.vehicle.setLaneChangeMode(veh_id,0b000000000000) # 마음대로 차선 변경 x 
        ## agent action ##
        #action-> 0: LK_const_vel, 1: LK_accel, 2: LK_decel, 3: LC_left, 4: LC_right
            
        if len(rlEnv.ego) !=0:
            ego_v_last = rlEnv.ego[-1][4]
        else:
            ego_v_last = traci.vehicle.getSpeed('ego')

        if('ego' in vehs):   
            tau = 0.74
            v_max = 22.22 #선두 차량 속도
            c0 = 1.98  
            LLP = traci.vehicle.getLateralLanePosition('ego') #LateralLanePosition
            if action == 6:
                traci.vehicle.setSpeedMode('ego',32)
                traci.vehicle.setLaneChangeMode('ego',0)
                # traci.vehicle.setAcceleration('ego',0,0.01)
                # self.__set_v('ego',ego_v_last)
                # if traci.vehicle.getLaneIndex('ego') ==1 or traci.vehicle.getLaneIndex('ego') ==2:
                #     traci.vehicle.changeLaneRelative('ego',-1,3)
                edgeID = 'E5'
                if traci.vehicle.getLaneIndex('ego') ==0 or traci.vehicle.getLaneIndex('ego') ==1:
                    lane =0
                elif traci.vehicle.getLaneIndex('ego') ==2:
                    lane =1
                if LLP>=0:
                    angle = 90+LLP*2
                elif LLP<=0:
                    angle = 90-LLP*2
                keepRoute =2 #기존 route에 제한 받지 않고 movetoxy 실행.
                matchThreshold =3.2
                if(len(rlEnv.ego)!=0):
                    last_ego_x = rlEnv.ego[-1][2]
                    last_ego_y = rlEnv.ego[-1][3]
                else:
                    last_ego_x,last_ego_y = traci.vehicle.getPosition('ego')
                ego_x,ego_y = traci.vehicle.getPosition('ego')
                ego_v = traci.vehicle.getSpeed('ego')
                ego_a = traci.vehicle.getAcceleration('ego')
                # ego_v = self.__get_v_x('ego')
                # ego_a = self.__get_a_x('ego')
                ego_v_y = self.__get_v_y('ego')
                ego_a_y = self.__get_a_y('ego')
                # ego_v_y_desire = self.__set_v_possible('ego',ego_v_y_desire)
                # acc variables
                v_preceding = traci.vehicle.getSpeed(self.__ACC_target_id('ego',action))
                c_desire = c0+tau*v_preceding
                c_front = traci.vehicle.getPosition(self.__ACC_target_id('ego',action))[0]-traci.vehicle.getPosition('ego')[0]
                ego_a_x_desire =self.__set_a_x_possible('ego',self.__ego_acc(ego_v,v_preceding,c_desire,c_front)) 
                # ego_a_x_desire = self.__ego_acc(ego_v,v_preceding,c_desire,c_front)
                # ego_v_x_desire = ego_v + ego_a_x_desire*self.step_length
                ego_v_x_next = ego_v +ego_a_x_desire*self.step_length

                kp_l = 0.8
                kd_l = 2
                if traci.vehicle.getLaneIndex('ego') == 2:
                    e_lateral =0-(3.2+LLP) #LLP 음수
                    ego_a_y_des = kp_l*e_lateral - kd_l*ego_v_y
                elif traci.vehicle.getLaneIndex('ego') == 1 and LLP>=0: 
                    e_lateral = 0-LLP #LLP 양수
                    ego_a_y_des = kp_l*e_lateral - kd_l*ego_v_y
                elif traci.vehicle.getLaneIndex('ego') == 1 and LLP<0: 
                    e_lateral = 0-(3.2+LLP) #LLP 음수
                    ego_a_y_des = kp_l*e_lateral - kd_l*ego_v_y
                elif traci.vehicle.getLaneIndex('ego') == 0 and LLP>=0: 
                    e_lateral = 0-LLP #LLP 양수
                    ego_a_y_des = kp_l*e_lateral - kd_l*ego_v_y
                else:
                    ego_a_y_des =0

                # print('LLP,Lane: ',LLP,", ",traci.vehicle.getLaneIndex('ego'))
                ego_a_y_desire  = self.__set_a_y_possible('ego',ego_a_y_des)
                
                # ego_a_y_desire  = self.__set_a_y_possible('ego',-0.5)
                ego_v_y_next = ego_v_y +ego_a_y_desire*self.step_length
                
                x_next = ego_x+ego_v_x_next*self.step_length #ACC
                # print('ego_x: ',ego_x)
                # print('x_next: ',x_next)
                # print('ego_v_x: ',ego_v)
                # print('ego_v_x_next: ',ego_v_x_next)
                # print('ego_a: ',ego_a)
                # print('ego_a_x_desire: ',ego_a_x_desire)
                y_next = ego_y+ego_v_y_next*self.step_length
                # print("ego_y",ego_y)
                # print("ego_v_y_next",ego_v_y_next)
                # print("ego_a_y_desire",ego_a_y_desire)
                # print("y_next: ",y_next)
                control = [traci.simulation.getTime(),self.__ego_acc(ego_v,v_preceding,c_desire,c_front),ego_a_y_des]
                rlEnv.ego_control.append(control)
                theta = np.arctan2(ego_y-last_ego_y,ego_x-last_ego_x)

                if(ego_y-last_ego_y>=0):
                    angle = 90-theta*10
                else:
                    angle = 90-theta*10

                if self.done != True and self.__ego_vehicle_LC_completed() !=True:
                    if(traci.vehicle.getLaneIndex('ego')==0 and traci.vehicle.getLateralLanePosition('ego')<=0):
                        print('stop moveToXY')
                        self.done = True
                    else:
                        # print(':::::::::::::::::::::::::::::::::::movetoXY')
                        traci.vehicle.moveToXY('ego',edgeID,lane,x_next,y_next,angle,keepRoute,matchThreshold)
                

        if('ego' in vehs and 'car.forward' in vehs and 'accel.rear' in vehs):
            nextstate = self.state('ego')            
            nextreward = self.__reward(rlEnv.step_num, action) # rear vehicle collision : -10, collision caused by ego : -20, LC_succeed_with_biggest_space : +20, LC_succeed_with_smaller_space : +10, step*-0.01
        else:
            nextstate = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0] ## it might wrong code
            nextreward = self.__reward(rlEnv.step_num, action)
            # print("else")
            # print(vehs)
            self.done = True  
        
        if len(traci.simulation.getCollidingVehiclesIDList()) !=0:
            log = traci.simulation.getCollisions()
            if str(log[0]).split(',')[1]== ' victim=ego' or str(log[0]).split(',')[0]== 'Collision(collider=ego' :
                print('done')
                self.collision_num +=1
                self.done = True

        for veh_id in vehs:
            # surrounding vehicles LK for first 2secs.
            # if veh_id[0] =='c'and traci.simulation.getTime()<2:
            #     traci.vehicle.setLaneChangeMode(veh_id,0b000000000000)

 
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
                ID = veh_id.split('.')
                velocity_y=0
                if veh_id== 'ego' and not (len(rlEnv.ego)==0):
                    velocity_y = (y1 - rlEnv.ego[-1][3])/self.step_length
                if veh_id== 'car.left0' and not (len(rlEnv.left0))==0:
                    velocity_y = (y1 - rlEnv.left0[-1][3])/self.step_length
                if veh_id== 'car.left1' and not (len(rlEnv.left1))==0:
                    velocity_y = (y1 - rlEnv.left1[-1][3])/self.step_length
                if veh_id== 'accel.rear' and not (len(rlEnv.rear))==0:
                    velocity_y = (y1 - rlEnv.rear[-1][3])/self.step_length
                if veh_id== 'car.right0' and not (len(rlEnv.right0))==0:
                    velocity_y = (y1 - rlEnv.right0[-1][3])/self.step_length
                if veh_id== 'car.right1' and not (len(rlEnv.right1))==0:
                    velocity_y = (y1 - rlEnv.right1[-1][3])/self.step_length
                if veh_id== 'car.left2' and not (len(rlEnv.left2))==0:
                    velocity_y = (y1 - rlEnv.left2[-1][3])/self.step_length
                if veh_id== 'car.left3' and not (len(rlEnv.left3))==0:
                    velocity_y = (y1 - rlEnv.left3[-1][3])/self.step_length
                if veh_id== 'car.left4' and not (len(rlEnv.left4))==0:
                    velocity_y = (y1 - rlEnv.left4[-1][3])/self.step_length
                if veh_id== 'car.left5' and not (len(rlEnv.left5))==0:
                    velocity_y = (y1 - rlEnv.left5[-1][3])/self.step_length
                if veh_id== 'car.left6' and not (len(rlEnv.left6))==0:
                    velocity_y = (y1 - rlEnv.left6[-1][3])/self.step_length
                if veh_id== 'car.left7' and not (len(rlEnv.left7))==0:
                    velocity_y = (y1 - rlEnv.left7[-1][3])/self.step_length
                if veh_id== 'car.forward' and not (len(rlEnv.forward))==0:
                    velocity_y = (y1 - rlEnv.forward[-1][3])/self.step_length
                if veh_id== 'car.right2' and not (len(rlEnv.right2))==0:
                    velocity_y = (y1 - rlEnv.right2[-1][3])/self.step_length
                if veh_id== 'car.right3' and not (len(rlEnv.right3))==0:
                    velocity_y = (y1 - rlEnv.right3[-1][3])/self.step_length
                if veh_id== 'car.right4' and not (len(rlEnv.right4))==0:
                    velocity_y = (y1 - rlEnv.right4[-1][3])/self.step_length
                if veh_id== 'car.right5' and not (len(rlEnv.right5))==0:
                    velocity_y = (y1 - rlEnv.right5[-1][3])/self.step_length

                vehicle_state.append(velocity_y)
                # print("v_y : ",traci.vehicle.getLateralSpeed(veh_id))
                
                
                acceleration_x =0
                
                
                acceleration_x= traci.vehicle.getAcceleration(veh_id)            
                vehicle_state.append(acceleration_x)
                acceleration_y=0
                if veh_id== 'ego' and not (len(rlEnv.ego)==0):
                    acceleration_y = (velocity_y - rlEnv.ego[-1][5])/self.step_length
                if veh_id== 'car.left0' and not (len(rlEnv.left0))==0:
                    acceleration_y = (velocity_y- rlEnv.left0[-1][5])/self.step_length
                if veh_id== 'car.left1' and not (len(rlEnv.left1))==0:
                    acceleration_y = (velocity_y - rlEnv.left1[-1][5])/self.step_length
                if veh_id== 'accel.rear' and not (len(rlEnv.rear))==0:
                    acceleration_y = (velocity_y - rlEnv.rear[-1][5])/self.step_length
                if veh_id== 'car.right0' and not (len(rlEnv.right0))==0:
                    acceleration_y = (velocity_y - rlEnv.right0[-1][5])/self.step_length
                if veh_id== 'car.right1' and not (len(rlEnv.right1))==0:
                    acceleration_y = (velocity_y - rlEnv.right1[-1][5])/self.step_length
                if veh_id== 'car.left2' and not (len(rlEnv.left2))==0:
                    acceleration_y = (velocity_y - rlEnv.left2[-1][5])/self.step_length
                if veh_id== 'car.left3' and not (len(rlEnv.left3))==0:
                    acceleration_y = (velocity_y - rlEnv.left3[-1][5])/self.step_length
                if veh_id== 'car.left4' and not (len(rlEnv.left4))==0:
                    acceleration_y = (velocity_y - rlEnv.left4[-1][5])/self.step_length
                if veh_id== 'car.left5' and not (len(rlEnv.left5))==0:
                    acceleration_y = (velocity_y - rlEnv.left5[-1][5])/self.step_length
                if veh_id== 'car.left6' and not (len(rlEnv.left6))==0:
                    acceleration_y = (velocity_y - rlEnv.left6[-1][5])/self.step_length
                if veh_id== 'car.left7' and not (len(rlEnv.left7))==0:
                    acceleration_y = (velocity_y - rlEnv.left7[-1][5])/self.step_length
                if veh_id== 'car.forward' and not (len(rlEnv.forward))==0:
                    acceleration_y = (velocity_y - rlEnv.forward[-1][5])/self.step_length
                if veh_id== 'car.right2' and not (len(rlEnv.right2))==0:
                    acceleration_y = (velocity_y - rlEnv.right2[-1][5])/self.step_length
                if veh_id== 'car.right3' and not (len(rlEnv.right3))==0:
                    acceleration_y = (velocity_y - rlEnv.right3[-1][5])/self.step_length
                if veh_id== 'car.right4' and not (len(rlEnv.right4))==0:
                    acceleration_y = (velocity_y - rlEnv.right4[-1][5])/self.step_length
                if veh_id== 'car.right5' and not (len(rlEnv.right5))==0:
                    acceleration_y = (velocity_y - rlEnv.right5[-1][5])/self.step_length

                vehicle_state.append(acceleration_y)
                
                
                vehicle_state.append(traci.vehicle.getAngle(veh_id))
            
                
                if(veh_id == 'ego'):
                    rlEnv.ego.append(vehicle_state)
                if(veh_id =='car.left0'):
                    rlEnv.left0.append(vehicle_state)
                if(veh_id=='car.left1'):
                    rlEnv.left1.append(vehicle_state)
                if(veh_id=='accel.rear'):
                    rlEnv.rear.append(vehicle_state)
                if(veh_id=='car.right0'):
                    rlEnv.right0.append(vehicle_state)
                if(veh_id=='car.right1'):
                    rlEnv.right1.append(vehicle_state)
                if(veh_id=='car.left2'):
                    rlEnv.left2.append(vehicle_state)
                if(veh_id=='car.left3'):
                    rlEnv.left3.append(vehicle_state)
                if(veh_id=='car.left4'):
                    rlEnv.left4.append(vehicle_state)
                if(veh_id=='car.left5'):
                    rlEnv.left5.append(vehicle_state)
                if(veh_id=='car.left6'):
                    rlEnv.left6.append(vehicle_state)
                if(veh_id=='car.left7'):
                    rlEnv.left7.append(vehicle_state)
                if(veh_id=='car.forward'):
                    rlEnv.forward.append(vehicle_state)
                if(veh_id=='car.right2'):
                    rlEnv.right2.append(vehicle_state)
                if(veh_id=='car.right3'):
                    rlEnv.right3.append(vehicle_state)
                if(veh_id=='car.right4'):
                    rlEnv.right4.append(vehicle_state)    
                if(veh_id=='car.right5'):
                    rlEnv.right5.append(vehicle_state)  


            ### ego_vehicle LC -> episode ends.
        
            if veh_id == 'ego':
                if self.__ego_vehicle_LC_start() and traci.simulation.getTime()>=0.01:
                    print('Lane change start')
                    self.LC_succeed_num +=1
                    rlEnv.ego_LC_start =True
                    traci.vehicle.setLaneChangeMode('ego',0)
                self.__ego_vehicle_LC_completed() # LC completed checking
                rlEnv.lane_buffer_ego = traci.vehicle.getLaneIndex('ego') # ego_car lane buffer 
                

            if traci.vehicle.getRoadID(veh_id) == 'E5' and veh_id[0] == 'a':
                traci.vehicle.setAccel(veh_id, '1')
                traci.vehicle.setDecel(veh_id, '0.00001')
                # traci.vehicle.setSpeedMode(veh_id,'0')
                self.__setvehiclestate('accel.rear')

            if (traci.vehicle.getRoadID(veh_id) == 'E5' and veh_id[0] == 'c'):
                c0 = 1.98
                # ID=veh_id.split('.')
                # if ID[1][0] =='r':
                #     tau = 1.6
                # else:
                #     tau = 1.36
                
                if veh_id == 'car.left0':
                    tau = rlEnv.vehicles_tau[0]                    
                elif veh_id =='car.left1':
                    tau = rlEnv.vehicles_tau[1]                    
                elif veh_id =='car.right0':
                    tau = rlEnv.vehicles_tau[2]                    
                elif veh_id =='car.right1':
                    tau = rlEnv.vehicles_tau[3]                    
                elif veh_id =='car.left2':
                    tau = rlEnv.vehicles_tau[4]               
                elif veh_id =='car.left3':
                    tau = rlEnv.vehicles_tau[5]                    
                elif veh_id =='car.left4':
                    tau = rlEnv.vehicles_tau[6]
                elif veh_id =='car.left5':
                    tau = rlEnv.vehicles_tau[7]
                elif veh_id =='car.left6':
                    tau = rlEnv.vehicles_tau[8]
                elif veh_id =='car.left7':
                    tau = rlEnv.vehicles_tau[9]
                elif veh_id =='car.forward':
                    tau = rlEnv.vehicles_tau[10]                    
                elif veh_id =='car.right2':
                    tau = rlEnv.vehicles_tau[11]                    
                elif veh_id =='car.right3':
                    tau = rlEnv.vehicles_tau[12]                    
                elif veh_id =='car.right4':
                    tau = rlEnv.vehicles_tau[13]
                elif veh_id =='car.right5':
                    tau = rlEnv.vehicles_tau[14]
                else:
                    tau = 1.36
                
                
                v_controled = traci.vehicle.getSpeed(veh_id)

                if traci.vehicle.getLeader(veh_id) is not None: # 선행 차량이 있을 때 ACC
                    Leader_id,c_front = traci.vehicle.getLeader(veh_id)
                    v_preceding= traci.vehicle.getSpeed(Leader_id)
                    c_desire = c0+tau*v_preceding
                    
                    ##### ACC with speed limit ########
                    speed_limit = 23
                    # traci.vehicle.setAcceleration(veh_id,self.__a_desire_with_speed_limit(v_controled,v_preceding,c_desire,c_front,speed_limit),20) #입력 가속도
                    self.__set_a(veh_id,self.__a_desire_with_speed_limit(v_controled,v_preceding,c_desire,c_front,speed_limit))
                    # self.__a_desire_with_speed_limit(v_controled,v_preceding,c_desire,c_front,speed_limit) #입력 가속도

                else: # 선두 차량 CC
                    self.__set_v(veh_id,22.22)
        vehs = traci.vehicle.getIDList()
        
        # print(rlEnv.step_num)
        rlEnv.step_num +=1
        
        self.ego_collision_happened()
        return nextstate,nextreward

    
    
    