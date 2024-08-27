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
from sympy import symbols #Derivative
import math
import matplotlib.pyplot as plt
import pylab
from datetime import datetime

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
    DEFAULT_VIEW = 'View #0'
    #class global variable
    last_state =[]
    lane_buffer_ego = 0
    ego_LC_success = False
    ego_LC_start = False
    ego_LC_completed = False
    last_Space1=0
    last_Space2=0
    last_Space3=0
    last_Space4=0
    S=[0,0,0,0,0,0,0,0,0]
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
    right6=[]
    right7=[]
    Target1=[]
    Target2=[]
    Target3=[]
    Target4=[]
    Target_left_car = ""
    Target_right_car =""
    crosstrack_error_term=[]
    heading_error_term=[]
    delta_term = []
    clearance_error_term =[]
    velocity_error_term = []
    Time=[]
    last_action = [[0,0]]
    Left_action = False
    Right_action = False
    rewards =[]
    toward_space_reward=[]
    toward_space_reward_long=[]
    toward_space_reward_lat=[]
    space1_count=0
    space2_count=0
    space3_count=0
    space4_count=0
    R1_count=0
    R2_count=0
    L1_count=0
    L2_count=0
    t_LC_start =0
    initial_x = 0
    initial_y = 0
    initial_target_y =0
    theta_i=0
    final_x=9999
    final_y=0
    step_num = 0
    coefficient=[0,0,0,0]
    Right_SD = False
    Left_SD = False
    Front_SD = True
    gui_on = True
    vehicles_tau=[random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27)]
    # vehicles_tau=[1.36]*20 ########### RL 검증
    tau_random = [random.uniform(0,1),random.uniform(0,1),random.uniform(0,1),random.uniform(0,1),random.uniform(0,1),random.uniform(0,1),random.uniform(0,1),random.uniform(0,1)]
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
        self.egoID = 'ego'
        


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
    def vehicle_position(self,ID):
        x,y = traci.vehicle.getPosition(ID)
        vehicle_length = traci.vehicle.getLength(ID)
        center_x = x-vehicle_length/2
        return [center_x, y]
    def reset(self):

        #initiate values of class rlEnv.
        rlEnv.last_state =[]
        rlEnv.lane_buffer_ego = 0
        rlEnv.ego_LC_success = False
        rlEnv.ego_LC_start = False
        rlEnv.ego_LC_completed = False
        rlEnv.last_Space1=0
        rlEnv.last_Space2=0
        rlEnv.last_Space3=0
        rlEnv.last_Space4=0
        rlEnv.S=[0,0,0,0,0,0,0,0,0]
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
        rlEnv.right6=[]
        rlEnv.right7=[]
        rlEnv.Target1=[]
        rlEnv.Target2=[]
        rlEnv.Target3=[]
        rlEnv.Target4=[]
        rlEnv.Target_left_car = ""
        rlEnv.Target_right_car =""
        rlEnv.last_action = [[0,0]]
        rlEnv.Left_action = False
        rlEnv.Right_action = False
        rlEnv.rewards =[]
        rlEnv.toward_space_reward=[]
        rlEnv.toward_space_reward_long=[]
        rlEnv.toward_space_reward_lat=[]
        rlEnv.t_LC_start =0
        rlEnv.initial_x = 0
        rlEnv.initial_y = 0
        rlEnv.initial_target_y =0
        rlEnv.theta_i=0
        rlEnv.final_x =9999
        rlEnv.final_y = 0
        rlEnv.step_num = 0
        rlEnv.coefficient=[0,0,0,0]
        rlEnv.crosstrack_error_term=[]
        rlEnv.heading_error_term=[]
        rlEnv.delta_term = []
        rlEnv.clearance_error_term =[]
        rlEnv.velocity_error_term = []    
        rlEnv.Time=[]
        rlEnv.Right_SD = False
        rlEnv.Left_SD = False
        rlEnv.Front_SD = True
        rlEnv.last_target_space =0
        rlEnv.last_vehicles = ['','']
        rlEnv.mode=[0,0,0,0,0,0]
        rlEnv.vehicles_tau=[random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27)]
        # rlEnv.vehicles_tau=[1.36]*20 ########### RL 검증
        rlEnv.tau_random = [random.uniform(0,1),random.uniform(0,1),random.uniform(0,1),random.uniform(0,1),random.uniform(0,1),random.uniform(0,1),random.uniform(0,1),random.uniform(0,1)]
        rlEnv.rearMaxSpeed = random.uniform(29.17,33.33)
        # if self.episode!=0: 
        #     self.sumo.close()
        #     sys.stdout.flush()
        # ego_rand = random.uniform(50,115)
        ego_initial_v = random.uniform(22.5,23)
        # ego_initial_v = 20
        forward_initial_v = random.uniform(21,23)
        # forward_initial_v = 20
        # rear_initial_v = random.uniform(27.77,29.17)
        rear_initial_v = random.uniform(25,27.77)
             
        # rear_initial_v = 27.77########### RL 검증
        # rear_initial_v = 17
        left_initial_v =[random.uniform(22.5,23),random.uniform(22.5,23),random.uniform(22.5,23),random.uniform(22.5,23),random.uniform(22.5,23),random.uniform(22.5,23),random.uniform(22.5,23),random.uniform(22.5,23)]
        right_initial_v =[random.uniform(22.5,23),random.uniform(22.5,23),random.uniform(22.5,23),random.uniform(22.5,23),random.uniform(22.5,23),random.uniform(22.5,23),random.uniform(22.5,23),random.uniform(22.5,23)]
        # left_initial_v = [22.5, 22.5, 22.5, 22.5, 22.5, 22.5, 22.5, 22.5]
        # right_initial_v = [22.5, 22.5, 22.5, 22.5, 22.5, 22.5, 22.5, 22.5]
        # ego_rand = random.uniform(70,100)
        # ego_rand = random.uniform(100,110)
        ego_rand = random.uniform(120,140)
        # ego_rand =135
        # ego_rand = random.uniform(98-15,98+15) ########### 
        # ego_rand = random.uniform(98-7,98+15) ########### RL 검증 space 3 or 4
        # ego_rand = 30+98
        forward_rand = random.uniform(30+150,30+170)
        # forward_rand = 180
        r0 =random.uniform(0,36)
        l0 =random.uniform(0,36)
        r_delta1 = random.uniform(31,40)
        r_delta2 = random.uniform(31,40)
        r_delta3 = random.uniform(31,40)
        r_delta4 = random.uniform(31,40)
        r_delta5 = random.uniform(31,40)
        r_delta6 = random.uniform(31,40)
        r_delta7 = random.uniform(31,40)
        l_delta1 = random.uniform(31,40)
        l_delta2 = random.uniform(31,40)
        l_delta3 = random.uniform(31,40)
        l_delta4 = random.uniform(31,40)
        l_delta5 = random.uniform(31,40)
        l_delta6 = random.uniform(31,40)
        l_delta7 = random.uniform(31,40)
        # r0 = 0
        # l0 = 0
        # r_delta1 = 31
        # r_delta2 = 31   #4번 위치!!!
        # r_delta3 = 31    #2번 위치!!!
        # r_delta4 = 31
        # r_delta5 = 31
        # r_delta6 = 31
        # r_delta7 = 31
        # l_delta1 = 31
        # l_delta2 = 31     #3번 위치!!!
        # l_delta3 = 31    #1번 위치!!!
        # l_delta4 = 31
        # l_delta5 = 31
        # l_delta6 = 31
        # l_delta7 = 31########### RL 검증

        r1 = r0 + r_delta1
        r2 = r1 + r_delta2
        r3 = r2 + r_delta3
        r4 = r3 + r_delta4
        r5 = r4 + r_delta5
        r6 = r5 + r_delta6
        r7 = r6 + r_delta7
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
        traci.vehicle.add(vehID='backupcar.forward',routeID='route0',typeID="car",depart='now',departLane='1',departPos=str(forward_rand+40), departSpeed=str(ego_initial_v), arrivalLane='current', arrivalPos='max', arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0, personNumber=0)
        traci.vehicle.add(vehID='backupcar.rear',routeID='route0',typeID="car",depart='2',departLane='1',departPos='0', departSpeed=str(ego_initial_v), arrivalLane='current', arrivalPos='max', arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0, personNumber=0)

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
        traci.vehicle.add(vehID='car.right6',routeID='route0',typeID="car",depart='now',departLane='0',departPos=str(r6), departSpeed=str(right_initial_v[6]), arrivalLane='current', arrivalPos='max', arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0, personNumber=0)
        traci.vehicle.add(vehID='car.right7',routeID='route0',typeID="car",depart='now',departLane='0',departPos=str(r7), departSpeed=str(right_initial_v[7]), arrivalLane='current', arrivalPos='max', arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0, personNumber=0)
        
        traci.simulationStep()
        return self.state('ego')

        
    def end(self):
        rlEnv.ego_LC_start=False
        rlEnv.ego_LC_completed=False
        traci.close()
        sys.stdout.flush()
    def save_action_csv(self,episode,name):
        col_action = ['time','action']
        df_ego_action = pd.DataFrame(rlEnv.last_action,columns=col_action)
        file_path = '/home/mds/Desktop/highway_episodic/DQN/RL_validation/data_log/data/'+name+'/action/'
        os.makedirs(file_path, exist_ok=True)
        os.chdir(file_path)
        df_ego_action.to_csv('log_data_ego_action'+str(episode)+'.csv')
        
    def save_csv(self,episode,name):
         
        col=['time','vehicle_id','x','y','v_x','v_y','a_x','a_y','heading_angle']
        col_target = ['time','num','clearance','x','v_x']
        col_control = ['time','desire_a_x','desire_a_y']
        col_action = ['time','action']
        col_reward = ['time','r1','r2','r2_1','r3','r4','r5','r6','r7','r8','r9','total']
        col_space_reward = ['time','first_size','second_size','third_size','forth_size' ]
        # col_episode_result = ['episode','collision','space1','space2','space3','space4']
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
        df_right6 = pd.DataFrame(rlEnv.right6,columns=col)
        df_right7 = pd.DataFrame(rlEnv.right7,columns=col)
        df_target1 = pd.DataFrame(rlEnv.Target1,columns=col_target)
        df_target2 = pd.DataFrame(rlEnv.Target2,columns=col_target)
        df_target3 = pd.DataFrame(rlEnv.Target3,columns=col_target)
        df_target4 = pd.DataFrame(rlEnv.Target4,columns=col_target)
        df_ego_control = pd.DataFrame(rlEnv.ego_control,columns=col_control)
        df_ego_action = pd.DataFrame(rlEnv.last_action,columns=col_action)
        df_reward = pd.DataFrame(rlEnv.rewards,columns=col_reward)
        df_space_reward = pd.DataFrame(rlEnv.toward_space_reward,columns=col_space_reward)
        df_space_reward_long = pd.DataFrame(rlEnv.toward_space_reward_long,columns=col_space_reward)
        df_space_reward_lat = pd.DataFrame(rlEnv.toward_space_reward_lat,columns=col_space_reward)
        # df_episode_result = pd.DataFrame([self.episode, self.collision_num,self.space1_count,self.space2_count,self.space3_count,self.space4_count],columns=col_episode_result)
        file_path = '/home/mds/Desktop/highway_episodic/DQN/RL_validation/data_log/data/'+'/'+name+'/'+str(episode)
        os.makedirs(file_path, exist_ok=True)
        os.chdir(file_path)
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
        df_right5.to_csv('log_data_right4.csv')
        df_right6.to_csv('log_data_right5.csv')
        df_target1.to_csv('log_data_target1.csv')
        df_target2.to_csv('log_data_target2.csv')
        df_target3.to_csv('log_data_target3.csv')
        df_target4.to_csv('log_data_target4.csv')
        df_ego_control.to_csv('log_data_ego_control.csv')
        df_ego_action.to_csv('log_data_ego_action.csv')
        df_reward.to_csv('log_data_reward.csv')
        df_space_reward.to_csv('log_data_space_reward.csv')
        df_space_reward_long.to_csv('log_data_space_reward_long.csv')
        df_space_reward_lat.to_csv('log_data_space_reward_lat.csv')
        # df_episode_result.to_csv('log_data_episode_result.csv')
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
    
    def __ego_vehicle_LC_completed(self,action):
        vehs = traci.vehicle.getIDList()
        if 'ego' in vehs:
            if rlEnv.ego_LC_start == True:
                if ((traci.vehicle.getLaneIndex('ego')== 2 and (action == 1 or action == 3)) or (traci.vehicle.getLaneIndex('ego')== 0 and (action == 2 or action == 4))) and traci.vehicle.getAngle('ego') >= 89.9 and traci.vehicle.getAngle('ego') <= 90.1 and np.abs(traci.vehicle.getLateralLanePosition('ego'))<=0.5:  
                # if ((traci.vehicle.getLaneIndex('ego')== 2 and (action == 1 or action == 3)) or (traci.vehicle.getLaneIndex('ego')== 0 and (action == 2 or action == 4))):
                    rlEnv.ego_LC_completed =True
                    self.done = True
                    print('Lanechange completed')
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
        
        if(len(rlEnv.ego)!=0):            
            v_x_cur = (traci.vehicle.getPosition(id)[0] - rlEnv.ego[-1][2])/dt
            a_x_cur = (v_x_cur -rlEnv.ego[-1][4])/dt
        else:
            v_x_cur= traci.vehicle.getSpeed(id)
            a_x_cur = traci.vehicle.getAcceleration(id)
        # v_x_cur= traci.vehicle.getSpeed(id)
        # a_x_cur = traci.vehicle.getAcceleration(id)
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
        if a_y_des>5:
            a_y_des = 5
        elif a_y_des<-5:
            a_y_des = -5
        else:
            a_y_des = a_y_des

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
    def __LC_goal_space_posistion(self,action):
        left_vehicles=[] #[|ego_x_pos - x_pos|, id]
        right_vehicles=[] #[|ego_x_pos - x_pos|, id]
        left_near_vehicles=[] #[x_pos]
        right_near_vehicles =[] #[x_pos]      
        vehs = traci.vehicle.getIDList()
        if 'ego' in vehs:
            id = 'ego'
            lane = traci.vehicle.getLaneIndex(id)
            if len(rlEnv.last_action) ==0:
                LLP = 0
            else:
                LLP = traci.vehicle.getLateralLanePosition('ego') #LateralLanePosition
                if np.abs(LLP)<0.15:
                    LLP = 0
            traci.vehicle.subscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 300.0, [tc.VAR_POSITION])

            if lane == 0 and LLP <0:
                if action == 5:
                    for v_id in traci.vehicle.getContextSubscriptionResults(str(id)):
                        if(traci.vehicle.getLaneIndex(v_id)==0) and not (v_id=='ego'):
                            left_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),traci.vehicle.getPosition(v_id)[0],v_id])
                    left_vehicles.sort()
                    for i in range(2):
                        left_near_vehicles.append([left_vehicles[:3][i][1],left_vehicles[:3][i][2]])# 1: left vehicle position 2: left vehicle id
                    left_near_vehicles.sort()
                    left_target_space = [(left_near_vehicles[1][0]+left_near_vehicles[0][0])/2,-8]
                    vehicles = [left_near_vehicles[1][1],left_near_vehicles[0][1]]
                    traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 300.0)
                    return left_target_space,vehicles
                elif action ==6:
                    for v_id in traci.vehicle.getContextSubscriptionResults(str(id)):
                        if(traci.vehicle.getLaneIndex(v_id)==0) and not (v_id=='ego'):
                            right_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),traci.vehicle.getPosition(v_id)[0],v_id])
                    right_vehicles.sort()
                    for i in range(2):
                        right_near_vehicles.append([right_vehicles[:3][i][1],right_vehicles[:3][i][2]])# 1: right vehicle position 2: right vehicle id
                    right_near_vehicles.sort()
                    right_target_space = [(right_near_vehicles[1][0]+right_near_vehicles[0][0])/2,-8]
                    vehicles = [right_near_vehicles[1][1],right_near_vehicles[0][1]]
                    print("Wrong action!!")
                    traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 300.0)
                    return right_target_space, vehicles
                else:
                    traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 300.0)
                    return rlEnv.last_target_space , rlEnv.last_vehicles
                        
            elif lane == 0 and LLP >=0:
                if action == 5:
                    # for v_id in traci.vehicle.getContextSubscriptionResults(str(id)):
                    #     if(traci.vehicle.getLaneIndex(v_id)==1) and not (v_id=='ego'):
                    #         left_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),traci.vehicle.getPosition(v_id)[0],v_id])
                    # left_vehicles.sort()
                    # if len(left_vehicles)<=1:
                    left_target_space = [traci.vehicle.getPosition('ego')[0]+50,-4.8]
                    vehicles = ['ego','ego'] #...
                    # else:
                    #     for i in range(2):
                    #         left_near_vehicles.append([left_vehicles[:3][i][1],left_vehicles[:3][i][2]])# 1: left vehicle position 2: left vehicle id
                    #     left_near_vehicles.sort()
                    #     left_target_space = [(left_near_vehicles[1][0]+left_near_vehicles[0][0])/2,-4.8]
                    #     vehicles = [left_near_vehicles[1][1],left_near_vehicles[0][1]]
                    traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 300.0)
                    return left_target_space,vehicles
                elif action ==6:
                    for v_id in traci.vehicle.getContextSubscriptionResults(str(id)):
                        if(traci.vehicle.getLaneIndex(v_id)==0) and not (v_id=='ego'):
                            right_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),traci.vehicle.getPosition(v_id)[0],v_id])
                    right_vehicles.sort()
                    for i in range(2):
                        right_near_vehicles.append([right_vehicles[:3][i][1],right_vehicles[:3][i][2]])# 1: right vehicle position 2: right vehicle id
                    right_near_vehicles.sort()
                    right_target_space = [(right_near_vehicles[1][0]+right_near_vehicles[0][0])/2,-8]
                    vehicles = [right_near_vehicles[1][1],right_near_vehicles[0][1]]
                    traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 300.0)
                    return right_target_space,vehicles
                else:
                    traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 300.0)
                    return rlEnv.last_target_space , rlEnv.last_vehicles
            elif lane == 1 and LLP<0:
                if action == 5:
                    # for v_id in traci.vehicle.getContextSubscriptionResults(str(id)):
                    #     if(traci.vehicle.getLaneIndex(v_id)==1) and not (v_id=='ego'):
                    #         left_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),traci.vehicle.getPosition(v_id)[0],v_id])
                    # left_vehicles.sort()
                    # if len(left_vehicles)<=1:
                    left_target_space = [traci.vehicle.getPosition('ego')[0]+50,-4.8]
                    vehicles = ['ego','ego']
                    # else:
                    #     for i in range(2):
                    #         left_near_vehicles.append([left_vehicles[:3][i][1],left_vehicles[:3][i][2]])# 1: left vehicle position 2: left vehicle id
                    #     left_near_vehicles.sort()
                    #     left_target_space = [(left_near_vehicles[1][0]+left_near_vehicles[0][0])/2,-4.8]
                    #     vehicles = [left_near_vehicles[1][1],left_near_vehicles[0][1]]
                    traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 300.0)
                    return left_target_space, vehicles
                elif action == 6:
                    for v_id in traci.vehicle.getContextSubscriptionResults(str(id)):
                        if(traci.vehicle.getLaneIndex(v_id)==0) and not (v_id=='ego'):
                            right_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),traci.vehicle.getPosition(v_id)[0],v_id])
                    right_vehicles.sort() 
                    for i in range(2):
                        right_near_vehicles.append([right_vehicles[:3][i][1],right_vehicles[:3][i][2]])# 1: right vehicle position 2: right vehicle id
                    right_near_vehicles.sort()
                    right_target_space = [(right_near_vehicles[1][0]+right_near_vehicles[0][0])/2,-8]
                    vehicles = [right_near_vehicles[1][1],right_near_vehicles[0][1]]
                    traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 300.0)
                    return right_target_space, vehicles
                else:
                    traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 300.0)
                    return rlEnv.last_target_space , rlEnv.last_vehicles
            elif lane == 1 and LLP==0:
                if action == 5:
                    for v_id in traci.vehicle.getContextSubscriptionResults(str(id)):
                        if(traci.vehicle.getLaneIndex(v_id)==2) and not (v_id=='ego'):
                            left_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),traci.vehicle.getPosition(v_id)[0],v_id])
                    left_vehicles.sort()
                    # print(left_vehicles)
                    # print(left_vehicles[:3])
                    for i in range(2):
                        left_near_vehicles.append([left_vehicles[:3][i][1],left_vehicles[:3][i][2]])# 1: left vehicle position 2: left vehicle id
                    left_near_vehicles.sort()
                    left_target_space = [(left_near_vehicles[1][0]+left_near_vehicles[0][0])/2,-1.6]
                    vehicles = [left_near_vehicles[1][1],left_near_vehicles[0][1]]
                    traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 300.0)
                    return left_target_space, vehicles
                elif action == 6:
                    for v_id in traci.vehicle.getContextSubscriptionResults(str(id)):
                        if(traci.vehicle.getLaneIndex(v_id)==0) and not (v_id=='ego'):
                            right_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),traci.vehicle.getPosition(v_id)[0],v_id])
                    right_vehicles.sort()
                    # print(right_vehicles)
                    # print(right_vehicles[:3])
                    for i in range(2):
                        right_near_vehicles.append([right_vehicles[:3][i][1],right_vehicles[:3][i][2]])# 1: right vehicle position 2: right vehicle id
                    right_near_vehicles.sort()
                    right_target_space = [(right_near_vehicles[1][0]+right_near_vehicles[0][0])/2,-8]
                    vehicles = [right_near_vehicles[1][1],right_near_vehicles[0][1]]
                    traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 300.0)
                    return right_target_space,vehicles
                else:
                    traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 300.0)
                    return rlEnv.last_target_space , rlEnv.last_vehicles
                        
            elif lane == 1 and LLP>0:
                if action == 5:
                    for v_id in traci.vehicle.getContextSubscriptionResults(str(id)):
                        if(traci.vehicle.getLaneIndex(v_id)==2) and not (v_id=='ego'):
                            left_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),traci.vehicle.getPosition(v_id)[0],v_id])
                    left_vehicles.sort()
                    for i in range(2):
                        left_near_vehicles.append([left_vehicles[:3][i][1],left_vehicles[:3][i][2]])# 1: left vehicle position 2: left vehicle id
                    left_near_vehicles.sort()
                    left_target_space = [(left_near_vehicles[1][0]+left_near_vehicles[0][0])/2,-1.6]
                    vehicles = [left_near_vehicles[1][1],left_near_vehicles[0][1]]
                    traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 300.0)
                    return left_target_space, vehicles
                elif action == 6:
                    # for v_id in traci.vehicle.getContextSubscriptionResults(str(id)):
                    #     if(traci.vehicle.getLaneIndex(v_id)==1) and not (v_id=='ego'):
                    #         right_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),traci.vehicle.getPosition(v_id)[0],v_id])
                    # right_vehicles.sort()
                    # if len(right_vehicles)<=1:
                    right_target_space = [traci.vehicle.getPosition('ego')[0]+50,-4.8]
                    vehicles = ['ego','ego']
                    # else:
                    #     for i in range(2):
                    #         right_near_vehicles.append([right_vehicles[:3][i][1],right_vehicles[:3][i][2]])# 1: right vehicle position 2: right vehicle id
                    #     right_near_vehicles.sort()
                    #     right_target_space = [(right_near_vehicles[1][0]+right_near_vehicles[0][0])/2,-4.8]
                    #     vehicles = [right_near_vehicles[1][1],right_near_vehicles[0][1]]
                    traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 300.0)
                    return right_target_space, vehicles
                else:
                    traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 300.0)
                    return rlEnv.last_target_space , rlEnv.last_vehicles
            
            elif lane == 2 and LLP<=0:
                if action == 5:
                    for v_id in traci.vehicle.getContextSubscriptionResults(str(id)):
                        if(traci.vehicle.getLaneIndex(v_id)==2) and not (v_id=='ego'):
                            left_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),traci.vehicle.getPosition(v_id)[0],v_id])
                    left_vehicles.sort()
                    for i in range(2):
                        left_near_vehicles.append([left_vehicles[:3][i][1],left_vehicles[:3][i][2]])# 1: left vehicle position 2: left vehicle id
                    left_near_vehicles.sort()
                    left_target_space = [(left_near_vehicles[1][0]+left_near_vehicles[0][0])/2,-1.6]
                    vehicles = [left_near_vehicles[1][1],left_near_vehicles[0][1]]
                    traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 300.0)
                    return left_target_space,vehicles
                elif action == 6:
                    # for v_id in traci.vehicle.getContextSubscriptionResults(str(id)):
                    #     if(traci.vehicle.getLaneIndex(v_id)==1) and not (v_id=='ego'):
                    #         right_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),traci.vehicle.getPosition(v_id)[0],v_id])
                    # right_vehicles.sort()
                    # if len(right_vehicles)<=1:
                    right_target_space = [traci.vehicle.getPosition('ego')[0]+50,-4.8]
                    vehicles = ['ego','ego']
                    # else:
                    #     for i in range(2):
                    #         right_near_vehicles.append([right_vehicles[:3][i][1],right_vehicles[:3][i][2]])# 1: right vehicle position 2: right vehicle id
                    #     right_near_vehicles.sort()
                    #     right_target_space = [(right_near_vehicles[1][0]+right_near_vehicles[0][0])/2,-4.8]
                    #     vehicles = [right_near_vehicles[1][1],right_near_vehicles[0][1]]
                    traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 300.0)
                    return right_target_space, vehicles
                else:
                    traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 300.0)
                    return rlEnv.last_target_space , rlEnv.last_vehicles
            elif lane == 2 and LLP>0:
                if action == 5:
                    for v_id in traci.vehicle.getContextSubscriptionResults(str(id)):
                        if(traci.vehicle.getLaneIndex(v_id)==2) and not (v_id=='ego'):
                            left_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),traci.vehicle.getPosition(v_id)[0],v_id])
                    left_vehicles.sort()
                    for i in range(2):
                        left_near_vehicles.append([left_vehicles[:3][i][1],left_vehicles[:3][i][2]])# 1: left vehicle position 2: left vehicle id
                    left_near_vehicles.sort()
                    left_target_space = [(left_near_vehicles[1][0]+left_near_vehicles[0][0])/2,-1.6]
                    print("Wrong action!!!")
                    vehicles = [left_near_vehicles[1][1],left_near_vehicles[0][1]]
                    traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 300.0)
                    return left_target_space, vehicles
                elif action == 6:
                    for v_id in traci.vehicle.getContextSubscriptionResults(str(id)):
                        if(traci.vehicle.getLaneIndex(v_id)==2) and not (v_id=='ego'):
                            right_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),traci.vehicle.getPosition(v_id)[0],v_id])
                    right_vehicles.sort()
                    for i in range(2):
                        right_near_vehicles.append([right_vehicles[:3][i][1],right_vehicles[:3][i][2]])# 1: right vehicle position 2: right vehicle id
                    right_near_vehicles.sort()
                    right_target_space = [(right_near_vehicles[1][0]+right_near_vehicles[0][0])/2,-1.6]
                    vehicles = [right_near_vehicles[1][1],right_near_vehicles[0][1]]
                    traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 300.0)
                    return right_target_space, vehicles
                else:
                    traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 300.0)
                    return rlEnv.last_target_space , rlEnv.last_vehicles
            else:
                traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 300.0)
                return rlEnv.last_target_space , rlEnv.last_vehicles
            
            
        else:
            traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 300.0)
            return  rlEnv.last_target_space, rlEnv.last_vehicles
    
    
    # def __LC_sin_path_des_a(self,goal_x_rel,goal_y_rel):
    #     x_t = rlEnv.ego[-1][3]
    #     v_x_cur = traci.vehicle.getSpeed('ego')
    #     a_x_cur = traci.vehicle.getAcceleration('ego')
    #     x_t += rlEnv.ego[-1][3] - rlEnv.ego[-2][3]
    #     des_a_y = goal_y_rel*(-(np.pi/goal_x_rel)**2*np.sin(np.pi/goal_x_rel*x_t-np.pi/2)*v_x_cur**2)/2 + goal_y_rel*(np.pi/goal_x_rel*np.cos(np.pi/goal_x_rel*x_t-np.pi/2)*a_x_cur)/2
        # return des_a_y    
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
        # print('a_d: ',a_d)
        if(a_d > 3):
            return 3
        elif(a_d <= 3 and a_d >= -5):
            return a_d
        elif(a_d< -5):
            return -5
        
    def __ego_a_desire(self, v_c,v_t,c_d,c,space_num): # desire acceleration by ACC, v_c : ego_vehicle's velocity, v_t : target velocity, c_d : desire clearance, c = actual clearance
        P=2.8
        P2=2
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
        P=2.8
        P2=2
        Ki=0
        Ki2 = 0
        Kd = 0
        Kd2 = 0
        # Ki=0.0021
        # Ki2 = 0.09
        # Kd = 0.0021
        # Kd2 = 0.99
        # #action4
        # Ki=0.001
        # Ki2 = 0.04
        # Kd = 0.002
        # Kd2 = 0.9
        # action3
        Ki=0.001
        Ki2 = 0.05
        Kd = 0.005
        Kd2 = 0.99
        rlEnv.integral_clearance_e[space_num] += Ki*(c_d -c)*self.step_length
        rlEnv.integral_velocity_e[space_num] += Ki2*(v_t-v_c)*self.step_length

        clearance_error = (c_d -c)
        velocity_error = (v_t-v_c)
        if len(rlEnv.pre_error_velocity)>0 and len(rlEnv.pre_error_velocity)>0:
            a_d = -P*self.__K1(v_c)*(c_d-c) -P2*self.__K2(v_c)*(v_t-v_c) -rlEnv.integral_clearance_e[space_num] +rlEnv.integral_velocity_e[space_num] -Kd*(clearance_error-rlEnv.pre_error_clearance[space_num])/self.step_length +Kd2*(velocity_error-rlEnv.pre_error_velocity[space_num])
        else:
            a_d = -P*self.__K1(v_c)*(c_d-c) -P2*self.__K2(v_c)*(v_t-v_c) -rlEnv.integral_clearance_e[space_num] +rlEnv.integral_velocity_e[space_num]
        # a_d = -self.__K1(v_c)*(c_d-c) - self.__K2(v_c)*(v_t-v_c)
        # print('ego_v: ',v_c)
        # print('target_v: ',v_t)
        rlEnv.clearance_error_term.append(-P*self.__K1(v_c)*(c_d-c))
        rlEnv.velocity_error_term.append(-P2*self.__K2(v_c)*(v_t-v_c))
        rlEnv.Time.append(traci.simulation.getTime())
        # plt.figure(1,figsize=(8,4))            
        # pylab.plot(rlEnv.Time, rlEnv.clearance_error_term, 'r',label='c_error_term')
        # pylab.plot(rlEnv.Time, rlEnv.velocity_error_term, 'b',label='vel_error_term')
        # pylab.xlabel("error_term")
        # pylab.ylabel("time")
        # pylab.savefig("/home/mds/Desktop/highway_episodic/DQN/RL_validation/pid_graph/pid_error_term.png")
        if(a_d >3):
            return 3
        elif(a_d <= 3 and a_d >= -5):
            return a_d
        elif(a_d< -3):
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
        dt = self.step_length    
        # goal_j_x = random.uniform(1,3)
        # max_a_x = random.uniform(1,2)
        # min_a_x = random.uniform(0,1)
        
        jerk = random.uniform(1,1.5)
        
        # a_des = random.uniform(1,2)
        a_des = random.uniform(0.5,1)
        a_cur = traci.vehicle.getAcceleration(id)
        if(a_cur+jerk*dt<a_des):
            traci.vehicle.setAcceleration(id,a_cur+jerk*dt,1)            
        elif(a_cur-jerk*dt>a_des):
            traci.vehicle.setAcceleration(id,a_cur-jerk*dt,1)            
        else:
            traci.vehicle.setAcceleration(id,a_des,1)

    def __setvehiclestate2(self, id):
        dt = self.step_length            
        jerk = 5
        a_des = 0
        a_cur = traci.vehicle.getAcceleration(id)
        if(a_cur+jerk*dt<a_des):
            traci.vehicle.setAcceleration(id,a_cur+jerk*dt,1)            
        elif(a_cur-jerk*dt>a_des):
            traci.vehicle.setAcceleration(id,a_cur-jerk*dt,1)            
        else:
            traci.vehicle.setAcceleration(id,a_des,1)

    def __set_a(self,id,a_des): #return 값은 jerk 값을 고려한 현실적인 가속도 값을 반환한다. # SUMO에 가속도를 입력한다.
        jerk = 5
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
        jerk = 5
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
    


    def __set_ego_a_movetoXY(self,id,a_des): #return 값은 jerk 값을 고려한 현실적인 가속도 값을 반환한다. # SUMO에 가속도를 입력한다.
        jerk = 5
        jerk_y = 5
        
        v_y_des = 0
        
        dt = self.step_length
        traci.vehicle.setSpeedMode('ego',32)
        traci.vehicle.setLaneChangeMode('ego',0)
        a_cur = traci.vehicle.getAcceleration(id)
        edgeID = "E5"
        lane = traci.vehicle.getLaneIndex('ego') 
        v_x_cur = traci.vehicle.getSpeed('ego')
        x_cur,y_cur = traci.vehicle.getPosition('ego')
        if len(rlEnv.ego)!=0:
            last_x = rlEnv.ego[-1][2]
            last_y = rlEnv.ego[-1][3]
            v_y_cur = (traci.vehicle.getPosition(id)[1] - rlEnv.ego[-1][3])/dt
            a_y_cur = (v_y_cur -rlEnv.ego[-1][5])/dt
        else:
            last_x = x_cur
            last_y = y_cur
            v_y_cur = 0
            a_y_cur = 0
        
        #################################################### 가장 가까운 lane의 중심으로 a_y_des 결정. P-gain K 사용.
        target_lane_y = [-8,-4.8,-1.6]
        K =2
        KD = 2
        dy_lane = []
        for i, lane_y in enumerate(target_lane_y):
            dy_lane.append([np.abs(lane_y -y_cur),i])
        dy_lane.sort()
        e_y = dy_lane[0][0]
        target_lane_index = dy_lane[0][1]
        if target_lane_y[target_lane_index] > y_cur:
            a_y_des = K*e_y-KD*v_y_cur
        elif target_lane_y[target_lane_index] < y_cur:
            a_y_des = -K*e_y-KD*v_y_cur
        else:
            a_y_des = 0
        # print('a_y_des: ',a_y_des)
        if a_y_des>5:
            a_y_des = 5
        elif a_y_des <-5:
            a_y_des = -5
        else:
            a_y_des = a_y_des
        # print('a_y_des_fixed: ',a_y_des)
        ### a_y_des가 0으로 수렴하도록 함.
        # if(v_y_cur >v_y_des):
        #     a_y_des = 2*(v_y_des-v_y_cur)
        # elif(v_y_cur<v_y_des):
        #     a_y_des = 2*(v_y_des-v_y_cur)
        # else:
        #     a_y_des = 0
        # y_next = y_cur + v_y_cur*dt+0.5*a_y_cur*dt**2
        if(a_y_cur+jerk_y*dt<a_y_des):
            v_y_next = v_y_cur+(a_y_cur+jerk_y*dt)*dt
            y_next = y_cur + v_y_next*dt
        elif(a_y_cur-jerk_y*dt>a_y_des):
            v_y_next = v_y_cur+(a_y_cur-jerk_y*dt)*dt
            y_next = y_cur + v_y_next*dt
        else:
            v_y_next = v_y_cur+a_y_des*dt
            y_next = y_cur + v_y_next*dt
            
        # print('y_cur: ',y_cur)
        # if len(rlEnv.ego) != 0:
        #     if rlEnv.ego[-1][8]<90:
        #         angle = rlEnv.ego[-1][8] +4*dt
        #     elif rlEnv.ego[-1][8]>90:
        #         angle = rlEnv.ego[-1][8] -4*dt
        #     else:
        #         angle =90
        # else:
        #     angle = 90
########################################################
        ego_y=y_cur   
        last_ego_y =last_y
        ego_x=x_cur
        last_ego_x =last_x
        theta = np.arctan2(ego_y-last_ego_y,ego_x-last_ego_x)

        if(ego_y-last_ego_y>=0):
            angle = 90-theta*180/np.pi
        else:
            angle = 90-theta*180/np.pi
            
        keepRoute = 2 #기존의 route에 제한 받지 안고 moveToXY 실행.
        matchThreshold = 3.2
        if(a_cur+jerk*dt<a_des):
            traci.vehicle.setAcceleration(id,a_cur+jerk*dt,1)
            v_next = v_x_cur + (a_cur+jerk*dt)*dt
            x_next = x_cur +v_next*dt      
            # return a_cur+jerk*dt
        elif(a_cur-jerk*dt>a_des):
            traci.vehicle.setAcceleration(id,a_cur-jerk*dt,1)
            v_next = v_x_cur + (a_cur-jerk*dt)*dt
            x_next = x_cur +v_next*dt 
            # return a_cur-jerk*dt
        else:
            traci.vehicle.setAcceleration(id,a_des,1)
            v_next = v_x_cur + a_des*dt
            x_next = x_cur +v_next*dt 
            # return a_des
        # theta = np.arctan2(y_cur-rlEnv.ego[-1][3],x_cur-rlEnv.ego[-1][2])
        # if(y_cur-rlEnv.ego[-1][3]>=0):
        #     angle = 90-theta*30
        # else:
        #     angle = 90-theta*30
        # print("angle:::::::::::::::::",angle)
        traci.vehicle.moveToXY('ego',"dummy",-1,x_next,y_next,angle,keepRoute,matchThreshold)
        return a_y_des
    
    def __set_ego_a_movetoXY2(self,id,a_des,action): #return 값은 jerk 값을 고려한 현실적인 가속도 값을 반환한다. # SUMO에 가속도를 입력한다.
        jerk = 5
        jerk_y = 5
        
        v_y_des = 0
        
        dt = self.step_length
        traci.vehicle.setSpeedMode('ego',32)
        traci.vehicle.setLaneChangeMode('ego',0)
        a_cur = traci.vehicle.getAcceleration(id)
        edgeID = "E5"
        lane = traci.vehicle.getLaneIndex('ego') 
        v_x_cur = traci.vehicle.getSpeed('ego')
        x_cur,y_cur = traci.vehicle.getPosition('ego')
        if len(rlEnv.ego)!=0:
            last_x = rlEnv.ego[-1][2]
            last_y = rlEnv.ego[-1][3]
            v_y_cur = (traci.vehicle.getPosition(id)[1] - rlEnv.ego[-1][3])/dt
            a_y_cur = (v_y_cur -rlEnv.ego[-1][5])/dt
        else:
            last_x = x_cur
            last_y = y_cur
            v_y_cur = 0
            a_y_cur = 0
        
        #################################################### 중앙 lane 1의 중심으로 a_y_des 결정. P-gain K 사용.
        target_lane_y = -4.8
        epsilon_d = 12

        if action == 1  and lane == 2 and x_cur > traci.vehicle.getPosition(rlEnv.Target_left_car)[0] + epsilon_d:
            target_lane_y = -1.6
        if action == 3 and lane == 2 and x_cur < traci.vehicle.getPosition(rlEnv.Target_left_car)[0] - epsilon_d:
            target_lane_y = -1.6
        if action == 2 and lane == 0 and x_cur > traci.vehicle.getPosition(rlEnv.Target_right_car)[0] + epsilon_d:
            target_lane_y = -8
        if action == 4 and lane == 0 and x_cur < traci.vehicle.getPosition(rlEnv.Target_right_car)[0] - epsilon_d:
            target_lane_y = -8

        K =2
        KD = 2
        e_y = np.abs(target_lane_y-y_cur)

        if target_lane_y > y_cur:
            a_y_des = K*e_y-KD*v_y_cur
        elif target_lane_y < y_cur:
            a_y_des = -K*e_y-KD*v_y_cur
        else:
            a_y_des = 0
        # print('a_y_des: ',a_y_des)

        if a_y_des>5:
            a_y_des = 5
        elif a_y_des <-5:
            a_y_des = -5
        else:
            a_y_des = a_y_des
        # print('a_y_des_fixed: ',a_y_des)
        

        ### a_y_des가 0으로 수렴하도록 함.
        # if(v_y_cur >v_y_des):
        #     a_y_des = 2*(v_y_des-v_y_cur)
        # elif(v_y_cur<v_y_des):
        #     a_y_des = 2*(v_y_des-v_y_cur)
        # else:
        #     a_y_des = 0
        # y_next = y_cur + v_y_cur*dt+0.5*a_y_cur*dt**2
        if(a_y_cur+jerk_y*dt<a_y_des):
            v_y_next = v_y_cur+(a_y_cur+jerk_y*dt)*dt
            y_next = y_cur + v_y_next*dt
        elif(a_y_cur-jerk_y*dt>a_y_des):
            v_y_next = v_y_cur+(a_y_cur-jerk_y*dt)*dt
            y_next = y_cur + v_y_next*dt
        else:
            v_y_next = v_y_cur+a_y_des*dt
            y_next = y_cur + v_y_next*dt
            
        # print('y_cur: ',y_cur)
        # if len(rlEnv.ego) != 0:
        #     if rlEnv.ego[-1][8]<90:
        #         angle = rlEnv.ego[-1][8] +4*dt
        #     elif rlEnv.ego[-1][8]>90:
        #         angle = rlEnv.ego[-1][8] -4*dt
        #     else:
        #         angle =90
        # else:
        #     angle = 90
########################################################
        ego_y=y_cur   
        last_ego_y =last_y
        ego_x=x_cur
        last_ego_x =last_x
        theta = np.arctan2(ego_y-last_ego_y,ego_x-last_ego_x)

        if(ego_y-last_ego_y>=0):
            angle = 90-theta*180/np.pi
        else:
            angle = 90-theta*180/np.pi
            
        keepRoute = 2 #기존의 route에 제한 받지 안고 moveToXY 실행.
        matchThreshold = 3.2
        if(a_cur+jerk*dt<a_des):
            traci.vehicle.setAcceleration(id,a_cur+jerk*dt,1)
            v_next = v_x_cur + (a_cur+jerk*dt)*dt
            x_next = x_cur +v_next*dt      
            # return a_cur+jerk*dt
        elif(a_cur-jerk*dt>a_des):
            traci.vehicle.setAcceleration(id,a_cur-jerk*dt,1)
            v_next = v_x_cur + (a_cur-jerk*dt)*dt
            x_next = x_cur +v_next*dt 
            # return a_cur-jerk*dt
        else:
            traci.vehicle.setAcceleration(id,a_des,1)
            v_next = v_x_cur + a_des*dt
            x_next = x_cur +v_next*dt 
            # return a_des
        # theta = np.arctan2(y_cur-rlEnv.ego[-1][3],x_cur-rlEnv.ego[-1][2])
        # if(y_cur-rlEnv.ego[-1][3]>=0):
        #     angle = 90-theta*30
        # else:
        #     angle = 90-theta*30
        # print("angle:::::::::::::::::",angle)
        traci.vehicle.moveToXY('ego',"dummy",-1,x_next,y_next,angle,keepRoute,matchThreshold)
        return a_y_des
        
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
    def __set_ego_v_movetoXY(self,id,v_des): # v_des에 따라 a_des을 결정하고 jerk 값이 고려된  속도를 시스템에 넣음.
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
        a_y_des = self.__set_ego_a_movetoXY(id,a_des)
        return a_des,a_y_des
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
    def minmaxscale(self,s_value,s_min,s_max):
        value = (s_value-s_min)/(s_max - s_min)
        if value >1:
            return 1
        elif value < 0:
            return 0
        else:
            return value

    # surounding vehicle (rel_x, rel_y) ->8
    # ego states: heading angle, lane num, ego_y ->3
    # space 1~4 (rel_x,rel_y), space size ->8
    def state(self,id = 'ego'):
        states = []
        vehs = traci.vehicle.getIDList()
        if id in vehs and rlEnv.Target_left_car in vehs and rlEnv.Target_right_car in vehs:
            if traci.vehicle.getLeader(rlEnv.Target_left_car)[0] == 'ego' and traci.vehicle.getLaneIndex('ego')==2:
                left_leader = traci.vehicle.getLeader('ego')[0]
            elif traci.vehicle.getLeader(rlEnv.Target_left_car)[0] == 'ego' and traci.vehicle.getLaneIndex('ego')==1:
                left_leader = 'car.left'+str(int(rlEnv.Target_left_car[-1])+1)
            else:
                left_leader = traci.vehicle.getLeader(rlEnv.Target_left_car)[0]
            if traci.vehicle.getFollower(rlEnv.Target_left_car)[0] =='ego' and traci.vehicle.getLaneIndex('ego')==2:
                left_follower = traci.vehicle.getFollower('ego')[0]
            elif traci.vehicle.getFollower(rlEnv.Target_left_car)[0] =='ego' and traci.vehicle.getLaneIndex('ego')==1:
                left_follower = 'car.left'+str(int(rlEnv.Target_left_car[-1])-1)
            else:
                left_follower = traci.vehicle.getFollower(rlEnv.Target_left_car)[0]
            
            if traci.vehicle.getLeader(rlEnv.Target_right_car)[0] == 'ego' and traci.vehicle.getLaneIndex('ego')==0:
                right_leader = traci.vehicle.getLeader('ego')[0]
            elif traci.vehicle.getLeader(rlEnv.Target_right_car)[0] == 'ego' and traci.vehicle.getLaneIndex('ego')==1:
                right_leader = 'car.right'+str(int(rlEnv.Target_right_car[-1])+1)
            else:
                right_leader = traci.vehicle.getLeader(rlEnv.Target_right_car)[0]
            if traci.vehicle.getFollower(rlEnv.Target_right_car)[0] =='ego' and traci.vehicle.getLaneIndex('ego')==0:
                right_follower = traci.vehicle.getFollower('ego')[0]
            elif traci.vehicle.getFollower(rlEnv.Target_right_car)[0] =='ego' and traci.vehicle.getLaneIndex('ego')==1:
                right_follower = 'car.right'+str(int(rlEnv.Target_right_car[-1])-1)
            else:
                right_follower = traci.vehicle.getFollower(rlEnv.Target_right_car)[0]

            rear_rel_x, rear_rel_y = np.asarray(traci.vehicle.getPosition('accel.rear')) - np.asarray(traci.vehicle.getPosition('ego'))
            # print('rear_rel_x: ',rear_rel_x)
            # print(rear_rel_x == None)
            # if rear_rel_x == None and len(rlEnv.rear) !=0:
            #     for sublist in reversed(rlEnv.rear):
            #         if sublist[2] == None:
            #             pass
            #         else:
            #             last_non_none = sublist[2]
            #             break
            #     rear_rel_x = last_non_none
            rear_v = traci.vehicle.getSpeed('accel.rear')
            forward_rel_x,forward_rel_y = np.asarray(traci.vehicle.getPosition('car.forward')) - np.asarray(traci.vehicle.getPosition('ego'))
            forward_v = traci.vehicle.getSpeed('car.forward')
            left_leader_rel_x,left_leader_rel_y = np.asarray(traci.vehicle.getPosition(left_leader)) - np.asarray(traci.vehicle.getPosition('ego'))
            left_leader_v = traci.vehicle.getSpeed(left_leader)
            left_rel_x, left_rel_y = np.asarray(traci.vehicle.getPosition(rlEnv.Target_left_car)) - np.asarray(traci.vehicle.getPosition('ego'))
            left_v = traci.vehicle.getSpeed(rlEnv.Target_left_car)
            left_follower_rel_x, left_follower_rel_y = np.asarray(traci.vehicle.getPosition(left_follower)) - np.asarray(traci.vehicle.getPosition('ego'))
            left_follower_v = traci.vehicle.getSpeed(left_follower)
            right_leader_rel_x, right_leader_rel_y = np.asarray(traci.vehicle.getPosition(right_leader)) - np.asarray(traci.vehicle.getPosition('ego'))
            right_leader_v = traci.vehicle.getSpeed(right_leader)
            right_rel_x, right_rel_y = np.asarray(traci.vehicle.getPosition(rlEnv.Target_right_car)) - np.asarray(traci.vehicle.getPosition('ego'))
            right_v = traci.vehicle.getSpeed(rlEnv.Target_right_car)
            right_follower_rel_x, right_follower_rel_y = np.asarray(traci.vehicle.getPosition(right_follower)) - np.asarray(traci.vehicle.getPosition('ego'))
            right_follower_v = traci.vehicle.getSpeed(right_follower)
            ego_heading_angle = traci.vehicle.getAngle('ego')
            # ego_lane_index = traci.vehicle.getLaneIndex('ego')
            ego_x = 0
            ego_y = traci.vehicle.getPosition('ego')[1]
            ego_v = traci.vehicle.getSpeed('ego')

            s1_rel_x, s1_rel_y = ((traci.vehicle.getPosition(rlEnv.Target_left_car)[0] + traci.vehicle.getPosition(left_leader)[0])/2 - traci.vehicle.getPosition('ego')[0]), (traci.vehicle.getPosition(rlEnv.Target_left_car)[1]- traci.vehicle.getPosition('ego')[1])
            s1_v = (traci.vehicle.getSpeed(rlEnv.Target_left_car) + traci.vehicle.getSpeed(left_leader))/2
            s2_rel_x, s2_rel_y = ((traci.vehicle.getPosition(rlEnv.Target_right_car)[0] + traci.vehicle.getPosition(right_leader)[0])/2 - traci.vehicle.getPosition('ego')[0]), (traci.vehicle.getPosition(rlEnv.Target_right_car)[1]- traci.vehicle.getPosition('ego')[1])
            s2_v = (traci.vehicle.getSpeed(rlEnv.Target_right_car) + traci.vehicle.getSpeed(right_leader))/2
            s3_rel_x, s3_rel_y = ((traci.vehicle.getPosition(rlEnv.Target_left_car)[0] + traci.vehicle.getPosition(left_follower)[0])/2 - traci.vehicle.getPosition('ego')[0]), (traci.vehicle.getPosition(rlEnv.Target_left_car)[1]- traci.vehicle.getPosition('ego')[1])
            s3_v = (traci.vehicle.getSpeed(rlEnv.Target_left_car) + traci.vehicle.getSpeed(left_follower))/2
            s4_rel_x, s4_rel_y = ((traci.vehicle.getPosition(rlEnv.Target_right_car)[0] + traci.vehicle.getPosition(right_follower)[0])/2 - traci.vehicle.getPosition('ego')[0]), (traci.vehicle.getPosition(rlEnv.Target_right_car)[1]- traci.vehicle.getPosition('ego')[1])
            s4_v = (traci.vehicle.getSpeed(rlEnv.Target_right_car) + traci.vehicle.getSpeed(right_follower))/2
            smid_rel_x, smid_rel_y = ((traci.vehicle.getPosition('accel.rear')[0] + traci.vehicle.getPosition('car.forward')[0])/2 - traci.vehicle.getPosition('ego')[0]), (traci.vehicle.getPosition('car.forward')[1]- traci.vehicle.getPosition('ego')[1])
            smid_v = (traci.vehicle.getSpeed('accel.rear')+traci.vehicle.getSpeed('car.forward'))/2
            space1_size = np.abs((traci.vehicle.getPosition(rlEnv.Target_left_car)[0] - traci.vehicle.getPosition(left_leader)[0]))
            space2_size = np.abs((traci.vehicle.getPosition(rlEnv.Target_right_car)[0] - traci.vehicle.getPosition(right_leader)[0]))
            space3_size = np.abs((traci.vehicle.getPosition(rlEnv.Target_left_car)[0] - traci.vehicle.getPosition(left_follower)[0]))
            space4_size = np.abs((traci.vehicle.getPosition(rlEnv.Target_right_car)[0] - traci.vehicle.getPosition(right_follower)[0]))
            spacemid_size = np.abs((traci.vehicle.getPosition('car.forward')[0] - traci.vehicle.getPosition('accel.rear')[0]))
            lane_width_double =3.2*2
            lane_width = 3.2
            max_detectable_range = 150
            space_size_range = 300
            rear_rel_x_scaled = self.minmaxscale(rear_rel_x,-max_detectable_range,max_detectable_range)
            rear_rel_y_scaled = self.minmaxscale(rear_rel_y,-lane_width_double,lane_width_double)
            rear_v_scaled = self.minmaxscale(rear_v,0,40)
            forward_rel_x_scaled = self.minmaxscale(forward_rel_x,-max_detectable_range,max_detectable_range)
            forward_rel_y_scaled = self.minmaxscale(forward_rel_y,-lane_width_double,lane_width_double)
            forward_v_scaled = self.minmaxscale(forward_v,0,40)
            left_leader_rel_x_scaled = self.minmaxscale(left_leader_rel_x,-max_detectable_range,max_detectable_range)
            left_leader_rel_y_scaled =self.minmaxscale(left_leader_rel_y,-lane_width_double,lane_width_double)
            left_leader_v_scaled = self.minmaxscale(left_leader_v,0,40)
            left_rel_x_scaled =self.minmaxscale(left_rel_x,-max_detectable_range,max_detectable_range)
            left_rel_y_scaled =self.minmaxscale(left_rel_y,-lane_width_double,lane_width_double)
            left_v_scaled = self.minmaxscale(left_v,0,40)
            left_follower_rel_x_scaled = self.minmaxscale(left_follower_rel_x,-max_detectable_range,max_detectable_range)
            left_follower_rel_y_scaled = self.minmaxscale(left_follower_rel_y,-lane_width_double,lane_width_double)
            left_follower_v_scaled = self.minmaxscale(left_follower_v,0,40)
            right_leader_rel_x_scaled = self.minmaxscale(right_leader_rel_x,-max_detectable_range,max_detectable_range)
            right_leader_rel_y_scaled =self.minmaxscale(right_leader_rel_y,-lane_width_double,lane_width_double)
            right_leader_v_scaled = self.minmaxscale(right_leader_v,0,40)
            right_rel_x_scaled =self.minmaxscale(right_rel_x,-max_detectable_range,max_detectable_range)
            right_rel_y_scaled =self.minmaxscale(right_rel_y,-lane_width_double,lane_width_double)
            right_v_scaled = self.minmaxscale(right_v,0,40)
            right_follower_rel_x_scaled = self.minmaxscale(right_follower_rel_x,-max_detectable_range,max_detectable_range)
            right_follower_rel_y_scaled = self.minmaxscale(right_follower_rel_y,-lane_width_double,lane_width_double)
            right_follower_v_scaled = self.minmaxscale(right_follower_v,0,40)
            ego_heading_angle_scaled =self.minmaxscale(ego_heading_angle,0,180)
            # ego_lane_index_scaled =ego_lane_index/10
            ego_x_scaled = ego_x
            ego_y_scaled = self.minmaxscale(ego_y,-1*lane_width*3,0)
            ego_v_scaled = self.minmaxscale(ego_v,0,40)

            s1_rel_x_scaled =self.minmaxscale(s1_rel_x,-max_detectable_range,max_detectable_range)
            s1_rel_y_scaled =self.minmaxscale(s1_rel_y,-lane_width_double,lane_width_double)
            s1_v_scaled = self.minmaxscale(s1_v,0,40)
            s2_rel_x_scaled =self.minmaxscale(s2_rel_x,-max_detectable_range,max_detectable_range)
            s2_rel_y_scaled =self.minmaxscale(s2_rel_y,-lane_width_double,lane_width_double)
            s2_v_scaled = self.minmaxscale(s2_v,0,40)
            s3_rel_x_scaled =self.minmaxscale(s3_rel_x,-max_detectable_range,max_detectable_range)
            s3_rel_y_scaled =self.minmaxscale(s3_rel_y,-lane_width_double,lane_width_double)
            s3_v_scaled = self.minmaxscale(s3_v,0,40)
            s4_rel_x_scaled =self.minmaxscale(s4_rel_x,-max_detectable_range,max_detectable_range)
            s4_rel_y_scaled =self.minmaxscale(s4_rel_y,-lane_width_double,lane_width_double)
            s4_v_scaled = self.minmaxscale(s4_v,0,40)
            smid_rel_x_scaled = self.minmaxscale(smid_rel_x,-max_detectable_range,max_detectable_range)
            smid_rel_y_scaled = self.minmaxscale(smid_rel_y,-max_detectable_range,max_detectable_range)
            smid_v_scaled = self.minmaxscale(smid_v,0,40)
            space1_size_scaled =self.minmaxscale(space1_size,0,space_size_range)
            space2_size_scaled =self.minmaxscale(space2_size,0,space_size_range)
            space3_size_scaled =self.minmaxscale(space3_size,0,space_size_range)
            space4_size_scaled =self.minmaxscale(space4_size,0,space_size_range)
            spacemid_size_scaled = self.minmaxscale(spacemid_size,0,space_size_range)
            states.append(rear_rel_x_scaled)
            states.append(rear_rel_y_scaled)
            states.append(rear_v_scaled)
            states.append(forward_rel_x_scaled)
            states.append(forward_rel_y_scaled)
            states.append(forward_v_scaled)
            states.append(left_leader_rel_x_scaled)
            states.append(left_leader_rel_y_scaled)
            states.append(left_leader_v_scaled)
            states.append(left_rel_x_scaled)
            states.append(left_rel_y_scaled)
            states.append(left_v_scaled)
            states.append(left_follower_rel_x_scaled)
            states.append(left_follower_rel_y_scaled)
            states.append(left_follower_v_scaled)
            states.append(right_leader_rel_x_scaled)
            states.append(right_leader_rel_y_scaled)
            states.append(right_leader_v_scaled)
            states.append(right_rel_x_scaled)
            states.append(right_rel_y_scaled)
            states.append(right_v_scaled)
            states.append(right_follower_rel_x_scaled)
            states.append(right_follower_rel_y_scaled)
            states.append(right_follower_v_scaled)
            
            states.append(ego_x_scaled)
            states.append(ego_y_scaled)
            states.append(ego_v_scaled)
            states.append(ego_heading_angle_scaled)
            # states.append(ego_lane_index_scaled)

            states.append(s1_rel_x_scaled)
            states.append(s1_rel_y_scaled)
            states.append(s1_v_scaled)
            states.append(space1_size_scaled)
            states.append(s2_rel_x_scaled)
            states.append(s2_rel_y_scaled)
            states.append(s2_v_scaled)
            states.append(space2_size_scaled)
            states.append(s3_rel_x_scaled)
            states.append(s3_rel_y_scaled)
            states.append(s3_v_scaled)
            states.append(space3_size_scaled)
            states.append(s4_rel_x_scaled)
            states.append(s4_rel_y_scaled)
            states.append(s4_v_scaled)
            states.append(space4_size_scaled)
            states.append(smid_rel_x_scaled)
            states.append(smid_rel_y_scaled)
            states.append(smid_v_scaled)
            states.append(spacemid_size_scaled)
        else:
            left_vehicles=[] #[|ego_x_pos - x_pos|, id]
            right_vehicles=[] #[|ego_x_pos - x_pos|, id]

            left_near_vehicles=[] #[x_pos]
            right_near_vehicles =[] #[x_pos]
            
            left_near_spaces=[] #[space_x_pos, space_size]
            right_near_spaces=[] #[space_x_pos, space_size]
            ego_to_left_spaces=[]
            ego_to_right_spaces=[]
        

            traci.vehicle.subscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 130.0, [tc.VAR_POSITION])
            # print(id,' subscribeContext')
            for v_id in traci.vehicle.getContextSubscriptionResults(str(id)):
                if(traci.vehicle.getLaneIndex(v_id)==2) and not (v_id=='ego'):
                    left_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),traci.vehicle.getPosition(v_id)[0],v_id])
                if(traci.vehicle.getLaneIndex(v_id)==0)and not (v_id=='ego') :
                    right_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),traci.vehicle.getPosition(v_id)[0],v_id])
                    
            traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 130.0)
            
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
            left_leader = left_near_vehicles[2][1] # left first: 왼쪽 차로에 있는 ego vehicle에서 가까운 3대 중 앞에서 첫번째 차량까지 거리
            left_car= left_near_vehicles[1][1] # left second: 왼쪽 차로에 있는 ego vehicle에서 가까운 3대 중 앞에서 두번째 차량까지 거리
            left_follower = left_near_vehicles[0][1] # left third: 왼쪽 차로에 있는 ego vehicle에서 가까운 3대 중 앞에서 세번째 차량까지 거리
            right_leader = right_near_vehicles[2][1] # right third: 오른쪽 차로에 있는 ego vehicle에서 가까운 3대 중 앞에서 첫번째 차량까지 거리
            right_car = right_near_vehicles[1][1] # right third: 오른쪽 차로에 있는 ego vehicle에서 가까운 3대 중 앞에서 두번째 차량까지 거리
            right_follower = right_near_vehicles[0][1] # right third: 오른쪽 차로에 있는 ego vehicle에서 가까운 3대 중 앞에서 세번째 차량까지 거리
            
            rear_rel_x, rear_rel_y = np.asarray(traci.vehicle.getPosition('accel.rear')) - np.asarray(traci.vehicle.getPosition('ego'))
            # if rear_rel_x == None:
            #     rear_rel_x = 0-traci.vehicle.getPosition('ego')[0]
                
            rear_v = traci.vehicle.getSpeed('accel.rear')
            forward_rel_x,forward_rel_y = np.asarray(traci.vehicle.getPosition('car.forward')) - np.asarray(traci.vehicle.getPosition('ego'))
            forward_v = traci.vehicle.getSpeed('car.forward')
            left_leader_rel_x,left_leader_rel_y = np.asarray(traci.vehicle.getPosition(left_leader)) - np.asarray(traci.vehicle.getPosition('ego'))
            left_leader_v = traci.vehicle.getSpeed(left_leader)
            left_rel_x, left_rel_y = np.asarray(traci.vehicle.getPosition(left_car)) - np.asarray(traci.vehicle.getPosition('ego'))
            left_v = traci.vehicle.getSpeed(left_car)
            left_follower_rel_x, left_follower_rel_y = np.asarray(traci.vehicle.getPosition(left_follower)) - np.asarray(traci.vehicle.getPosition('ego'))
            left_follower_v = traci.vehicle.getSpeed(left_follower)
            right_leader_rel_x, right_leader_rel_y = np.asarray(traci.vehicle.getPosition(right_leader)) - np.asarray(traci.vehicle.getPosition('ego'))
            right_leader_v = traci.vehicle.getSpeed(right_leader)
            right_rel_x, right_rel_y = np.asarray(traci.vehicle.getPosition(right_car)) - np.asarray(traci.vehicle.getPosition('ego'))
            right_v = traci.vehicle.getSpeed(right_car)
            right_follower_rel_x, right_follower_rel_y = np.asarray(traci.vehicle.getPosition(right_follower)) - np.asarray(traci.vehicle.getPosition('ego'))
            right_follower_v = traci.vehicle.getSpeed(right_follower)
            ego_heading_angle = traci.vehicle.getAngle('ego')
            # ego_lane_index = traci.vehicle.getLaneIndex('ego')
            ego_x = 0
            ego_y = traci.vehicle.getPosition('ego')[1]
            ego_v = traci.vehicle.getSpeed('ego')

            s1_rel_x, s1_rel_y = ((traci.vehicle.getPosition(left_car)[0] + traci.vehicle.getPosition(left_leader)[0])/2 - traci.vehicle.getPosition('ego')[0]), (traci.vehicle.getPosition(left_car)[1]- traci.vehicle.getPosition('ego')[1])
            s1_v = (traci.vehicle.getSpeed(left_car) + traci.vehicle.getSpeed(left_leader))/2
            s2_rel_x, s2_rel_y = ((traci.vehicle.getPosition(right_car)[0] + traci.vehicle.getPosition(right_leader)[0])/2 - traci.vehicle.getPosition('ego')[0]), (traci.vehicle.getPosition(right_car)[1]- traci.vehicle.getPosition('ego')[1])
            s2_v = (traci.vehicle.getSpeed(right_car) + traci.vehicle.getSpeed(right_leader))/2
            s3_rel_x, s3_rel_y = ((traci.vehicle.getPosition(left_car)[0] + traci.vehicle.getPosition(left_follower)[0])/2 - traci.vehicle.getPosition('ego')[0]), (traci.vehicle.getPosition(left_car)[1]- traci.vehicle.getPosition('ego')[1])
            s3_v = (traci.vehicle.getSpeed(left_car) + traci.vehicle.getSpeed(left_follower))/2
            s4_rel_x, s4_rel_y = ((traci.vehicle.getPosition(right_car)[0] + traci.vehicle.getPosition(right_follower)[0])/2 - traci.vehicle.getPosition('ego')[0]), (traci.vehicle.getPosition(right_car)[1]- traci.vehicle.getPosition('ego')[1])
            s4_v = (traci.vehicle.getSpeed(right_car) + traci.vehicle.getSpeed(right_follower))/2
            smid_rel_x, smid_rel_y = ((traci.vehicle.getPosition('accel.rear')[0] + traci.vehicle.getPosition('car.forward')[0])/2 - traci.vehicle.getPosition('ego')[0]), (traci.vehicle.getPosition('car.forward')[1]- traci.vehicle.getPosition('ego')[1])
            smid_v = (traci.vehicle.getSpeed('accel.rear')+traci.vehicle.getSpeed('car.forward'))/2
            spacemid_size = np.abs((traci.vehicle.getPosition('car.forward')[0] - traci.vehicle.getPosition('accel.rear')[0]))
            space1_size = np.abs((traci.vehicle.getPosition(left_car)[0] - traci.vehicle.getPosition(left_leader)[0]))
            space2_size = np.abs((traci.vehicle.getPosition(right_car)[0] - traci.vehicle.getPosition(right_leader)[0]))
            space3_size = np.abs((traci.vehicle.getPosition(left_car)[0] - traci.vehicle.getPosition(left_follower)[0]))
            space4_size = np.abs((traci.vehicle.getPosition(right_car)[0] - traci.vehicle.getPosition(right_follower)[0]))


            lane_width_double =3.2*2
            lane_width = 3.2
            max_detectable_range = 150
            space_size_range = 300
            rear_rel_x_scaled = self.minmaxscale(rear_rel_x,-max_detectable_range,max_detectable_range)
            rear_rel_y_scaled = self.minmaxscale(rear_rel_y,-lane_width_double,lane_width_double)
            rear_v_scaled = self.minmaxscale(rear_v,0,40)
            forward_rel_x_scaled = self.minmaxscale(forward_rel_x,-max_detectable_range,max_detectable_range)
            forward_rel_y_scaled = self.minmaxscale(forward_rel_y,-lane_width_double,lane_width_double)
            forward_v_scaled = self.minmaxscale(forward_v,0,40)
            left_leader_rel_x_scaled = self.minmaxscale(left_leader_rel_x,-max_detectable_range,max_detectable_range)
            left_leader_rel_y_scaled =self.minmaxscale(left_leader_rel_y,-lane_width_double,lane_width_double)
            left_leader_v_scaled = self.minmaxscale(left_leader_v,0,40)
            left_rel_x_scaled =self.minmaxscale(left_rel_x,-max_detectable_range,max_detectable_range)
            left_rel_y_scaled =self.minmaxscale(left_rel_y,-lane_width_double,lane_width_double)
            left_v_scaled = self.minmaxscale(left_v,0,40)
            left_follower_rel_x_scaled = self.minmaxscale(left_follower_rel_x,-max_detectable_range,max_detectable_range)
            left_follower_rel_y_scaled = self.minmaxscale(left_follower_rel_y,-lane_width_double,lane_width_double)
            left_follower_v_scaled = self.minmaxscale(left_follower_v,0,40)
            right_leader_rel_x_scaled = self.minmaxscale(right_leader_rel_x,-max_detectable_range,max_detectable_range)
            right_leader_rel_y_scaled =self.minmaxscale(right_leader_rel_y,-lane_width_double,lane_width_double)
            right_leader_v_scaled = self.minmaxscale(right_leader_v,0,40)
            right_rel_x_scaled =self.minmaxscale(right_rel_x,-max_detectable_range,max_detectable_range)
            right_rel_y_scaled =self.minmaxscale(right_rel_y,-lane_width_double,lane_width_double)
            right_v_scaled = self.minmaxscale(right_v,0,40)
            right_follower_rel_x_scaled = self.minmaxscale(right_follower_rel_x,-max_detectable_range,max_detectable_range)
            right_follower_rel_y_scaled = self.minmaxscale(right_follower_rel_y,-lane_width_double,lane_width_double)
            right_follower_v_scaled = self.minmaxscale(right_follower_v,0,40)
            ego_heading_angle_scaled =self.minmaxscale(ego_heading_angle,0,180)
            # ego_lane_index_scaled =ego_lane_index/10
            ego_x_scaled = ego_x
            ego_y_scaled = self.minmaxscale(ego_y,-1*lane_width*3,0)
            ego_v_scaled = self.minmaxscale(ego_v,0,40)
           
            s1_rel_x_scaled =self.minmaxscale(s1_rel_x,-max_detectable_range,max_detectable_range)
            s1_rel_y_scaled =self.minmaxscale(s1_rel_y,-lane_width_double,lane_width_double)
            s1_v_scaled = self.minmaxscale(s1_v,0,40)
            s2_rel_x_scaled =self.minmaxscale(s2_rel_x,-max_detectable_range,max_detectable_range)
            s2_rel_y_scaled =self.minmaxscale(s2_rel_y,-lane_width_double,lane_width_double)
            s2_v_scaled = self.minmaxscale(s2_v,0,40)
            s3_rel_x_scaled =self.minmaxscale(s3_rel_x,-max_detectable_range,max_detectable_range)
            s3_rel_y_scaled =self.minmaxscale(s3_rel_y,-lane_width_double,lane_width_double)
            s3_v_scaled = self.minmaxscale(s3_v,0,40)
            s4_rel_x_scaled =self.minmaxscale(s4_rel_x,-max_detectable_range,max_detectable_range)
            s4_rel_y_scaled =self.minmaxscale(s4_rel_y,-lane_width_double,lane_width_double)
            s4_v_scaled = self.minmaxscale(s4_v,0,40)
            smid_rel_x_scaled = self.minmaxscale(smid_rel_x,-max_detectable_range,max_detectable_range)
            smid_rel_y_scaled = self.minmaxscale(smid_rel_y,-max_detectable_range,max_detectable_range)
            smid_v_scaled = self.minmaxscale(smid_v,0,40)
            space1_size_scaled =self.minmaxscale(space1_size,0,space_size_range)
            space2_size_scaled =self.minmaxscale(space2_size,0,space_size_range)
            space3_size_scaled =self.minmaxscale(space3_size,0,space_size_range)
            space4_size_scaled =self.minmaxscale(space4_size,0,space_size_range)
            spacemid_size_scaled = self.minmaxscale(spacemid_size,0,space_size_range)
        
            states.append(rear_rel_x_scaled)
            states.append(rear_rel_y_scaled)
            states.append(rear_v_scaled)
            states.append(forward_rel_x_scaled)
            states.append(forward_rel_y_scaled)
            states.append(forward_v_scaled)
            states.append(left_leader_rel_x_scaled)
            states.append(left_leader_rel_y_scaled)
            states.append(left_leader_v_scaled)
            states.append(left_rel_x_scaled)
            states.append(left_rel_y_scaled)
            states.append(left_v_scaled)
            states.append(left_follower_rel_x_scaled)
            states.append(left_follower_rel_y_scaled)
            states.append(left_follower_v_scaled)
            states.append(right_leader_rel_x_scaled)
            states.append(right_leader_rel_y_scaled)
            states.append(right_leader_v_scaled)
            states.append(right_rel_x_scaled)
            states.append(right_rel_y_scaled)
            states.append(right_v_scaled)
            states.append(right_follower_rel_x_scaled)
            states.append(right_follower_rel_y_scaled)
            states.append(right_follower_v_scaled)
            
            states.append(ego_x_scaled)
            states.append(ego_y_scaled)
            states.append(ego_v_scaled)
            states.append(ego_heading_angle_scaled)
            # states.append(ego_lane_index_scaled)
            
            states.append(s1_rel_x_scaled)
            states.append(s1_rel_y_scaled)
            states.append(s1_v_scaled)
            states.append(space1_size_scaled)
            states.append(s2_rel_x_scaled)
            states.append(s2_rel_y_scaled)
            states.append(s2_v_scaled)
            states.append(space2_size_scaled)
            states.append(s3_rel_x_scaled)
            states.append(s3_rel_y_scaled)
            states.append(s3_v_scaled)
            states.append(space3_size_scaled)
            states.append(s4_rel_x_scaled)
            states.append(s4_rel_y_scaled)
            states.append(s4_v_scaled)
            states.append(space4_size_scaled)
            states.append(smid_rel_x_scaled)
            states.append(smid_rel_y_scaled)
            states.append(smid_v_scaled)
            states.append(spacemid_size_scaled)
        states = np.array(states)
        rlEnv.last_state = states
        # print('def state: ',states)
        return states
            


    # def state(self,id):       
    #     states = [] # ego_forward_clearance, ego_rear_clearance, ego_lane_num, ego_to_left_spaces, ego_to_right_spaces, left_space_size, right_space_size
    #     vehs = traci.vehicle.getIDList()
    #     if id in vehs and rlEnv.Target_left_car in vehs and rlEnv.Target_right_car in vehs:
    #         if traci.vehicle.getLeader(rlEnv.Target_left_car)[0] == 'ego':
    #             left_leader = traci.vehicle.getLeader('ego')[0]
    #         else:
    #             left_leader = traci.vehicle.getLeader(rlEnv.Target_left_car)[0]
    #         if traci.vehicle.getFollower(rlEnv.Target_left_car)[0] =='ego':
    #             left_follower = traci.vehicle.getFollower('ego')[0]
    #         else:
    #             left_follower = traci.vehicle.getFollower(rlEnv.Target_left_car)[0]
            
    #         if traci.vehicle.getLeader(rlEnv.Target_right_car)[0] == 'ego':
    #             right_leader = traci.vehicle.getLeader('ego')[0]
    #         else:
    #             right_leader = traci.vehicle.getLeader(rlEnv.Target_right_car)[0]
    #         if traci.vehicle.getFollower(rlEnv.Target_right_car)[0] =='ego':
    #             right_follower = traci.vehicle.getFollower('ego')[0]
    #         else:
    #             right_follower = traci.vehicle.getFollower(rlEnv.Target_right_car)[0]
    #         # print('state1')
    #         s1 = np.sqrt(((traci.vehicle.getPosition(rlEnv.Target_left_car)[0] + traci.vehicle.getPosition(left_leader)[0])/2 - traci.vehicle.getPosition('ego')[0])**2 +(traci.vehicle.getPosition(rlEnv.Target_left_car)[1]- traci.vehicle.getPosition('ego')[1])**2)#자차와 왼쪽 차로 전방 빈공간의 중점 까지 상대 거리
    #         s1_scaled = self.minmaxscale(s1,0,200)
    #         s2 = np.sqrt(((traci.vehicle.getPosition(rlEnv.Target_right_car)[0] + traci.vehicle.getPosition(right_leader)[0])/2 - traci.vehicle.getPosition('ego')[0])**2+(traci.vehicle.getPosition(rlEnv.Target_right_car)[1]- traci.vehicle.getPosition('ego')[1])**2) #자차와 오른쪽 차로 전방 빈공간의 중점 까지 상대 거리
    #         s2_scaled = self.minmaxscale(s2,0,200)
    #         s3 = np.sqrt(((traci.vehicle.getPosition(rlEnv.Target_left_car)[0] + traci.vehicle.getPosition(left_follower)[0])/2 - traci.vehicle.getPosition('ego')[0])**2+(traci.vehicle.getPosition(rlEnv.Target_left_car)[1]- traci.vehicle.getPosition('ego')[1])**2) #자차와 왼쪽 차로 후방 빈공간의 중점 까지 상대 거리            
    #         s3_scaled = self.minmaxscale(s3,0,200)
    #         s4 = np.sqrt(((traci.vehicle.getPosition(rlEnv.Target_right_car)[0] + traci.vehicle.getPosition(right_follower)[0])/2 - traci.vehicle.getPosition('ego')[0])**2+(traci.vehicle.getPosition(rlEnv.Target_right_car)[1]- traci.vehicle.getPosition('ego')[1])**2)#자차와 오른쪽 차로 후방 빈공간의 중점 까지 상대 거리  
    #         s4_scaled = self.minmaxscale(s4,0,200)
    #         space1_size = np.abs((traci.vehicle.getPosition(rlEnv.Target_left_car)[0] - traci.vehicle.getPosition(left_leader)[0]))
    #         space1_size_scaled = self.minmaxscale(space1_size,0,100)
    #         space2_size = np.abs((traci.vehicle.getPosition(rlEnv.Target_right_car)[0] - traci.vehicle.getPosition(right_leader)[0]))
    #         space2_size_scaled = self.minmaxscale(space2_size,0,100)
    #         space3_size = np.abs((traci.vehicle.getPosition(rlEnv.Target_left_car)[0] - traci.vehicle.getPosition(left_follower)[0]))
    #         space3_size_scaled = self.minmaxscale(space3_size,0,100)
    #         space4_size = np.abs((traci.vehicle.getPosition(rlEnv.Target_right_car)[0] - traci.vehicle.getPosition(right_follower)[0]))
    #         space4_size_scaled = self.minmaxscale(space4_size,0,100)
    #         ego_to_left_first = traci.vehicle.getPosition(left_leader)[0] -traci.vehicle.getPosition('ego')[0] # left first: 왼쪽 차로에 있는 ego vehicle에서 3대 중 앞에서 첫번째 차량까지 거리
    #         ego_to_left_first_scaled = self.minmaxscale(ego_to_left_first,-100,100)
    #         ego_to_left_second = traci.vehicle.getPosition(rlEnv.Target_left_car)[0] -traci.vehicle.getPosition('ego')[0] # left second: 왼쪽 차로에 있는 ego vehicle에서 가까운 3대 중 앞에서 두번째 차량까지 거리
    #         ego_to_left_second_scaled = self.minmaxscale(ego_to_left_second,-100,100)
    #         ego_to_left_third = traci.vehicle.getPosition(left_follower)[0] - traci.vehicle.getPosition('ego')[0] # left third: 왼쪽 차로에 있는 ego vehicle에서 가까운 3대 중 앞에서 세번째 차량까지 거리
    #         ego_to_left_third_scaled = self.minmaxscale(ego_to_left_third,-100,100)
    #         ego_to_right_first = traci.vehicle.getPosition(right_leader)[0] - traci.vehicle.getPosition('ego')[0] # right third: 오른쪽 차로에 있는 ego vehicle에서 가까운 3대 중 앞에서 첫번째 차량까지 거리
    #         ego_to_right_first_scaled = self.minmaxscale(ego_to_right_first,-100,100)
    #         ego_to_right_second = traci.vehicle.getPosition(rlEnv.Target_right_car)[0] - traci.vehicle.getPosition('ego')[0]# right third: 오른쪽 차로에 있는 ego vehicle에서 가까운 3대 중 앞에서 두번째 차량까지 거리
    #         ego_to_right_second_scaled = self.minmaxscale(ego_to_right_second,-100,100)
    #         ego_to_right_third = traci.vehicle.getPosition(right_leader)[0] - traci.vehicle.getPosition('ego')[0] # right third: 오른쪽 차로에 있는 ego vehicle에서 가까운 3대 중 앞에서 세번째 차량까지 거리
    #         ego_to_right_third_scaled = self.minmaxscale(ego_to_right_third,-100,100)
            
    #         # ego_to_left_first_y = traci.vehicle.getPosition(left_leader)[1] -traci.vehicle.getPosition('ego')[1] # left first: 왼쪽 차로에 있는 ego vehicle에서 3대 중 앞에서 첫번째 차량까지 거리
    #         # ego_to_left_second_y = traci.vehicle.getPosition(rlEnv.Target_left_car)[1] -traci.vehicle.getPosition('ego')[1] # left second: 왼쪽 차로에 있는 ego vehicle에서 가까운 3대 중 앞에서 두번째 차량까지 거리
    #         # ego_to_left_third_y = traci.vehicle.getPosition(left_follower)[1] - traci.vehicle.getPosition('ego')[1] # left third: 왼쪽 차로에 있는 ego vehicle에서 가까운 3대 중 앞에서 세번째 차량까지 거리
    #         # ego_to_right_first_y = traci.vehicle.getPosition(right_leader)[1] - traci.vehicle.getPosition('ego')[1] # right third: 오른쪽 차로에 있는 ego vehicle에서 가까운 3대 중 앞에서 첫번째 차량까지 거리
    #         # ego_to_right_second_y = traci.vehicle.getPosition(rlEnv.Target_right_car)[1] - traci.vehicle.getPosition('ego')[1]# right third: 오른쪽 차로에 있는 ego vehicle에서 가까운 3대 중 앞에서 두번째 차량까지 거리
    #         # ego_to_right_third_y = traci.vehicle.getPosition(right_follower)[1] - traci.vehicle.getPosition('ego')[1] # right third: 오른쪽 차로에 있는 ego vehicle에서 가까운 3대 중 앞에서 세번째 차량까지 거리
            
    #         # print('rlEnv.Target_left_car: ',rlEnv.Target_left_car,'rlEnv.Target_right_car: ',rlEnv.Target_right_car)
    #         # print('s1: ',s1,'s2: ',s2,'s3: ',s3,'s4: ',s4)
    #         # print('space1_size: ',space1_size,'space2_size: ',space2_size,'space3_size: ',space3_size,'space4_size: ',space4_size)
    #         # print('ego_to_left_first: ',ego_to_left_first,'ego_to_left_second: ',ego_to_left_second,'ego_to_left_third: ',ego_to_left_third)
    #         # print('ego_to_right_first: ',ego_to_right_first,'ego_to_right_second: ',ego_to_right_second,'ego_to_right_third: ',ego_to_right_third)
                            
    #         if traci.vehicle.getLeader(id) is not None:
    #             leader_id,forward_clearance = traci.vehicle.getLeader(id)
    #             if forward_clearance>120:# 앞 차량인지 가능 거리 =120
    #                 forward_clearance =120
    #             forward_rel_y = traci.vehicle.getPosition(leader_id)[1] - traci.vehicle.getPosition('ego')[1]
    #         else:
    #             forward_clearance =120
    #             forward_rel_y=0
    #         if type(traci.vehicle.getFollower(id)) is not None and traci.vehicle.getFollower(id)[0] != '':
    #             follower_id, rear_clearance = traci.vehicle.getFollower(id)                 
    #             if rear_clearance >120:
    #                 rear_clearance = 120
    #             rear_rel_y = traci.vehicle.getPosition(follower_id)[1] - traci.vehicle.getPosition('ego')[1]
    #         else:
    #             rear_clearance =120 # 뒤 차량인지 가능 거리 =120
    #             rear_rel_y=0

    #         forward_clearance_scaled = self.minmaxscale(forward_clearance,0,120)
    #         rear_clearance_scaled = self.minmaxscale(rear_clearance,0,120)
    #         rel_v_rear = traci.vehicle.getSpeed('ego') - traci.vehicle.getSpeed('accel.rear')
    #         rel_v_rear_scaled = self.minmaxscale(rel_v_rear,-20,20)
    #         rel_v_forward = traci.vehicle.getSpeed('ego') - traci.vehicle.getSpeed('car.forward')
    #         rel_v_forward_scaled = self.minmaxscale(rel_v_forward,-20,20)
    #         rel_v_left=traci.vehicle.getSpeed('ego') - traci.vehicle.getSpeed(rlEnv.Target_left_car)
    #         rel_v_left_scaled = self.minmaxscale(rel_v_left,-20,20)
    #         rel_v_right=traci.vehicle.getSpeed('ego') - traci.vehicle.getSpeed(rlEnv.Target_right_car)
    #         rel_v_right_scaled = self.minmaxscale(rel_v_right,-20,20)
    #         rel_v_left_forward = traci.vehicle.getSpeed('ego') - traci.vehicle.getSpeed(traci.vehicle.getLeader(rlEnv.Target_left_car)[0])
    #         rel_v_left_forward_scaled = self.minmaxscale(rel_v_left_forward,-20,20)
    #         rel_v_left_rear = traci.vehicle.getSpeed('ego') - traci.vehicle.getSpeed(traci.vehicle.getFollower(rlEnv.Target_left_car)[0])
    #         rel_v_left_rear_scaled = self.minmaxscale(rel_v_left_rear,-20,20)
    #         rel_v_right_forward = traci.vehicle.getSpeed('ego') - traci.vehicle.getSpeed(traci.vehicle.getLeader(rlEnv.Target_right_car)[0])
    #         rel_v_right_forward_scaled = self.minmaxscale(rel_v_right_forward,-20,20)
    #         rel_v_right_rear = traci.vehicle.getSpeed('ego') - traci.vehicle.getSpeed(traci.vehicle.getFollower(rlEnv.Target_right_car)[0])
    #         rel_v_right_rear_scaled = self.minmaxscale(rel_v_right_rear,-20,20)
    #         ego_cur_lane = traci.vehicle.getLaneIndex('ego')
    #         ego_cur_lane_scaled = ego_cur_lane/10
    #         if len(rlEnv.ego)!=0:
    #             ego_y = rlEnv.ego[-1][3]
    #             ego_v_y= rlEnv.ego[-1][5]
    #             ego_heading_angle = rlEnv.ego[-1][8]
    #         else:
    #             ego_y = -4.8 #initial ego_y
    #             ego_v_y= 0
    #             ego_heading_angle = 90
    #         lane_width = 3.2
    #         ego_y_scaled = self.minmaxscale(ego_y,-1*lane_width*3,0)
    #         ego_v_y_scaled = self.minmaxscale(ego_v_y,0,40) 
    #         ego_heading_angle_scaled = self.minmaxscale(ego_heading_angle,0,180) 

    #         # states.append(forward_clearance)
    #         # states.append(rear_clearance)
                        
    #         # states.append(s1)
    #         # states.append(s3)
    #         # states.append(s2)
    #         # states.append(s4)

    #         # states.append(space1_size)
    #         # states.append(space3_size)
    #         # states.append(space2_size)
    #         # states.append(space4_size)

    #         # states.append(ego_to_left_first)
    #         # states.append(ego_to_left_second)
    #         # states.append(ego_to_left_third)
    #         # states.append(ego_to_right_first)
    #         # states.append(ego_to_right_second)
    #         # states.append(ego_to_right_third)

    #         # states.append(rel_v_rear)
    #         # states.append(rel_v_forward)

    #         # states.append(rel_v_left)
    #         # states.append(rel_v_left_forward)
    #         # states.append(rel_v_left_rear)
    #         # states.append(rel_v_right)
    #         # states.append(rel_v_right_forward)
    #         # states.append(rel_v_right_rear)

    #         # states.append(ego_cur_lane)
    #         # states.append(ego_y)
    #         # states.append(ego_v_y)
    #         # states.append(ego_heading_angle)
    #         states.append(forward_clearance_scaled)
    #         states.append(rear_clearance_scaled)
                        
    #         states.append(s1_scaled)
    #         states.append(s3_scaled)
    #         states.append(s2_scaled)
    #         states.append(s4_scaled)

    #         states.append(space1_size_scaled)
    #         states.append(space3_size_scaled)
    #         states.append(space2_size_scaled)
    #         states.append(space4_size_scaled)

    #         states.append(ego_to_left_first_scaled)
    #         states.append(ego_to_left_second_scaled)
    #         states.append(ego_to_left_third_scaled)
    #         states.append(ego_to_right_first_scaled)
    #         states.append(ego_to_right_second_scaled)
    #         states.append(ego_to_right_third_scaled)

    #         states.append(rel_v_rear_scaled)
    #         states.append(rel_v_forward_scaled)

    #         states.append(rel_v_left_scaled)
    #         states.append(rel_v_left_forward_scaled)
    #         states.append(rel_v_left_rear_scaled)
    #         states.append(rel_v_right_scaled)
    #         states.append(rel_v_right_forward_scaled)
    #         states.append(rel_v_right_rear_scaled)

    #         states.append(ego_cur_lane_scaled)
    #         states.append(ego_y_scaled)
    #         states.append(ego_v_y_scaled)
    #         states.append(ego_heading_angle_scaled)

    #         states = np.array(states)

    #         # print('states',states)
    #     else:
    #         left_vehicles=[] #[|ego_x_pos - x_pos|, id]
    #         right_vehicles=[] #[|ego_x_pos - x_pos|, id]
    #         left_near_vehicles=[] #[x_pos]
    #         right_near_vehicles =[] #[x_pos]
            
    #         left_near_spaces=[] #[space_x_pos, space_size]
    #         right_near_spaces=[] #[space_x_pos, space_size]
    #         ego_to_left_spaces=[]
    #         ego_to_right_spaces=[]
        

    #         traci.vehicle.subscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 120.0, [tc.VAR_POSITION])
    #         # print(id,' subscribeContext')
    #         for v_id in traci.vehicle.getContextSubscriptionResults(str(id)):
    #             if(traci.vehicle.getLaneIndex(v_id)==2) and not (v_id=='ego'):
    #                 #left_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),v_id]) -> id check
    #                 left_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),traci.vehicle.getPosition(v_id)[0],v_id])
    #             if(traci.vehicle.getLaneIndex(v_id)==0)and not (v_id=='ego') :
    #                 right_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),traci.vehicle.getPosition(v_id)[0],v_id])
                    
            
            
    #         left_vehicles.sort()
    #         right_vehicles.sort()
    #         # if len(left_vehicles) >=3:
    #         # print('left_near_vehicles: ',left_vehicles[:2])
    #         # print('right_near_vehicles: ',right_vehicles[:2])
            
    #         for i in range(3): # getting near 3 vehicles x_pos each left and right
    #             left_near_vehicles.append([left_vehicles[:3][i][1],left_vehicles[:3][i][2]])# 1: left vehicle position 2: left vehicle id
    #             right_near_vehicles.append([right_vehicles[:3][i][1],right_vehicles[:3][i][2]])# 1: right vehicle position 2: right vehicle id
                
    #         left_near_vehicles.sort()
    #         right_near_vehicles.sort()
    #         # print('left_near_vehicles: ',left_near_vehicles)
    #         # print('right_near_vehicles: ',right_near_vehicles)
    #         # print('left_near_vehicles[0][0]: ', left_near_vehicles[0][0])
    #         # print('left_near_vehicles[0][1]: ', left_near_vehicles[0][1])
    #         # print('left_near_vehicles[1][0]: ', left_near_vehicles[1][0])
    #         # print('left_near_vehicles[1][1]: ', left_near_vehicles[1][1])
    #         ego_to_left_first = left_near_vehicles[2][0] - traci.vehicle.getPosition('ego')[0] # left first: 왼쪽 차로에 있는 ego vehicle에서 가까운 3대 중 앞에서 첫번째 차량까지 거리
    #         ego_to_left_second = left_near_vehicles[1][0] -traci.vehicle.getPosition('ego')[0] # left second: 왼쪽 차로에 있는 ego vehicle에서 가까운 3대 중 앞에서 두번째 차량까지 거리
    #         ego_to_left_third = left_near_vehicles[0][0] - traci.vehicle.getPosition('ego')[0] # left third: 왼쪽 차로에 있는 ego vehicle에서 가까운 3대 중 앞에서 세번째 차량까지 거리
    #         ego_to_right_first = right_near_vehicles[2][0] - traci.vehicle.getPosition('ego')[0] # right third: 오른쪽 차로에 있는 ego vehicle에서 가까운 3대 중 앞에서 첫번째 차량까지 거리
    #         ego_to_right_second = right_near_vehicles[1][0] - traci.vehicle.getPosition('ego')[0]# right third: 오른쪽 차로에 있는 ego vehicle에서 가까운 3대 중 앞에서 두번째 차량까지 거리
    #         ego_to_right_third = right_near_vehicles[0][0] - traci.vehicle.getPosition('ego')[0] # right third: 오른쪽 차로에 있는 ego vehicle에서 가까운 3대 중 앞에서 세번째 차량까지 거리

    #         left_near_spaces.append([left_near_vehicles[1][0]+(left_near_vehicles[2][0]-left_near_vehicles[1][0])/2,left_near_vehicles[2][0]-left_near_vehicles[1][0]]) #ego vehicle 주위 두 개의 왼쪽 빈공간 중 앞에서 첫번째 빈공간의 위치. 빈공간의 크기
    #         left_near_spaces.append([left_near_vehicles[0][0]+(left_near_vehicles[1][0]-left_near_vehicles[0][0])/2,left_near_vehicles[1][0]-left_near_vehicles[0][0]]) #ego vehicle 주위 두 개의 왼쪽 빈공간 중 앞에서 두번째 빈공간의 위치, 빈공간의 크기
    #         right_near_spaces.append([right_near_vehicles[1][0]+(right_near_vehicles[2][0]-right_near_vehicles[1][0])/2, right_near_vehicles[2][0]-right_near_vehicles[1][0]]) #ego vehicle 주위 두 개의 오른쪽 빈공간 중 앞에서 첫번째 빈공간의 위치, 빈공간의 크기
    #         right_near_spaces.append([right_near_vehicles[0][0]+(right_near_vehicles[1][0]-right_near_vehicles[0][0])/2, right_near_vehicles[1][0]-right_near_vehicles[0][0]]) #ego vehicle 주위 두 개의 오른쪽 빈공간 중 앞에서 두번째 빈공간의 위치, 빈공간의 크기
            

    #         ego_to_left_spaces.append(left_near_spaces[0][0]- traci.vehicle.getPosition('ego')[0])#ego vehicle에서 주위 두 개의 왼쪽 빈공간 중 앞에서 첫번째 빈공간의 중점까지 거리
    #         ego_to_left_spaces.append(left_near_spaces[1][0]- traci.vehicle.getPosition('ego')[0])#ego vehicle에서 주위 두 개의 왼쪽 빈공간 중 앞에서 두번째 빈공간의 중점까지 거리
    #         ego_to_right_spaces.append(right_near_spaces[0][0]-traci.vehicle.getPosition('ego')[0])#ego vehicle에서 주위 두 개의 오른쪽 빈공간 중 앞에서 첫번째 빈공간의 중점까지 거리
    #         ego_to_right_spaces.append(right_near_spaces[1][0]-traci.vehicle.getPosition('ego')[0])#ego vehicle에서 주위 두 개의 오른쪽 빈공간 중 앞에서 두번째 빈공간의 중점까지 거리

                
    #         if traci.vehicle.getLeader(id) is not None:
    #             leader_id,forward_clearance = traci.vehicle.getLeader(id)
    #             if forward_clearance>120:
    #                 forward_clearance =120
    #         else:
    #             forward_clearance =120
    #         if type(traci.vehicle.getFollower(id)) is not None and traci.vehicle.getFollower(id)[0] != '':
    #             follower_id, rear_clearance = traci.vehicle.getFollower(id) 
                
    #             if rear_clearance >120:
    #                 rear_clearance = 120
    #         else:
    #             rear_clearance =120
    #         rel_v_rear = traci.vehicle.getSpeed('ego') - traci.vehicle.getSpeed('accel.rear')
    #         rel_v_forward = traci.vehicle.getSpeed('ego') - traci.vehicle.getSpeed('car.forward')
    #         rel_v_left = traci.vehicle.getSpeed('ego') - traci.vehicle.getSpeed(left_near_vehicles[1][1])
    #         rel_v_left_forward = traci.vehicle.getSpeed('ego') - traci.vehicle.getSpeed(left_near_vehicles[2][1])
    #         rel_v_left_rear = traci.vehicle.getSpeed('ego') - traci.vehicle.getSpeed(left_near_vehicles[0][1])
    #         rel_v_right = traci.vehicle.getSpeed('ego') - traci.vehicle.getSpeed(right_near_vehicles[1][1])
    #         rel_v_right_forward = traci.vehicle.getSpeed('ego') - traci.vehicle.getSpeed(right_near_vehicles[2][1])
    #         rel_v_right_rear = traci.vehicle.getSpeed('ego') - traci.vehicle.getSpeed(right_near_vehicles[0][1])
    #         ego_cur_lane = traci.vehicle.getLaneIndex('ego')
    #         if len(rlEnv.ego)!=0:
    #             ego_y = rlEnv.ego[-1][3]
    #             ego_v_y= rlEnv.ego[-1][5]
    #             ego_heading_angle = rlEnv.ego[-1][8]
    #         else:
    #             ego_y = -4.8
    #             ego_v_y= 0
    #             ego_heading_angle = 90

    #         forward_clearance_scaled = self.minmaxscale(forward_clearance,0,120)
    #         rear_clearance_scaled = self.minmaxscale(rear_clearance,0,120)
    #         s1_scaled = self.minmaxscale(ego_to_left_spaces[0],0,200)
    #         s2_scaled = self.minmaxscale(ego_to_right_spaces[0],0,200)
    #         s3_scaled = self.minmaxscale(ego_to_left_spaces[1],0,200)
    #         s4_scaled = self.minmaxscale(ego_to_right_spaces[1],0,200)
    #         space1_size_scaled = self.minmaxscale(left_near_spaces[0][1],0,100)
    #         space2_size_scaled = self.minmaxscale(right_near_spaces[0][1],0,100)
    #         space3_size_scaled = self.minmaxscale(left_near_spaces[1][1],0,100)
    #         space4_size_scaled = self.minmaxscale(right_near_spaces[1][1],0,100)
    #         ego_to_left_first_scaled = self.minmaxscale(ego_to_left_first,-100,100)
    #         ego_to_left_second_scaled = self.minmaxscale(ego_to_left_second,-100,100)
    #         ego_to_left_third_scaled = self.minmaxscale(ego_to_left_third,-100,100)
    #         ego_to_right_first_scaled = self.minmaxscale(ego_to_right_first,-100,100)
    #         ego_to_right_second_scaled = self.minmaxscale(ego_to_right_second,-100,100)
    #         ego_to_right_third_scaled = self.minmaxscale(ego_to_right_third,-100,100)
    #         rel_v_rear_scaled = self.minmaxscale(rel_v_rear,-20,20)
    #         rel_v_forward_scaled = self.minmaxscale(rel_v_forward,-20,20)
    #         rel_v_left_scaled = self.minmaxscale(rel_v_left,-20,20)
    #         rel_v_right_scaled = self.minmaxscale(rel_v_right,-20,20)
    #         rel_v_left_forward_scaled = self.minmaxscale(rel_v_left_forward,-20,20)
    #         rel_v_left_rear_scaled = self.minmaxscale(rel_v_left_rear,-20,20)
    #         rel_v_right_forward_scaled = self.minmaxscale(rel_v_right_forward,-20,20)
    #         rel_v_right_rear_scaled = self.minmaxscale(rel_v_right_rear,-20,20)
    #         ego_cur_lane_scaled = ego_cur_lane/10
    #         lane_width = 3.2
    #         ego_y_scaled = self.minmaxscale(ego_y,-1*lane_width*3,0)
    #         ego_v_y_scaled = self.minmaxscale(ego_v_y,0,40) 
    #         ego_heading_angle_scaled = self.minmaxscale(ego_heading_angle,0,180) 

    #         # states.append(forward_clearance)
    #         # states.append(rear_clearance)
                        
    #         # states.append(ego_to_left_spaces[0])
    #         # states.append(ego_to_left_spaces[1])
    #         # states.append(ego_to_right_spaces[0])
    #         # states.append(ego_to_right_spaces[1])

    #         # states.append(left_near_spaces[0][1])
    #         # states.append(left_near_spaces[1][1])
    #         # states.append(right_near_spaces[0][1])
    #         # states.append(right_near_spaces[1][1])

    #         # states.append(ego_to_left_first)
    #         # states.append(ego_to_left_second)
    #         # states.append(ego_to_left_third)
    #         # states.append(ego_to_right_first)
    #         # states.append(ego_to_right_second)
    #         # states.append(ego_to_right_third)

    #         # states.append(rel_v_rear)
    #         # states.append(rel_v_forward)
    #         # states.append(rel_v_left)
    #         # states.append(rel_v_left_forward)
    #         # states.append(rel_v_left_rear)
    #         # states.append(rel_v_right)
    #         # states.append(rel_v_right_forward)
    #         # states.append(rel_v_right_rear)
    #         # states.append(ego_cur_lane)
    #         # states.append(ego_y)
    #         # states.append(ego_v_y)
    #         # states.append(ego_heading_angle)
    #         states.append(forward_clearance_scaled)
    #         states.append(rear_clearance_scaled)
                        
    #         states.append(s1_scaled)
    #         states.append(s3_scaled)
    #         states.append(s2_scaled)
    #         states.append(s4_scaled)

    #         states.append(space1_size_scaled)
    #         states.append(space3_size_scaled)
    #         states.append(space2_size_scaled)
    #         states.append(space4_size_scaled)

    #         states.append(ego_to_left_first_scaled)
    #         states.append(ego_to_left_second_scaled)
    #         states.append(ego_to_left_third_scaled)
    #         states.append(ego_to_right_first_scaled)
    #         states.append(ego_to_right_second_scaled)
    #         states.append(ego_to_right_third_scaled)

    #         states.append(rel_v_rear_scaled)
    #         states.append(rel_v_forward_scaled)

    #         states.append(rel_v_left_scaled)
    #         states.append(rel_v_left_forward_scaled)
    #         states.append(rel_v_left_rear_scaled)
    #         states.append(rel_v_right_scaled)
    #         states.append(rel_v_right_forward_scaled)
    #         states.append(rel_v_right_rear_scaled)

    #         states.append(ego_cur_lane_scaled)
    #         states.append(ego_y_scaled)
    #         states.append(ego_v_y_scaled)
    #         states.append(ego_heading_angle_scaled)
    #         traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 120.0)
    #         states = np.array(states)
    #     rlEnv.last_state = states
    #     return states
    def __toward_empty_space_reward(self, id, action):
        
        def calculate_distance(point1, point2):
            return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)
        
        def get_space_size(target_car, car):
            return np.abs(traci.vehicle.getPosition(target_car)[0] - traci.vehicle.getPosition(car)[0])
        
        def get_target_car_rear_SD_LC_point_distance(target, ego_pos, SD_LC):
            return np.abs((traci.vehicle.getPosition(target)[0] - SD_LC) - ego_pos[0])
        def get_target_car_forward_SD_LC_point_distance(target, ego_pos, SD_LC):
            return np.abs((traci.vehicle.getPosition(target)[0] + SD_LC) - ego_pos[0])
        
        def get_midpoint_distance(target, car, ego_pos):
            midpoint_x = (traci.vehicle.getPosition(target)[0] + traci.vehicle.getPosition(car)[0]) / 2
            midpoint_y = traci.vehicle.getPosition(target)[1]
            return calculate_distance((midpoint_x, midpoint_y), ego_pos)

        def reward_for_space(space, ego_pos, action, sd_lc_dist, dist_to_space, weight, action_codes):
            longitudinal_term = 1 if (
                (action_codes[0] == 'L' and action == 3 and (ego_pos[0] < traci.vehicle.getPosition(rlEnv.Target_left_car)[0] - self.SD_LC_with_respect_to_target_space('ego', 3, 'front') and ego_pos[0] > traci.vehicle.getPosition(left_follower)[0] + self.SD_LC_with_respect_to_target_space('ego', 3,'back'))) or
                (action_codes[0] == 'L' and action == 1 and (ego_pos[0] > traci.vehicle.getPosition(rlEnv.Target_left_car)[0] + self.SD_LC_with_respect_to_target_space('ego', 1,'back') and ego_pos[0] < traci.vehicle.getPosition(left_leader)[0] - self.SD_LC_with_respect_to_target_space('ego', 1,'front'))) or
                (action_codes[0] == 'R' and action == 4 and (ego_pos[0] < traci.vehicle.getPosition(rlEnv.Target_right_car)[0] - self.SD_LC_with_respect_to_target_space('ego', 4,'front') and ego_pos[0] > traci.vehicle.getPosition(right_follower)[0] + self.SD_LC_with_respect_to_target_space('ego', 4,'back'))) or
                (action_codes[0] == 'R' and action == 2 and (ego_pos[0] > traci.vehicle.getPosition(rlEnv.Target_right_car)[0] + self.SD_LC_with_respect_to_target_space('ego', 2,'back') and ego_pos[0] < traci.vehicle.getPosition(right_leader)[0] - self.SD_LC_with_respect_to_target_space('ego', 2,'front')))
            ) else 2 ** (-sd_lc_dist)
            
            lateral_distance_error = np.abs((-1.6 if action_codes[0] == 'L' else -8) - ego_pos[1])
            lateral_term = w_l * 2 ** (-lateral_distance_error) if dist_to_space <= np.sqrt((space[0] / 2) ** 2 + 3.2 ** 2) else 0
            
            return (longitudinal_term + lateral_term) * weight, longitudinal_term* weight, lateral_term* weight

        space_reward = []
        space_reward_long = []
        space_reward_lat = []

        space_reward.append(traci.simulation.getTime())
        space_reward_long.append(traci.simulation.getTime())
        space_reward_lat.append(traci.simulation.getTime())
        value = 0
        vehs = traci.vehicle.getIDList()
        if id in vehs and rlEnv.Target_left_car in vehs and rlEnv.Target_right_car in vehs:
            ego_pos = traci.vehicle.getPosition('ego') 
            if traci.vehicle.getLeader(rlEnv.Target_left_car)[0] == 'ego' and traci.vehicle.getLaneIndex('ego')==2:
                left_leader = traci.vehicle.getLeader('ego')[0]
            elif traci.vehicle.getLeader(rlEnv.Target_left_car)[0] == 'ego' and traci.vehicle.getLaneIndex('ego')==1:
                left_leader = 'car.left'+str(int(rlEnv.Target_left_car[-1])+1)
            else:
                left_leader = traci.vehicle.getLeader(rlEnv.Target_left_car)[0]
            if traci.vehicle.getFollower(rlEnv.Target_left_car)[0] =='ego' and traci.vehicle.getLaneIndex('ego')==2:
                left_follower = traci.vehicle.getFollower('ego')[0]
            elif traci.vehicle.getFollower(rlEnv.Target_left_car)[0] =='ego' and traci.vehicle.getLaneIndex('ego')==1:
                left_follower = 'car.left'+str(int(rlEnv.Target_left_car[-1])-1)
            else:
                left_follower = traci.vehicle.getFollower(rlEnv.Target_left_car)[0]
            
            if traci.vehicle.getLeader(rlEnv.Target_right_car)[0] == 'ego' and traci.vehicle.getLaneIndex('ego')==0:
                right_leader = traci.vehicle.getLeader('ego')[0]
            elif traci.vehicle.getLeader(rlEnv.Target_right_car)[0] == 'ego' and traci.vehicle.getLaneIndex('ego')==1:
                right_leader = 'car.right'+str(int(rlEnv.Target_right_car[-1])+1)
            else:
                right_leader = traci.vehicle.getLeader(rlEnv.Target_right_car)[0]
            if traci.vehicle.getFollower(rlEnv.Target_right_car)[0] =='ego' and traci.vehicle.getLaneIndex('ego')==0:
                right_follower = traci.vehicle.getFollower('ego')[0]
            elif traci.vehicle.getFollower(rlEnv.Target_right_car)[0] =='ego' and traci.vehicle.getLaneIndex('ego')==1:
                right_follower = 'car.right'+str(int(rlEnv.Target_right_car[-1])-1)
            else:
                right_follower = traci.vehicle.getFollower(rlEnv.Target_right_car)[0]

            spaces = [
                [get_space_size(rlEnv.Target_left_car, left_leader), get_midpoint_distance(rlEnv.Target_left_car, left_leader, ego_pos), 'L1',get_target_car_forward_SD_LC_point_distance(rlEnv.Target_left_car, ego_pos, self.SD_LC_with_respect_to_target_space('ego', 1,'back'))],
                [get_space_size(rlEnv.Target_right_car, right_leader), get_midpoint_distance(rlEnv.Target_right_car, right_leader, ego_pos), 'R1',get_target_car_forward_SD_LC_point_distance(rlEnv.Target_left_car, ego_pos, self.SD_LC_with_respect_to_target_space('ego', 2,'back'))],
                [get_space_size(rlEnv.Target_left_car, left_follower), get_midpoint_distance(rlEnv.Target_left_car, left_follower, ego_pos), 'L2',get_target_car_rear_SD_LC_point_distance(rlEnv.Target_left_car, ego_pos, self.SD_LC_with_respect_to_target_space('ego', 3,'front'))],
                [get_space_size(rlEnv.Target_right_car, right_follower), get_midpoint_distance(rlEnv.Target_right_car, right_follower, ego_pos), 'R2',get_target_car_rear_SD_LC_point_distance(rlEnv.Target_left_car, ego_pos, self.SD_LC_with_respect_to_target_space('ego', 4,'front'))]
            ]
            spaces.sort()

            

            space_weights = [1, 0.75, 0.5, 0.25]
            # w_l = 0.2
            w_l =2
            
            total_space_size = sum(space[0] for space in spaces)

            for i, space in enumerate(reversed(spaces)):
                if (space[2] == 'L1' and action == 1) or (space[2] == 'R1' and action == 2) or (space[2] == 'L2' and action == 3) or (space[2] == 'R2' and action == 4):
                    reward, long_term, lat_term = reward_for_space(
                        space, ego_pos, action, space[3], space[1], space_weights[i], space[2]
                    )
                    value += reward
                    space_reward.append(reward)
                    space_reward_long.append(long_term)
                    space_reward_lat.append(lat_term)
                else:
                    space_reward.append(0)
                    space_reward_long.append(0)
                    space_reward_lat.append(0)
        rlEnv.toward_space_reward.append(space_reward)
        rlEnv.toward_space_reward_long.append(space_reward_long)
        rlEnv.toward_space_reward_lat.append(space_reward_lat)
        
        return value
    
    # def __toward_biggest_empty_space_reward(self,id,action):
    #     space_reward=[]
    #     space_reward_long=[]
    #     space_reward_lat=[]
    #     space_reward.append(traci.simulation.getTime())
    #     space_reward_long.append(traci.simulation.getTime())
    #     space_reward_lat.append(traci.simulation.getTime())
    #     vehs = traci.vehicle.getIDList()
    #     if id in vehs and rlEnv.Target_left_car in vehs and rlEnv.Target_right_car in vehs:        
    #         if traci.vehicle.getLeader(rlEnv.Target_left_car)[0] == 'ego':
    #             left_leader = traci.vehicle.getLeader('ego')[0]                
    #         else:
    #             left_leader = traci.vehicle.getLeader(rlEnv.Target_left_car)[0]
    #         if traci.vehicle.getFollower(rlEnv.Target_left_car)[0] =='ego':
    #             left_follower = traci.vehicle.getFollower('ego')[0]
    #         else:
    #             left_follower = traci.vehicle.getFollower(rlEnv.Target_left_car)[0]
            
    #         if traci.vehicle.getLeader(rlEnv.Target_right_car)[0] == 'ego':
    #             right_leader = traci.vehicle.getLeader('ego')[0]
    #         else:
    #             right_leader = traci.vehicle.getLeader(rlEnv.Target_right_car)[0]
    #         if traci.vehicle.getFollower(rlEnv.Target_right_car)[0] =='ego':
    #             right_follower = traci.vehicle.getFollower('ego')[0]
    #         else:
    #             right_follower = traci.vehicle.getFollower(rlEnv.Target_right_car)[0]
    #         # print('left_leader: ',left_leader)
    #         # print('right_leader: ',right_leader)
    #         # print('right_follower: ',right_follower)
    #         # print('left_follower: ',left_follower)
    #         #distance to space with respect to SD_LC
    #         SD_LC_s1 = np.abs((traci.vehicle.getPosition(rlEnv.Target_left_car)[0] + self.SD_LC_with_respect_to_target_space('ego',1,'back')) - traci.vehicle.getPosition('ego')[0])
    #         SD_LC_s1_1 = np.abs((traci.vehicle.getPosition(left_leader)[0] - self.SD_LC_with_respect_to_target_space('ego',1,'front')) - traci.vehicle.getPosition('ego')[0])
    #         SD_LC_s2 = np.abs((traci.vehicle.getPosition(rlEnv.Target_right_car)[0] + self.SD_LC_with_respect_to_target_space('ego',2,'back')) - traci.vehicle.getPosition('ego')[0])
    #         SD_LC_s2_1 = np.abs((traci.vehicle.getPosition(right_leader)[0] - self.SD_LC_with_respect_to_target_space('ego',2,'front')) - traci.vehicle.getPosition('ego')[0])
    #         SD_LC_s3 = np.abs((traci.vehicle.getPosition(rlEnv.Target_left_car)[0] - self.SD_LC_with_respect_to_target_space('ego',3,'front')) - traci.vehicle.getPosition('ego')[0])
    #         SD_LC_s3_1 = np.abs((traci.vehicle.getPosition(left_follower)[0] + self.SD_LC_with_respect_to_target_space('ego',3,'back')) - traci.vehicle.getPosition('ego')[0])
    #         SD_LC_s4 = np.abs((traci.vehicle.getPosition(rlEnv.Target_right_car)[0] - self.SD_LC_with_respect_to_target_space('ego',4,'front')) - traci.vehicle.getPosition('ego')[0])
    #         SD_LC_s4_1 = np.abs((traci.vehicle.getPosition(right_follower)[0] + self.SD_LC_with_respect_to_target_space('ego',4,'back')) - traci.vehicle.getPosition('ego')[0])
    #         #distance to space
    #         s1 = np.sqrt(((traci.vehicle.getPosition(rlEnv.Target_left_car)[0] + traci.vehicle.getPosition(left_leader)[0])/2 - traci.vehicle.getPosition('ego')[0])**2 +(traci.vehicle.getPosition(rlEnv.Target_left_car)[1]- traci.vehicle.getPosition('ego')[1])**2)#자차와 왼쪽 차로 전방 빈공간의 중점까지 상대 거리
    #         s2 = np.sqrt(((traci.vehicle.getPosition(rlEnv.Target_right_car)[0] + traci.vehicle.getPosition(right_leader)[0])/2 - traci.vehicle.getPosition('ego')[0])**2+(traci.vehicle.getPosition(rlEnv.Target_right_car)[1]- traci.vehicle.getPosition('ego')[1])**2) #자차와 오른쪽 차로 전방 빈공간의 중점까지 상대 거리
    #         s3 = np.sqrt(((traci.vehicle.getPosition(rlEnv.Target_left_car)[0] + traci.vehicle.getPosition(left_follower)[0])/2 - traci.vehicle.getPosition('ego')[0])**2+(traci.vehicle.getPosition(rlEnv.Target_left_car)[1]- traci.vehicle.getPosition('ego')[1])**2) #자차와 왼쪽 차로 후방 빈공간의 중점까지 상대 거리            
    #         s4 = np.sqrt(((traci.vehicle.getPosition(rlEnv.Target_right_car)[0] + traci.vehicle.getPosition(right_follower)[0])/2 - traci.vehicle.getPosition('ego')[0])**2+(traci.vehicle.getPosition(rlEnv.Target_right_car)[1]- traci.vehicle.getPosition('ego')[1])**2)#자차와 오른쪽 차로 후방 빈공간의 중점까지 상대 거리  
    #         #space size
    #         space1_size = np.abs((traci.vehicle.getPosition(rlEnv.Target_left_car)[0] - traci.vehicle.getPosition(left_leader)[0]))
    #         space2_size = np.abs((traci.vehicle.getPosition(rlEnv.Target_right_car)[0] - traci.vehicle.getPosition(right_leader)[0]))
    #         space3_size = np.abs((traci.vehicle.getPosition(rlEnv.Target_left_car)[0] - traci.vehicle.getPosition(left_follower)[0]))
    #         space4_size = np.abs((traci.vehicle.getPosition(rlEnv.Target_right_car)[0] - traci.vehicle.getPosition(right_follower)[0]))
    #         # print("space1_size: ",space1_size)
    #         # print("space2_size: ",space2_size)
    #         # print("space3_size: ",space3_size)
    #         # print("space4_size: ",space4_size)
    #         space1 = [space1_size,s1,'L1'] #left front
    #         space2 = [space2_size,s2,'R1'] #right front
    #         space3 = [space3_size,s3,'L2'] #left back
    #         space4 = [space4_size,s4,'R2'] #right back
    #         order =[space1,space2,space3,space4]
    #         order.sort()
    #         dist_to_biggest_empty_space = order[-1][1]
    #         dist_to_second_biggest_empty_space = order[-2][1]
    #         dist_to_third_biggest_empty_space = order[-3][1]
    #         dist_to_forth_biggest_empty_space = order[-4][1]
    #         #space의 SD_LC 까지 거리
    #         SD_LC_space1 = [space1_size,SD_LC_s1,SD_LC_s1_1,'L1'] #left front
    #         SD_LC_space2 = [space2_size,SD_LC_s2,SD_LC_s2_1,'R1'] #right front
    #         SD_LC_space3 = [space3_size,SD_LC_s3,SD_LC_s3_1,'L2'] #left back
    #         SD_LC_space4 = [space4_size,SD_LC_s4,SD_LC_s4_1,'R2'] #right back
    #         SD_LC_order = [SD_LC_space1,SD_LC_space2,SD_LC_space3,SD_LC_space4]
    #         SD_LC_order.sort()
    #         SD_LC_dist_to_biggest_empty_space = SD_LC_order[-1][1]
    #         SD_LC_dist_to_second_biggest_empty_space = SD_LC_order[-2][1]
    #         SD_LC_dist_to_third_biggest_empty_space = SD_LC_order[-3][1]
    #         SD_LC_dist_to_forth_biggest_empty_space = SD_LC_order[-4][1]
    #         SD_LC_dist_to_biggest_empty_space2 = SD_LC_order[-1][2]
    #         SD_LC_dist_to_second_biggest_empty_space2 = SD_LC_order[-2][2]
    #         SD_LC_dist_to_third_biggest_empty_space2 = SD_LC_order[-3][2]
    #         SD_LC_dist_to_forth_biggest_empty_space2 = SD_LC_order[-4][2]
    #         value =0
    #         total_space_size = space1_size+space2_size+space3_size+space4_size
    #         # print('1st space weight: ',(order[-1][0]/total_space_size))
    #         # print('2nd space weight: ',(order[-2][0]/total_space_size))
    #         # print('3rd space weight: ',(order[-3][0]/total_space_size))
    #         # print('4th space weight: ',(order[-4][0]/total_space_size))
    #         # print('ego_y: ',traci.vehicle.getPosition('ego')[1])
    #         # if dist_to_biggest_empty_space<=np.sqrt((order[-1][0]/2)**2+3.2**2) and(order[-1][2] =='L1' and action == 1 or order[-1][2] =='R1' and action == 2 or order[-1][2] =='L2' and action == 3 or order[-1][2] =='R2' and action == 4): #If ego vehicle is next to the open space then the reward adding start.
    #         w_l = 0.2
    #         biggest_size_value = 1
    #         second_biggest_size_value = 0.75
    #         third_biggest_size_value = 0.5
    #         forth_biggest_size_value = 0.25
    #         if ((order[-1][2] =='L1' and action == 1) or (order[-1][2] =='R1' and action == 2) or (order[-1][2] =='L2' and action == 3) or (order[-1][2] =='R2' and action == 4)): #If ego vehicle is next to the open space then the reward adding start.                 
    #             # print(order[-1][2])
    #             if order[-1][2][0] == 'L' :
                    
    #                 if(action == 3 and (traci.vehicle.getPosition('ego')[0] < traci.vehicle.getPosition(rlEnv.Target_left_car)[0] - self.SD_LC_with_respect_to_target_space('ego',3,'front') and traci.vehicle.getPosition('ego')[0] > traci.vehicle.getPosition(right_follower)[0] + self.SD_LC_with_respect_to_target_space('ego',3,'back'))) or (action == 1 and (traci.vehicle.getPosition('ego')[0] > traci.vehicle.getPosition(rlEnv.Target_left_car)[0] + self.SD_LC_with_respect_to_target_space('ego',1) and traci.vehicle.getPosition('ego')[0] < traci.vehicle.getPosition(left_leader)[0] - self.SD_LC_with_respect_to_target_space('ego',1,'front'))):
    #                     longitudinal_term = 0.05
    #                 elif(action == 3 and (traci.vehicle.getPosition('ego')[0] >= traci.vehicle.getPosition(rlEnv.Target_left_car)[0] - self.SD_LC_with_respect_to_target_space('ego',3,'front'))) or (action == 1 and (traci.vehicle.getPosition('ego')[0] <= traci.vehicle.getPosition(rlEnv.Target_left_car)[0] + self.SD_LC_with_respect_to_target_space('ego',1))):
    #                     longitudinal_term = 2**(-SD_LC_dist_to_biggest_empty_space)
    #                 elif(action == 3 and (traci.vehicle.getPosition('ego')[0] <= traci.vehicle.getPosition(rlEnv.Target_left_car)[0] + self.SD_LC_with_respect_to_target_space('ego',3,'back'))) or (action == 1 and (traci.vehicle.getPosition('ego')[0] >= traci.vehicle.getPosition(left_leader)[0] - self.SD_LC_with_respect_to_target_space('ego',1,'front'))):
    #                     longitudinal_term = 2**(-SD_LC_dist_to_biggest_empty_space2)
    #                 else:
    #                     longitudinal_term = 0

    #                 if dist_to_biggest_empty_space<=np.sqrt((order[-1][0]/2)**2+3.2**2):
    #                     lateral_distance_error = np.abs(-1.6 -traci.vehicle.getPosition('ego')[1])                        
    #                     lateral_term = w_l*2**(-lateral_distance_error)
                        
    #                     value+= longitudinal_term+lateral_term
    #                     space_reward.append(longitudinal_term +lateral_term)
    #                     space_reward_lat.append(lateral_term)
    #                     # print('lateral_distance_error: ',lateral_distance_error) 
    #                     # print('space_reward_lat : ',lateral_term)
    #                 else:
    #                     # print('space_reward_lat : 0')                       
    #                     value+= longitudinal_term
    #                     space_reward.append(longitudinal_term)
    #                     space_reward_lat.append(0)
                    
    #                 space_reward_long.append(longitudinal_term)   
    #                 # print("space_reward_long : ",longitudinal_term)                 
    #                 # print('1st-')  
                                     
                    
    #             elif order[-1][2][0] == 'R': # order[-1][2][0] == 'R':
    #                 if (action == 4 and (traci.vehicle.getPosition('ego')[0] < traci.vehicle.getPosition(rlEnv.Target_right_car)[0] - self.SD_LC_with_respect_to_target_space('ego',4))) or (action == 2 and (traci.vehicle.getPosition('ego')[0] > traci.vehicle.getPosition(rlEnv.Target_right_car)[0] + self.SD_LC_with_respect_to_target_space('ego',2))):
    #                     longitudinal_term = 0.05
    #                 else:
    #                     longitudinal_term = 2**(-SD_LC_dist_to_biggest_empty_space)

    #                 if dist_to_biggest_empty_space<=np.sqrt((order[-1][0]/2)**2+3.2**2):
    #                     lateral_distance_error = np.abs(-8 -traci.vehicle.getPosition('ego')[1])
    #                     lateral_term = w_l*2**(-lateral_distance_error)
    #                     value+= longitudinal_term+lateral_term
    #                     space_reward.append(longitudinal_term+lateral_term)
    #                     space_reward_lat.append(lateral_term)
    #                     # print('lateral_distance_error: ',lateral_distance_error)
    #                     # print('space_reward_lat : ',lateral_term)
    #                 else:                        
    #                     value+=longitudinal_term 
    #                     space_reward.append(longitudinal_term)
    #                     space_reward_lat.append(0)
    #                     # print('space_reward_lat : 0')
    #                 # print('1st')                    
    #                 space_reward_long.append(longitudinal_term)
    #                 # print("space_reward_long : ",longitudinal_term)
                    
                                        
    #             else:
    #                 space_reward.append(0)
    #                 space_reward_long.append(0)
    #                 space_reward_lat.append(0)
    #                 value+= 0
    #         else:
    #             space_reward.append(0)
    #             space_reward_long.append(0)
    #             space_reward_lat.append(0)
                      
    #         # if dist_to_second_biggest_empty_space<=np.sqrt((order[-2][0]/2)**2+3.2**2) and(order[-2][2] =='L1' and action == 1 or order[-2][2] =='R1' and action == 2 or order[-2][2] =='L2' and action == 3 or order[-2][2] =='R2' and action == 4): 
    #         if ((order[-2][2] =='L1' and action == 1) or (order[-2][2] =='R1' and action == 2) or (order[-2][2] =='L2' and action == 3) or (order[-2][2] =='R2' and action == 4)): 
    #             # print(order[-2][2])
    #             if order[-2][2][0] == 'L':
    #                 if (action == 3 and (traci.vehicle.getPosition('ego')[0] < traci.vehicle.getPosition(rlEnv.Target_left_car)[0] - self.SD_LC_with_respect_to_target_space('ego',3))) or (action == 1 and (traci.vehicle.getPosition('ego')[0] > traci.vehicle.getPosition(rlEnv.Target_left_car)[0] + self.SD_LC_with_respect_to_target_space('ego',1))):
    #                     longitudinal_term = 0.05
    #                 else:
    #                     longitudinal_term = 2**(-SD_LC_dist_to_second_biggest_empty_space)
    #                 if dist_to_second_biggest_empty_space<=np.sqrt((order[-2][0]/2)**2+3.2**2):
    #                     lateral_distance_error = np.abs(-1.6 -traci.vehicle.getPosition('ego')[1])
    #                     lateral_term = w_l*2**(-lateral_distance_error)
    #                     value+= (longitudinal_term+lateral_term)*second_biggest_size_value
    #                     space_reward.append((longitudinal_term+lateral_term)*second_biggest_size_value)
    #                     space_reward_lat.append(lateral_term*second_biggest_size_value)
    #                     # print('space_reward_lat: ',lateral_term*second_biggest_size_value)
    #                     # print('lateral_distance_error: ',lateral_distance_error)
    #                 else:
    #                     value+= (longitudinal_term)*second_biggest_size_value
    #                     space_reward.append(longitudinal_term*second_biggest_size_value)
    #                     space_reward_lat.append(0)
    #                     # print('space_reward_lat: ',0)
                        
    #                 # print('2nd-')                    
    #                 space_reward_long.append(longitudinal_term*second_biggest_size_value)
    #                 # print('space_reward_long: ',longitudinal_term*second_biggest_size_value)
                    
    #             elif order[-2][2][0] == 'R': # order[-1][2][0] == 'R':
    #                 if (action == 4 and (traci.vehicle.getPosition('ego')[0] < traci.vehicle.getPosition(rlEnv.Target_right_car)[0] - self.SD_LC_with_respect_to_target_space('ego',4))) or (action == 2 and (traci.vehicle.getPosition('ego')[0] > traci.vehicle.getPosition(rlEnv.Target_right_car)[0] + self.SD_LC_with_respect_to_target_space('ego',2))):
    #                     longitudinal_term = 0.05
    #                 else:
    #                     longitudinal_term = 2**(-SD_LC_dist_to_second_biggest_empty_space)

    #                 if dist_to_second_biggest_empty_space<=np.sqrt((order[-2][0]/2)**2+3.2**2):
    #                     lateral_distance_error = np.abs(-8 -traci.vehicle.getPosition('ego')[1])
    #                     lateral_term = w_l*2**(-lateral_distance_error)
    #                     value+= (longitudinal_term+lateral_term)*second_biggest_size_value
    #                     space_reward.append((longitudinal_term+lateral_term)*second_biggest_size_value)
    #                     space_reward_lat.append(lateral_term*second_biggest_size_value) 
    #                     # print('space_reward_lat: ',lateral_term*second_biggest_size_value)
    #                     # print('lateral_distance_error: ',lateral_distance_error)
    #                 else:
    #                     value+= longitudinal_term*second_biggest_size_value
    #                     space_reward.append(longitudinal_term*second_biggest_size_value)
    #                     space_reward_lat.append(0)
    #                     # print('space_reward_lat: ',0)
    #                 # print('2nd')                    
    #                 space_reward_long.append(longitudinal_term*second_biggest_size_value)                    
    #                 # print('space_reward_long: ',longitudinal_term*second_biggest_size_value)                 
    #             else:
    #                 space_reward.append(0)
    #                 space_reward_long.append(0)
    #                 space_reward_lat.append(0)
    #                 value+= 0
    #         else:
    #             space_reward.append(0)
    #             space_reward_long.append(0)
    #             space_reward_lat.append(0)
                
    #         # if dist_to_third_biggest_empty_space<=np.sqrt((order[-3][0]/2)**2+3.2**2) and(order[-3][2] =='L1' and action == 1 or order[-3][2] =='R1' and action == 2 or order[-3][2] =='L2' and action == 3 or order[-3][2] =='R2' and action == 4):
    #         if ((order[-3][2] =='L1' and action == 1) or (order[-3][2] =='R1' and action == 2) or (order[-3][2] =='L2' and action == 3) or (order[-3][2] =='R2' and action == 4)):
    #             # print(order[-3][2])
    #             if order[-3][2][0] == 'L':
    #                 if (action == 3 and (traci.vehicle.getPosition('ego')[0] < traci.vehicle.getPosition(rlEnv.Target_left_car)[0] - self.SD_LC_with_respect_to_target_space('ego',3))) or (action == 1 and (traci.vehicle.getPosition('ego')[0] > traci.vehicle.getPosition(rlEnv.Target_left_car)[0] + self.SD_LC_with_respect_to_target_space('ego',1))):
    #                     longitudinal_term = 0.05
    #                 else:
    #                     longitudinal_term = 2**(-SD_LC_dist_to_third_biggest_empty_space)
    #                 if dist_to_third_biggest_empty_space<=np.sqrt((order[-3][0]/2)**2+3.2**2):
    #                     lateral_distance_error = np.abs(-1.6 -traci.vehicle.getPosition('ego')[1])
    #                     lateral_term = w_l*2**(-lateral_distance_error)
    #                     value+= (longitudinal_term+lateral_term)*third_biggest_size_value
    #                     space_reward.append((longitudinal_term+lateral_term)*third_biggest_size_value)
    #                     space_reward_lat.append(lateral_term*third_biggest_size_value)
    #                     # print('space_reward_lat: ',lateral_term*third_biggest_size_value)
    #                     # print('lateral_distance_error: ',lateral_distance_error)
    #                 else:
    #                     value+= longitudinal_term*third_biggest_size_value
    #                     space_reward.append(longitudinal_term*third_biggest_size_value)
    #                     space_reward_lat.append(0)
    #                     # print('space_reward_lat: ',0)
    #                 # print('3rd-')                    
    #                 space_reward_long.append(longitudinal_term*third_biggest_size_value)
    #                 # print('space_reward_long: ',longitudinal_term*third_biggest_size_value)
                                       
    #             elif order[-3][2][0] == 'R': # order[-1][2][0] == 'R':
    #                 if (action == 4 and (traci.vehicle.getPosition('ego')[0] < traci.vehicle.getPosition(rlEnv.Target_right_car)[0] - self.SD_LC_with_respect_to_target_space('ego',4))) or (action == 2 and (traci.vehicle.getPosition('ego')[0] > traci.vehicle.getPosition(rlEnv.Target_right_car)[0] + self.SD_LC_with_respect_to_target_space('ego',2))):
    #                     longitudinal_term = 0.05
    #                 else:
    #                     longitudinal_term = 2**(-SD_LC_dist_to_third_biggest_empty_space)
    #                 if dist_to_third_biggest_empty_space<=np.sqrt((order[-3][0]/2)**2+3.2**2):
    #                     lateral_distance_error = np.abs(-8 -traci.vehicle.getPosition('ego')[1])
    #                     lateral_term = w_l*2**(-lateral_distance_error)
    #                     value+= (longitudinal_term+lateral_term)*third_biggest_size_value
    #                     space_reward.append((longitudinal_term+lateral_term)*third_biggest_size_value )                       
    #                     space_reward_lat.append(lateral_term*third_biggest_size_value)
    #                     # print('space_reward_lat: ',lateral_term*third_biggest_size_value)
    #                     # print('lateral_distance_error: ',lateral_distance_error)
    #                 else:
    #                     value+= longitudinal_term*third_biggest_size_value
    #                     space_reward.append(longitudinal_term*third_biggest_size_value)
    #                     space_reward_lat.append(0)
    #                     # print('space_reward_lat: ',0)
    #                 # print('3rd')
                    
                    
    #                 space_reward_long.append(longitudinal_term*third_biggest_size_value)
    #                 # print('space_reward_long: ',longitudinal_term*third_biggest_size_value)
                                        
    #             else:
    #                 space_reward.append(0)
    #                 space_reward_long.append(0)
    #                 space_reward_lat.append(0)
    #                 value+= 0
    #         else:
    #             space_reward.append(0)
    #             space_reward_long.append(0)
    #             space_reward_lat.append(0)
                
    #         # if dist_to_forth_biggest_empty_space<=np.sqrt((order[-4][0]/2)**2+3.2**2) and(order[-4][2] =='L1' and action == 1 or order[-4][2] =='R1' and action == 2 or order[-4][2] =='L2' and action == 3 or order[-4][2] =='R2' and action == 4):
    #         if ((order[-4][2] =='L1' and action == 1) or (order[-4][2] =='R1' and action == 2) or (order[-4][2] =='L2' and action == 3) or (order[-4][2] =='R2' and action == 4)):
    #             # print(order[-4 ][2])
    #             if order[-4][2][0] == 'L':
    #                 if (action == 3 and (traci.vehicle.getPosition('ego')[0] < traci.vehicle.getPosition(rlEnv.Target_left_car)[0] - self.SD_LC_with_respect_to_target_space('ego',3))) or (action == 1 and (traci.vehicle.getPosition('ego')[0] > traci.vehicle.getPosition(rlEnv.Target_left_car)[0] + self.SD_LC_with_respect_to_target_space('ego',1))):
    #                     longitudinal_term = 0.05
    #                 else:
    #                     longitudinal_term = 2**(-SD_LC_dist_to_forth_biggest_empty_space)
    #                 if dist_to_forth_biggest_empty_space<=np.sqrt((order[-4][0]/2)**2+3.2**2):
    #                     lateral_distance_error = np.abs(-1.6 -traci.vehicle.getPosition('ego')[1])
    #                     lateral_term = w_l*2**(-lateral_distance_error)
    #                     value+= (longitudinal_term+lateral_term)*forth_biggest_size_value
    #                     space_reward.append((longitudinal_term+lateral_term)*forth_biggest_size_value)
    #                     space_reward_lat.append(lateral_term*forth_biggest_size_value)
    #                     # print('space_reward_lat: ',lateral_term*forth_biggest_size_value)
    #                     # print('lateral_distance_error: ',lateral_distance_error)
    #                 else:
    #                     value+= longitudinal_term*forth_biggest_size_value 
    #                     space_reward.append(longitudinal_term*forth_biggest_size_value)
    #                     space_reward_lat.append(0)
    #                     # print('space_reward_lat: ',0)
    #                 # print('4th-')                    
    #                 space_reward_long.append(longitudinal_term*forth_biggest_size_value)
    #                 # print('space_reward_long: ',longitudinal_term*forth_biggest_size_value)
                                       
    #             elif order[-4][2][0] == 'R': # order[-1][2][0] == 'R':
    #                 if (action == 4 and (traci.vehicle.getPosition('ego')[0] < traci.vehicle.getPosition(rlEnv.Target_right_car)[0] - self.SD_LC_with_respect_to_target_space('ego',4))) or (action == 2 and (traci.vehicle.getPosition('ego')[0] > traci.vehicle.getPosition(rlEnv.Target_right_car)[0] + self.SD_LC_with_respect_to_target_space('ego',2))):
    #                     longitudinal_term = 0.05
    #                 else:
    #                     longitudinal_term = 2**(-SD_LC_dist_to_forth_biggest_empty_space)
    #                 if dist_to_forth_biggest_empty_space<=np.sqrt((order[-4][0]/2)**2+3.2**2):
    #                     lateral_distance_error = np.abs(-8 -traci.vehicle.getPosition('ego')[1])
    #                     lateral_term = w_l*2**(-lateral_distance_error)
    #                     value+= (longitudinal_term+lateral_term)*forth_biggest_size_value
    #                     space_reward.append((longitudinal_term+lateral_term)*forth_biggest_size_value)
    #                     space_reward_lat.append(lateral_term*forth_biggest_size_value)
    #                     # print('space_reward_lat: ',lateral_term*forth_biggest_size_value)
    #                     # print('lateral_distance_error: ',lateral_distance_error)
    #                 else:
    #                     value+= longitudinal_term*forth_biggest_size_value
    #                     space_reward.append(longitudinal_term*forth_biggest_size_value)
    #                     space_reward_lat.append(0)
    #                     # print('space_reward_lat: ',0)
    #                 # print('4th')                    
    #                 space_reward_long.append(longitudinal_term*forth_biggest_size_value)
    #                 # print('space_reward_long: ',longitudinal_term*forth_biggest_size_value)
                    
                                     
    #             else:
    #                 space_reward.append(0)
    #                 space_reward_long.append(0)
    #                 space_reward_lat.append(0)
    #                 value+= 0
    #         else:
    #             space_reward.append(0)
    #             space_reward_long.append(0)
    #             space_reward_lat.append(0)
    #         rlEnv.toward_space_reward.append(space_reward)
    #         rlEnv.toward_space_reward_long.append(space_reward_long)
    #         rlEnv.toward_space_reward_lat.append(space_reward_lat)
    #         return value
    #     else:
    #         space_reward.append(0)
    #         space_reward.append(0)
    #         space_reward.append(0)
    #         space_reward.append(0)
    #         space_reward_long.append(0)
    #         space_reward_lat.append(0)
    #         space_reward_long.append(0)
    #         space_reward_lat.append(0)
    #         space_reward_long.append(0)
    #         space_reward_lat.append(0)
    #         space_reward_long.append(0)
    #         space_reward_lat.append(0)
    #         rlEnv.toward_space_reward.append(space_reward)
    #         rlEnv.toward_space_reward_long.append(space_reward_long)
    #         rlEnv.toward_space_reward_lat.append(space_reward_lat)
    #         return 0
        
    # def __toward_biggest_empty_space_reward(self,id,action):
    #     space_reward=[]
    #     space_reward_long=[]
    #     space_reward_lat=[]
    #     space_reward.append(traci.simulation.getTime())
    #     space_reward_long.append(traci.simulation.getTime())
    #     space_reward_lat.append(traci.simulation.getTime())
    #     vehs = traci.vehicle.getIDList()
    #     if id in vehs and rlEnv.Target_left_car in vehs and rlEnv.Target_right_car in vehs:        
    #         if traci.vehicle.getLeader(rlEnv.Target_left_car)[0] == 'ego':
    #             left_leader = traci.vehicle.getLeader('ego')[0]
    #         else:
    #             left_leader = traci.vehicle.getLeader(rlEnv.Target_left_car)[0]
    #         if traci.vehicle.getFollower(rlEnv.Target_left_car)[0] =='ego':
    #             left_follower = traci.vehicle.getFollower('ego')[0]
    #         else:
    #             left_follower = traci.vehicle.getFollower(rlEnv.Target_left_car)[0]
            
    #         if traci.vehicle.getLeader(rlEnv.Target_right_car)[0] == 'ego':
    #             right_leader = traci.vehicle.getLeader('ego')[0]
    #         else:
    #             right_leader = traci.vehicle.getLeader(rlEnv.Target_right_car)[0]
    #         if traci.vehicle.getFollower(rlEnv.Target_right_car)[0] =='ego':
    #             right_follower = traci.vehicle.getFollower('ego')[0]
    #         else:
    #             right_follower = traci.vehicle.getFollower(rlEnv.Target_right_car)[0]
    #         #distance to space
    #         s1 = np.sqrt(((traci.vehicle.getPosition(rlEnv.Target_left_car)[0] + traci.vehicle.getPosition(left_leader)[0])/2 - traci.vehicle.getPosition('ego')[0])**2 +(traci.vehicle.getPosition(rlEnv.Target_left_car)[1]- traci.vehicle.getPosition('ego')[1])**2)#자차와 왼쪽 차로 전방 빈공간의 중점까지 상대 거리
    #         s2 = np.sqrt(((traci.vehicle.getPosition(rlEnv.Target_right_car)[0] + traci.vehicle.getPosition(right_leader)[0])/2 - traci.vehicle.getPosition('ego')[0])**2+(traci.vehicle.getPosition(rlEnv.Target_right_car)[1]- traci.vehicle.getPosition('ego')[1])**2) #자차와 오른쪽 차로 전방 빈공간의 중점까지 상대 거리
    #         s3 = np.sqrt(((traci.vehicle.getPosition(rlEnv.Target_left_car)[0] + traci.vehicle.getPosition(left_follower)[0])/2 - traci.vehicle.getPosition('ego')[0])**2+(traci.vehicle.getPosition(rlEnv.Target_left_car)[1]- traci.vehicle.getPosition('ego')[1])**2) #자차와 왼쪽 차로 후방 빈공간의 중점까지 상대 거리            
    #         s4 = np.sqrt(((traci.vehicle.getPosition(rlEnv.Target_right_car)[0] + traci.vehicle.getPosition(right_follower)[0])/2 - traci.vehicle.getPosition('ego')[0])**2+(traci.vehicle.getPosition(rlEnv.Target_right_car)[1]- traci.vehicle.getPosition('ego')[1])**2)#자차와 오른쪽 차로 후방 빈공간의 중점까지 상대 거리  
    #         #space size
    #         space1_size = np.abs((traci.vehicle.getPosition(rlEnv.Target_left_car)[0] - traci.vehicle.getPosition(left_leader)[0]))
    #         space2_size = np.abs((traci.vehicle.getPosition(rlEnv.Target_right_car)[0] - traci.vehicle.getPosition(right_leader)[0]))
    #         space3_size = np.abs((traci.vehicle.getPosition(rlEnv.Target_left_car)[0] - traci.vehicle.getPosition(left_follower)[0]))
    #         space4_size = np.abs((traci.vehicle.getPosition(rlEnv.Target_right_car)[0] - traci.vehicle.getPosition(right_follower)[0]))
    #         # print("space1_size: ",space1_size)
    #         # print("space2_size: ",space2_size)
    #         # print("space3_size: ",space3_size)
    #         # print("space4_size: ",space4_size)
    #         space1 = [space1_size,s1,'L1'] #left front
    #         space2 = [space2_size,s2,'R1'] #right front
    #         space3 = [space3_size,s3,'L2'] #left back
    #         space4 = [space4_size,s4,'R2'] #right back
    #         order =[space1,space2,space3,space4]
    #         order.sort()
    #         dist_to_biggest_empty_space = order[-1][1]
    #         dist_to_second_biggest_empty_space = order[-2][1]
    #         dist_to_third_biggest_empty_space = order[-3][1]
    #         dist_to_forth_biggest_empty_space = order[-4][1]
    #         value =0
    #         total_space_size = space1_size+space2_size+space3_size+space4_size
    #         # print('1st space weight: ',(order[-1][0]/total_space_size))
    #         # print('2nd space weight: ',(order[-2][0]/total_space_size))
    #         # print('3rd space weight: ',(order[-3][0]/total_space_size))
    #         # print('4th space weight: ',(order[-4][0]/total_space_size))
    #         # print('ego_y: ',traci.vehicle.getPosition('ego')[1])
    #         # if dist_to_biggest_empty_space<=np.sqrt((order[-1][0]/2)**2+3.2**2) and(order[-1][2] =='L1' and action == 1 or order[-1][2] =='R1' and action == 2 or order[-1][2] =='L2' and action == 3 or order[-1][2] =='R2' and action == 4): #If ego vehicle is next to the open space then the reward adding start.
    #         if ((order[-1][2] =='L1' and action == 1) or (order[-1][2] =='R1' and action == 2) or (order[-1][2] =='L2' and action == 3) or (order[-1][2] =='R2' and action == 4)): #If ego vehicle is next to the open space then the reward adding start.
                                
    #             # print(order[-1][2])
    #             if order[-1][2][0] == 'L' :
    #                 if dist_to_biggest_empty_space<=np.sqrt((order[-1][0]/2)**2+3.2**2):
    #                     lateral_distance_error = np.abs(-1.6 -traci.vehicle.getPosition('ego')[1])
    #                     value+= 2**(-dist_to_biggest_empty_space) +10*2**(-lateral_distance_error)
    #                     space_reward.append(2**(-dist_to_biggest_empty_space) +10*2**(-lateral_distance_error))
    #                     space_reward_lat.append(10*2**(-lateral_distance_error))
    #                     # print('lateral_distance_error: ',lateral_distance_error) 
    #                 else:
    #                     value+= 2**(-dist_to_biggest_empty_space)
    #                     space_reward.append(2**(-dist_to_biggest_empty_space))
    #                     space_reward_lat.append(0)
                    
    #                 space_reward_long.append(2**(-dist_to_biggest_empty_space))                    
    #                 # print('1st-')  
                                     
                    
    #             elif order[-1][2][0] == 'R': # order[-1][2][0] == 'R':
    #                 if dist_to_biggest_empty_space<=np.sqrt((order[-1][0]/2)**2+3.2**2):
    #                     lateral_distance_error = np.abs(-8 -traci.vehicle.getPosition('ego')[1])
    #                     value+= 2**(-dist_to_biggest_empty_space) +10*2**(-lateral_distance_error)
    #                     space_reward.append(2**(-dist_to_biggest_empty_space) +10*2**(-lateral_distance_error))
    #                     space_reward_lat.append(10*2**(-lateral_distance_error))
    #                     # print('lateral_distance_error: ',lateral_distance_error)
    #                 else:                        
    #                     value+= 2**(-dist_to_biggest_empty_space) 
    #                     space_reward.append(2**(-dist_to_biggest_empty_space))
    #                     space_reward_lat.append(0)
    #                 # print('1st')                    
    #                 space_reward_long.append(2**(-dist_to_biggest_empty_space))
                    
                                        
    #             else:
    #                 space_reward.append(0)
    #                 space_reward_long.append(0)
    #                 space_reward_lat.append(0)
    #                 value+= 0
    #         else:
    #             space_reward.append(0)
    #             space_reward_long.append(0)
    #             space_reward_lat.append(0)
                      
    #         # if dist_to_second_biggest_empty_space<=np.sqrt((order[-2][0]/2)**2+3.2**2) and(order[-2][2] =='L1' and action == 1 or order[-2][2] =='R1' and action == 2 or order[-2][2] =='L2' and action == 3 or order[-2][2] =='R2' and action == 4): 
    #         if ((order[-2][2] =='L1' and action == 1) or (order[-2][2] =='R1' and action == 2) or (order[-2][2] =='L2' and action == 3) or (order[-2][2] =='R2' and action == 4)): 
    #             # print(order[-2][2])
    #             if order[-2][2][0] == 'L':
    #                 if dist_to_second_biggest_empty_space<=np.sqrt((order[-2][0]/2)**2+3.2**2):
    #                     lateral_distance_error = np.abs(-1.6 -traci.vehicle.getPosition('ego')[1])
    #                     value+= (2**(-dist_to_second_biggest_empty_space)+10*2**(-lateral_distance_error))*0.25
    #                     space_reward.append((2**(-dist_to_second_biggest_empty_space)+10*2**(-lateral_distance_error))*0.25)
    #                     space_reward_lat.append(10*2**(-lateral_distance_error)*0.25)
    #                     # print('lateral_distance_error: ',lateral_distance_error)
    #                 else:
    #                     value+= (2**(-dist_to_second_biggest_empty_space))*0.25
    #                     space_reward.append((2**(-dist_to_second_biggest_empty_space))*0.25)
    #                     space_reward_lat.append(0)
                        
    #                 # print('2nd-')                    
    #                 space_reward_long.append(2**(-dist_to_second_biggest_empty_space)*0.25)
                    
                    
    #             elif order[-2][2][0] == 'R': # order[-1][2][0] == 'R':
    #                 if dist_to_second_biggest_empty_space<=np.sqrt((order[-2][0]/2)**2+3.2**2):
    #                     lateral_distance_error = np.abs(-8 -traci.vehicle.getPosition('ego')[1])
    #                     value+= (2**(-dist_to_second_biggest_empty_space)+10*2**(-lateral_distance_error))*0.25
    #                     space_reward.append((2**(-dist_to_second_biggest_empty_space)+10*2**(-lateral_distance_error))*0.25)
    #                     space_reward_lat.append(10*2**(-lateral_distance_error)*0.25) 
    #                     # print('lateral_distance_error: ',lateral_distance_error)
    #                 else:
    #                     value+= (2**(-dist_to_second_biggest_empty_space))*0.25 
    #                     space_reward.append((2**(-dist_to_second_biggest_empty_space))*0.25)
    #                     space_reward_lat.append(0)
    #                 # print('2nd')                    
    #                 space_reward_long.append(2**(-dist_to_second_biggest_empty_space)*0.25)                    
                                     
    #             else:
    #                 space_reward.append(0)
    #                 space_reward_long.append(0)
    #                 space_reward_lat.append(0)
    #                 value+= 0
    #         else:
    #             space_reward.append(0)
    #             space_reward_long.append(0)
    #             space_reward_lat.append(0)
                
    #         # if dist_to_third_biggest_empty_space<=np.sqrt((order[-3][0]/2)**2+3.2**2) and(order[-3][2] =='L1' and action == 1 or order[-3][2] =='R1' and action == 2 or order[-3][2] =='L2' and action == 3 or order[-3][2] =='R2' and action == 4):
    #         if ((order[-3][2] =='L1' and action == 1) or (order[-3][2] =='R1' and action == 2) or (order[-3][2] =='L2' and action == 3) or (order[-3][2] =='R2' and action == 4)):
    #             # print(order[-3][2])
    #             if order[-3][2][0] == 'L':
    #                 if dist_to_third_biggest_empty_space<=np.sqrt((order[-3][0]/2)**2+3.2**2):
    #                     lateral_distance_error = np.abs(-1.6 -traci.vehicle.getPosition('ego')[1])
    #                     value+= (2**(-dist_to_third_biggest_empty_space)+10*2**(-lateral_distance_error))*0.25
    #                     space_reward.append((2**(-dist_to_third_biggest_empty_space)+10*2**(-lateral_distance_error))*0.25)
    #                     space_reward_lat.append(10*2**(-lateral_distance_error)*0.25)
    #                     # print('lateral_distance_error: ',lateral_distance_error)
    #                 else:
    #                     value+= (2**(-dist_to_third_biggest_empty_space))*0.25
    #                     space_reward.append((2**(-dist_to_third_biggest_empty_space))*0.25)
    #                     space_reward_lat.append(0)
    #                 # print('3rd-')                    
    #                 space_reward_long.append(2**(-dist_to_third_biggest_empty_space)*0.25)
                    
                                       
    #             elif order[-3][2][0] == 'R': # order[-1][2][0] == 'R':
    #                 if dist_to_third_biggest_empty_space<=np.sqrt((order[-3][0]/2)**2+3.2**2):
    #                     lateral_distance_error = np.abs(-8 -traci.vehicle.getPosition('ego')[1])
    #                     value+= (2**(-dist_to_third_biggest_empty_space)+10*2**(-lateral_distance_error))*0.25 
    #                     space_reward.append((2**(-dist_to_third_biggest_empty_space)+10*2**(-lateral_distance_error))*0.25 )                       
    #                     space_reward_lat.append(10*2**(-lateral_distance_error)*0.25)
    #                     # print('lateral_distance_error: ',lateral_distance_error)
    #                 else:
    #                     value+= (2**(-dist_to_third_biggest_empty_space))*0.25
    #                     space_reward.append((2**(-dist_to_third_biggest_empty_space))*0.25)
    #                     space_reward_lat.append(0)
    #                 # print('3rd')
                    
                    
    #                 space_reward_long.append(2**(-dist_to_third_biggest_empty_space)*0.25)
                    
                                        
    #             else:
    #                 space_reward.append(0)
    #                 space_reward_long.append(0)
    #                 space_reward_lat.append(0)
    #                 value+= 0
    #         else:
    #             space_reward.append(0)
    #             space_reward_long.append(0)
    #             space_reward_lat.append(0)
                
    #         # if dist_to_forth_biggest_empty_space<=np.sqrt((order[-4][0]/2)**2+3.2**2) and(order[-4][2] =='L1' and action == 1 or order[-4][2] =='R1' and action == 2 or order[-4][2] =='L2' and action == 3 or order[-4][2] =='R2' and action == 4):
    #         if ((order[-4][2] =='L1' and action == 1) or (order[-4][2] =='R1' and action == 2) or (order[-4][2] =='L2' and action == 3) or (order[-4][2] =='R2' and action == 4)):
    #             # print(order[-4 ][2])
    #             if order[-4][2][0] == 'L':
    #                 if dist_to_forth_biggest_empty_space<=np.sqrt((order[-4][0]/2)**2+3.2**2):
    #                     lateral_distance_error = np.abs(-1.6 -traci.vehicle.getPosition('ego')[1])
    #                     value+= (2**(-dist_to_forth_biggest_empty_space)+10*2**(-lateral_distance_error))*0.25 
    #                     space_reward.append((2**(-dist_to_forth_biggest_empty_space)+10*2**(-lateral_distance_error))*0.25)
    #                     space_reward_lat.append(10*2**(-lateral_distance_error)*0.25)
    #                     # print('lateral_distance_error: ',lateral_distance_error)
    #                 else:
    #                     value+= (2**(-dist_to_forth_biggest_empty_space))*0.25 
    #                     space_reward.append((2**(-dist_to_forth_biggest_empty_space))*0.25)
    #                     space_reward_lat.append(0)
    #                 # print('4th-')                    
    #                 space_reward_long.append(2**(-dist_to_forth_biggest_empty_space)*0.25)
                    
                                       
    #             elif order[-4][2][0] == 'R': # order[-1][2][0] == 'R':
    #                 if dist_to_forth_biggest_empty_space<=np.sqrt((order[-4][0]/2)**2+3.2**2):
    #                     lateral_distance_error = np.abs(-8 -traci.vehicle.getPosition('ego')[1])
    #                     value+= (2**(-dist_to_forth_biggest_empty_space)+10*2**(-lateral_distance_error))*0.25
    #                     space_reward.append((2**(-dist_to_forth_biggest_empty_space)+10*2**(-lateral_distance_error))*0.25)
    #                     space_reward_lat.append(10*2**(-lateral_distance_error)*0.25 )
    #                     # print('lateral_distance_error: ',lateral_distance_error)
    #                 else:
    #                     value+= (2**(-dist_to_forth_biggest_empty_space))*0.25
    #                     space_reward.append((2**(-dist_to_forth_biggest_empty_space))*0.25)
    #                     space_reward_lat.append(0)
    #                 # print('4th')                    
    #                 space_reward_long.append(2**(-dist_to_forth_biggest_empty_space)*0.25 )
                    
                                     
    #             else:
    #                 space_reward.append(0)
    #                 space_reward_long.append(0)
    #                 space_reward_lat.append(0)
    #                 value+= 0
    #         else:
    #             space_reward.append(0)
    #             space_reward_long.append(0)
    #             space_reward_lat.append(0)
    #         rlEnv.toward_space_reward.append(space_reward)
    #         rlEnv.toward_space_reward_long.append(space_reward_long)
    #         rlEnv.toward_space_reward_lat.append(space_reward_lat)
    #         return value
    #     else:
    #         space_reward.append(0)
    #         space_reward.append(0)
    #         space_reward.append(0)
    #         space_reward.append(0)
    #         space_reward_long.append(0)
    #         space_reward_lat.append(0)
    #         space_reward_long.append(0)
    #         space_reward_lat.append(0)
    #         space_reward_long.append(0)
    #         space_reward_lat.append(0)
    #         space_reward_long.append(0)
    #         space_reward_lat.append(0)
    #         rlEnv.toward_space_reward.append(space_reward)
    #         rlEnv.toward_space_reward_long.append(space_reward_long)
    #         rlEnv.toward_space_reward_lat.append(space_reward_lat)
    #         return 0
        
    # def __toward_biggest_empty_space_reward(self,id):
    #     space_reward=[]
    #     space_reward_long=[]
    #     space_reward_lat=[]
    #     space_reward.append(traci.simulation.getTime())
    #     space_reward_long.append(traci.simulation.getTime())
    #     space_reward_lat.append(traci.simulation.getTime())
    #     vehs = traci.vehicle.getIDList()
    #     if id in vehs and rlEnv.Target_left_car in vehs and rlEnv.Target_right_car in vehs:        
    #         if traci.vehicle.getLeader(rlEnv.Target_left_car)[0] == 'ego':
    #             left_leader = traci.vehicle.getLeader('ego')[0]
    #         else:
    #             left_leader = traci.vehicle.getLeader(rlEnv.Target_left_car)[0]
    #         if traci.vehicle.getFollower(rlEnv.Target_left_car)[0] =='ego':
    #             left_follower = traci.vehicle.getFollower('ego')[0]
    #         else:
    #             left_follower = traci.vehicle.getFollower(rlEnv.Target_left_car)[0]
            
    #         if traci.vehicle.getLeader(rlEnv.Target_right_car)[0] == 'ego':
    #             right_leader = traci.vehicle.getLeader('ego')[0]
    #         else:
    #             right_leader = traci.vehicle.getLeader(rlEnv.Target_right_car)[0]
    #         if traci.vehicle.getFollower(rlEnv.Target_right_car)[0] =='ego':
    #             right_follower = traci.vehicle.getFollower('ego')[0]
    #         else:
    #             right_follower = traci.vehicle.getFollower(rlEnv.Target_right_car)[0]
    #         #distance to space
    #         s1 = np.sqrt(((traci.vehicle.getPosition(rlEnv.Target_left_car)[0] + traci.vehicle.getPosition(left_leader)[0])/2 - traci.vehicle.getPosition('ego')[0])**2 +(traci.vehicle.getPosition(rlEnv.Target_left_car)[1]- traci.vehicle.getPosition('ego')[1])**2)#자차와 왼쪽 차로 전방 빈공간의 중점까지 상대 거리
    #         s2 = np.sqrt(((traci.vehicle.getPosition(rlEnv.Target_right_car)[0] + traci.vehicle.getPosition(right_leader)[0])/2 - traci.vehicle.getPosition('ego')[0])**2+(traci.vehicle.getPosition(rlEnv.Target_right_car)[1]- traci.vehicle.getPosition('ego')[1])**2) #자차와 오른쪽 차로 전방 빈공간의 중점까지 상대 거리
    #         s3 = np.sqrt(((traci.vehicle.getPosition(rlEnv.Target_left_car)[0] + traci.vehicle.getPosition(left_follower)[0])/2 - traci.vehicle.getPosition('ego')[0])**2+(traci.vehicle.getPosition(rlEnv.Target_left_car)[1]- traci.vehicle.getPosition('ego')[1])**2) #자차와 왼쪽 차로 후방 빈공간의 중점까지 상대 거리            
    #         s4 = np.sqrt(((traci.vehicle.getPosition(rlEnv.Target_right_car)[0] + traci.vehicle.getPosition(right_follower)[0])/2 - traci.vehicle.getPosition('ego')[0])**2+(traci.vehicle.getPosition(rlEnv.Target_right_car)[1]- traci.vehicle.getPosition('ego')[1])**2)#자차와 오른쪽 차로 후방 빈공간의 중점까지 상대 거리  
    #         #space size
    #         space1_size = np.abs((traci.vehicle.getPosition(rlEnv.Target_left_car)[0] - traci.vehicle.getPosition(left_leader)[0]))
    #         space2_size = np.abs((traci.vehicle.getPosition(rlEnv.Target_right_car)[0] - traci.vehicle.getPosition(right_leader)[0]))
    #         space3_size = np.abs((traci.vehicle.getPosition(rlEnv.Target_left_car)[0] - traci.vehicle.getPosition(left_follower)[0]))
    #         space4_size = np.abs((traci.vehicle.getPosition(rlEnv.Target_right_car)[0] - traci.vehicle.getPosition(right_follower)[0]))
    #         # print("space1_size: ",space1_size)
    #         # print("space2_size: ",space2_size)
    #         # print("space3_size: ",space3_size)
    #         # print("space4_size: ",space4_size)
    #         space1 = [space1_size,s1,'L1'] #left front
    #         space2 = [space2_size,s2,'R1'] #right front
    #         space3 = [space3_size,s3,'L2'] #left back
    #         space4 = [space4_size,s4,'R2'] #right back
    #         order =[space1,space2,space3,space4]
    #         order.sort()
    #         dist_to_biggest_empty_space = order[-1][1]
    #         dist_to_second_biggest_empty_space = order[-2][1]
    #         dist_to_third_biggest_empty_space = order[-3][1]
    #         dist_to_forth_biggest_empty_space = order[-4][1]
    #         value =0
    #         total_space_size = space1_size+space2_size+space3_size+space4_size
    #         # print('1st space weight: ',(order[-1][0]/total_space_size))
    #         # print('2nd space weight: ',(order[-2][0]/total_space_size))
    #         # print('3rd space weight: ',(order[-3][0]/total_space_size))
    #         # print('4th space weight: ',(order[-4][0]/total_space_size))
    #         # print('ego_y: ',traci.vehicle.getPosition('ego')[1])
    #         if dist_to_biggest_empty_space<=np.sqrt((order[-1][0]/2)**2+3.2**2): #If ego vehicle is next to the open space then the reward adding start.
    #             # print(order[-1][2])
    #             if order[-1][2][0] == 'L' and traci.vehicle.getPosition('ego')[1]>=-4.8:
    #                 lateral_distance_error = np.abs(-1.6 -traci.vehicle.getPosition('ego')[1])
    #                 space_reward.append(2**(-dist_to_biggest_empty_space) +10*2**(-lateral_distance_error))
    #                 space_reward_long.append(2**(-dist_to_biggest_empty_space))
    #                 space_reward_lat.append(10*2**(-lateral_distance_error))
    #                 # print('1st-')                    
    #                 value+= 2**(-dist_to_biggest_empty_space) +10*2**(-lateral_distance_error)
    #             elif order[-1][2][0] == 'R' and traci.vehicle.getPosition('ego')[1]<=-4.8: # order[-1][2][0] == 'R':
    #                 lateral_distance_error = np.abs(-8 -traci.vehicle.getPosition('ego')[1])
    #                 # print('1st')
    #                 space_reward.append(2**(-dist_to_biggest_empty_space) +10*2**(-lateral_distance_error))
    #                 space_reward_long.append(2**(-dist_to_biggest_empty_space))
    #                 space_reward_lat.append(10*2**(-lateral_distance_error))
    #                 value+= 2**(-dist_to_biggest_empty_space) +10*2**(-lateral_distance_error)                    
    #             else:
    #                 space_reward.append(0)
    #                 space_reward_long.append(0)
    #                 space_reward_lat.append(0)
    #                 value+= 0
    #         else:
    #             space_reward.append(0)
    #             space_reward_long.append(0)
    #             space_reward_lat.append(0)
                      
    #         if dist_to_second_biggest_empty_space<=np.sqrt((order[-2][0]/2)**2+3.2**2): 
    #             # print(order[-2][2])
    #             if order[-2][2][0] == 'L' and traci.vehicle.getPosition('ego')[1]>=-4.8:
    #                 lateral_distance_error = np.abs(-1.6 -traci.vehicle.getPosition('ego')[1])
    #                 # print('2nd-')
    #                 space_reward.append((2**(-dist_to_second_biggest_empty_space)+10*2**(-lateral_distance_error))*0.25)
    #                 space_reward_long.append(2**(-dist_to_second_biggest_empty_space)*0.25)
    #                 space_reward_lat.append(10*2**(-lateral_distance_error)*0.25)
    #                 value+= (2**(-dist_to_second_biggest_empty_space)+10*2**(-lateral_distance_error))*0.25
    #             elif order[-2][2][0] == 'R' and traci.vehicle.getPosition('ego')[1]<=-4.8: # order[-1][2][0] == 'R':
    #                 lateral_distance_error = np.abs(-8 -traci.vehicle.getPosition('ego')[1])
    #                 # print('2nd')
    #                 space_reward.append((2**(-dist_to_second_biggest_empty_space)+10*2**(-lateral_distance_error))*0.25)
    #                 space_reward_long.append(2**(-dist_to_second_biggest_empty_space)*0.25)
    #                 space_reward_lat.append(10*2**(-lateral_distance_error)*0.25)
    #                 value+= (2**(-dist_to_second_biggest_empty_space)+10*2**(-lateral_distance_error))*0.25                  
    #             else:
    #                 space_reward.append(0)
    #                 space_reward_long.append(0)
    #                 space_reward_lat.append(0)
    #                 value+= 0
    #         else:
    #             space_reward.append(0)
    #             space_reward_long.append(0)
    #             space_reward_lat.append(0)
                
    #         if dist_to_third_biggest_empty_space<=np.sqrt((order[-3][0]/2)**2+3.2**2):
    #             # print(order[-3][2])
    #             if order[-3][2][0] == 'L' and traci.vehicle.getPosition('ego')[1]>=-4.8:
    #                 lateral_distance_error = np.abs(-1.6 -traci.vehicle.getPosition('ego')[1])
    #                 # print('3rd-')
    #                 space_reward.append((2**(-dist_to_third_biggest_empty_space)+10*2**(-lateral_distance_error))*0.25)
    #                 space_reward_long.append(2**(-dist_to_third_biggest_empty_space)*0.25)
    #                 space_reward_lat.append(10*2**(-lateral_distance_error)*0.25)
    #                 value+= (2**(-dist_to_third_biggest_empty_space)+10*2**(-lateral_distance_error))*0.25                   
    #             elif order[-3][2][0] == 'R' and traci.vehicle.getPosition('ego')[1]<=-4.8: # order[-1][2][0] == 'R':
    #                 lateral_distance_error = np.abs(-8 -traci.vehicle.getPosition('ego')[1])
    #                 # print('3rd')
    #                 space_reward.append((2**(-dist_to_third_biggest_empty_space)+10*2**(-lateral_distance_error))*0.25)
    #                 space_reward_long.append(round(max([-1,-(dist_to_third_biggest_empty_space-15)**3/75]),3)*0.25)
    #                 space_reward_lat.append(10*2**(-lateral_distance_error)*0.25)
    #                 value+= (2**(-dist_to_third_biggest_empty_space)+10*2**(-lateral_distance_error))*0.25                    
    #             else:
    #                 space_reward.append(0)
    #                 space_reward_long.append(0)
    #                 space_reward_lat.append(0)
    #                 value+= 0
    #         else:
    #             space_reward.append(0)
    #             space_reward_long.append(0)
    #             space_reward_lat.append(0)
                
    #         if dist_to_forth_biggest_empty_space<=np.sqrt((order[-4][0]/2)**2+3.2**2):
    #             # print(order[-4 ][2])
    #             if order[-4][2][0] == 'L' and traci.vehicle.getPosition('ego')[1]>=-4.8:
    #                 lateral_distance_error = np.abs(-1.6 -traci.vehicle.getPosition('ego')[1])
    #                 # print('4th-')
    #                 space_reward.append((2**(-dist_to_forth_biggest_empty_space)+10*2**(-lateral_distance_error))*0.25)
    #                 space_reward_long.append(2**(-dist_to_forth_biggest_empty_space)*0.25)
    #                 space_reward_lat.append(10*2**(-lateral_distance_error)*0.25)
    #                 value+= (2**(-dist_to_forth_biggest_empty_space)+10*2**(-lateral_distance_error))*0.25                    
    #             elif order[-4][2][0] == 'R' and traci.vehicle.getPosition('ego')[1]<=-4.8: # order[-1][2][0] == 'R':
    #                 lateral_distance_error = np.abs(-8 -traci.vehicle.getPosition('ego')[1])
    #                 # print('4th')
    #                 space_reward.append((2**(-dist_to_forth_biggest_empty_space)+10*2**(-lateral_distance_error))*0.25)
    #                 space_reward_long.append(2**(-dist_to_forth_biggest_empty_space)*0.25 )
    #                 space_reward_lat.append(10*2**(-lateral_distance_error)*0.25 )
    #                 value+= (2**(-dist_to_forth_biggest_empty_space)+10*2**(-lateral_distance_error))*0.25                 
    #             else:
    #                 space_reward.append(0)
    #                 space_reward_long.append(0)
    #                 space_reward_lat.append(0)
    #                 value+= 0
    #         else:
    #             space_reward.append(0)
    #             space_reward_long.append(0)
    #             space_reward_lat.append(0)
    #         rlEnv.toward_space_reward.append(space_reward)
    #         rlEnv.toward_space_reward_long.append(space_reward_long)
    #         rlEnv.toward_space_reward_lat.append(space_reward_lat)
    #         return value
    #     else:
    #         space_reward.append(0)
    #         space_reward.append(0)
    #         space_reward.append(0)
    #         space_reward.append(0)
    #         space_reward_long.append(0)
    #         space_reward_lat.append(0)
    #         space_reward_long.append(0)
    #         space_reward_lat.append(0)
    #         space_reward_long.append(0)
    #         space_reward_lat.append(0)
    #         space_reward_long.append(0)
    #         space_reward_lat.append(0)
    #         rlEnv.toward_space_reward.append(space_reward)
    #         rlEnv.toward_space_reward_long.append(space_reward_long)
    #         rlEnv.toward_space_reward_lat.append(space_reward_lat)
    #         return 0
    # def __toward_biggest_empty_space_reward(self,id):
    #     space_reward=[]
    #     space_reward_long=[]
    #     space_reward_lat=[]
    #     space_reward.append(traci.simulation.getTime())
    #     space_reward_long.append(traci.simulation.getTime())
    #     space_reward_lat.append(traci.simulation.getTime())
    #     vehs = traci.vehicle.getIDList()
    #     if id in vehs and rlEnv.Target_left_car in vehs and rlEnv.Target_right_car in vehs:        
    #         if traci.vehicle.getLeader(rlEnv.Target_left_car)[0] == 'ego':
    #             left_leader = traci.vehicle.getLeader('ego')[0]
    #         else:
    #             left_leader = traci.vehicle.getLeader(rlEnv.Target_left_car)[0]
    #         if traci.vehicle.getFollower(rlEnv.Target_left_car)[0] =='ego':
    #             left_follower = traci.vehicle.getFollower('ego')[0]
    #         else:
    #             left_follower = traci.vehicle.getFollower(rlEnv.Target_left_car)[0]
            
    #         if traci.vehicle.getLeader(rlEnv.Target_right_car)[0] == 'ego':
    #             right_leader = traci.vehicle.getLeader('ego')[0]
    #         else:
    #             right_leader = traci.vehicle.getLeader(rlEnv.Target_right_car)[0]
    #         if traci.vehicle.getFollower(rlEnv.Target_right_car)[0] =='ego':
    #             right_follower = traci.vehicle.getFollower('ego')[0]
    #         else:
    #             right_follower = traci.vehicle.getFollower(rlEnv.Target_right_car)[0]
    #         #distance to space
    #         s1 = np.sqrt(((traci.vehicle.getPosition(rlEnv.Target_left_car)[0] + traci.vehicle.getPosition(left_leader)[0])/2 - traci.vehicle.getPosition('ego')[0])**2 +(traci.vehicle.getPosition(rlEnv.Target_left_car)[1]- traci.vehicle.getPosition('ego')[1])**2)#자차와 왼쪽 차로 전방 빈공간의 중점까지 상대 거리
    #         s2 = np.sqrt(((traci.vehicle.getPosition(rlEnv.Target_right_car)[0] + traci.vehicle.getPosition(right_leader)[0])/2 - traci.vehicle.getPosition('ego')[0])**2+(traci.vehicle.getPosition(rlEnv.Target_right_car)[1]- traci.vehicle.getPosition('ego')[1])**2) #자차와 오른쪽 차로 전방 빈공간의 중점까지 상대 거리
    #         s3 = np.sqrt(((traci.vehicle.getPosition(rlEnv.Target_left_car)[0] + traci.vehicle.getPosition(left_follower)[0])/2 - traci.vehicle.getPosition('ego')[0])**2+(traci.vehicle.getPosition(rlEnv.Target_left_car)[1]- traci.vehicle.getPosition('ego')[1])**2) #자차와 왼쪽 차로 후방 빈공간의 중점까지 상대 거리            
    #         s4 = np.sqrt(((traci.vehicle.getPosition(rlEnv.Target_right_car)[0] + traci.vehicle.getPosition(right_follower)[0])/2 - traci.vehicle.getPosition('ego')[0])**2+(traci.vehicle.getPosition(rlEnv.Target_right_car)[1]- traci.vehicle.getPosition('ego')[1])**2)#자차와 오른쪽 차로 후방 빈공간의 중점까지 상대 거리  
    #         #space size
    #         space1_size = np.abs((traci.vehicle.getPosition(rlEnv.Target_left_car)[0] - traci.vehicle.getPosition(left_leader)[0]))
    #         space2_size = np.abs((traci.vehicle.getPosition(rlEnv.Target_right_car)[0] - traci.vehicle.getPosition(right_leader)[0]))
    #         space3_size = np.abs((traci.vehicle.getPosition(rlEnv.Target_left_car)[0] - traci.vehicle.getPosition(left_follower)[0]))
    #         space4_size = np.abs((traci.vehicle.getPosition(rlEnv.Target_right_car)[0] - traci.vehicle.getPosition(right_follower)[0]))
    #         print("space1_size: ",space1_size)
    #         print("space2_size: ",space2_size)
    #         print("space3_size: ",space3_size)
    #         print("space4_size: ",space4_size)
    #         space1 = [space1_size,s1,'L1'] #left front
    #         space2 = [space2_size,s2,'R1'] #right front
    #         space3 = [space3_size,s3,'L2'] #left back
    #         space4 = [space4_size,s4,'R2'] #right back
    #         order =[space1,space2,space3,space4]
    #         order.sort()
    #         dist_to_biggest_empty_space = order[-1][1]
    #         dist_to_second_biggest_empty_space = order[-2][1]
    #         dist_to_third_biggest_empty_space = order[-3][1]
    #         dist_to_forth_biggest_empty_space = order[-4][1]
    #         value =0
    #         total_space_size = space1_size+space2_size+space3_size+space4_size
    #         # print('1st space weight: ',(order[-1][0]/total_space_size))
    #         # print('2nd space weight: ',(order[-2][0]/total_space_size))
    #         # print('3rd space weight: ',(order[-3][0]/total_space_size))
    #         # print('4th space weight: ',(order[-4][0]/total_space_size))
    #         print('ego_y: ',traci.vehicle.getPosition('ego')[1])
    #         if dist_to_biggest_empty_space<=np.sqrt((order[-1][0]/2)**2+3.2**2): #If ego vehicle is next to the open space then the reward adding start.
    #             print(order[-1][2])
    #             if order[-1][2][0] == 'L' and traci.vehicle.getPosition('ego')[1]>=-4.8:
    #                 lateral_distance_error = np.abs(-1.6 -traci.vehicle.getPosition('ego')[1])
    #                 space_reward.append(round(max([-1,-(dist_to_biggest_empty_space-15)**3/75]),3) +10*2**(-lateral_distance_error))
    #                 space_reward_long.append(round(max([-1,-(dist_to_biggest_empty_space-15)**3/75]),3))
    #                 space_reward_lat.append(10*2**(-lateral_distance_error))
    #                 print('1st-')                    
    #                 value+= round(max([-1,-(dist_to_biggest_empty_space-15)**3/75]),3) +10*2**(-lateral_distance_error)
    #             elif order[-1][2][0] == 'R' and traci.vehicle.getPosition('ego')[1]<=-4.8: # order[-1][2][0] == 'R':
    #                 lateral_distance_error = np.abs(-8 -traci.vehicle.getPosition('ego')[1])
    #                 print('1st')
    #                 space_reward.append(round(max([-1,-(dist_to_biggest_empty_space-15)**3/75]),3) +10*2**(-lateral_distance_error))
    #                 space_reward_long.append(round(max([-1,-(dist_to_biggest_empty_space-15)**3/75]),3))
    #                 space_reward_lat.append(10*2**(-lateral_distance_error))
    #                 value+= round(max([-1,-(dist_to_biggest_empty_space-15)**3/75]),3) +10*2**(-lateral_distance_error)                    
    #             else:
    #                 space_reward.append(0)
    #                 space_reward_long.append(0)
    #                 space_reward_lat.append(0)
    #                 value+= 0
    #         else:
    #             space_reward.append(0)
    #             space_reward_long.append(0)
    #             space_reward_lat.append(0)
                      
    #         if dist_to_second_biggest_empty_space<=np.sqrt((order[-2][0]/2)**2+3.2**2): 
    #             print(order[-2][2])
    #             if order[-2][2][0] == 'L' and traci.vehicle.getPosition('ego')[1]>=-4.8:
    #                 lateral_distance_error = np.abs(-1.6 -traci.vehicle.getPosition('ego')[1])
    #                 print('2nd-')
    #                 space_reward.append((round(max([-1,-(dist_to_second_biggest_empty_space-15)**3/75]),3)+10*2**(-lateral_distance_error))*0.25)
    #                 space_reward_long.append(round(max([-1,-(dist_to_second_biggest_empty_space-15)**3/75]),3)*0.25)
    #                 space_reward_lat.append(10*2**(-lateral_distance_error)*0.25)
    #                 value+= (round(max([-1,-(dist_to_second_biggest_empty_space-15)**3/75]),3)+10*2**(-lateral_distance_error))*0.25
    #             elif order[-2][2][0] == 'R' and traci.vehicle.getPosition('ego')[1]<=-4.8: # order[-1][2][0] == 'R':
    #                 lateral_distance_error = np.abs(-8 -traci.vehicle.getPosition('ego')[1])
    #                 print('2nd')
    #                 space_reward.append((round(max([-1,-(dist_to_second_biggest_empty_space-15)**3/75]),3)+10*2**(-lateral_distance_error))*0.25)
    #                 space_reward_long.append(round(max([-1,-(dist_to_second_biggest_empty_space-15)**3/75]),3)*0.25)
    #                 space_reward_lat.append(10*2**(-lateral_distance_error)*0.25)
    #                 value+= (round(max([-1,-(dist_to_second_biggest_empty_space-15)**3/75]),3)+10*2**(-lateral_distance_error))*0.25                  
    #             else:
    #                 space_reward.append(0)
    #                 space_reward_long.append(0)
    #                 space_reward_lat.append(0)
    #                 value+= 0
    #         else:
    #             space_reward.append(0)
    #             space_reward_long.append(0)
    #             space_reward_lat.append(0)
                
    #         if dist_to_third_biggest_empty_space<=np.sqrt((order[-3][0]/2)**2+3.2**2):
    #             print(order[-3][2])
    #             if order[-3][2][0] == 'L' and traci.vehicle.getPosition('ego')[1]>=-4.8:
    #                 lateral_distance_error = np.abs(-1.6 -traci.vehicle.getPosition('ego')[1])
    #                 print('3rd-')
    #                 space_reward.append((round(max([-1,-(dist_to_third_biggest_empty_space-15)**3/75]),3)+10*2**(-lateral_distance_error))*0.25)
    #                 space_reward_long.append(round(max([-1,-(dist_to_third_biggest_empty_space-15)**3/75]),3)*0.25)
    #                 space_reward_lat.append(10*2**(-lateral_distance_error)*0.25)
    #                 value+= (round(max([-1,-(dist_to_third_biggest_empty_space-15)**3/75]),3)+10*2**(-lateral_distance_error))*0.25                   
    #             elif order[-3][2][0] == 'R' and traci.vehicle.getPosition('ego')[1]<=-4.8: # order[-1][2][0] == 'R':
    #                 lateral_distance_error = np.abs(-8 -traci.vehicle.getPosition('ego')[1])
    #                 print('3rd')
    #                 space_reward.append((round(max([-1,-(dist_to_third_biggest_empty_space-15)**3/75]),3)+10*2**(-lateral_distance_error))*0.25)
    #                 space_reward_long.append(round(max([-1,-(dist_to_third_biggest_empty_space-15)**3/75]),3)*0.25)
    #                 space_reward_lat.append(10*2**(-lateral_distance_error)*0.25)
    #                 value+= (round(max([-1,-(dist_to_third_biggest_empty_space-15)**3/75]),3)+10*2**(-lateral_distance_error))*0.25                    
    #             else:
    #                 space_reward.append(0)
    #                 space_reward_long.append(0)
    #                 space_reward_lat.append(0)
    #                 value+= 0
    #         else:
    #             space_reward.append(0)
    #             space_reward_long.append(0)
    #             space_reward_lat.append(0)
                
    #         if dist_to_forth_biggest_empty_space<=np.sqrt((order[-4][0]/2)**2+3.2**2):
    #             print(order[-4 ][2])
    #             if order[-4][2][0] == 'L' and traci.vehicle.getPosition('ego')[1]>=-4.8:
    #                 lateral_distance_error = np.abs(-1.6 -traci.vehicle.getPosition('ego')[1])
    #                 print('4th-')
    #                 space_reward.append((round(max([-1,-(dist_to_forth_biggest_empty_space-15)**3/75]),3)+10*2**(-lateral_distance_error))*0.25)
    #                 space_reward_long.append(round(max([-1,-(dist_to_forth_biggest_empty_space-15)**3/75]),3)*0.25)
    #                 space_reward_lat.append(10*2**(-lateral_distance_error)*0.25)
    #                 value+= (round(max([-1,-(dist_to_forth_biggest_empty_space-15)**3/75]),3)+10*2**(-lateral_distance_error))*0.25                    
    #             elif order[-4][2][0] == 'R' and traci.vehicle.getPosition('ego')[1]<=-4.8: # order[-1][2][0] == 'R':
    #                 lateral_distance_error = np.abs(-8 -traci.vehicle.getPosition('ego')[1])
    #                 print('4th')
    #                 space_reward.append((round(max([-1,-(dist_to_forth_biggest_empty_space-15)**3/75]),3)+10*2**(-lateral_distance_error))*0.25)
    #                 space_reward_long.append(round(max([-1,-(dist_to_forth_biggest_empty_space-15)**3/75]),3)*0.25 )
    #                 space_reward_lat.append(10*2**(-lateral_distance_error)*0.25 )
    #                 value+= (round(max([-1,-(dist_to_forth_biggest_empty_space-15)**3/75]),3)+10*2**(-lateral_distance_error))*0.25                 
    #             else:
    #                 space_reward.append(0)
    #                 space_reward_long.append(0)
    #                 space_reward_lat.append(0)
    #                 value+= 0
    #         else:
    #             space_reward.append(0)
    #             space_reward_long.append(0)
    #             space_reward_lat.append(0)
    #         rlEnv.toward_space_reward.append(space_reward)
    #         rlEnv.toward_space_reward_long.append(space_reward_long)
    #         rlEnv.toward_space_reward_lat.append(space_reward_lat)
    #         return value
    #     else:
    #         space_reward.append(0)
    #         space_reward.append(0)
    #         space_reward.append(0)
    #         space_reward.append(0)
    #         space_reward_long.append(0)
    #         space_reward_lat.append(0)
    #         space_reward_long.append(0)
    #         space_reward_lat.append(0)
    #         space_reward_long.append(0)
    #         space_reward_lat.append(0)
    #         space_reward_long.append(0)
    #         space_reward_lat.append(0)
    #         rlEnv.toward_space_reward.append(space_reward)
    #         rlEnv.toward_space_reward_long.append(space_reward_long)
    #         rlEnv.toward_space_reward_lat.append(space_reward_lat)
    #         return 0
        

    # def __toward_biggest_empty_space_reward(self,id):
    #     left_vehicles=[] #[|ego_x_pos - x_pos|, id]
    #     right_vehicles=[] #[|ego_x_pos - x_pos|, id]
    #     left_near_vehicles=[] #[x_pos]
    #     right_near_vehicles =[] #[x_pos]
        
    #     left_near_spaces=[] #[space_x_pos, space_size]
    #     right_near_spaces=[] #[space_x_pos, space_size]
        
    #     chosen_space_size=0  ## need to check if it is right intialization
    #     chosen_space_number = 0 ## need to check if it is right intialization
        
    #     if id:
    #         traci.vehicle.subscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 120.0, [tc.VAR_POSITION])
    #         # print(id,' subscribeContext')
    #         for v_id in traci.vehicle.getContextSubscriptionResults(str(id)):
    #             if(traci.vehicle.getLaneIndex(v_id)==2) and not (v_id=='ego'):
    #                 #left_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),v_id]) -> id check
    #                 left_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),traci.vehicle.getPosition(v_id)[0]])
    #             if(traci.vehicle.getLaneIndex(v_id)==0) and not (v_id=='ego'):
    #                 right_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),traci.vehicle.getPosition(v_id)[0]])
            
            
    #         left_vehicles.sort()
    #         right_vehicles.sort()
    #         traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 120.0)
    #     for i in range(3): # getting near 3 vehicles x_pos each left and right
    #         left_near_vehicles.append(left_vehicles[:3][i][1])
    #         right_near_vehicles.append(right_vehicles[:3][i][1])
                
    #     left_near_vehicles.sort()
    #     right_near_vehicles.sort()
    #     # print('left_near_vehicles: ',left_near_vehicles)
    #     left_near_spaces.append([left_near_vehicles[1],left_near_vehicles[0],left_near_vehicles[1]-left_near_vehicles[0]])
    #     left_near_spaces.append([left_near_vehicles[2],left_near_vehicles[1],left_near_vehicles[2]-left_near_vehicles[1]])
    #     right_near_spaces.append([right_near_vehicles[1],right_near_vehicles[0], right_near_vehicles[1]-right_near_vehicles[0]])
    #     right_near_spaces.append([right_near_vehicles[2],right_near_vehicles[1], right_near_vehicles[2]-right_near_vehicles[1]])
    #     # print('left_near_spaces: ',left_near_spaces)
    #     empty_space_dictionary = {"L0":left_near_spaces[0],"R0":right_near_spaces[0],"L1":left_near_spaces[1],"R1":right_near_spaces[1]}

    #     empty_space_dictionary_sorted_by_space = sorted(empty_space_dictionary.items(), key=lambda x: x[1][2],reverse=True)
    #     # print("empty_space_dictionary_sorted_by_space: ",empty_space_dictionary_sorted_by_space)
    #     dist_to_biggest_empty_space = np.abs(traci.vehicle.getPosition('ego')[0]-(empty_space_dictionary_sorted_by_space[0][1][1]+(empty_space_dictionary_sorted_by_space[0][1][0]-empty_space_dictionary_sorted_by_space[0][1][1])/2))
    #     dist_to_second_biggest_empty_space = np.abs(traci.vehicle.getPosition('ego')[0]-(empty_space_dictionary_sorted_by_space[1][1][1]+(empty_space_dictionary_sorted_by_space[1][1][0]-empty_space_dictionary_sorted_by_space[1][1][1])/2))
    #     dist_to_third_biggest_empty_space = np.abs(traci.vehicle.getPosition('ego')[0]-(empty_space_dictionary_sorted_by_space[2][1][1]+(empty_space_dictionary_sorted_by_space[2][1][0]-empty_space_dictionary_sorted_by_space[2][1][1])/2))
    #     dist_to_forth_biggest_empty_space = np.abs(traci.vehicle.getPosition('ego')[0]-(empty_space_dictionary_sorted_by_space[3][1][1]+(empty_space_dictionary_sorted_by_space[3][1][0]-empty_space_dictionary_sorted_by_space[3][1][1])/2))
    #     # print('num1: ', dist_to_biggest_empty_space)
    #     # print('num2: ', dist_to_second_biggest_empty_space)
    #     # print('num3: ', dist_to_third_biggest_empty_space)
    #     # print('num4: ', dist_to_forth_biggest_empty_space)
    #     if empty_space_dictionary_sorted_by_space[0][0][0] == 'L':
    #         lateral_distance_error = np.abs(-1.6 -traci.vehicle.getPosition('ego')[1]) #1차로 중앙으로 부터 자차까지 거리
    #     elif empty_space_dictionary_sorted_by_space[0][0][0] == 'R':
    #         lateral_distance_error = np.abs(-8 -traci.vehicle.getPosition('ego')[1])   #3차로 중앙으로 부터 자차까지 거리
        
    #     if empty_space_dictionary_sorted_by_space[1][0][0] == 'L':
    #         lateral_distance_error2 = np.abs(-1.6 -traci.vehicle.getPosition('ego')[1]) #1차로 중앙으로 부터 자차까지 거리
    #     elif empty_space_dictionary_sorted_by_space[1][0][0] == 'R':
    #         lateral_distance_error2 = np.abs(-8 -traci.vehicle.getPosition('ego')[1])   #3차로 중앙으로 부터 자차까지 거리
        
    #     if empty_space_dictionary_sorted_by_space[2][0][0] == 'L':
    #         lateral_distance_error3 = np.abs(-1.6 -traci.vehicle.getPosition('ego')[1]) #1차로 중앙으로 부터 자차까지 거리
    #     elif empty_space_dictionary_sorted_by_space[2][0][0] == 'R':
    #         lateral_distance_error3 = np.abs(-8 -traci.vehicle.getPosition('ego')[1])   #3차로 중앙으로 부터 자차까지 거리
        
    #     if empty_space_dictionary_sorted_by_space[3][0][0] == 'L':
    #         lateral_distance_error4 = np.abs(-1.6 -traci.vehicle.getPosition('ego')[1]) #1차로 중앙으로 부터 자차까지 거리
    #     elif empty_space_dictionary_sorted_by_space[3][0][0] == 'R':
    #         lateral_distance_error4 = np.abs(-8 -traci.vehicle.getPosition('ego')[1])   #3차로 중앙으로 부터 자차까지 거리

    #     # if(round(np.exp(-dist_to_biggest_empty_space),2)*100 <0):
    #     #     print('.........................Negative.............................')
    #     # return round(np.exp(-dist_to_biggest_empty_space),2)

    #     # 1차식
    #     # print("toward_biggest_space reward: ",round(-dist_to_biggest_empty_space/50 + 10,2))
    #     # return round(-dist_to_biggest_empty_space/50 + 10,2)

    #     # 2차식
    #     # print('dist: ',dist_to_biggest_empty_space)
    #     # print("toward_biggest_space reward: ",round(max([-1,-(dist_to_biggest_empty_space-15)**3/75]),2))
            
        
    #     if dist_to_biggest_empty_space<=8:
    #         lateral_distance_to_side_lane = lateral_distance_error
    #         # print("lateral_distance_to_side_lane: ",10*2**(-lateral_distance_to_side_lane))
    #         # print("dist_to_biggest_empty_space: ",round(max([-1,-(dist_to_biggest_empty_space-15)**3/75]),3))
    #         return round(max([-1,-(dist_to_biggest_empty_space-15)**3/75]),3) +10*2**(-lateral_distance_to_side_lane)
            
    #     elif dist_to_second_biggest_empty_space<=8: 
    #         lateral_distance_to_side_lane = lateral_distance_error2
    #         return (round(max([-1,-(dist_to_second_biggest_empty_space-15)**3/75]),3)+10*2**(-lateral_distance_to_side_lane))*0.75
    #     elif dist_to_third_biggest_empty_space<=8:
    #         lateral_distance_to_side_lane = lateral_distance_error3
    #         return (round(max([-1,-(dist_to_third_biggest_empty_space-15)**3/75]),3)+10*2**(-lateral_distance_to_side_lane))*0.5
    #     elif dist_to_forth_biggest_empty_space<=8:
    #         lateral_distance_to_side_lane = lateral_distance_error4
    #         return (round(max([-1,-(dist_to_forth_biggest_empty_space-15)**3/75]),3)+10*2**(-lateral_distance_to_side_lane))*0.25
    #     else:
    #         return 0
    def __chosen_space_size_reward(self): #시뮬레이션 시작 처음 근접 좌우 차량 기준 앞뒤 공간으로 차로 변경 시 보상.
        vehs = traci.vehicle.getIDList()
        if rlEnv.Target_left_car in vehs and rlEnv.Target_right_car in vehs and 'ego' in vehs:
            if traci.vehicle.getLeader(rlEnv.Target_left_car)[0] == 'ego' and traci.vehicle.getLaneIndex('ego')==2:
                left_leader = traci.vehicle.getLeader('ego')[0]
            elif traci.vehicle.getLeader(rlEnv.Target_left_car)[0] == 'ego' and traci.vehicle.getLaneIndex('ego')==1:
                left_leader = 'car.left'+str(int(rlEnv.Target_left_car[-1])+1)
            else:
                left_leader = traci.vehicle.getLeader(rlEnv.Target_left_car)[0]
            if traci.vehicle.getFollower(rlEnv.Target_left_car)[0] =='ego' and traci.vehicle.getLaneIndex('ego')==2:
                left_follower = traci.vehicle.getFollower('ego')[0]
            elif traci.vehicle.getFollower(rlEnv.Target_left_car)[0] =='ego' and traci.vehicle.getLaneIndex('ego')==1:
                left_follower = 'car.left'+str(int(rlEnv.Target_left_car[-1])-1)
            else:
                left_follower = traci.vehicle.getFollower(rlEnv.Target_left_car)[0]
            
            if traci.vehicle.getLeader(rlEnv.Target_right_car)[0] == 'ego' and traci.vehicle.getLaneIndex('ego')==0:
                right_leader = traci.vehicle.getLeader('ego')[0]
            elif traci.vehicle.getLeader(rlEnv.Target_right_car)[0] == 'ego' and traci.vehicle.getLaneIndex('ego')==1:
                right_leader = 'car.right'+str(int(rlEnv.Target_right_car[-1])+1)
            else:
                right_leader = traci.vehicle.getLeader(rlEnv.Target_right_car)[0]
            if traci.vehicle.getFollower(rlEnv.Target_right_car)[0] =='ego' and traci.vehicle.getLaneIndex('ego')==0:
                right_follower = traci.vehicle.getFollower('ego')[0]
            elif traci.vehicle.getFollower(rlEnv.Target_right_car)[0] =='ego' and traci.vehicle.getLaneIndex('ego')==1:
                right_follower = 'car.right'+str(int(rlEnv.Target_right_car[-1])-1)
            else:
                right_follower = traci.vehicle.getFollower(rlEnv.Target_right_car)[0]
            
            # print('left_leader: ',left_leader)
            # print('right_leader: ',right_leader)
            # print('left_follower',left_follower)
            # print('right_follower',right_follower)
            # ego vehicle의 위치
            ego_chosen_space = None
            if traci.vehicle.getLaneIndex('ego') == 2:
                if traci.vehicle.getPosition('ego')[0] > traci.vehicle.getPosition(rlEnv.Target_left_car)[0]:
                    ego_chosen_space = 'L1'
                else:
                    ego_chosen_space = 'L2'
            if traci.vehicle.getLaneIndex('ego') == 0:
                if traci.vehicle.getPosition('ego')[0] > traci.vehicle.getPosition(rlEnv.Target_right_car)[0]:
                    ego_chosen_space = 'R1'
                else:
                    ego_chosen_space = 'R2'
            
            space1_size = np.abs((traci.vehicle.getPosition(rlEnv.Target_left_car)[0] - traci.vehicle.getPosition(left_leader)[0]))
            space2_size = np.abs((traci.vehicle.getPosition(rlEnv.Target_right_car)[0] - traci.vehicle.getPosition(right_leader)[0]))
            space3_size = np.abs((traci.vehicle.getPosition(rlEnv.Target_left_car)[0] - traci.vehicle.getPosition(left_follower)[0]))
            space4_size = np.abs((traci.vehicle.getPosition(rlEnv.Target_right_car)[0] - traci.vehicle.getPosition(right_follower)[0]))
            
            space1 = [space1_size,'L1'] #left front
            space2 = [space2_size,'R1'] #right front
            space3 = [space3_size,'L2'] #left back
            space4 = [space4_size,'R2'] #right back
            order =[space1,space2,space3,space4]
            order.sort()

            print('order: ',order)
            if ego_chosen_space == order[-1][1]: #LC가 끝났을 때 위치가 가장 큰 크기의 공간과 일치,LC 성공 했을 경우 
                print('ego_chosen_space: ',ego_chosen_space)
                print('1st biggest space')
                return 400 ,'1',ego_chosen_space
            elif ego_chosen_space == order[-2][1]:#LC가 끝났을 때 위치가 두번째 큰 크기의 공간과 일치,LC 성공 했을 경우 
                print('ego_chosen_space: ',ego_chosen_space)
                print('2nd biggest space')
                return 300, '2',ego_chosen_space
            elif ego_chosen_space == order[-3][1]:#LC가 끝났을 때 위치가 세번째 큰 크기의 공간과 일치,LC 성공 했을 경우 
                print('3rd biggest space')
                return 200, '3',ego_chosen_space
            else: #LC가 끝났을 때 위치가 네번째 큰 크기의 공간과 일치,LC 성공 했을 경우
                print('ego_chosen_space: ',ego_chosen_space) 
                print('4th biggest space')
                return 100, '4',ego_chosen_space
        else:
            print('else space')
            return 50 ,None,None#그 외의 공간으로 LC 성공 했을 경우 
        
    # def __chosen_space_size_reward(self): #시뮬레이션 시작 처음 근접 좌우 차량 기준 앞뒤 공간으로 차로 변경 시 보상.
    #     vehs = traci.vehicle.getIDList()
    #     if rlEnv.Target_left_car in vehs and rlEnv.Target_right_car in vehs and 'ego' in vehs:
    #         if traci.vehicle.getLeader(rlEnv.Target_left_car)[0] == 'ego':
    #             left_leader = traci.vehicle.getLeader('ego')[0]
    #         else:
    #             left_leader = traci.vehicle.getLeader(rlEnv.Target_left_car)[0]
    #         if traci.vehicle.getFollower(rlEnv.Target_left_car)[0] =='ego':
    #             left_follower = traci.vehicle.getFollower('ego')[0]
    #         else:
    #             left_follower = traci.vehicle.getFollower(rlEnv.Target_left_car)[0]
            
    #         if traci.vehicle.getLeader(rlEnv.Target_right_car)[0] == 'ego':
    #             right_leader = traci.vehicle.getLeader('ego')[0]
    #         else:
    #             right_leader = traci.vehicle.getLeader(rlEnv.Target_right_car)[0]
    #         if traci.vehicle.getFollower(rlEnv.Target_right_car)[0] =='ego':
    #             right_follower = traci.vehicle.getFollower('ego')[0]
    #         else:
    #             right_follower = traci.vehicle.getFollower(rlEnv.Target_right_car)[0]
            
    #         # print('left_leader: ',left_leader)
    #         # print('right_leader: ',right_leader)
    #         # print('left_follower',left_follower)
    #         # print('right_follower',right_follower)
            
    #         s1 = np.sqrt(((traci.vehicle.getPosition(rlEnv.Target_left_car)[0] + traci.vehicle.getPosition(left_leader)[0])/2 - traci.vehicle.getPosition('ego')[0])**2 +(traci.vehicle.getPosition(rlEnv.Target_left_car)[1]- traci.vehicle.getPosition('ego')[1])**2)#자차와 왼쪽 차로 전방 빈공간의 중점 까지 상대 거리
    #         s2 = np.sqrt(((traci.vehicle.getPosition(rlEnv.Target_right_car)[0] + traci.vehicle.getPosition(right_leader)[0])/2 - traci.vehicle.getPosition('ego')[0])**2+(traci.vehicle.getPosition(rlEnv.Target_right_car)[1]- traci.vehicle.getPosition('ego')[1])**2) #자차와 오른쪽 차로 전방 빈공간의 중점 까지 상대 거리
    #         s3 = np.sqrt(((traci.vehicle.getPosition(rlEnv.Target_left_car)[0] + traci.vehicle.getPosition(left_follower)[0])/2 - traci.vehicle.getPosition('ego')[0])**2+(traci.vehicle.getPosition(rlEnv.Target_left_car)[1]- traci.vehicle.getPosition('ego')[1])**2) #자차와 왼쪽 차로 후방 빈공간의 중점 까지 상대 거리            
    #         s4 = np.sqrt(((traci.vehicle.getPosition(rlEnv.Target_right_car)[0] + traci.vehicle.getPosition(right_follower)[0])/2 - traci.vehicle.getPosition('ego')[0])**2+(traci.vehicle.getPosition(rlEnv.Target_right_car)[1]- traci.vehicle.getPosition('ego')[1])**2)#자차와 오른쪽 차로 후방 빈공간의 중점 까지 상대 거리  
    #         space1_size = np.abs((traci.vehicle.getPosition(rlEnv.Target_left_car)[0] - traci.vehicle.getPosition(left_leader)[0])/2)
    #         space2_size = np.abs((traci.vehicle.getPosition(rlEnv.Target_right_car)[0] - traci.vehicle.getPosition(right_leader)[0])/2)
    #         space3_size = np.abs((traci.vehicle.getPosition(rlEnv.Target_left_car)[0] - traci.vehicle.getPosition(left_follower)[0])/2)
    #         space4_size = np.abs((traci.vehicle.getPosition(rlEnv.Target_right_car)[0] - traci.vehicle.getPosition(right_follower)[0])/2)
            
    #         space1 = [space1_size,'L1'] #left front
    #         space2 = [space2_size,'R1'] #right front
    #         space3 = [space3_size,'L2'] #left back
    #         space4 = [space4_size,'R2'] #right back
    #         sspace1 = [s1,'L1']
    #         sspace2 = [s2,'R1']
    #         sspace3 = [s3,'L2']
    #         sspace4 = [s4,'R2']
    #         order =[space1,space2,space3,space4]
    #         order.sort()
    #         order2 =[sspace1,sspace2,sspace3,sspace4]
    #         order2.sort()
    #         print('order: ',order)
    #         print('order2: ',order2)
    #         if order2[0][1] == order[-1][1]: #가장 가까운 위치의 공간과 가장 큰 크기의 공간과 일치,LC 성공 했을 경우 
    #             print(order2[-1][1],order[-1][1])
    #             print('1st biggest space')
    #             return 400 ,'1'
    #         elif order2[0][1] == order[-2][1]:#가장 가까운 위치의 공간과 두번째 큰 크기의 공간과 일치,LC 성공 했을 경우 
    #             print('2nd biggest space')
    #             return 300, '2'
    #         elif order2[0][1] == order[-3][1]:#가장 가까운 위치의 공간과 세번째 큰 크기의 공간과 일치,LC 성공 했을 경우 
    #             print('3rd biggest space')
    #             return 200, '3'
    #         else: #가장 가까운 위치의 공간과 네번째 큰 크기의 공간과 일치,LC 성공 했을 경우 
    #             print('4th biggest space')
    #             return 100, '4'
    #     else:
    #         print('else space')
    #         return 50 #그 외의 공간으로 LC 성공 했을 경우 
        # if traci.vehicle.getLaneIndex('ego') == 2:
        #     if order[-1][2] == 'L1':
        #         return 300
        #     elif order[-1][2] == 'L2'
        # elif traci.vehicle.getLaneIndex('ego') == 0:
        #     return
        # else:
        #     return 0
        

        


    # def __chosen_space_size_reward(self,id):
    #     left_vehicles=[] #[|ego_x_pos - x_pos|, id]
    #     right_vehicles=[] #[|ego_x_pos - x_pos|, id]
    #     left_near_vehicles=[] #[x_pos]
    #     right_near_vehicles =[] #[x_pos]
        
    #     left_near_spaces=[] #[space_x_pos, space_size]
    #     right_near_spaces=[] #[space_x_pos, space_size]
        
    #     chosen_space_size=0  ## need to check if it is right intialization
    #     chosen_space_number = 0 ## need to check if it is right intialization
    #     if id:
    #         traci.vehicle.subscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 120.0, [tc.VAR_POSITION])
    #         # print(id,' subscribeContext')
    #         for v_id in traci.vehicle.getContextSubscriptionResults(str(id)):
    #             if(traci.vehicle.getLaneIndex(v_id)==2) and not (v_id=='ego'):
    #                 #left_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),v_id]) -> id check
    #                 left_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),traci.vehicle.getPosition(v_id)[0]])
    #             if(traci.vehicle.getLaneIndex(v_id)==0) and not (v_id=='ego'):
    #                 right_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),traci.vehicle.getPosition(v_id)[0]])
            
            
    #         left_vehicles.sort()
    #         right_vehicles.sort()
    #         traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 120.0)
    #     for i in range(2): # getting near 3 vehicles x_pos each left and right
    #         left_near_vehicles.append(left_vehicles[:3][i][1])
    #         right_near_vehicles.append(right_vehicles[:3][i][1])
                
    #     left_near_vehicles.sort()
    #     right_near_vehicles.sort()
        
    #     left_near_spaces.append([left_near_vehicles[1],left_near_vehicles[0],left_near_vehicles[1]-left_near_vehicles[0]])
    #     right_near_spaces.append([right_near_vehicles[1],right_near_vehicles[0], right_near_vehicles[1]-right_near_vehicles[0]])
        
    #     left_near_spaces.sort()
    #     # print(left_near_spaces)
    #     right_near_spaces.sort()
    #     # print(right_near_spaces)
    #     empty_space_dictionary = {"L0":left_near_spaces[0],"R0":right_near_spaces[0]}

    #     empty_space_dictionary_sorted_by_space = sorted(empty_space_dictionary.items(), key=lambda x: x[1][2],reverse=True)
    #     print('empty_space_dictionary_sorted_by_space: ',empty_space_dictionary_sorted_by_space)

    #     for i in range(2):
    #         if empty_space_dictionary_sorted_by_space[i][1][0]> traci.vehicle.getPosition('ego')[0] and empty_space_dictionary_sorted_by_space[i][1][1] <traci.vehicle.getPosition('ego')[0] :

    #             if (traci.vehicle.getAngle('ego')>90 and empty_space_dictionary_sorted_by_space[i][0][0] == "R") or (traci.vehicle.getAngle('ego')<90 and empty_space_dictionary_sorted_by_space[i][0][0] == "L"):
    #                 chosen_space_size = empty_space_dictionary_sorted_by_space[i][1][2]
    #                 chosen_space_number = i

    #     ## 0 : biggest space is chosen, 1: second big space is chosen, 2 : third big space is chosen, 3: fourth(last) big space is chosen
    #     distance_to_center = np.abs(traci.vehicle.getPosition('ego')[0]-(empty_space_dictionary_sorted_by_space[chosen_space_number][1][1]+(empty_space_dictionary_sorted_by_space[chosen_space_number][1][0]-empty_space_dictionary_sorted_by_space[chosen_space_number][1][1])/2))
    #     # print('distance_to_center: ',distance_to_center)
    #     if chosen_space_number == 0:            
    #         print('biggest space reward ')
    #         return 300 -distance_to_center
    #     elif chosen_space_number == 1:            
    #         print('2nd space reward ')
    #         return 150-distance_to_center

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
                return x_forward-des_clearance
    def __Front_SD_check(self):
        id = 'ego'
        time_gap_LK = 1.36
        c_LK = 2 #min clearance for lane keeping
        ego_v_x = traci.vehicle.getSpeed(id)
        ego_x_pos = traci.vehicle.getPosition(id)[0]
        ego_a_x = traci.vehicle.getAcceleration(id)

        if traci.vehicle.getLeader(id) is not None:
            Leader_id,x_forward = traci.vehicle.getLeader(id)
            rel_v_with_front = traci.vehicle.getSpeed(Leader_id) - ego_v_x
            v_p = traci.vehicle.getSpeed(Leader_id)
            des_clearance = c_LK + time_gap_LK*v_p #앞차 속도에 따른 앞차와의 ACC 안전거리 (safe)
            
            if (x_forward > des_clearance) or ((x_forward <= des_clearance) and (ego_v_x<=traci.vehicle.getSpeed(Leader_id)) and (ego_a_x <= 0)):
                rlEnv.Front_SD = True
            else:
                # print('Forward_SD_negative_reward: ',des_clearance -x_forward)
                rlEnv.Front_SD = False
        
                
    # def __LaneChange_SD_negative_reward(self): # SD_LC 침범 안하면 reward 0, SD_LC 침범한 거리 만큼 negative reward , 앞뒤 차량 SD_LC 침범시 reward -50
    #     id = 'ego'
    #     time_gap_LC_1 =1
    #     time_gap_LC_2 =0.5 
    #     c_LC =12 # min clearance for lane change
    #     ego_v_x = traci.vehicle.getSpeed(id)
    #     ego_x_pos = traci.vehicle.getPosition(id)[0]

    #     if traci.vehicle.getLeader(id) is not None:
    #         Leader_id,x_forward = traci.vehicle.getLeader(id)
    #     if type(traci.vehicle.getFollower(id)) is not None and traci.vehicle.getFollower(id)[0] != '':
    #         follower_id, x_back = traci.vehicle.getFollower(id) 
        


    #     if traci.vehicle.getLeader(id) is not None and type(traci.vehicle.getFollower(id)) is not None and traci.vehicle.getFollower(id)[0] != '':
    #         front_relative_velocity_term = max( [traci.vehicle.getSpeed(id) - traci.vehicle.getSpeed(Leader_id) ,0] )*time_gap_LC_1
    #         front_minimum_clearance_trem = max([traci.vehicle.getSpeed(id)*time_gap_LC_2,c_LC])
    #         front_SD_LC = front_relative_velocity_term +front_minimum_clearance_trem
    #         back_relative_velocity_term = max( [traci.vehicle.getSpeed(follower_id)-traci.vehicle.getSpeed(id),0] )*time_gap_LC_1
    #         back__minimum_clearance_trem = max([traci.vehicle.getSpeed(follower_id)*time_gap_LC_2,c_LC])
    #         back_SD_LC = back_relative_velocity_term + back__minimum_clearance_trem
    #         # print('back_SD_LC: ',back_SD_LC)
    #         # print('front_SD_LC: ',front_SD_LC)
    #         if((traci.vehicle.getPosition(Leader_id)[0]-ego_x_pos) >= front_SD_LC  and (ego_x_pos -traci.vehicle.getPosition(follower_id)[0])>= back_SD_LC):
    #             # print('LaneChange_SD_negative_reward -> None ')
    #             return 0
    #         elif((traci.vehicle.getPosition(Leader_id)[0]-ego_x_pos) < front_SD_LC  and (ego_x_pos -traci.vehicle.getPosition(follower_id)[0])>= back_SD_LC):
    #             # print('LaneChange_SD_negative_reward-> forward ')
    #             return int((traci.vehicle.getPosition(Leader_id)[0]-ego_x_pos) - front_SD_LC )
    #         elif((traci.vehicle.getPosition(Leader_id)[0]-ego_x_pos) >= front_SD_LC  and (ego_x_pos -traci.vehicle.getPosition(follower_id)[0]) < back_SD_LC):
    #             # print('LaneChange_SD_negative_reward-> rear ')
    #             return int((ego_x_pos -traci.vehicle.getPosition(follower_id)[0]) - back_SD_LC)
    #         else:
    #             # print('LaneChange_SD_negative_reward-> forward and rear')
    #             return-50  
    def SD_LC_with_respect_to_target_space(self,id,action,direction): #action 1~4 = target space 1~4, action 에 따라 SD_LC를 return /// direction은 해당 action의 SD 중 앞에 것, 뒤에 것중 하나를 지정.
        # if action == 1:
        #     if traci.vehicle.getLeader(rlEnv.Target_left_car)[0] == 'ego':
        #         left_leader = traci.vehicle.getLeader('ego')[0]
        #     else:
        #         left_leader = traci.vehicle.getLeader(rlEnv.Target_left_car)[0]
        #     left_follower = rlEnv.Target_left_car

        # elif action == 2:
        #     if traci.vehicle.getLeader(rlEnv.Target_right_car)[0] == 'ego':
        #         right_leader = traci.vehicle.getLeader('ego')[0]
        #     else:
        #         right_leader = traci.vehicle.getLeader(rlEnv.Target_right_car)[0]
        #     right_follower = rlEnv.Target_right_car

        # elif action == 3:
        #     left_leader = rlEnv.Target_left_car
        #     if traci.vehicle.getFollower(rlEnv.Target_left_car)[0] =='ego':
        #         left_follower = traci.vehicle.getFollower('ego')[0]
        #     else:
        #         left_follower = traci.vehicle.getFollower(rlEnv.Target_left_car)[0] 

        # elif action == 4:
        #     right_follower = rlEnv.Target_right_car
        #     if traci.vehicle.getFollower(rlEnv.Target_right_car)[0] =='ego':
        #         right_follower = traci.vehicle.getFollower('ego')[0]
        #     else:
        #         right_follower = traci.vehicle.getFollower(rlEnv.Target_right_car)[0]

        # else:
        #     print('!!!!!!!!!!!!!!!!!!!!!!!!!!! check def SD_LC_with_respect_to_target_space')
        vehs = traci.vehicle.getIDList()
        if rlEnv.Target_left_car in vehs and rlEnv.Target_right_car in vehs and 'ego' in vehs:

            if traci.vehicle.getLeader(rlEnv.Target_left_car)[0] == 'ego' and traci.vehicle.getLaneIndex('ego')==2:
                left_leader = traci.vehicle.getLeader('ego')[0]
            elif traci.vehicle.getLeader(rlEnv.Target_left_car)[0] == 'ego' and traci.vehicle.getLaneIndex('ego')==1:
                left_leader = 'car.left'+str(int(rlEnv.Target_left_car[-1])+1)
            else:
                left_leader = traci.vehicle.getLeader(rlEnv.Target_left_car)[0]
            if traci.vehicle.getFollower(rlEnv.Target_left_car)[0] =='ego' and traci.vehicle.getLaneIndex('ego')==2:
                left_follower = traci.vehicle.getFollower('ego')[0]
            elif traci.vehicle.getFollower(rlEnv.Target_left_car)[0] =='ego' and traci.vehicle.getLaneIndex('ego')==1:
                left_follower = 'car.left'+str(int(rlEnv.Target_left_car[-1])-1)
            else:
                left_follower = traci.vehicle.getFollower(rlEnv.Target_left_car)[0]
            
            if traci.vehicle.getLeader(rlEnv.Target_right_car)[0] == 'ego' and traci.vehicle.getLaneIndex('ego')==0:
                right_leader = traci.vehicle.getLeader('ego')[0]
            elif traci.vehicle.getLeader(rlEnv.Target_right_car)[0] == 'ego' and traci.vehicle.getLaneIndex('ego')==1:
                right_leader = 'car.right'+str(int(rlEnv.Target_right_car[-1])+1)
            else:
                right_leader = traci.vehicle.getLeader(rlEnv.Target_right_car)[0]
            if traci.vehicle.getFollower(rlEnv.Target_right_car)[0] =='ego' and traci.vehicle.getLaneIndex('ego')==0:
                right_follower = traci.vehicle.getFollower('ego')[0]
            elif traci.vehicle.getFollower(rlEnv.Target_right_car)[0] =='ego' and traci.vehicle.getLaneIndex('ego')==1:
                right_follower = 'car.right'+str(int(rlEnv.Target_right_car[-1])-1)
            else:
                right_follower = traci.vehicle.getFollower(rlEnv.Target_right_car)[0]
        else:
            left_leader, left_follower, right_leader, right_follower = self.reset_left_right_target_vehicle()

        # print('::::::::::::::::::::::::::left_leader: ',left_leader)
        # print('::::::::::::::::::::::::::left_follower: ',left_follower)
        id = 'ego'
        time_gap_LC_1 =1
        time_gap_LC_2 =0.5 
        car_overall_length =0
        c_LC =12 +car_overall_length # min clearance for lane change        
        vehs = traci.vehicle.getIDList()#direction은 space의 중심을 기준으로 앞뒤로 나뉨.
        if rlEnv.Target_left_car in vehs and rlEnv.Target_right_car in vehs and 'ego' in vehs:  
            if action == 1 and direction == 'back':
                relative_velocity_term = max([traci.vehicle.getSpeed(rlEnv.Target_left_car)-traci.vehicle.getSpeed(id),0] )*time_gap_LC_1
                minimum_clearance_trem = max([traci.vehicle.getSpeed(rlEnv.Target_left_car)*time_gap_LC_2,c_LC])
                SD_LC_back = relative_velocity_term + minimum_clearance_trem
                # print('space1 SD_LC: ',SD_LC)
                # space1_size = (traci.vehicle.getPosition(traci.vehicle.getLeader(rlEnv.Target_left_car)[0])[0] -traci.vehicle.getPosition(rlEnv.Target_left_car)[0])
                # return  min([SD_LC_back, space1_size/2])
                # print('Space1_SD_LC_back')
                return SD_LC_back
            elif action == 1 and direction == 'front':
                relative_velocity_term = max([traci.vehicle.getSpeed(id)-traci.vehicle.getSpeed(left_leader),0] )*time_gap_LC_1
                minimum_clearance_trem = max([traci.vehicle.getSpeed(id)*time_gap_LC_2,c_LC])
                SD_LC_front = relative_velocity_term + minimum_clearance_trem
                # print('space1 SD_LC: ',SD_LC)
                # space1_size = (traci.vehicle.getPosition(traci.vehicle.getLeader(rlEnv.Target_left_car)[0])[0] -traci.vehicle.getPosition(rlEnv.Target_left_car)[0])
                # return  min([SD_LC_front, space1_size/2])
                # print('Space1_SD_LC_front')
                return SD_LC_front
            elif action == 2 and direction == 'back':
                relative_velocity_term = max([traci.vehicle.getSpeed(rlEnv.Target_right_car)-traci.vehicle.getSpeed(id),0] )*time_gap_LC_1
                minimum_clearance_trem = max([traci.vehicle.getSpeed(rlEnv.Target_right_car)*time_gap_LC_2,c_LC])
                SD_LC_back = relative_velocity_term + minimum_clearance_trem
                # print('space2 SD_LC: ',SD_LC)
                # space2_size = (traci.vehicle.getPosition(traci.vehicle.getLeader(rlEnv.Target_right_car)[0])[0] -traci.vehicle.getPosition(rlEnv.Target_right_car)[0])
                # return  min([SD_LC_back, space2_size/2])
                # print('Space2_SD_LC_back')
                return SD_LC_back
            elif action == 2 and direction == 'front':
                relative_velocity_term = max([traci.vehicle.getSpeed(id)-traci.vehicle.getSpeed(right_leader),0] )*time_gap_LC_1
                minimum_clearance_trem = max([traci.vehicle.getSpeed(id)*time_gap_LC_2,c_LC])
                SD_LC_front = relative_velocity_term + minimum_clearance_trem
                # print('space2 SD_LC: ',SD_LC)
                # space2_size = (traci.vehicle.getPosition(traci.vehicle.getLeader(rlEnv.Target_right_car)[0])[0] -traci.vehicle.getPosition(rlEnv.Target_right_car)[0])
                # return  min([SD_LC_front, space2_size/2])
                # print('Space2_SD_LC_front')
                return SD_LC_front
            elif action == 3 and direction =='front':
                relative_velocity_term = max( [traci.vehicle.getSpeed(id) - traci.vehicle.getSpeed(rlEnv.Target_left_car) ,0] )*time_gap_LC_1
                minimum_clearance_trem = max([traci.vehicle.getSpeed(id)*time_gap_LC_2,c_LC])
                SD_LC_front = relative_velocity_term +minimum_clearance_trem
                # print('space3 SD_LC: ',SD_LC)
                # space3_size = (traci.vehicle.getPosition(rlEnv.Target_left_car)[0]-traci.vehicle.getPosition(traci.vehicle.getFollower(rlEnv.Target_left_car)[0])[0])
                # return min([SD_LC_front, space3_size/2])
                # print('Space3_SD_LC_front')
                return SD_LC_front
            elif action == 3 and direction =='back':
                relative_velocity_term = max([traci.vehicle.getSpeed(left_follower)- traci.vehicle.getSpeed(id), 0] )*time_gap_LC_1
                minimum_clearance_trem = max([traci.vehicle.getSpeed(left_follower)*time_gap_LC_2,c_LC])
                SD_LC_back = relative_velocity_term +minimum_clearance_trem
                # print('space3 SD_LC: ',SD_LC)
                # space3_size = (traci.vehicle.getPosition(rlEnv.Target_left_car)[0]-traci.vehicle.getPosition(traci.vehicle.getFollower(rlEnv.Target_left_car)[0])[0])
                # return min([SD_LC_back, space3_size/2])
                # print('Space3_SD_LC_back')
                return SD_LC_back
            elif action == 4 and direction == 'front':
                relative_velocity_term = max( [traci.vehicle.getSpeed(id) - traci.vehicle.getSpeed(rlEnv.Target_right_car) ,0] )*time_gap_LC_1
                minimum_clearance_trem = max([traci.vehicle.getSpeed(id)*time_gap_LC_2,c_LC])
                SD_LC_front = relative_velocity_term +minimum_clearance_trem
                # print('space4 SD_LC: ',SD_LC)
                # space4_size = (traci.vehicle.getPosition(rlEnv.Target_right_car)[0]-traci.vehicle.getPosition(traci.vehicle.getFollower(rlEnv.Target_right_car)[0])[0])
                # return min([SD_LC_front, space4_size/2])
                # print('Space4_SD_LC_front')
                return SD_LC_front
            elif action == 4 and direction == 'back':
                relative_velocity_term = max([traci.vehicle.getSpeed(right_follower)- traci.vehicle.getSpeed(id), 0] )*time_gap_LC_1
                minimum_clearance_trem = max([traci.vehicle.getSpeed(right_follower)*time_gap_LC_2,c_LC])
                SD_LC_back = relative_velocity_term +minimum_clearance_trem
                # print('space4 SD_LC: ',SD_LC)
                # space4_size = (traci.vehicle.getPosition(rlEnv.Target_right_car)[0]-traci.vehicle.getPosition(traci.vehicle.getFollower(rlEnv.Target_right_car)[0])[0])
                # return min([SD_LC_back, space4_size/2])
                # print('Space4_SD_LC_back')
                return SD_LC_back
            else:
                return 0

    def __SD_check(self,action):
        if (action == 1 or action == 3) and rlEnv.Left_action == True:
            action = 5
        if (action == 2 or action == 4) and rlEnv.Right_action == True:
            action = 6
        id = 'ego'
        time_gap_LC_1 =1
        time_gap_LC_2 =0.5 
        c_LC =12 # min clearance for lane change
        ego_v_x = traci.vehicle.getSpeed(id)
        ego_x_pos = traci.vehicle.getPosition(id)[0]
        if traci.vehicle.getLeftLeaders(id) is not None and len(traci.vehicle.getLeftLeaders(id)) !=0:
            Left_leader_id= traci.vehicle.getLeftLeaders(id)
        if traci.vehicle.getRightLeaders(id) is not None and len(traci.vehicle.getRightLeaders(id)) !=0:
            Right_leader_id= traci.vehicle.getRightLeaders(id)
        LEFT_FOLLOWER =[]
        RIGHT_FOLLOWER=[]
        traci.vehicle.subscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 200.0, [tc.VAR_POSITION])
        # print(id,' subscribeContext')
        for v in traci.vehicle.getContextSubscriptionResults(str(id)):
            Left_follower_id = self.__find_followers(id, v,LEFT_FOLLOWER,RIGHT_FOLLOWER)[0]
            Right_follower_id = self.__find_followers(id, v,LEFT_FOLLOWER,RIGHT_FOLLOWER)[1]
        traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 200.0)
        if traci.vehicle.getLaneIndex(id) == 1: 
            if len(traci.vehicle.getLeftLeaders(id)) !=0 and Left_follower_id is not None: # 왼쪽 차로 앞 뒤 차량이 100m 안에 있다는 가정.
                front_relative_velocity_term = max( [traci.vehicle.getSpeed(id) - traci.vehicle.getSpeed(Left_leader_id[0][0]) ,0] )*time_gap_LC_1
                front_minimum_clearance_trem = max([traci.vehicle.getSpeed(id)*time_gap_LC_2,c_LC])
                front_SD_LC = front_relative_velocity_term +front_minimum_clearance_trem
                # print('Left_leader_id[0][0]: ',Left_leader_id[0][0])
                back_relative_velocity_term = max( [traci.vehicle.getSpeed(Left_follower_id)-traci.vehicle.getSpeed(id),0] )*time_gap_LC_1
                back__minimum_clearance_trem = max([traci.vehicle.getSpeed(Left_follower_id)*time_gap_LC_2,c_LC])
                back_SD_LC = back_relative_velocity_term + back__minimum_clearance_trem
                # print('Left_follower_id: ',Left_follower_id)
                if((traci.vehicle.getPosition(Left_leader_id[0][0])[0]-ego_x_pos) >= front_SD_LC  and (ego_x_pos -traci.vehicle.getPosition(Left_follower_id)[0])>= back_SD_LC):
                    rlEnv.Left_SD = True
                else:
                    rlEnv.Left_SD = False
        
            if len(traci.vehicle.getRightLeaders(id))!=0 and Right_follower_id is not None: # 오른쪽 차로 앞 뒤 차량이 100m 안에 있다는 가정.
                front_relative_velocity_term = max( [traci.vehicle.getSpeed(id) - traci.vehicle.getSpeed(Right_leader_id[0][0]) ,0] )*time_gap_LC_1
                front_minimum_clearance_trem = max([traci.vehicle.getSpeed(id)*time_gap_LC_2,c_LC])
                front_SD_LC = front_relative_velocity_term +front_minimum_clearance_trem
                # print('Right_leader_id[0][0]: ',Right_leader_id[0][0])
                back_relative_velocity_term = max( [traci.vehicle.getSpeed(Right_follower_id)-traci.vehicle.getSpeed(id),0] )*time_gap_LC_1
                back__minimum_clearance_trem = max([traci.vehicle.getSpeed(Right_follower_id)*time_gap_LC_2,c_LC])
                back_SD_LC = back_relative_velocity_term + back__minimum_clearance_trem
                # print('Right_follower_id: ',Right_follower_id)
                if((traci.vehicle.getPosition(Right_leader_id[0][0])[0]-ego_x_pos) >=front_SD_LC  and (ego_x_pos -traci.vehicle.getPosition(Right_follower_id)[0])>=back_SD_LC):
                    rlEnv.Right_SD = True
                else:
                    rlEnv.Right_SD = False

        if traci.vehicle.getLeader(id) is not None:
            Leader_id,x_forward = traci.vehicle.getLeader(id)
        if type(traci.vehicle.getFollower(id)) is not None and traci.vehicle.getFollower(id)[0] != '':
            follower_id, x_back = traci.vehicle.getFollower(id) 
        if action == 5 and traci.vehicle.getLaneIndex(id) == 2:
            front_relative_velocity_term = max( [traci.vehicle.getSpeed(id) - traci.vehicle.getSpeed(Leader_id) ,0] )*time_gap_LC_1
            front_minimum_clearance_trem = max([traci.vehicle.getSpeed(id)*time_gap_LC_2,c_LC])
            front_SD_LC = front_relative_velocity_term +front_minimum_clearance_trem
            back_relative_velocity_term = max( [traci.vehicle.getSpeed(follower_id)-traci.vehicle.getSpeed(id),0] )*time_gap_LC_1
            back__minimum_clearance_trem = max([traci.vehicle.getSpeed(follower_id)*time_gap_LC_2,c_LC])
            back_SD_LC = back_relative_velocity_term + back__minimum_clearance_trem
            # print('back_SD_LC: ',back_SD_LC)
            # print('front_SD_LC: ',front_SD_LC)
            if((traci.vehicle.getPosition(Leader_id)[0]-ego_x_pos) >= front_SD_LC  and (ego_x_pos -traci.vehicle.getPosition(follower_id)[0])>= back_SD_LC):
                rlEnv.Left_SD = True
            else:
                rlEnv.Left_SD = False
        if action == 6 and traci.vehicle.getLaneIndex(id) == 0:
            front_relative_velocity_term = max( [traci.vehicle.getSpeed(id) - traci.vehicle.getSpeed(Leader_id) ,0] )*time_gap_LC_1
            front_minimum_clearance_trem = max([traci.vehicle.getSpeed(id)*time_gap_LC_2,c_LC])
            front_SD_LC = front_relative_velocity_term +front_minimum_clearance_trem
            back_relative_velocity_term = max( [traci.vehicle.getSpeed(follower_id)-traci.vehicle.getSpeed(id),0] )*time_gap_LC_1
            back__minimum_clearance_trem = max([traci.vehicle.getSpeed(follower_id)*time_gap_LC_2,c_LC])
            back_SD_LC = back_relative_velocity_term + back__minimum_clearance_trem
            # print('back_SD_LC: ',back_SD_LC)
            # print('front_SD_LC: ',front_SD_LC)
            if((traci.vehicle.getPosition(Leader_id)[0]-ego_x_pos) >= front_SD_LC  and (ego_x_pos -traci.vehicle.getPosition(follower_id)[0])>= back_SD_LC):
                rlEnv.Right_SD = True
            else:
                rlEnv.Right_SD = False
                



    def __LaneChange_SD_negative_reward(self,action): # SD_LC 침범 안하면 reward 0, SD_LC 침범한 거리 만큼 negative reward , 앞뒤 차량 SD_LC 침범시 reward -50
        if (action == 1 or action == 3) and rlEnv.Left_action == True:
            action = 5
        if (action == 2 or action == 4) and rlEnv.Right_action == True:
            action =6
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
        if traci.vehicle.getLeftLeaders(id) is not None and len(traci.vehicle.getLeftLeaders(id)) !=0:
            Left_leader_id= traci.vehicle.getLeftLeaders(id)
        if traci.vehicle.getRightLeaders(id) is not None and len(traci.vehicle.getRightLeaders(id)) !=0:
            Right_leader_id= traci.vehicle.getRightLeaders(id)
        
        
        LEFT_FOLLOWER =[]
        RIGHT_FOLLOWER=[]
        traci.vehicle.subscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 200.0, [tc.VAR_POSITION])
        # print(id,' subscribeContext')
        for v in traci.vehicle.getContextSubscriptionResults(str(id)):
            Left_follower_id = self.__find_followers(id, v,LEFT_FOLLOWER,RIGHT_FOLLOWER)[0]
            Right_follower_id = self.__find_followers(id, v,LEFT_FOLLOWER,RIGHT_FOLLOWER)[1]
        traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 200.0)
        if traci.vehicle.getLaneIndex(id) == 0 or traci.vehicle.getLaneIndex(id) == 2:
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
                return (traci.vehicle.getPosition(Leader_id)[0]-ego_x_pos) - front_SD_LC 
            elif((traci.vehicle.getPosition(Leader_id)[0]-ego_x_pos) >= front_SD_LC  and (ego_x_pos -traci.vehicle.getPosition(follower_id)[0]) < back_SD_LC):
                # print('LaneChange_SD_negative_reward-> rear ')
                return (ego_x_pos -traci.vehicle.getPosition(follower_id)[0]) - back_SD_LC
            else:
                # print('LaneChange_SD_negative_reward-> forward and rear')
                return-50

        elif action == 5 and traci.vehicle.getLaneIndex(id) == 1: #왼쪽 차로로 이동
            if len(traci.vehicle.getLeftLeaders(id)) !=0 and Left_follower_id is not None: # 왼쪽 차로 앞 뒤 차량이 100m 안에 있다는 가정.
                front_relative_velocity_term = max( [traci.vehicle.getSpeed(id) - traci.vehicle.getSpeed(Left_leader_id[0][0]) ,0] )*time_gap_LC_1
                front_minimum_clearance_trem = max([traci.vehicle.getSpeed(id)*time_gap_LC_2,c_LC])
                front_SD_LC = front_relative_velocity_term +front_minimum_clearance_trem
                back_relative_velocity_term = max( [traci.vehicle.getSpeed(Left_follower_id)-traci.vehicle.getSpeed(id),0] )*time_gap_LC_1
                back__minimum_clearance_trem = max([traci.vehicle.getSpeed(Left_follower_id)*time_gap_LC_2,c_LC])
                back_SD_LC = back_relative_velocity_term + back__minimum_clearance_trem

                if((traci.vehicle.getPosition(Left_leader_id[0][0])[0]-ego_x_pos) >= front_SD_LC  and (ego_x_pos -traci.vehicle.getPosition(Left_follower_id)[0])>= back_SD_LC):
                    # print("LaneChange_SD_negative_reward : left 0")
                    return 0                    
                elif((traci.vehicle.getPosition(Left_leader_id[0][0])[0]-ego_x_pos) < front_SD_LC  and (ego_x_pos -traci.vehicle.getPosition(Left_follower_id)[0])>= back_SD_LC):
                    # print('LaneChange_SD_negative_reward : left forward   ',(traci.vehicle.getPosition(Left_leader_id[0][0])[0]-ego_x_pos) - front_SD_LC )
                    return (traci.vehicle.getPosition(Left_leader_id[0][0])[0]-ego_x_pos) - front_SD_LC 
                elif((traci.vehicle.getPosition(Left_leader_id[0][0])[0]-ego_x_pos) >= front_SD_LC  and (ego_x_pos -traci.vehicle.getPosition(Left_follower_id)[0]) < back_SD_LC):
                    # print('LaneChange_SD_negative_reward : left rear   ',(ego_x_pos -traci.vehicle.getPosition(Left_follower_id)[0]) - back_SD_LC)
                    return (ego_x_pos -traci.vehicle.getPosition(Left_follower_id)[0]) - back_SD_LC
                else:
                    # print('LaneChange_SD_negative_reward : left forward and rear    ',-50)
                    return-50
            else:
                return 0
                

        elif action == 6  and traci.vehicle.getLaneIndex(id) == 1: #오른쪽 차로로 이동
            if len(traci.vehicle.getRightLeaders(id))!=0 and Right_follower_id is not None: # 오른쪽 차로 앞 뒤 차량이 100m 안에 있다는 가정.
                front_relative_velocity_term = max( [traci.vehicle.getSpeed(id) - traci.vehicle.getSpeed(Right_leader_id[0][0]) ,0] )*time_gap_LC_1
                front_minimum_clearance_trem = max([traci.vehicle.getSpeed(id)*time_gap_LC_2,c_LC])
                front_SD_LC = front_relative_velocity_term +front_minimum_clearance_trem
                back_relative_velocity_term = max( [traci.vehicle.getSpeed(Right_follower_id)-traci.vehicle.getSpeed(id),0] )*time_gap_LC_1
                back__minimum_clearance_trem = max([traci.vehicle.getSpeed(Right_follower_id)*time_gap_LC_2,c_LC])
                back_SD_LC = back_relative_velocity_term + back__minimum_clearance_trem
                if((traci.vehicle.getPosition(Right_leader_id[0][0])[0]-ego_x_pos) >=front_SD_LC  and (ego_x_pos -traci.vehicle.getPosition(Right_follower_id)[0])>=back_SD_LC):
                    # print('LaneChange_SD_negative_reward : right 0')
                    return 0
                elif((traci.vehicle.getPosition(Right_leader_id[0][0])[0]-ego_x_pos) <front_SD_LC  and (ego_x_pos -traci.vehicle.getPosition(Right_follower_id)[0])>=back_SD_LC): 
                    # print('LaneChange_SD_negative_reward : right forward     ',(traci.vehicle.getPosition(Right_leader_id[0][0])[0]-ego_x_pos) - front_SD_LC)
                    return (traci.vehicle.getPosition(Right_leader_id[0][0])[0]-ego_x_pos) - front_SD_LC
                elif((traci.vehicle.getPosition(Right_leader_id[0][0])[0]-ego_x_pos) >=front_SD_LC  and (ego_x_pos -traci.vehicle.getPosition(Right_follower_id)[0])<back_SD_LC):
                    # print('LaneChange_SD_negative_reward : right rear     ',(ego_x_pos -traci.vehicle.getPosition(Right_follower_id)[0]) - back_SD_LC)
                    return (ego_x_pos -traci.vehicle.getPosition(Right_follower_id)[0]) - back_SD_LC
                else:
                    # print('LaneChange_SD_negative_reward : right forward and rear     ',-50)
                    return -50
            else:
                return 0
        else:
            return 0
        
        
    def __reward(self, step_num, action):
        
        Reward = []
        Reward.append(traci.simulation.getTime())
        w1= 0.01 # collision negative reward
        w2 =0.01 # SD_LC not satisfied negative reward
        w2_1 = 0.005 #SD_LK not satisfied negative reward
        w3 = 0.2 # distance to biggest empty space positive reward
        w4 = 0.6 # LK continue if rear accel is far away positive reward or if rear accel is closer give negative reward
        w5 = 0.01 # LC succeed positive reward
        w6 = 0.01 # LK when no danger detected        
        w7 = 0 # to target lane positive reward
        w8 = 0.5 # no collision until rear vehicle pass positive reward
        w9 = 0 # once LC start keep last step action positive reward
        w10 = 0.01 # SD_LC target space 
        reward = 0
        vehs = traci.vehicle.getIDList()
        ###negative rewards            
        #collision
        if('ego' in vehs): 
            LLP = traci.vehicle.getLateralLanePosition('ego')
            Lane = traci.vehicle.getLaneIndex('ego')
        else:
            LLP =0
            Lane =1
        if('ego' in vehs and 'accel.rear' in vehs):  
            rear_clearance = traci.vehicle.getPosition('ego')[0] - traci.vehicle.getPosition('accel.rear')[0]
            ego_v = traci.vehicle.getSpeed('ego')
            rear_v = traci.vehicle.getSpeed('accel.rear')
            rear_TTC = rear_clearance/(rear_v-ego_v)
        else:
            rear_TTC = 0
        self.__SD_check(action)
        self.__Front_SD_check()
        # print('rlEnv.Left_SD: ',rlEnv.Left_SD )
        # print('rlEnv.Right_SD: ',rlEnv.Right_SD)
        # print('rlEnv.Front_SD: ',rlEnv.Front_SD)
        if action == 7 and Lane == 1 and np.abs(LLP)<=0.56 and (rear_TTC<2.4  and (rlEnv.Left_SD == False and rlEnv.Right_SD == False))and rlEnv.Front_SD:
            # print('in action 7 reward')
            reward=0 #reward 0
            Reward.append(0)
            Reward.append(0)
            Reward.append(0)
            Reward.append(0)
            Reward.append(0)
            Reward.append(0)
            Reward.append(0)
            Reward.append(0)
            Reward.append(0)
            Reward.append(0)
        else:
            if len(traci.simulation.getCollidingVehiclesIDList()) !=0:
                # print('negative reward')
                log = traci.simulation.getCollisions()
                if str(log[0]).split(',')[1]== ' victim=ego': 
                    print(str(log[0]).split(',')[0])
                    # if str(log[0]).split(',')[0]== 'Collision(collider=accel.rear' and ((np.abs(LLP)>0.56 and Lane ==1) or (Lane ==2 or 0)):
                    #     print("LC tried hard")
                    #     reward -=50*w1
                    #     Reward.append(-50*w1)
                    # else:
                    # print('Rear vehicle collision negative reward: ',-100*w1)   
                    # print('collision negative reward: ',-200*w1)
                    reward -= 150*w1
                    Reward.append(-150*w1)
                # collision caused by ego
                elif str(log[0]).split(',')[0]== 'Collision(collider=ego' :
                    # print('ego cause collision negative reward: ',-200*w1)
                    reward -= 300*w1
                    Reward.append(-300*w1)
            else:
                Reward.append(0)
                reward +=0
            if (action == 1 or action == 2 or action == 3 or action == 4):
                # print('LaneChange_SD_negative_reward: ',self.__LaneChange_SD_negative_reward(action)*w2)
                reward += self.__LaneChange_SD_negative_reward(action)*w2
                Reward.append(self.__LaneChange_SD_negative_reward(action)*w2)
            else:
                Reward.append(0)
                reward +=0
            
            if('ego' in vehs and 'accel.rear' in vehs):           
            
                if traci.vehicle.getLaneIndex('ego')==1 and 'car.forward' in vehs:
                    # print('Forward_SD_negative_reward: ',self.__Forward_SD_negative_reward()*w2)
                    reward += self.__Forward_SD_negative_reward()*w2_1
                    Reward.append(self.__Forward_SD_negative_reward()*w2_1)
                else:
                    Reward.append(0)
                    reward +=0

                rear_clearance = traci.vehicle.getPosition('ego')[0] - traci.vehicle.getPosition('accel.rear')[0]
                # print('rear_clearance: ',rear_clearance)
                # if(rear_TTC<=100 and rear_clearance>=0): 
                # print("rear_TTC: ",rear_TTC)               
                if(rear_TTC<=13.2 and rear_clearance>=0): # 차로 변경을 위한 가장큰 빈공간의 중앙에 가까워질 수 록 보상을 준다.
                    # print(self.__toward_biggest_empty_space_reward('ego'))
                    # print('toward_biggest_empty_space_reward: ',self.__toward_biggest_empty_space_reward('ego')*w3)
                    value = self.__toward_empty_space_reward('ego',action)*w3
                    reward += value
                    Reward.append(value)
                else:
                    Reward.append(0)
                    reward +=0
                if traci.vehicle.getLaneIndex('ego')==1:                
                    # if rear_clearance>70 and (action ==0 or action ==1 or action ==2):
                    #     # print('LK continue reward: ', 1*w4)
                    #     reward +=1*w4
                    if(rlEnv.ego_LC_success ==False and rear_TTC<=13.2 and rear_clearance >=0 and traci.vehicle.getLaneIndex('ego')==1): #후방 차량 충돌 판단 가능 범위
                        # print('Danger LK continue reward: ' ,-1*w4)
                        reward -= 1.1**(10-rear_clearance)*w4
                        Reward.append(-1.1**(10-rear_clearance)*w4)
                    else:
                        Reward.append(0)
                        reward +=0
                else:
                    Reward.append(0)
                    reward +=0
                # if (rear_clearance<-6):
                #     reward += np.abs(rear_clearance)*w4
                # print('LC completed: ',self.__ego_vehicle_LC_completed())
                if (self.done and len(traci.simulation.getCollidingVehiclesIDList()) ==0):
                    print('check right zero value if collision else 200 or 150 or 100 or 50')
                    print('LC success reward: ',self.__chosen_space_size_reward()[0]*w5 )
                    print('chosen space: ',self.__chosen_space_size_reward()[1])
                    if self.__chosen_space_size_reward()[1] =='1':
                        rlEnv.space1_count +=1
                    elif self.__chosen_space_size_reward()[1] == '2':
                        rlEnv.space2_count +=1
                    elif self.__chosen_space_size_reward()[1] == '3':
                        rlEnv.space3_count +=1
                    else:
                        rlEnv.space4_count +=1
                    if self.__chosen_space_size_reward()[2] =='R1':
                        rlEnv.R1_count +=1
                    elif self.__chosen_space_size_reward()[2] == 'R2':
                        rlEnv.R2_count +=1
                    elif self.__chosen_space_size_reward()[2] == 'L1':
                        rlEnv.L1_count +=1
                    else:
                        rlEnv.L2_count +=1

                    reward += self.__chosen_space_size_reward()[0]*w5
                    Reward.append(self.__chosen_space_size_reward()[0]*w5)
                else:
                    Reward.append(0)
                    reward +=0
                
                # vehs = traci.vehicle.getIDList()
                
                
                if(rear_TTC>13.2 and (action == 7)): #safe state LK positive reward                    
                    # print('safe state LK positive reward: ',1*w6)
                    reward += (rear_TTC)*w6
                    Reward.append((rear_TTC)*w6)  
                else:
                    Reward.append(0)
                    reward +=0    
  
                if len(rlEnv.last_action)>0 and rear_TTC<13.2 and (((action == 1 or action == 3) and rlEnv.Left_SD == True) or ((action == 2 or action ==  4) and rlEnv.Right_SD == True)):                    
                    if rlEnv.Left_action == True and (action == 1 or action == 3):
                        if Lane == 1 and LLP >0:
                            reward+= LLP*w7
                            Reward.append(LLP*w7)
                        elif Lane ==2 and LLP <0:
                            reward+= (3.2+LLP)*w7
                            Reward.append((3.2+LLP)*w7)
                        else:
                            reward+=0
                            Reward.append(0)
                    elif rlEnv.Right_action == False and (action == 2 or action ==  4):
                        if Lane == 1 and LLP <0:
                            reward+= -LLP*w7
                            Reward.append(-LLP*w7)
                        elif Lane ==0 and LLP>0:
                            reward+= (3.2-LLP)*w7
                            Reward.append((3.2-LLP)*w7)
                        else:
                            reward+=0
                            Reward.append(0)
                    else:
                        reward+=0
                        Reward.append(0)

                else:
                    reward+=0
                    Reward.append(0)
                
                if rear_clearance<= -29.7: # if rear_clearance >=-30 episode ends
                    reward += 30*w8
                    Reward.append(30*w8)
                else:
                    reward += 0
                    Reward.append(0)

            else:
                Reward.append(0)
                Reward.append(0)
                Reward.append(0)
                Reward.append(0)
                Reward.append(0)
                Reward.append(0)
                Reward.append(0)
                reward +=0
            # print('rlEnv.last_action: ',rlEnv.last_action[-1][1])
            # print('rlEnv.Left_action: ',rlEnv.Left_action)
            # print('rlEnv.Right_action: ',rlEnv.Right_action)
            if ((Lane == 1 or Lane == 2) and action == 1 and rlEnv.Left_action == True and rlEnv.last_action[-1][1] == 1) or ((Lane == 1 or Lane == 2) and action == 3 and rlEnv.Left_action == True and rlEnv.last_action[-1][1] == 3) or ((Lane == 1 or Lane == 0) and action == 2 and rlEnv.Right_action == True and rlEnv.last_action[-1][1] == 2) or ((Lane == 1 or Lane == 0) and action == 4 and rlEnv.Right_action == True and rlEnv.last_action[-1][1] == 4):
                # print('continue LC positive reward')
                reward +=1*w9
                Reward.append(1*w9)
            else:
                reward += 0
                Reward.append(0)
                
        Reward.append(reward)
        rlEnv.rewards.append(Reward)
        return reward
    
    # def __next_state(self):
    #     next_states =[]

    #     return next_states
    def LC_cubic_polynomial_path(self,x_initial,y_initial,x_final,y_final,rear_v_id,forward_v_id,theta_i,action):
        #위치에 따른 차로 변경에 걸리는 시간 지정.
        # if traci.vehicle.getLaneIndex('ego') == 1:
        #     t_LC = 4
        # else:
        #     t_LC = 2
        lane = traci.vehicle.getLaneIndex('ego')
        LLP = traci.vehicle.getLateralLanePosition('ego') #LateralLanePosition
        t_LC =4
        if lane == 1:
            t_LC =4
            # if LLP >=0 and action== 5:
            #     t_LC = 4 
            # elif LLP<0 and action== 5:
            #     t_LC = 2.2
            # if LLP >0 and action==6:
            #     t_LC =2.2
            # elif LLP<=0 and action==6:
            #     t_LC = 4
        elif lane == 0:
            if LLP>=0 and action==5:
                t_LC =3
            elif LLP<0 and action==5:
                t_LC =0
            if LLP>=0 and action ==6:
                t_LC =2.2
        elif lane == 2:
            if LLP>0 and  action == 6:
                t_LC = 0
            elif LLP<=0 and action ==6:
                t_LC =2.2
            if LLP<=0 and action ==5:
                t_LC = 3
        # t_LC=t_LC+1

        
        initial_rear_x = traci.vehicle.getPosition(rear_v_id)[0]
        initial_forward_x = traci.vehicle.getPosition(forward_v_id)[0]
        initial_rear_v = traci.vehicle.getSpeed(rear_v_id)
        initial_forward_v = traci.vehicle.getSpeed(forward_v_id)
        initial_rear_a = traci.vehicle.getAcceleration(rear_v_id)
        initial_forward_a = traci.vehicle.getAcceleration(rear_v_id)
        x_final = ((initial_rear_x+t_LC*initial_rear_v+0.5*initial_rear_a*t_LC**2)+(initial_forward_x+t_LC*initial_forward_v+0.5*initial_forward_a*t_LC**2))/2 -x_initial
        y_final = y_final-y_initial
        # print('y_final: ',y_final)
        #짧은 LC
        if np.abs(y_final) < 3.15 and lane ==1:
            d_target_space = (80*np.abs(y_final)/3.2)
            t_LC = d_target_space/traci.vehicle.getSpeed('ego')
            # if t_LC<1.5:
            #     t_LC= 1.5
            if t_LC <1:
                t_LC = 1
            x_final = ((initial_rear_x+t_LC*initial_rear_v+0.5*initial_rear_a*t_LC**2)+(initial_forward_x+t_LC*initial_forward_v+0.5*initial_forward_a*t_LC**2))/2 -x_initial
        # print('x_final: ',x_final)
        # print('t_LC: ',t_LC)
        

        if x_final <=0:
            x_cur =0
            y_cur =0
            coefficient = [0,0,0,0]
        else:
            x_cur = np.arange(0,x_final,0.1)
            y_cur = np.tan(theta_i)*x_cur+((3*y_final-2*x_final*np.tan(theta_i))/x_final**2)*x_cur**2+((x_final*np.tan(theta_i)-2*y_final)/x_final**3)*x_cur**3
            coefficient = [0,np.tan(theta_i),((3*y_final-2*x_final*np.tan(theta_i))/x_final**2),((x_final*np.tan(theta_i)-2*y_final)/x_final**3)]
        
        if self.gui_on:
            plt.figure(0,figsize=(8,4))
            pylab.plot(x_cur, y_cur, 'b')
            pylab.xlabel("x")
            pylab.ylabel("y")
            pylab.savefig("/home/mds/Desktop/highway_episodic/DQN/RL_validation/path/graph.png")
            pylab.clf()


        # path = list(zip(x_cur,y_cur))
        
        # print(path)
        # x = symbols('x')
        # y = symbols('y')
        # yprime = 2*coefficient[2]*x+3*coefficient[3]*x**2
        # I = symbols.Integral(symbols.root(1+yprime**2))
      
        return coefficient, x_final, y_final
    def K(self,y_prime,y_pprime):
        # return np.abs(y_pprime/(1+y_prime**2)**(3/2))
        return y_pprime/(1+y_prime**2)**(3/2)

    def stanley(self,coef,heading_angle,x_cur,v_cur): #heading_angle(degree)
        if coef == [0,0,0,0]:
            return 0
        else:
            max_rad=0.524 #I added margin maixmum is 0.6526438369 [rad]
            min_rad=-0.524 #-30 degree
            soft_term=1
            if len(rlEnv.ego) !=0:
                prev_delta = rlEnv.ego[-1][8]
            else:
                prev_delta = 90

            p_gain=1*0.03
            h_gain=10*0.03
            
            #short distance
            # if rlEnv.final_x < 70:
            #     # print("short distance gain")
            #     p_gain = 0.6*0.03
            #     h_gain = 6*0.03

            dt=0.01
            pre_view_point =  x_cur+v_cur*dt
            # pre_view_point =  x_cur
            delta = (90-heading_angle)*np.pi/180 #delta(radian)
            x = symbols('x')
            fx = coef[0]+coef[1]*x+coef[2]*x**2+coef[3]*x**3
            fprime = coef[1]+2*coef[2]*x+3*coef[3]*x**2
            f_2_prime = 2*coef[2]+6*coef[3]*x
            n = fprime.subs({x:pre_view_point})
        
            cte=fx.subs({x: pre_view_point})

            if cte >3 :
                cte=3
            elif cte<-3:
                cte=-3
            
            crosstrack_error_term=math.atan((p_gain*cte)/(pre_view_point+soft_term))
            heading_error_term=h_gain*(math.atan(n)-delta)
            
            
            delta= crosstrack_error_term+ heading_error_term

            if max_rad < delta:
                delta=max_rad
            elif delta< min_rad: 
                delta=min_rad


            # print("delta_return: ",delta)
            # rlEnv.crosstrack_error_term.append(crosstrack_error_term)
            # rlEnv.heading_error_term.append(heading_error_term)
            # rlEnv.delta_term.append(delta)

            # plt.figure(1,figsize=(8,4))
            # pylab.plot(rlEnv.crosstrack_error_term,label = 'crosstrack_error_term',color ='b')
            # pylab.plot(rlEnv.heading_error_term,label = 'heading_error_term',color ='r')
            # pylab.legend()
            # pylab.xlabel("step")
            # pylab.ylabel("error")
            # pylab.savefig("/home/mds/Desktop/highway_episodic/DQN/RL_validation/stanley_graph/stanley_error_graph.png")
            # pylab.clf()
            # plt.figure(2,figsize=(8,4))
            # pylab.plot(rlEnv.delta_term,label = 'delta_term',color ='g')
            # pylab.legend()
            # pylab.xlabel("step")
            # pylab.ylabel("error")
            # pylab.savefig("/home/mds/Desktop/highway_episodic/DQN/RL_validation/stanley_graph/delta_term_graph.png")
            # pylab.clf()
            return delta #error를 보상해주는 delta 값.
    def LC_Left(self,action,v_target,c_target):
        if action == 1 or action == 3:
            action =5
        vehs = traci.vehicle.getIDList()
        if('ego' in vehs):  
        
            tau = 0.74
            v_max = 22.22 #선두 차량 속도
            c0 = 1.98  
            LLP = traci.vehicle.getLateralLanePosition('ego') #LateralLanePosition
            if action == 5:
                traci.vehicle.setSpeedMode('ego',32)
                traci.vehicle.setLaneChangeMode('ego',0)

                edgeID = 'E5'

                if traci.vehicle.getLaneIndex('ego') ==0 and LLP<0:
                    lane =0
                elif traci.vehicle.getLaneIndex('ego') ==0 and LLP>=0:
                    lane =1 
                elif traci.vehicle.getLaneIndex('ego') ==1 and LLP<0:
                    lane =1
                elif traci.vehicle.getLaneIndex('ego') ==1 and LLP>=0:
                    lane =2
                elif traci.vehicle.getLaneIndex('ego') ==2 and LLP<0:
                    lane =2
                elif traci.vehicle.getLaneIndex('ego') ==2 and LLP>=0:
                    lane =2
                    print("wrong Left LC")
                else:
                    lane = 0
                keepRoute =2 #기존 route에 제한 받지 않고 movetoxy 실행.
                matchThreshold =3.2 #차로 폭
                if(len(rlEnv.ego)!=0):
                    last_ego_x = rlEnv.ego[-1][2]
                    last_ego_y = rlEnv.ego[-1][3]
                    last_ego_v_y = rlEnv.ego[-1][5]
                else:
                    last_ego_x,last_ego_y = traci.vehicle.getPosition('ego')
                    last_ego_v_y=0
                ego_x,ego_y = traci.vehicle.getPosition('ego')
                ego_v = traci.vehicle.getSpeed('ego')
                ego_a = traci.vehicle.getAcceleration('ego')
                ego_v_y = self.__get_v_y('ego')
                ego_a_y = self.__get_a_y('ego')

                xy ,ids = self.__LC_goal_space_posistion(action)
                rlEnv.last_target_space = xy
                rlEnv.last_vehicles = ids
                target_x, target_y = xy[0],xy[1]
                rear_v_id,forward_v_id = ids
                if ego_x-rlEnv.initial_x>=rlEnv.final_x:
                    path_done = True
                else:
                    path_done = False
                
                if path_done == True and traci.vehicle.getLaneIndex('ego') ==1 and LLP <= 0 and LLP >= -0.4 and traci.vehicle.getAngle('ego')>=89.9:
                    print("---------------------------------------LC_succeed")
                    rlEnv.initial_x = ego_x
                    rlEnv.initial_y = ego_y
                    target_y = -1.6
                    rlEnv.theta_i = (90-traci.vehicle.getAngle('ego'))*np.pi/180

                    left_vehicles=[] #[|ego_x_pos - x_pos|, id]
                    left_near_vehicles=[] #[x_pos]   
                    vehs = traci.vehicle.getIDList()
                    if 'ego' in vehs:
                        id = 'ego'                        
                        traci.vehicle.subscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 200.0, [tc.VAR_POSITION])
                    for v_id in traci.vehicle.getContextSubscriptionResults(str(id)):
                        if(traci.vehicle.getLaneIndex(v_id)==2) and not (v_id=='ego'):
                            left_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),traci.vehicle.getPosition(v_id)[0],v_id])
                    left_vehicles.sort()
                    for i in range(2):
                        left_near_vehicles.append([left_vehicles[:3][i][1],left_vehicles[:3][i][2]])# 1: left vehicle position 2: left vehicle id
                    left_near_vehicles.sort()                    
                    vehicles = [left_near_vehicles[1][1],left_near_vehicles[0][1]]
                    forward_v_id,rear_v_id = vehicles
                    LC_succeed =True
                else:
                    LC_succeed = False

                if rlEnv.Left_action == False:
                    # print('rlEnv.Left_action == False')
                    t = 0
                    rlEnv.t_LC_start = traci.simulation.getTime()
                    x0=0
                    y0=0
                    rlEnv.initial_x = ego_x
                    rlEnv.initial_y = ego_y
                    rlEnv.initial_target_y = target_y
                    rlEnv.theta_i = (90-traci.vehicle.getAngle('ego'))*np.pi/180
                    rlEnv.coefficient,rlEnv.final_x,rlEnv.final_y =self.LC_cubic_polynomial_path(rlEnv.initial_x,rlEnv.initial_y,target_x,target_y,rear_v_id,forward_v_id,rlEnv.theta_i,action)                    

                else:
                    # print('rlEnv.Left_action == True')
                    t= traci.simulation.getTime()-rlEnv.t_LC_start
                    x0= rlEnv.initial_x
                    y0= rlEnv.initial_y
                    if path_done:                       
                        rlEnv.coefficient = [target_y-ego_y,0,0,0]
                    if LC_succeed:                     
                        rlEnv.coefficient,rlEnv.final_x,rlEnv.final_y =self.LC_cubic_polynomial_path(rlEnv.initial_x,rlEnv.initial_y,target_x,target_y,rear_v_id,forward_v_id,rlEnv.theta_i,action)
                    
                        
                

                x_local = ego_x - rlEnv.initial_x
                # delta = self.stanley(rlEnv.coefficient,traci.vehicle.getAngle('ego'),x_local,ego_v)
                
                x = ego_x -rlEnv.initial_x
                y = rlEnv.coefficient[2]*x**2+rlEnv.coefficient[3]*x**3
                y_prime = 2*rlEnv.coefficient[2]*x+3*rlEnv.coefficient[3]*x**2
                y_pprime = 2*rlEnv.coefficient[2]+6*rlEnv.coefficient[3]*x
                theta_n = np.arctan(y_prime)


                delta = self.stanley(rlEnv.coefficient,traci.vehicle.getAngle('ego'),x_local,ego_v)
                # delta = 0
                theta_n += delta

                u = ego_v/np.cos(theta_n)
                # ego_a_y_des = u**2*self.K(y_prime,y_pprime)
                ego_a_y_des = u**2*self.K(y_prime,y_pprime)+u**2*np.tan(delta)
                ego_a_y_desire = self.__set_a_y_possible('ego',ego_a_y_des)    
                ego_v_y_next = ego_v_y + ego_a_y_desire*self.step_length
                y_next = ego_y+ego_v_y_next*self.step_length
                # print('ego_a_y_desire: ',ego_a_y_desire)
                # acc variables
                # if self.__ACC_target_id('ego',action):
                #     v_preceding = traci.vehicle.getSpeed(self.__ACC_target_id('ego',action))
                #     c_front = traci.vehicle.getPosition(self.__ACC_target_id('ego',action))[0]-traci.vehicle.getPosition('ego')[0]
                # else:
                #     v_preceding =22.22
                #     c_front = 2
                c_desire = 0
                ego_a_x_desire =self.__set_a_x_possible('ego',self.__ego_acc(ego_v,v_target,c_desire,c_target))
                # print('ego_a_x_desire: ',ego_a_x_desire)
                ego_v_x_next = ego_v +ego_a_x_desire*self.step_length
                x_next = ego_x+ego_v_x_next*self.step_length #ACC:
                control = [traci.simulation.getTime(),self.__ego_acc(ego_v,v_target,c_desire,c_target),ego_a_y_des]
                rlEnv.ego_control.append(control)
                theta = np.arctan2(ego_y-last_ego_y,ego_x-last_ego_x)
                theta +=delta

                if(ego_y-last_ego_y>=0):
                    angle = 90-theta*180/np.pi
                else:
                    angle = 90-theta*180/np.pi
                # if self.done != True and self.__ego_vehicle_LC_completed() !=True :
                if(traci.vehicle.getLaneIndex('ego')==2 and traci.vehicle.getLateralLanePosition('ego')>=0.5 and action ==5):
                    print('stop moveToXY')
                    self.done = True
                else:
                    traci.vehicle.moveToXY('ego',edgeID,lane,x_next,y_next,angle,keepRoute,matchThreshold)  
        if rlEnv.final_x < ego_x-rlEnv.initial_x:
            rlEnv.Left_action = False
        else:
            rlEnv.Left_action = True
        # print('rlEnv.final_x(goal distance): ',rlEnv.final_x)
        # print('ego_x-rlEnv.initial_x(driven distance): ',ego_x-rlEnv.initial_x)

    def LC_Right(self, action,v_target,c_target):
        vehs = traci.vehicle.getIDList()
        if action == 2 or action == 4:
            action = 6
        if('ego' in vehs):  
            tau = 0.74
            v_max = 22.22 #선두 차량 속도
            c0 = 1.98 
            LLP = traci.vehicle.getLateralLanePosition('ego') #LateralLanePosition
            if action == 6:
                traci.vehicle.setSpeedMode('ego',32)
                traci.vehicle.setLaneChangeMode('ego',0)

                edgeID = 'E5'
                if traci.vehicle.getLaneIndex('ego') ==2 and LLP >=0:
                    lane =2 
                elif traci.vehicle.getLaneIndex('ego') ==2 and LLP <0:
                    lane =1
                elif traci.vehicle.getLaneIndex('ego') ==1 and LLP >=0:
                    lane =1
                elif traci.vehicle.getLaneIndex('ego') ==1 and LLP <0:
                    lane =0
                elif traci.vehicle.getLaneIndex('ego') ==0 and LLP >=0:
                    lane =0
                elif traci.vehicle.getLaneIndex('ego') ==0 and LLP <0:
                    lane =0
                    print("wrong Right LC")
                else:
                    lane =2


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
                ego_v_y = self.__get_v_y('ego')
                ego_a_y = self.__get_a_y('ego')

                xy ,ids = self.__LC_goal_space_posistion(action)
                rlEnv.last_target_space = xy
                rlEnv.last_vehicles = ids
                target_x, target_y = xy[0],xy[1]
                rear_v_id,forward_v_id = ids

                # if ego_y+ego_v_y*self.step_length <= target_y:
                #     LC_success = True
                # else:
                #     LC_success = False
                if ego_x-rlEnv.initial_x>=rlEnv.final_x:
                    path_done = True
                else:
                    path_done = False

                if path_done == True and traci.vehicle.getLaneIndex('ego') == 1 and LLP <=0.4 and LLP>=0 and traci.vehicle.getAngle('ego')<=90.1:
                    print("---------------------------------------LC_succeed")
                    print("LLP: ",LLP)
                    rlEnv.initial_x = ego_x
                    rlEnv.initial_y = ego_y
                    target_y = -8
                    rlEnv.theta_i = (90-traci.vehicle.getAngle('ego'))*np.pi/180

                    left_vehicles=[] #[|ego_x_pos - x_pos|, id]
                    right_vehicles=[] #[|ego_x_pos - x_pos|, id]
                    left_near_vehicles=[] #[x_pos]
                    right_near_vehicles =[] #[x_pos]      
                    vehs = traci.vehicle.getIDList()
                    if 'ego' in vehs:
                        id = 'ego'                        
                        traci.vehicle.subscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 200.0, [tc.VAR_POSITION])
                        for v_id in traci.vehicle.getContextSubscriptionResults(str(id)):
                            if(traci.vehicle.getLaneIndex(v_id)==0) and not (v_id=='ego'):
                                right_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),traci.vehicle.getPosition(v_id)[0],v_id])
                        right_vehicles.sort() 
                        for i in range(2):
                            right_near_vehicles.append([right_vehicles[:3][i][1],right_vehicles[:3][i][2]])# 1: right vehicle position 2: right vehicle id
                        right_near_vehicles.sort()
                        right_target_space = [(right_near_vehicles[1][0]+right_near_vehicles[0][0])/2,-8]
                        vehicles = [right_near_vehicles[1][1],right_near_vehicles[0][1]]
                    forward_v_id,rear_v_id = vehicles
                    LC_succeed = True                    
                else:
                    LC_succeed = False
    
                if rlEnv.Right_action == False:
                    # print('rlEnv.Right_action == False')
                    t = 0
                    rlEnv.t_LC_start = traci.simulation.getTime()
                    rlEnv.initial_x = ego_x
                    rlEnv.initial_y = ego_y
                    rlEnv.initial_target_y = target_y
                    rlEnv.theta_i = (90-traci.vehicle.getAngle('ego'))*np.pi/180
                    rlEnv.coefficient,rlEnv.final_x,rlEnv.final_y =self.LC_cubic_polynomial_path(rlEnv.initial_x,rlEnv.initial_y,target_x,target_y,rear_v_id,forward_v_id,rlEnv.theta_i,action)
                else:
                    # print('rlEnv.Right_action == True')
                    t= traci.simulation.getTime()-rlEnv.t_LC_start                        
                    if path_done:
                        rlEnv.coefficient = [target_y-ego_y,0,0,0]
                    if LC_succeed:                                                     
                        rlEnv.coefficient,rlEnv.final_x,rlEnv.final_y =self.LC_cubic_polynomial_path(rlEnv.initial_x,rlEnv.initial_y,target_x,target_y,rear_v_id,forward_v_id,rlEnv.theta_i,action)
                    
                
                x_local = ego_x - rlEnv.initial_x
                x = ego_x -rlEnv.initial_x
                y = rlEnv.coefficient[2]*x**2+rlEnv.coefficient[3]*x**3
                y_prime = 2*rlEnv.coefficient[2]*x+3*rlEnv.coefficient[3]*x**2
                y_pprime = 2*rlEnv.coefficient[2]+6*rlEnv.coefficient[3]*x
                theta_n = np.arctan(y_prime)


                delta = self.stanley(rlEnv.coefficient,traci.vehicle.getAngle('ego'),x_local,ego_v)
                # delta =0
                theta_n += delta
                u = ego_v/np.cos(theta_n)
                ego_a_y_des = u**2*self.K(y_prime,y_pprime)+u**2*np.tan(delta)
                ego_a_y_desire = self.__set_a_y_possible('ego',ego_a_y_des)
                ego_v_y_next = ego_v_y +ego_a_y_desire*self.step_length
                y_next = ego_y+ego_v_y_next*self.step_length

                # if self.__ACC_target_id('ego',action):
                #     v_preceding = traci.vehicle.getSpeed(self.__ACC_target_id('ego',action))
                #     c_front = traci.vehicle.getPosition(self.__ACC_target_id('ego',action))[0]-traci.vehicle.getPosition('ego')[0]
                # else:
                #     v_preceding =22.22
                #     c_front = 2

                c_desire = 0
                ego_a_x_desire =self.__set_a_x_possible('ego',self.__ego_acc(ego_v,v_target,c_desire,c_target)) 
                ego_v_x_next = ego_v +ego_a_x_desire*self.step_length                               
                x_next = ego_x+ego_v_x_next*self.step_length #ACC

                control = [traci.simulation.getTime(),self.__ego_acc(ego_v,v_target,c_desire,c_target),ego_a_y_des]
                rlEnv.ego_control.append(control)
                theta = np.arctan2(ego_y-last_ego_y,ego_x-last_ego_x)

                if(ego_y-last_ego_y>=0):
                    angle = 90-theta*180/np.pi
                else:
                    angle = 90-theta*180/np.pi

                # if self.done != True and self.__ego_vehicle_LC_completed() !=True:
                if(traci.vehicle.getLaneIndex('ego')==0 and traci.vehicle.getLateralLanePosition('ego')<=-0.5 and action ==6):
                    print('stop moveToXY')
                    self.done = True
                else:
                    # print(':::::::::::::::::::::::::::::::::::movetoXY')
                    # traci.vehicle.moveToXY('ego',edgeID,lane,x_next,y_next,angle,keepRoute,matchThreshold)
                    traci.vehicle.moveToXY('ego',edgeID,lane,x_next,y_next,angle,keepRoute,matchThreshold)
        if rlEnv.final_x < ego_x-rlEnv.initial_x:
            rlEnv.Right_action = False
        else:            
            rlEnv.Right_action = True
        # print('rlEnv.final_x(goal distance): ',rlEnv.final_x)
        # print('ego_x-rlEnv.initial_x(driven distance): ',ego_x-rlEnv.initial_x)
    def other_mode_reset(self,action):
        for i in range(6):
            if i != action:
                rlEnv.mode[i] = 0  

    def Target_point_visualization(self,TargetID,TargetPoint):
        epsilon = 0.4
        x,y = TargetPoint
        # print('TargetPoint: ',x,y)
        Point = [
                (x-epsilon, y-epsilon),
                (x-epsilon, y+epsilon),
                (x+epsilon, y+epsilon),
                (x+epsilon, y-epsilon)
            ]
        # 모든 다각형 ID를 가져옵니다.
        polygon_ids = traci.polygon.getIDList()
        new_polygon_id = TargetID + '_target_point'
        # 확인하려는 다각형 ID가 이미 존재하는지 확인합니다.
        # if new_polygon_id in polygon_ids:
        #     print(f"ID '{new_polygon_id}' already exists!")
        # else:
        #     print(f"ID '{new_polygon_id}' is unique and can be used.")
        for polygon_id in polygon_ids:
            # print(polygon_id)
            # print(polygon_id.split('_')[-1])
            if polygon_id.split('_')[-1] == 'point':
                traci.polygon.remove(polygon_id, 0) 

        # if new_polygon_id in polygon_ids:
        #     traci.polygon.remove(TargetID + '_target_point', 0)              
        traci.polygon.add(TargetID + '_target_point',Point,color=(0, 0, 255, 255), fill=True, layer=5)

    def SD_visualization(self,TargetID,SD_end,SD_direction,mode): #SD_direction: target car's back or front
        epsilon = 0.2
        vehicle_width = 1.8 +epsilon
        vehicle_length = 5
        if TargetID == 'accel.rear':
            vehicle_width = 2.4 +epsilon
            vehicle_length = 12
        x,y = self.vehicle_position(TargetID)
        polygon_ids = traci.polygon.getIDList()
        # 기존 폴리곤 제거
        # for polygon_id in polygon_ids:
        #     if polygon_id.split('_')[-1] == 'front' or  polygon_id.split('_')[-1] == 'back':
        #         traci.polygon.remove(polygon_id, 0) 

        if SD_direction == 'front':
            SD_front = [
                (x, y - vehicle_width / 2 ),
                (x, y + vehicle_width / 2),
                (x+SD_end, y + vehicle_width / 2),
                (x+SD_end, y - vehicle_width / 2)
            ]
            # 기존 폴리곤 제거

            for polygon_id in polygon_ids:
                if polygon_id == TargetID + '_SD_'+str(mode)+'_front':
                    traci.polygon.remove(TargetID + '_SD_'+str(mode)+'_front', 0)
            # if traci.simulation.getTime() >0.03:
            #     traci.polygon.remove(TargetID + '_SD_'+str(mode)+'_front', 0)
            # 새로운 폴리곤 추가 - 주황색으로 설정 
            traci.polygon.add(TargetID + '_SD_'+str(mode)+'_front',SD_front,color=(255, 165, 0, 255), fill=True, layer=4)
        if SD_direction == 'back':
            SD_back = [
                (x, y - vehicle_width / 2 ),
                (x, y + vehicle_width / 2),
                (x-SD_end, y + vehicle_width / 2),
                (x-SD_end, y - vehicle_width / 2)
            ]
            # 기존 폴리곤 제거
            for polygon_id in polygon_ids:
                if polygon_id == TargetID + '_SD_'+str(mode)+'_back':
                    traci.polygon.remove(TargetID + '_SD_'+str(mode)+'_back', 0)
            # if traci.simulation.getTime() >0.03:
            #     traci.polygon.remove(TargetID + '_SD_'+str(mode)+'_back', 0)

            # 새로운 폴리곤 추가 - 주황색으로 설정        
            traci.polygon.add(TargetID + '_SD_'+str(mode)+'_back',SD_back,color=(255, 165, 0, 255), fill=True, layer=4)
    def reset_left_right_target_vehicle(self):
        if traci.simulation.getTime() >= 0.03: # 기준이되는 옆차로 차량을 시뮬레이션 시작할 때 설정한다.(-> 후방 차량의 추돌 위험이 감지 될 때 마다 기준이 되는 옆차로 차량의 위치를 갱신하는게 맞다.)
            left_vehicles = []
            right_vehicles = []
            traci.vehicle.subscribeContext('ego', tc.CMD_GET_VEHICLE_VARIABLE, 120.0, [tc.VAR_POSITION]) 
            for v_id in traci.vehicle.getContextSubscriptionResults('ego'):
                if(traci.vehicle.getLaneIndex(v_id)==2) and not (v_id=='ego'):
                    left_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),v_id])
                if(traci.vehicle.getLaneIndex(v_id)==0) and not (v_id=='ego'):
                    right_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),v_id])
            left_vehicles.sort() 
            right_vehicles.sort() 
            rlEnv.Target_left_car = left_vehicles[0][1]
            rlEnv.Target_right_car = right_vehicles[0][1]
            print('New Target_left_car: ',rlEnv.Target_left_car)
            print('New Target_right_car: ',rlEnv.Target_right_car)
            traci.vehicle.unsubscribeContext('ego', tc.CMD_GET_VEHICLE_VARIABLE, 120.0)

        if traci.vehicle.getLeader(rlEnv.Target_left_car)[0] == 'ego' and traci.vehicle.getLaneIndex('ego')==2:
            left_leader = traci.vehicle.getLeader('ego')[0]
        elif traci.vehicle.getLeader(rlEnv.Target_left_car)[0] == 'ego' and traci.vehicle.getLaneIndex('ego')==1:
            left_leader = 'car.left'+str(int(rlEnv.Target_left_car[-1])+1)
        else:
            left_leader = traci.vehicle.getLeader(rlEnv.Target_left_car)[0]
        if traci.vehicle.getFollower(rlEnv.Target_left_car)[0] =='ego' and traci.vehicle.getLaneIndex('ego')==2:
            left_follower = traci.vehicle.getFollower('ego')[0]
        elif traci.vehicle.getFollower(rlEnv.Target_left_car)[0] =='ego' and traci.vehicle.getLaneIndex('ego')==1:
            left_follower = 'car.left'+str(int(rlEnv.Target_left_car[-1])-1)
        else:
            left_follower = traci.vehicle.getFollower(rlEnv.Target_left_car)[0]
        
        if traci.vehicle.getLeader(rlEnv.Target_right_car)[0] == 'ego' and traci.vehicle.getLaneIndex('ego')==0:
            right_leader = traci.vehicle.getLeader('ego')[0]
        elif traci.vehicle.getLeader(rlEnv.Target_right_car)[0] == 'ego' and traci.vehicle.getLaneIndex('ego')==1:
            right_leader = 'car.right'+str(int(rlEnv.Target_right_car[-1])+1)
        else:
            right_leader = traci.vehicle.getLeader(rlEnv.Target_right_car)[0]
        if traci.vehicle.getFollower(rlEnv.Target_right_car)[0] =='ego' and traci.vehicle.getLaneIndex('ego')==0:
            right_follower = traci.vehicle.getFollower('ego')[0]
        elif traci.vehicle.getFollower(rlEnv.Target_right_car)[0] =='ego' and traci.vehicle.getLaneIndex('ego')==1:
            right_follower = 'car.right'+str(int(rlEnv.Target_right_car[-1])-1)
        else:
            right_follower = traci.vehicle.getFollower(rlEnv.Target_right_car)[0]

        return left_leader, left_follower, right_leader, right_follower


    def step(self, action): #action-> 0: LK_const_vel, 1: LK_accel, 2: LK_decel
        # print('final_y: ', rlEnv.final_y)
        
        if traci.vehicle.getPosition('accel.rear')[0] >traci.vehicle.getPosition('ego')[0]+30:
            self.done = True
        else:
            self.done =False

        if traci.vehicle.getPosition('ego')[1] >0 or traci.vehicle.getPosition('ego')[1] <-9.6:
            self.done =True
        else:
            self.done =False



        
        # rlEnv.vehicles_tau[1] =2.27 ##### RL 검증 space 3
        # rlEnv.vehicles_tau[4] =2.27 #space1
        # rlEnv.vehicles_tau[3] =2.27 #space 4
        # rlEnv.vehicles_tau[11] =2.27 #space 2
        # vehs = traci.vehicle.getIDList()
        traci.simulationStep()
        
        # if 'car.forward' in vehs:
        #     traci.simulationStep()
        # else:
        #     print('forward end1')            
        #     self.done=True
        vehs = traci.vehicle.getIDList()
        if traci.simulation.getTime() > 0.03 and ('accel.rear' not in vehs):
            # x = traci.vehicle.getPosition('ego')[0]
            # rear_pos = x-100
            # rear_initial_v = traci.vehicle.getSpeed('ego')            
            # traci.vehicle.add(vehID='accel.rear',routeID='route0',typeID="accel_truck",depart='now',departLane='1',departPos=str(rear_pos), departSpeed=str(rear_initial_v), arrivalLane='current', arrivalPos='max', arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0, personNumber=0)
            self.done=True
            # traci.close()
        if traci.simulation.getTime() > 0.03 and ('car.forward' not in vehs):
            # x = traci.vehicle.getPosition('ego')[0]
            # forward_pos = x+100
            # forward_initial_v = traci.vehicle.getSpeed('ego')
            # traci.vehicle.add(vehID='car.forward',routeID='route0',typeID="car",depart='now',departLane='1',departPos=str(forward_pos), departSpeed=str(forward_initial_v), arrivalLane='current', arrivalPos='max', arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0, personNumber=0)
            self.done=True
            # sys.stdout.flush()
        # print('rlEnv.Target_right_car: ',rlEnv.Target_right_car)
        # print('rlEnv.Target_left_car: ',rlEnv.Target_left_car)
        # print('time: ',traci.simulation.getTime())
        if traci.simulation.getTime() == 0.03: # 기준이되는 옆차로 차량을 시뮬레이션 시작할 때 설정한다.(-> 후방 차량의 추돌 위험이 감지 될 때 마다 기준이 되는 옆차로 차량의 위치를 갱신하는게 맞다.)
            left_vehicles = []
            right_vehicles = []
            traci.vehicle.subscribeContext('ego', tc.CMD_GET_VEHICLE_VARIABLE, 120.0, [tc.VAR_POSITION]) 
            for v_id in traci.vehicle.getContextSubscriptionResults('ego'):
                if(traci.vehicle.getLaneIndex(v_id)==2) and not (v_id=='ego'):
                    left_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),v_id])
                if(traci.vehicle.getLaneIndex(v_id)==0) and not (v_id=='ego'):
                    right_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),v_id])
            left_vehicles.sort() 
            right_vehicles.sort() 
            rlEnv.Target_left_car = left_vehicles[0][1]
            rlEnv.Target_right_car = right_vehicles[0][1]
            print('Initial Target_left_car: ',rlEnv.Target_left_car)
            print('Initial Target_right_car: ',rlEnv.Target_right_car)
            traci.vehicle.unsubscribeContext('ego', tc.CMD_GET_VEHICLE_VARIABLE, 120.0)

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
            if self.gui_on:
                traci.gui.trackVehicle(traci.gui.DEFAULT_VIEW, self.egoID)
            
        if len(rlEnv.ego) !=0:
            ego_v_last = rlEnv.ego[-1][4]
        else:
            ego_v_last = traci.vehicle.getSpeed('ego')

        left_vehicles = []
        right_vehicles = []
        left_near_vehicles = []
        right_near_vehicles = []
        
        if('ego' in vehs and 'car.forward' in vehs and 'accel.rear' in vehs): 
            LLP = traci.vehicle.getLateralLanePosition('ego')
            # print('LLP: ',LLP)
            traci.vehicle.setLaneChangeMode('ego',0) 
            rear_relative_distance = self.vehicle_position('ego')[0] - self.vehicle_position('accel.rear')[0]
            rear_relative_velocity = traci.vehicle.getSpeed('accel.rear')-traci.vehicle.getSpeed('ego')
            TTC= rear_relative_distance/rear_relative_velocity
            # print('TTC: ',TTC)

            ### LK mode
            if traci.vehicle.getLeader('ego') is not None:
                Leader_id,x_forward = traci.vehicle.getLeader('ego')
            else:
                print('cannot detect leader car in sumo')
                Leader_id ='car.forward'
                x_forward = self.vehicle_position(Leader_id)[0]-self.vehicle_position('ego')[0]
            Forward_id = 'car.forward'
            if type(traci.vehicle.getFollower('ego')) is not None and traci.vehicle.getFollower('ego')[0] != '':
                follower_id, x_back = traci.vehicle.getFollower('ego') 
            else:
                follower_id = 'ego' 
                x_back = 0
            Anti_RVC_on = False
            c_LK = 2 #min clearance for lane keeping
            time_gap_LC_prepare = 0.74
            time_gap_LK = 1.36  #95% percentile time_gap (most safe) //2.27 -> safe 1.36s -> 중간값   0.74s -> 5% persentile 가장 위험.
            # time_gap_LK = 1.2 #95% percentile time_gap (most safe) //2.27 -> safe 1.36s -> 중간값   0.74s -> 5% persentile 가장 위험.
            v_p = traci.vehicle.getSpeed(Leader_id)
            v_forward = traci.vehicle.getSpeed('car.forward')
            des_clearance = c_LK + time_gap_LK*v_p #앞차 속도에 따른 앞차와의 ACC 안전거리 (safe)
            des_clearance2 = c_LK + time_gap_LC_prepare*v_p #앞차 속도에 따른 앞차와의 ACC 안전거리 (aggressive)
            SD_LK_forward = c_LK + time_gap_LK*v_forward
            if rear_relative_distance<=50:
                Anti_RVC_on = True
            else:
                Anti_RVC_on = False

            # traci.vehicle.subscribeContext('ego', tc.CMD_GET_VEHICLE_VARIABLE, 120.0, [tc.VAR_POSITION])
            # # print(id,' subscribeContext')
            # for v_id in traci.vehicle.getContextSubscriptionResults('ego'):
            #     if(traci.vehicle.getLaneIndex(v_id)==2) and not (v_id=='ego'):
            #         #left_vehicles.append([np.abs(self.vehicle_position('ego')[0]- self.vehicle_position(v_id)[0]),v_id]) -> id check
            #         left_vehicles.append([np.abs(self.vehicle_position('ego')[0]- self.vehicle_position(v_id)[0]),self.vehicle_position(v_id)[0],v_id])
            #     if(traci.vehicle.getLaneIndex(v_id)==0) and not (v_id=='ego'):
            #         right_vehicles.append([np.abs(self.vehicle_position('ego')[0]- self.vehicle_position(v_id)[0]),self.vehicle_position(v_id)[0],v_id])
            # traci.vehicle.unsubscribeContext('ego', tc.CMD_GET_VEHICLE_VARIABLE, 120.0)
            # if len(left_vehicles)<3:
            #     left_vehicles.append([100,0,'virtual_left_at_start_point'])
            #     left_vehicles.append([100,1000,'virtual_left_at_end_point'])
            # if len(right_vehicles)<3:
            #     right_vehicles.append([100,0,'virtual_right_at_start_point'])
            #     right_vehicles.append([100,1000,'virtual_right_at_end_point'])
            # left_vehicles.sort()
            # right_vehicles.sort()
            
            # # print('right_vehicles: ',right_vehicles)
            # for i in range(3): # getting near 3 vehicles x_pos each left and right
            #     left_near_vehicles.append([left_vehicles[:3][i][1],left_vehicles[:3][i][2]])                
            #     right_near_vehicles.append([right_vehicles[:3][i][1],right_vehicles[:3][i][2]])
                
            # left_near_vehicles.sort()
            # right_near_vehicles.sort()
                
            # print('left_near_vehicles: ',left_near_vehicles)
            # print('right_near_vehicles: ',right_near_vehicles)
            
            if rlEnv.Target_left_car in vehs and rlEnv.Target_right_car in vehs and 'ego' in vehs:
                if traci.vehicle.getLeader(rlEnv.Target_left_car)[0] == 'ego' and traci.vehicle.getLaneIndex('ego')==2:
                    left_leader = traci.vehicle.getLeader('ego')[0]
                elif traci.vehicle.getLeader(rlEnv.Target_left_car)[0] == 'ego' and traci.vehicle.getLaneIndex('ego')==1:
                    left_leader = 'car.left'+str(int(rlEnv.Target_left_car[-1])+1)
                else:
                    left_leader = traci.vehicle.getLeader(rlEnv.Target_left_car)[0]
                if traci.vehicle.getFollower(rlEnv.Target_left_car)[0] =='ego' and traci.vehicle.getLaneIndex('ego')==2:
                    left_follower = traci.vehicle.getFollower('ego')[0]
                elif traci.vehicle.getFollower(rlEnv.Target_left_car)[0] =='ego' and traci.vehicle.getLaneIndex('ego')==1:
                    left_follower = 'car.left'+str(int(rlEnv.Target_left_car[-1])-1)
                else:
                    left_follower = traci.vehicle.getFollower(rlEnv.Target_left_car)[0]
                
                if traci.vehicle.getLeader(rlEnv.Target_right_car)[0] == 'ego' and traci.vehicle.getLaneIndex('ego')==0:
                    right_leader = traci.vehicle.getLeader('ego')[0]
                elif traci.vehicle.getLeader(rlEnv.Target_right_car)[0] == 'ego' and traci.vehicle.getLaneIndex('ego')==1:
                    right_leader = 'car.right'+str(int(rlEnv.Target_right_car[-1])+1)
                else:
                    right_leader = traci.vehicle.getLeader(rlEnv.Target_right_car)[0]
                if traci.vehicle.getFollower(rlEnv.Target_right_car)[0] =='ego' and traci.vehicle.getLaneIndex('ego')==0:
                    right_follower = traci.vehicle.getFollower('ego')[0]
                elif traci.vehicle.getFollower(rlEnv.Target_right_car)[0] =='ego' and traci.vehicle.getLaneIndex('ego')==1:
                    right_follower = 'car.right'+str(int(rlEnv.Target_right_car[-1])-1)
                else:
                    right_follower = traci.vehicle.getFollower(rlEnv.Target_right_car)[0]
                # print('left_leader: ',left_leader)
                # print('right_leader: ',right_leader)
                # print('left_follower: ',left_follower)
                # print('right_follower: ',right_follower)
                # print('traci.vehicle.getLeader(rlEnv.Target_left_car)[0]: ',traci.vehicle.getLeader(rlEnv.Target_left_car)[0])
                # print('traci.vehicle.getLeader(rlEnv.Target_right_car)[0]: ',traci.vehicle.getLeader(rlEnv.Target_right_car)[0])
                # print('traci.vehicle.getFollower(rlEnv.Target_left_car)[0]: ',traci.vehicle.getFollower(rlEnv.Target_left_car)[0])
                # print('traci.vehicle.getFollower(rlEnv.Target_right_car)[0]: ',traci.vehicle.getFollower(rlEnv.Target_right_car)[0])
                # print('traci.vehicle.getLeader(ego)[0]: ',traci.vehicle.getLeader('ego')[0])
                # print('traci.vehicle.getFollower(ego)[0]: ',traci.vehicle.getFollower('ego')[0])


                # print('self.SD_LC_with_respect_to_target_space(ego,1): ',self.SD_LC_with_respect_to_target_space('ego',1))
                # print('self.SD_LC_with_respect_to_target_space(ego,2): ',self.SD_LC_with_respect_to_target_space('ego',2))
                # print('self.SD_LC_with_respect_to_target_space(ego,3): ',self.SD_LC_with_respect_to_target_space('ego',3))
                # print('self.SD_LC_with_respect_to_target_space(ego,4): ',self.SD_LC_with_respect_to_target_space('ego',4))
                ego_x = self.vehicle_position('ego')[0]
                s1_back = (self.vehicle_position(rlEnv.Target_left_car)[0] + self.SD_LC_with_respect_to_target_space('ego',1,'back')) #자차와 왼쪽 차로 옆차량 SD_LC 위치
                s1_front = (self.vehicle_position(left_leader)[0] - self.SD_LC_with_respect_to_target_space('ego',1,'front')) #자차와 왼쪽 차로 옆차량의 앞차량 SD_LC 위치(옆차로 앞에 빈공간 앞쪽 SD_LC 위치)
                s2_back = (self.vehicle_position(rlEnv.Target_right_car)[0] + self.SD_LC_with_respect_to_target_space('ego',2,'back'))   
                s2_front = (self.vehicle_position(right_leader)[0] - self.SD_LC_with_respect_to_target_space('ego',2,'front')) 
                s3_front = (self.vehicle_position(rlEnv.Target_left_car)[0] - self.SD_LC_with_respect_to_target_space('ego',3,'front'))           
                s3_back = (self.vehicle_position(left_follower)[0] + self.SD_LC_with_respect_to_target_space('ego',3,'back'))           
                s4_front = (self.vehicle_position(rlEnv.Target_right_car)[0] - self.SD_LC_with_respect_to_target_space('ego',4,'front')) 
                s4_back = (self.vehicle_position(right_follower)[0] + self.SD_LC_with_respect_to_target_space('ego',4,'back')) 
                space1 = (self.vehicle_position(rlEnv.Target_left_car)[0] + self.vehicle_position(left_leader)[0])/2
                space2 = (self.vehicle_position(rlEnv.Target_right_car)[0] + self.vehicle_position(right_leader)[0])/2
                space3 = (self.vehicle_position(rlEnv.Target_left_car)[0] + self.vehicle_position(left_follower)[0])/2
                space4 = (self.vehicle_position(rlEnv.Target_right_car)[0] + self.vehicle_position(right_follower)[0])/2
            else:
                s1_back= rlEnv.S[1]
                s1_front = rlEnv.S[5]
                s2_back= rlEnv.S[2]
                s2_front= rlEnv.S[6]
                s3_front= rlEnv.S[3]
                s3_back= rlEnv.S[7]
                s4_front= rlEnv.S[4]
                s4_back= rlEnv.S[8]
                space1 = rlEnv.last_Space1
                space2 = rlEnv.last_Space2
                space3 = rlEnv.last_Space3
                space4 = rlEnv.last_Space4
                left_leader, left_follower, right_leader, right_follower = self.reset_left_right_target_vehicle()

                

            # ss1 = (left_near_vehicles[1][0]+left_near_vehicles[2][0])/2 - self.vehicle_position('ego')[0]  #왼쪽 차로 가장 가까운 전방 빈공간의 중점 까지 x축 방향 거리
            # ss2 = (right_near_vehicles[1][0]+right_near_vehicles[2][0])/2 - self.vehicle_position('ego')[0] #오른쪽 차로 가장 가까운 전방 빈공간의 중점 까지 x축 방향 거리
            # ss3 = (left_near_vehicles[0][0]+left_near_vehicles[1][0])/2 - self.vehicle_position('ego')[0] #왼쪽 차로 가장 가까운 후방 빈공간의 중점 까지 x축 방향 거리
            # ss4 = (right_near_vehicles[0][0]+right_near_vehicles[1][0])/2 - self.vehicle_position('ego')[0] #오른쪽 차로 가장 가까운 후방 빈공간의 중점 까지 x축 방향 거리
            # sspace1 = (left_near_vehicles[1][0]+left_near_vehicles[2][0])/2
            # sspace2 = (right_near_vehicles[1][0]+right_near_vehicles[2][0])/2
            # sspace3 = (left_near_vehicles[0][0]+left_near_vehicles[1][0])/2
            # sspace4 = (right_near_vehicles[0][0]+right_near_vehicles[1][0])/2
            # print('s1,s2,s3,s4: ',s1,' ',s2,' ',s3,' ',s4)
            # print('ss1,ss2,ss3,ss4: ',ss1,' ',ss2,' ',ss3,' ',ss4)
            # print('space1, space2, space3, space4: ',space1,' ',space2,' ',space3,' ',space4)
            # print('sspace1, sspace2, sspace3, sspace4: ',sspace1,' ',sspace2,' ',sspace3,' ',sspace4)
            ego_x = self.vehicle_position('ego')[0]
            v_ego = traci.vehicle.getSpeed('ego')
            accel_weight = 1
            c_des = -1.5
            c_des2 = 3.5
            epsilon = 0.5
            v_road_max = 23
            SD_LK = des_clearance
            alpha = (x_forward - SD_LK)/x_forward
            
            ########## visualize SD ###########
            if self.gui_on:
                self.SD_visualization('car.forward',SD_LK_forward,'back','LK')
                self.SD_visualization(rlEnv.Target_left_car,self.SD_LC_with_respect_to_target_space('ego',1,'back'),'front','LC')
                self.SD_visualization(left_leader,self.SD_LC_with_respect_to_target_space('ego',1,'front'),'back','LC')
                self.SD_visualization(rlEnv.Target_right_car,self.SD_LC_with_respect_to_target_space('ego',2,'back'),'front','LC')
                self.SD_visualization(right_leader,self.SD_LC_with_respect_to_target_space('ego',2,'front'),'back','LC')
                self.SD_visualization(left_follower,self.SD_LC_with_respect_to_target_space('ego',3,'back'),'front','LC')
                self.SD_visualization(rlEnv.Target_left_car,self.SD_LC_with_respect_to_target_space('ego',3,'front'),'back','LC')
                self.SD_visualization(right_follower,self.SD_LC_with_respect_to_target_space('ego',4,'back'),'front','LC')
                self.SD_visualization(rlEnv.Target_right_car,self.SD_LC_with_respect_to_target_space('ego',4,'front'),'back','LC')

            ###################################

            if action == 0:
                if self.gui_on:
                    self.Target_point_visualization('ego',[0,0])
                self.other_mode_reset(action)  
                if Anti_RVC_on == True: #LK_mode
                    # print('Anti_RVC_on')
                    if des_clearance<=x_forward:
                        speed = max(traci.vehicle.getSpeed(follower_id),v_road_max)
                    else:
                        speed = (traci.vehicle.getSpeed(follower_id)*(x_forward) + traci.vehicle.getSpeed(Leader_id)*(2*x_back))/(x_forward+2*x_back)
                    # self.__set_ego_v('ego',speed)
                    control = [traci.simulation.getTime(),self.__set_ego_v_movetoXY('ego',speed)[0],self.__set_ego_v_movetoXY('ego',speed)[1]]
                else:
                    # print('Anti_RVC_off')
                    # self.__set_ego_v('ego',ego_v_last)
                    if x_forward <= SD_LK:
                        speed = min(traci.vehicle.getSpeed(Leader_id),v_road_max)
                    else:
                        speed = min(alpha*v_road_max+(1-alpha)*traci.vehicle.getSpeed(Leader_id),v_road_max) 
                    control = [traci.simulation.getTime(),self.__set_ego_v_movetoXY('ego',speed)[0],self.__set_ego_v_movetoXY('ego',speed)[1]]
                rlEnv.ego_control.append(control)
            elif action == 1:               
                self.other_mode_reset(action)
                if rlEnv.last_action[-1][1] != 1:
                    rlEnv.Left_action = False 


                if ego_x >= s1_back+epsilon and ego_x <= s1_front-epsilon: # LC 모드 진입 조건
                    rlEnv.mode[1] = 2 #LC mode

                elif (ego_x< s1_back+epsilon or ego_x>s1_front-epsilon) and rlEnv.mode[1] !=2:
                    rlEnv.mode[1] = 1
                
                if ego_x < s1_back or ego_x > s1_front: # LC 모드 탈출,(preLC 모드 재진입조건)
                    # print(':::::::::::::::::::::::preLC 조건')
                    rlEnv.mode[1] = 1 #preLC mode

                if self.vehicle_position(Forward_id)[0] - SD_LK_forward <= s1_back and traci.vehicle.getLaneIndex('ego')==1: 
                    # print(':::::::::::::::::::::::LC 탈출 조건')
                    rlEnv.mode[1] = 1

                if rlEnv.mode[1] == 1:                   
                    if ego_x > s1_front -epsilon:
                        if self.vehicle_position(Forward_id)[0] - SD_LK_forward <= s1_front - epsilon:                        
                            v_t = traci.vehicle.getSpeed(Forward_id)
                            c_t = self.vehicle_position(Forward_id)[0] - SD_LK_forward
                            if self.gui_on:
                                self.Target_point_visualization(Forward_id,[self.vehicle_position(Forward_id)[0]- SD_LK_forward,self.vehicle_position(Forward_id)[1]])
                        else:
                            # v_t = max(min(traci.vehicle.getSpeed(left_leader),traci.vehicle.getSpeed(Forward_id)),traci.vehicle.getSpeed(rlEnv.Target_left_car))
                            v_t = traci.vehicle.getSpeed(left_leader)
                            c_t = s1_front - epsilon
                            if self.gui_on:
                                self.Target_point_visualization(left_leader,[s1_front - epsilon,self.vehicle_position(left_leader)[1]])
                        rlEnv.Target1.append([traci.simulation.getTime(),1,s1_front,space1,v_t]) 
                        # print([0,1,0,0,0,0])
                        a_y_des = self.__set_ego_a_movetoXY2('ego',self.__ego_a_desire2(v_ego,v_t,c_des2,c_t-ego_x,1),action) #desire clearance =3는 3값 만큼 타깃 지점에 가까워 져도 덜 감속함. 차량의 움직임 방향으로 3m 더 멀리 타깃으로 지정                      
                        control = [traci.simulation.getTime(),self.__ego_a_desire2(v_ego,v_t,c_des2,c_t-ego_x,1),a_y_des]
                        rlEnv.Left_action = False     
                        rlEnv.ego_control.append(control)
                    else:
                        if self.vehicle_position(Forward_id)[0] - SD_LK_forward <= s1_back+epsilon:
                            c_des = 0
                            v_t = traci.vehicle.getSpeed(Forward_id)
                            c_t = self.vehicle_position(Forward_id)[0] - SD_LK_forward
                            if self.gui_on:
                                self.Target_point_visualization(Forward_id,[self.vehicle_position(Forward_id)[0] - SD_LK_forward,self.vehicle_position(Forward_id)[1]])
                        else:
                            v_t = max(min(traci.vehicle.getSpeed(left_leader),traci.vehicle.getSpeed(Forward_id)),traci.vehicle.getSpeed(rlEnv.Target_left_car))
                            c_t = s1_back+epsilon
                            if self.gui_on:
                                self.Target_point_visualization(rlEnv.Target_left_car,[s1_back+epsilon,self.vehicle_position(rlEnv.Target_left_car)[1]])
                        rlEnv.Target1.append([traci.simulation.getTime(),1,s1_back,space1,v_t]) 
                        # print([0,1,0,0,0,0])
                        a_y_des = self.__set_ego_a_movetoXY2('ego',self.__ego_a_desire(v_ego,v_t,c_des,c_t-ego_x,1),action) #desire clearance = 2는 2값 만큼 타깃 지점에 가까워 져도 덜 감속함. 차량의 움직임 방향으로 2m 더 멀리 타깃으로 지정                    
                        control = [traci.simulation.getTime(),self.__ego_a_desire(v_ego,v_t,c_des,c_t-ego_x,1),a_y_des]
                        rlEnv.Left_action = False     
                        rlEnv.ego_control.append(control)                        

                if rlEnv.mode[1] == 2:
                    # print([0,2,0,0,0,0])
                    if (traci.vehicle.getLaneIndex('ego') == 1):
                        if(self.vehicle_position('car.forward')[0]-ego_x<SD_LK_forward):
                            a = ego_x -(s1_back + epsilon)
                            b = ego_x - (self.vehicle_position('car.forward')[0]-SD_LK_forward)
                            v_t = (b*traci.vehicle.getSpeed(rlEnv.Target_left_car)+a*traci.vehicle.getSpeed('car.forward'))/(a+b)
                            c_t = s1_back+epsilon+a/(a+b)*(self.vehicle_position('car.forward')[0]-SD_LK_forward -(s1_back+epsilon))- ego_x
                            if self.gui_on:
                                self.Target_point_visualization('weighted_point_with_Forward',[s1_back+epsilon+a/(a+b)*(self.vehicle_position('car.forward')[0]-SD_LK_forward -(s1_back+epsilon)),self.vehicle_position(rlEnv.Target_left_car)[1]])
                        else:
                            a = ego_x - (s1_back+epsilon)
                            b = s1_front-epsilon - ego_x
                            v_t = (b*traci.vehicle.getSpeed(rlEnv.Target_left_car)+a*traci.vehicle.getSpeed(left_leader))/(a+b)
                            c_t = s1_front - epsilon - ego_x
                            if self.gui_on:
                                self.Target_point_visualization(left_leader,[s1_front - epsilon,self.vehicle_position(left_leader)[1]])
                    else: # laneindex == 2 (TargetLane)
                        a = ego_x - (s1_back+epsilon)
                        b = s1_front-epsilon - ego_x
                        v_t = (b*traci.vehicle.getSpeed(rlEnv.Target_left_car)+a*traci.vehicle.getSpeed(left_leader))/(a+b)
                        c_t = s1_front - epsilon - ego_x
                        if self.gui_on:
                            self.Target_point_visualization(left_leader,[s1_front - epsilon,self.vehicle_position(left_leader)[1]])
                    self.LC_Left(action,v_t,c_t)
                  
            elif action == 2: 
                self.other_mode_reset(action)
                if rlEnv.last_action[-1][1] != 2:
                    rlEnv.Right_action = False 
                if ego_x >= s2_back+epsilon and ego_x<= s2_front-epsilon: #LC 모드 진입 조건
                    rlEnv.mode[2] =2 #LC mode
                elif (ego_x< s2_back+epsilon or ego_x>s2_front-epsilon) and rlEnv.mode[2] !=2:
                    rlEnv.mode[2] = 1
                if ego_x < s2_back or ego_x > s2_front: #LC 모드 탈출,(preLC 모드 재진입조건)
                    # print(':::::::::::::::::::::::preLC 조건')
                    rlEnv.mode[2] = 1 #preLC mode
                if self.vehicle_position(Forward_id)[0] - SD_LK_forward <= s2_back and traci.vehicle.getLaneIndex('ego')==1: 
                    # print(':::::::::::::::::::::::LC 탈출 조건')
                    rlEnv.mode[2] = 1
                if rlEnv.mode[2] == 1:  
                    if ego_x > s2_front -epsilon:
                        if self.vehicle_position(Forward_id)[0] - SD_LK_forward <= s2_front - epsilon:
                            v_t = traci.vehicle.getSpeed(Forward_id)
                            c_t = self.vehicle_position(Forward_id)[0] - SD_LK_forward
                            if self.gui_on:
                                self.Target_point_visualization(Forward_id,[self.vehicle_position(Forward_id)[0]- SD_LK_forward,self.vehicle_position(Forward_id)[1]])
                        else:
                            # v_t = max(min(traci.vehicle.getSpeed(right_leader),traci.vehicle.getSpeed(Forward_id)),traci.vehicle.getSpeed(rlEnv.Target_right_car))
                            v_t = traci.vehicle.getSpeed(right_leader)
                            c_t = s2_front - epsilon
                            if self.gui_on:
                                self.Target_point_visualization(right_leader,[s2_front - epsilon,self.vehicle_position(right_leader)[1]])
                        rlEnv.Target2.append([traci.simulation.getTime(),2,s2_front,space2,v_t]) 
                        # print([0,0,1,0,0,0]) 
                        a_y_des = self.__set_ego_a_movetoXY2('ego',self.__ego_a_desire2(v_ego,v_t,c_des2,c_t-ego_x,2),action) #desire clearance =3는 3값 만큼 타깃 지점에 가까워 져도 덜 감속함. 차량의 움직임 방향으로 3m 더 멀리 타깃으로 지정                      
                        control = [traci.simulation.getTime(),self.__ego_a_desire2(v_ego,v_t,c_des2,c_t-ego_x,2),a_y_des]
                        rlEnv.Right_action = False     
                        rlEnv.ego_control.append(control)
                    else:
                        if self.vehicle_position(Forward_id)[0] - SD_LK_forward <= s2_back+epsilon:
                            c_des = 0
                            v_t = traci.vehicle.getSpeed(Forward_id)
                            c_t = self.vehicle_position(Forward_id)[0] - SD_LK_forward
                            if self.gui_on:
                                self.Target_point_visualization(Forward_id,[self.vehicle_position(Forward_id)[0] - SD_LK_forward,self.vehicle_position(Forward_id)[1]])
                        else:
                            v_t = max(min(traci.vehicle.getSpeed(right_leader),traci.vehicle.getSpeed(Forward_id)),traci.vehicle.getSpeed(rlEnv.Target_right_car))
                            c_t = s2_back+epsilon
                            if self.gui_on:
                                self.Target_point_visualization(rlEnv.Target_right_car,[s2_back+epsilon,self.vehicle_position(rlEnv.Target_right_car)[1]])
                        rlEnv.Target2.append([traci.simulation.getTime(),2,s2_back,space2,v_t]) 
                        # print([0,0,1,0,0,0])
                        a_y_des = self.__set_ego_a_movetoXY2('ego',self.__ego_a_desire(v_ego,v_t,c_des,c_t-ego_x,2),action) #desire clearance = 2는 2값 만큼 타깃 지점에 가까워 져도 덜 감속함. 차량의 움직임 방향으로 2m 더 멀리 타깃으로 지정                    
                        control = [traci.simulation.getTime(),self.__ego_a_desire(v_ego,v_t,c_des,c_t-ego_x,2),a_y_des]
                        rlEnv.Right_action = False     
                        rlEnv.ego_control.append(control) 
                if rlEnv.mode[2] == 2:
                    # print([0,0,2,0,0,0])
                    if (traci.vehicle.getLaneIndex('ego') == 1):
                        if(self.vehicle_position('car.forward')[0]-ego_x<SD_LK_forward):
                            a = ego_x -(s2_back + epsilon)
                            b = ego_x - (self.vehicle_position('car.forward')[0]-SD_LK_forward)
                            v_t = (b*traci.vehicle.getSpeed(rlEnv.Target_right_car)+a*traci.vehicle.getSpeed('car.forward'))/(a+b)
                            c_t = s2_back+epsilon+a/(a+b)*(self.vehicle_position('car.forward')[0]-SD_LK_forward -(s2_back+epsilon))- ego_x
                            if self.gui_on:
                                self.Target_point_visualization('weighted_point_with_Forward',[s2_back+epsilon+a/(a+b)*(self.vehicle_position('car.forward')[0]-SD_LK_forward -(s2_back+epsilon)),self.vehicle_position(rlEnv.Target_right_car)[1]])
                        else:
                            a = ego_x - (s2_back+epsilon)
                            b = s2_front-epsilon - ego_x
                            v_t = (b*traci.vehicle.getSpeed(rlEnv.Target_right_car)+a*traci.vehicle.getSpeed(right_leader))/(a+b)
                            c_t = s2_front - epsilon - ego_x
                            if self.gui_on:
                                self.Target_point_visualization(right_leader,[s2_front - epsilon,self.vehicle_position(right_leader)[1]])
                    else: # laneindex == 0 (TargetLane)
                        a = ego_x - (s2_back+epsilon)
                        b = s2_front-epsilon - ego_x
                        v_t = (b*traci.vehicle.getSpeed(rlEnv.Target_right_car)+a*traci.vehicle.getSpeed(right_leader))/(a+b)
                        c_t = s2_front - epsilon - ego_x
                        if self.gui_on:
                            self.Target_point_visualization(right_leader,[s2_front - epsilon,self.vehicle_position(right_leader)[1]])
                    self.LC_Right(action,v_t,c_t)  

            elif action == 3:                
                self.other_mode_reset(action)   
                if rlEnv.last_action[-1][1] != 3:
                    rlEnv.Left_action = False
                if ego_x >= s3_back+epsilon and ego_x <= s3_front-epsilon: # LC 모드 진입 조건
                    rlEnv.mode[3] = 2 #LC mode
                elif (ego_x< s3_back+epsilon or ego_x>s3_front-epsilon) and rlEnv.mode[3] !=2:
                    rlEnv.mode[3] = 1
                if ego_x < s3_back or ego_x > s3_front: # LC 모드 탈출,(preLC 모드 재진입조건)
                    # print(':::::::::::::::::::::::preLC 조건')
                    rlEnv.mode[3] = 1 #preLC mode
                if self.vehicle_position(Forward_id)[0] - SD_LK_forward <= s3_back and traci.vehicle.getLaneIndex('ego')==1: 
                    # print(':::::::::::::::::::::::LC 탈출 조건')
                    rlEnv.mode[3] = 1

                if rlEnv.mode[3] == 1:                   
                    if ego_x > s3_front -epsilon:
                        if self.vehicle_position(Forward_id)[0] - SD_LK_forward <= s3_front - epsilon:
                            v_t = traci.vehicle.getSpeed(Forward_id)
                            c_t = self.vehicle_position(Forward_id)[0] - SD_LK_forward
                            if self.gui_on:
                                self.Target_point_visualization(Forward_id,[self.vehicle_position(Forward_id)[0]- SD_LK_forward,self.vehicle_position(Forward_id)[1]])
                        else:
                            v_t = traci.vehicle.getSpeed(rlEnv.Target_left_car)
                            c_t = s3_front - epsilon
                            if self.gui_on:
                                self.Target_point_visualization(left_leader,[s3_front - epsilon,self.vehicle_position(rlEnv.Target_left_car)[1]])
                        rlEnv.Target3.append([traci.simulation.getTime(),3,s3_front,space3,v_t]) 
                        # print([0,0,0,1,0,0])
                        a_y_des = self.__set_ego_a_movetoXY2('ego',self.__ego_a_desire2(v_ego,v_t,c_des2,c_t-ego_x,3),action) #desire clearance =3는 3값 만큼 타깃 지점에 가까워 져도 덜 감속함. 차량의 움직임 방향으로 3m 더 멀리 타깃으로 지정                      
                        control = [traci.simulation.getTime(),self.__ego_a_desire2(v_ego,v_t,c_des2,c_t-ego_x,3),a_y_des]
                        rlEnv.Left_action = False     
                        rlEnv.ego_control.append(control)
                    else:
                        if self.vehicle_position(Forward_id)[0] - SD_LK_forward <= s3_back+epsilon:
                            c_des = 0
                            v_t = traci.vehicle.getSpeed(Forward_id)
                            c_t = self.vehicle_position(Forward_id)[0] - SD_LK_forward
                            if self.gui_on:
                                self.Target_point_visualization(Forward_id,[self.vehicle_position(Forward_id)[0] - SD_LK_forward,self.vehicle_position(Forward_id)[1]])
                        else:
                            v_t = max(min(traci.vehicle.getSpeed(rlEnv.Target_left_car),traci.vehicle.getSpeed(Forward_id)),traci.vehicle.getSpeed(left_follower))
                            c_t = s3_back+epsilon
                            if self.gui_on:
                                self.Target_point_visualization(left_follower,[s3_back+epsilon,self.vehicle_position(left_follower)[1]])
                        rlEnv.Target3.append([traci.simulation.getTime(),3,s3_back,space3,v_t]) 
                        # print([0,0,0,1,0,0])
                        a_y_des = self.__set_ego_a_movetoXY2('ego',self.__ego_a_desire(v_ego,v_t,c_des,c_t-ego_x,3),action) #desire clearance = 2는 2값 만큼 타깃 지점에 가까워 져도 덜 감속함. 차량의 움직임 방향으로 2m 더 멀리 타깃으로 지정                    
                        control = [traci.simulation.getTime(),self.__ego_a_desire(v_ego,v_t,c_des,c_t-ego_x,3),a_y_des]
                        rlEnv.Left_action = False     
                        rlEnv.ego_control.append(control)
                if rlEnv.mode[3] == 2:
                    # print([0,0,0,2,0,0])
                    if (traci.vehicle.getLaneIndex('ego') == 1):
                        if(self.vehicle_position('car.forward')[0]-ego_x<SD_LK_forward):
                            a = ego_x -(s3_back + epsilon)
                            b = ego_x - (self.vehicle_position('car.forward')[0]-SD_LK_forward)
                            v_t = (b*traci.vehicle.getSpeed(left_follower)+a*traci.vehicle.getSpeed('car.forward'))/(a+b)
                            c_t = s3_back+epsilon+a/(a+b)*(self.vehicle_position('car.forward')[0]-SD_LK_forward -(s3_back+epsilon))- ego_x
                            if self.gui_on:
                                self.Target_point_visualization('weighted_point_with_Forward',[s3_back+epsilon+a/(a+b)*(self.vehicle_position('car.forward')[0]-SD_LK_forward -(s3_back+epsilon)),self.vehicle_position(left_follower)[1]])
                        else:
                            a = ego_x - (s3_back+epsilon)
                            b = s3_front-epsilon - ego_x
                            v_t = (b*traci.vehicle.getSpeed(left_follower)+a*traci.vehicle.getSpeed(rlEnv.Target_left_car))/(a+b)
                            c_t = s3_front - epsilon - ego_x
                            if self.gui_on:
                                self.Target_point_visualization(rlEnv.Target_left_car,[s3_front - epsilon,self.vehicle_position(rlEnv.Target_left_car)[1]])
                    else: # laneindex == 2 (TargetLane)
                        a = ego_x - (s3_back+epsilon)
                        b = s3_front-epsilon - ego_x
                        v_t = (b*traci.vehicle.getSpeed(left_follower)+a*traci.vehicle.getSpeed(rlEnv.Target_left_car))/(a+b)
                        c_t = s3_front - epsilon - ego_x
                        if self.gui_on:
                            self.Target_point_visualization(rlEnv.Target_left_car,[s3_front - epsilon,self.vehicle_position(rlEnv.Target_left_car)[1]])
                    self.LC_Left(action,v_t,c_t)

            elif action == 4:                  
                if rlEnv.last_action[-1][1] != 4:
                    rlEnv.Right_action = False 
                if ego_x >= s4_back+epsilon and ego_x <= s4_front-epsilon: # LC 모드 진입 조건
                    rlEnv.mode[4] = 2 #LC mode
                elif (ego_x< s4_back+epsilon or ego_x>s4_front-epsilon) and rlEnv.mode[4] !=2:
                    rlEnv.mode[4] = 1
                if ego_x < s4_back or ego_x > s4_front: # LC 모드 탈출,(preLC 모드 재진입조건)
                    # print(':::::::::::::::::::::::preLC 조건')
                    rlEnv.mode[4] = 1 #preLC mode
                if self.vehicle_position(Forward_id)[0] - SD_LK_forward <= s4_back and traci.vehicle.getLaneIndex('ego')==1: 
                    # print(':::::::::::::::::::::::LC 탈출 조건')
                    rlEnv.mode[4] = 1

                if rlEnv.mode[4] == 1:                   
                    if ego_x > s4_front -epsilon:
                        if self.vehicle_position(Forward_id)[0] - SD_LK_forward <= s4_front - epsilon:
                            v_t = traci.vehicle.getSpeed(Forward_id)
                            c_t = self.vehicle_position(Forward_id)[0] - SD_LK_forward
                            if self.gui_on:
                                self.Target_point_visualization(Forward_id,[self.vehicle_position(Forward_id)[0]- SD_LK_forward,self.vehicle_position(Forward_id)[1]])
                        else:
                            # v_t = max(min(traci.vehicle.getSpeed(rlEnv.Target_right_car),traci.vehicle.getSpeed(Forward_id)),traci.vehicle.getSpeed(right_follower))
                            v_t = traci.vehicle.getSpeed(rlEnv.Target_right_car)
                            c_t = s4_front - epsilon
                            if self.gui_on:
                                self.Target_point_visualization(rlEnv.Target_right_car,[s4_front - epsilon,self.vehicle_position(rlEnv.Target_right_car)[1]])
                        rlEnv.Target4.append([traci.simulation.getTime(),4,s4_front,space4,v_t]) 
                        # print([0,0,0,0,1,0])
                        a_y_des = self.__set_ego_a_movetoXY2('ego',self.__ego_a_desire2(v_ego,v_t,c_des2,c_t-ego_x,4),action) #desire clearance =3는 3값 만큼 타깃 지점에 가까워 져도 덜 감속함. 차량의 움직임 방향으로 3m 더 멀리 타깃으로 지정                      
                        control = [traci.simulation.getTime(),self.__ego_a_desire2(v_ego,v_t,c_des2,c_t-ego_x,4),a_y_des]
                        rlEnv.Right_action = False     
                        rlEnv.ego_control.append(control)
                    else:
                        if self.vehicle_position(Forward_id)[0] - SD_LK_forward <= s4_back+epsilon:
                            c_des = 0
                            v_t = traci.vehicle.getSpeed(Forward_id)
                            c_t = self.vehicle_position(Forward_id)[0] - SD_LK_forward
                            if self.gui_on:
                                self.Target_point_visualization(Forward_id,[self.vehicle_position(Forward_id)[0] - SD_LK_forward,self.vehicle_position(Forward_id)[1]])
                        else:
                            v_t = max(min(traci.vehicle.getSpeed(rlEnv.Target_right_car),traci.vehicle.getSpeed(Forward_id)),traci.vehicle.getSpeed(right_follower))
                            c_t = s4_back+epsilon
                            if self.gui_on:
                                self.Target_point_visualization(right_follower,[s4_back+epsilon,self.vehicle_position(right_follower)[1]])
                        rlEnv.Target4.append([traci.simulation.getTime(),4,s4_back,space4,v_t]) 
                        # print([0,0,0,0,1,0])
                        a_y_des = self.__set_ego_a_movetoXY2('ego',self.__ego_a_desire(v_ego,v_t,c_des,c_t-ego_x,4),action) #desire clearance = 2는 2값 만큼 타깃 지점에 가까워 져도 덜 감속함. 차량의 움직임 방향으로 2m 더 멀리 타깃으로 지정                    
                        control = [traci.simulation.getTime(),self.__ego_a_desire(v_ego,v_t,c_des,c_t-ego_x,4),a_y_des]
                        rlEnv.Right_action = False     
                        rlEnv.ego_control.append(control)

                if rlEnv.mode[4] == 2:
                    # print([0,0,0,0,2,0])
                    if (traci.vehicle.getLaneIndex('ego') == 1):
                        if(self.vehicle_position('car.forward')[0]-ego_x<SD_LK_forward):
                            a = ego_x -(s4_back + epsilon)
                            b = ego_x - (self.vehicle_position('car.forward')[0]-SD_LK_forward)
                            v_t = (b*traci.vehicle.getSpeed(right_follower)+a*traci.vehicle.getSpeed('car.forward'))/(a+b)
                            c_t = s4_back+epsilon+a/(a+b)*(self.vehicle_position('car.forward')[0]-SD_LK_forward -(s4_back+epsilon))- ego_x
                            if self.gui_on:
                                self.Target_point_visualization('weighted_point_with_Forward',[s4_back+epsilon+a/(a+b)*(self.vehicle_position('car.forward')[0]-SD_LK_forward -(s4_back+epsilon)),self.vehicle_position(right_follower)[1]])
                        else:
                            a = ego_x - (s4_back+epsilon)
                            b = s4_front-epsilon - ego_x
                            v_t = (b*traci.vehicle.getSpeed(right_follower)+a*traci.vehicle.getSpeed(rlEnv.Target_right_car))/(a+b)
                            c_t = s4_front - epsilon - ego_x
                            if self.gui_on:
                                self.Target_point_visualization(rlEnv.Target_right_car,[s4_front - epsilon,self.vehicle_position(rlEnv.Target_right_car)[1]])
                    else: # laneindex == 2 (TargetLane)
                        a = ego_x - (s4_back+epsilon)
                        b = s4_front-epsilon - ego_x
                        v_t = (b*traci.vehicle.getSpeed(right_follower)+a*traci.vehicle.getSpeed(rlEnv.Target_right_car))/(a+b)
                        c_t = s4_front - epsilon - ego_x
                        if self.gui_on:
                            self.Target_point_visualization(rlEnv.Target_right_car,[s4_front - epsilon,self.vehicle_position(rlEnv.Target_right_car)[1]])
                    self.LC_Right(action,v_t,c_t)

            elif action == 7:
                if self.gui_on:
                    self.Target_point_visualization('ego',[0,0])
                control = [traci.simulation.getTime(),self.__set_ego_v_movetoXY('ego',ego_v_last)[0],self.__set_ego_v_movetoXY('ego',ego_v_last)[1]]
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
                
            rlEnv.last_Space1=space1
            rlEnv.last_Space2=space2
            rlEnv.last_Space3=space3
            rlEnv.last_Space4=space4
            rlEnv.S = [0,s1_back,s2_back,s3_front,s4_front,s1_front,s2_front,s3_back,s4_back]
        
         
        
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
                if veh_id== 'car.right6' and not (len(rlEnv.right6))==0:
                    velocity_y = (y1 - rlEnv.right6[-1][3])/self.step_length
                if veh_id== 'car.right7' and not (len(rlEnv.right7))==0:
                    velocity_y = (y1 - rlEnv.right7[-1][3])/self.step_length

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
                if veh_id== 'car.right6' and not (len(rlEnv.right6))==0:
                    acceleration_y = (velocity_y - rlEnv.right6[-1][5])/self.step_length
                if veh_id== 'car.right7' and not (len(rlEnv.right7))==0:
                    acceleration_y = (velocity_y - rlEnv.right7[-1][5])/self.step_length

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
                if(veh_id=='car.right6'):
                    rlEnv.right6.append(vehicle_state)    
                if(veh_id=='car.right7'):
                    rlEnv.right7.append(vehicle_state)   


            ### ego_vehicle LC -> episode ends.
        
            if veh_id == 'ego':
                if self.__ego_vehicle_LC_start() and traci.simulation.getTime()>=0.01:
                    print('Lane change start')
                    self.LC_succeed_num +=1
                    rlEnv.ego_LC_start =True
                    traci.vehicle.setLaneChangeMode('ego',0)
                self.__ego_vehicle_LC_completed(action) # LC completed checking
                rlEnv.lane_buffer_ego = traci.vehicle.getLaneIndex('ego') # ego_car lane buffer  
                   
                      
            
            if traci.vehicle.getRoadID(veh_id) == 'E5' and veh_id[0] == 'a':
                traci.vehicle.setAccel(veh_id, '1')
                traci.vehicle.setDecel(veh_id, '0.00001')
                # traci.vehicle.setSpeedMode(veh_id,'0')
                if traci.vehicle.getSpeed('accel.rear') <= rlEnv.rearMaxSpeed:
                    self.__setvehiclestate('accel.rear')
                else:
                    self.__setvehiclestate2('accel.rear')
                

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
                elif veh_id =='car.right6':
                    tau = rlEnv.vehicles_tau[15]
                elif veh_id =='car.right7':
                    tau = rlEnv.vehicles_tau[16]
                else:
                    tau = 1.36
                
                
                v_controled = traci.vehicle.getSpeed(veh_id)

                if traci.vehicle.getLeader(veh_id) is not None: # 선행 차량이 있을 때 ACC
                    Leader_id,c_front = traci.vehicle.getLeader(veh_id)
                    v_preceding= traci.vehicle.getSpeed(Leader_id)
                    if Leader_id == 'ego'and rlEnv.tau_random[int(veh_id[-1])] <= 0.5: # 50% 확률로 앞으로 끼어드는 ego vehicle 양보 x
                        tau = 0
                    c_desire = c0+tau*v_preceding
                    
                    ##### ACC with speed limit ########
                    speed_limit = 23
                    # traci.vehicle.setAcceleration(veh_id,self.__a_desire_with_speed_limit(v_controled,v_preceding,c_desire,c_front,speed_limit),20) #입력 가속도
                    self.__set_a(veh_id,self.__a_desire_with_speed_limit(v_controled,v_preceding,c_desire,c_front,speed_limit))
                    # self.__a_desire_with_speed_limit(v_controled,v_preceding,c_desire,c_front,speed_limit) #입력 가속도

                else: # 선두 차량 CC
                    self.__set_v(veh_id,22.22)
                    self.__set_v('backupcar.forward',22.22)
                    # self.__set_v('car.forward',23)
                    # self.__set_v(veh_id,23)
        vehs = traci.vehicle.getIDList()
        if 'car.forward' not in vehs:
            print('forward end')            
            self.done=True

        # print(rlEnv.step_num)
        if 'ego'in vehs and rlEnv.Target_left_car in vehs and rlEnv.Target_right_car in vehs and traci.simulation.getTime()>0.03:
            
            last_target_left =''    
            last_target_right =''  
            if (traci.vehicle.getPosition('ego')[0]> traci.vehicle.getPosition(traci.vehicle.getLeader(rlEnv.Target_left_car)[0])[0]):
                last_target_left = rlEnv.Target_left_car
                rlEnv.Target_left_car = traci.vehicle.getLeader(rlEnv.Target_left_car)[0]                
                print('New Target_left_car: ',rlEnv.Target_left_car)                
            if (traci.vehicle.getPosition('ego')[0]<traci.vehicle.getPosition(traci.vehicle.getFollower(rlEnv.Target_left_car)[0])[0]):
                last_target_left = rlEnv.Target_left_car
                rlEnv.Target_left_car = traci.vehicle.getFollower(rlEnv.Target_left_car)[0]                
                print('New Target_left_car: ',rlEnv.Target_left_car)  
                
            if(traci.vehicle.getPosition('ego')[0]> traci.vehicle.getPosition(traci.vehicle.getLeader(rlEnv.Target_right_car)[0])[0]):
                last_target_right = rlEnv.Target_right_car
                rlEnv.Target_right_car = traci.vehicle.getLeader(rlEnv.Target_right_car)[0]
                print('New Target_right_car: ',rlEnv.Target_right_car) 
            if(traci.vehicle.getPosition('ego')[0]<traci.vehicle.getPosition(traci.vehicle.getFollower(rlEnv.Target_right_car)[0])[0]):
                last_target_right = rlEnv.Target_right_car
                rlEnv.Target_right_car = traci.vehicle.getFollower(rlEnv.Target_right_car)[0]
                print('New Target_right_car: ',rlEnv.Target_right_car)
            if last_target_left != '':
                polygon_ids = traci.polygon.getIDList()
                for polygon_id in polygon_ids:  
                    if polygon_id == last_target_left+ '_SD_LC'+'_front':
                        traci.polygon.remove(last_target_left+ '_SD_LC'+'_front', 0) 
                    if polygon_id == last_target_left+ '_SD_LC'+'_back':
                        traci.polygon.remove(last_target_left+ '_SD_LC'+'_back', 0) 
                    if polygon_id == traci.vehicle.getLeader(last_target_left)[0]+ '_SD_LC'+'_back':
                        traci.polygon.remove(traci.vehicle.getLeader(last_target_left)[0]+ '_SD_LC'+'_back', 0) 
                    if polygon_id == traci.vehicle.getFollower(last_target_left)[0]+ '_SD_LC'+'_front':
                        traci.polygon.remove(traci.vehicle.getFollower(last_target_left)[0]+ '_SD_LC'+'_front', 0) 
            if last_target_right != '':
                polygon_ids = traci.polygon.getIDList()
                for polygon_id in polygon_ids: 
                    if polygon_id == last_target_right+ '_SD_LC'+'_front':
                        traci.polygon.remove(last_target_right+ '_SD_LC'+'_front', 0) 
                    if polygon_id == last_target_right+ '_SD_LC'+'_back':
                        traci.polygon.remove(last_target_right+ '_SD_LC'+'_back', 0) 
                    if polygon_id == traci.vehicle.getLeader(last_target_right)[0]+ '_SD_LC'+'_back':
                        traci.polygon.remove(traci.vehicle.getLeader(last_target_right)[0]+ '_SD_LC'+'_back', 0) 
                    if polygon_id == traci.vehicle.getFollower(last_target_right)[0]+ '_SD_LC'+'_front':
                        traci.polygon.remove(traci.vehicle.getFollower(last_target_right)[0]+ '_SD_LC'+'_front', 0) 
                
        if('ego' in vehs and 'car.forward' in vehs and 'accel.rear' in vehs):
            nextstate = self.state('ego')            
            reward = self.__reward(rlEnv.step_num, action) # rear vehicle collision : -10, collision caused by ego : -20, LC_succeed_with_biggest_space : +20, LC_succeed_with_smaller_space : +10, step*-0.01
        else:
            nextstate = rlEnv.last_state ## it might wrong code
            self.done = True 
            reward = self.__reward(rlEnv.step_num, action)
            print("else!!!!!!!!!!!!!!!!!!!!!!!")
            
                
        rlEnv.step_num +=1    
        self.ego_collision_happened()
        rlEnv.last_action.append([traci.simulation.getTime(),action])
        return nextstate, reward
    
