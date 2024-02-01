import os
import sys
import time
from typing import DefaultDict
import numpy as np
import random
import traci.constants as tc

from xml.etree.ElementTree import parse
from collections import defaultdict

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
    step_num = 0

    vehicles_tau=[random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27)]
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
            #  '--collision.action', 'warn',
            #  '--collision.stoptime','5',
            '--collision.action', 'remove',
            '--collision.mingap-factor', '0',
            #  '--time-to-teleport','10',
            # '--collision-output','colliderSpeed',
            '--step-length', str(self.step_length),
            '--no-step-log',
            '--quit-on-end','true']
        
        self.sumo.start(sumo_cmd)

    def reset(self):
        # if self.episode!=0: 
        #     self.sumo.close()
        #     sys.stdout.flush()
        # ego_rand = random.uniform(50,115)
        ego_rand = random.uniform(50,70)
        r0 =random.uniform(0,30)
        l0 =random.uniform(0,30)
        r_delta1 = random.uniform(28,38)
        r_delta2 = random.uniform(28,38)
        r_delta3 = random.uniform(28,38)
        r_delta4 = random.uniform(28,38)
        r_delta5 = random.uniform(28,38)
        l_delta1 = random.uniform(28,38)
        l_delta2 = random.uniform(28,38)
        l_delta3 = random.uniform(28,38)
        l_delta4 = random.uniform(28,38)
        l_delta5 = random.uniform(28,38)
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
        self.done = False
        self.start_simulation()
        # print('episode : ',self.episode)
        self.episode+=1
        
        traci.simulationStep()
        traci.vehicle.add(vehID='ego',routeID='route0',typeID="ego_car",depart='now',departLane='1',departPos=str(ego_rand), departSpeed='22.22', arrivalLane='current', arrivalPos='max', arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0, personNumber=0)
        traci.vehicle.add(vehID='accel.rear',routeID='route0',typeID="accel_truck",depart='now',departLane='1',departPos='0', departSpeed='28', arrivalLane='current', arrivalPos='max', arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0, personNumber=0)
        traci.vehicle.add(vehID='car.forward',routeID='route0',typeID="car",depart='now',departLane='1',departPos='110', departSpeed='22.22', arrivalLane='current', arrivalPos='max', arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0, personNumber=0)

        traci.vehicle.add(vehID='car.left0',routeID='route0',typeID="car",depart='now',departLane='2',departPos=str(l0), departSpeed='22.22', arrivalLane='current', arrivalPos='max', arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0, personNumber=0)
        traci.vehicle.add(vehID='car.left1',routeID='route0',typeID="car",depart='now',departLane='2',departPos=str(l1), departSpeed='22.22', arrivalLane='current', arrivalPos='max', arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0, personNumber=0)
        traci.vehicle.add(vehID='car.left2',routeID='route0',typeID="car",depart='now',departLane='2',departPos=str(l2), departSpeed='22.22', arrivalLane='current', arrivalPos='max', arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0, personNumber=0)
        traci.vehicle.add(vehID='car.left3',routeID='route0',typeID="car",depart='now',departLane='2',departPos=str(l3), departSpeed='22.22', arrivalLane='current', arrivalPos='max', arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0, personNumber=0)
        traci.vehicle.add(vehID='car.left4',routeID='route0',typeID="car",depart='now',departLane='2',departPos=str(l4), departSpeed='22.22', arrivalLane='current', arrivalPos='max', arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0, personNumber=0)
        traci.vehicle.add(vehID='car.left5',routeID='route0',typeID="car",depart='now',departLane='2',departPos=str(l5), departSpeed='22.22', arrivalLane='current', arrivalPos='max', arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0, personNumber=0)
        
        traci.vehicle.add(vehID='car.right0',routeID='route0',typeID="car",depart='now',departLane='0',departPos=str(r0), departSpeed='22.22', arrivalLane='current', arrivalPos='max', arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0, personNumber=0)
        traci.vehicle.add(vehID='car.right1',routeID='route0',typeID="car",depart='now',departLane='0',departPos=str(r1), departSpeed='22.22', arrivalLane='current', arrivalPos='max', arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0, personNumber=0)
        traci.vehicle.add(vehID='car.right2',routeID='route0',typeID="car",depart='now',departLane='0',departPos=str(r2), departSpeed='22.22', arrivalLane='current', arrivalPos='max', arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0, personNumber=0)
        traci.vehicle.add(vehID='car.right3',routeID='route0',typeID="car",depart='now',departLane='0',departPos=str(r3), departSpeed='22.22', arrivalLane='current', arrivalPos='max', arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0, personNumber=0)
        traci.vehicle.add(vehID='car.right4',routeID='route0',typeID="car",depart='now',departLane='0',departPos=str(r4), departSpeed='22.22', arrivalLane='current', arrivalPos='max', arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0, personNumber=0)
        traci.vehicle.add(vehID='car.right5',routeID='route0',typeID="car",depart='now',departLane='0',departPos=str(r5), departSpeed='22.22', arrivalLane='current', arrivalPos='max', arrivalSpeed='current', fromTaz='', toTaz='', line='', personCapacity=0, personNumber=0)
        
        traci.simulationStep()
        return self.state('ego')

        
    def end(self):
        rlEnv.ego_LC_start=False
        rlEnv.ego_LC_completed=False
        traci.close()
        sys.stdout.flush()

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
                if traci.vehicle.getAngle('ego') >= 89.9 and traci.vehicle.getAngle('ego') <= 90.1:                    
                    rlEnv.ego_LC_completed =True
                    self.done = True
                    print('Lanechange completed')
                    if self.done:
                        print('done = True')
                    return True
                else:
                    return False
                
            
            
    def __K1(self, v): #optimal gain (input v : velocity output K1 :optimal gain upper bound)
        if(v>=0 and v<=11.1111):
            return 0.35
        elif(v>11.1111 and v<=19.4444):
            return -0.018*v+0.55
        else:
            return 0.2

    def __K2(self, v):#optimal gain (input v : velocity output K2 :optimal gain lower bound)
        if(v>=0 and v<=11.1111):
            return -1.22
        elif(v>11.1111 and v<=19.4444):
            return 0.0444*v-1.7133
        else:
            return -0.85
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

    def __a_desire_with_speed_limit(self, v_c,v_p,c_d,c,speed_limit):
        v_define = min(speed_limit,v_p) # 앞차 속도와 규정 속도 중 작은 것을 따름.
        
        if(v_c > speed_limit):
            a_d = - self.__K2(v_c)*(speed_limit-v_c)
        else:
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

    
    def __set_v(self,id,v_des): # v_des에 따라 a_des을 결정하고 jerk 값이 고려된  속도를 시스템에 넣음.
        dt = self.step_length
        Kp =0.9 # p gain
        v_cur = traci.vehicle.getSpeed(id)
        if (v_des - v_cur)> self.__a_max(v_cur):
            a_des = self.__a_max(v_cur)
        elif (v_des - v_cur) <= self.__a_max(v_cur) and (v_des - v_cur)>self.__a_min(v_cur): #a_max(20) = 1,a_min(20) = -1
            a_des = (v_des - v_cur)*Kp
        else:
            a_des = self.__a_min(v_cur)
        # a = set_a(id,a_des)
        traci.vehicle.setAcceleration(id,a_des,20) #입력 가속도

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
            traci.vehicle.subscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 60.0, [tc.VAR_POSITION])
            # print(id,' subscribeContext')
            for v_id in traci.vehicle.getContextSubscriptionResults(str(id)):
                if(traci.vehicle.getLaneIndex(v_id)==2):
                    #left_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),v_id]) -> id check
                    left_vehicles.append([round(np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0])),round(traci.vehicle.getPosition(v_id)[0])])
                if(traci.vehicle.getLaneIndex(v_id)==0):
                    right_vehicles.append([round(np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0])),round(traci.vehicle.getPosition(v_id)[0])])
            
            
            left_vehicles.sort()
            right_vehicles.sort()
            # if len(left_vehicles) >=3:
            for i in range(2): # getting near 3 vehicles x_pos each left and right
                left_near_vehicles.append(left_vehicles[:3][i][1])
                right_near_vehicles.append(right_vehicles[:3][i][1])
                
            left_near_vehicles.sort()
            right_near_vehicles.sort()
            
            left_near_spaces.append([round(left_near_vehicles[0]+(left_near_vehicles[1]-left_near_vehicles[0])/2),left_near_vehicles[1]-left_near_vehicles[0]])
            right_near_spaces.append([round(right_near_vehicles[0]+(right_near_vehicles[1]-right_near_vehicles[0])/2), right_near_vehicles[1]-right_near_vehicles[0]])

            ego_to_left_spaces.append(left_near_spaces[0][0]- round(traci.vehicle.getPosition('ego')[0]))
            ego_to_right_spaces.append(right_near_spaces[0][0]-round(traci.vehicle.getPosition('ego')[0]))
                
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
                
            states.append(round(forward_clearance/5))
            states.append(round(rear_clearance/5))
                        
            states.append(round(ego_to_left_spaces[0]/5)+10) # mapping negative numbers to positive number by add 5
            
            states.append(round(ego_to_right_spaces[0]/5)+10)# mapping negative numbers to positive number by add 5
            
            states.append(round(left_near_spaces[0][1]/5))
            
            states.append(round(right_near_spaces[0][1]/5))
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
            traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 60.0)
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
            traci.vehicle.subscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 60.0, [tc.VAR_POSITION])
            # print(id,' subscribeContext')
            for v_id in traci.vehicle.getContextSubscriptionResults(str(id)):
                if(traci.vehicle.getLaneIndex(v_id)==2) and not (v_id=='ego'):
                    #left_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),v_id]) -> id check
                    left_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),traci.vehicle.getPosition(v_id)[0]])
                if(traci.vehicle.getLaneIndex(v_id)==0) and not (v_id=='ego'):
                    right_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),traci.vehicle.getPosition(v_id)[0]])
            
            
            left_vehicles.sort()
            right_vehicles.sort()

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
        
        return round(np.exp(-dist_to_biggest_empty_space),2)*10


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
            traci.vehicle.subscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 60.0, [tc.VAR_POSITION])
            # print(id,' subscribeContext')
            for v_id in traci.vehicle.getContextSubscriptionResults(str(id)):
                if(traci.vehicle.getLaneIndex(v_id)==2) and not (v_id=='ego'):
                    #left_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),v_id]) -> id check
                    left_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),traci.vehicle.getPosition(v_id)[0]])
                if(traci.vehicle.getLaneIndex(v_id)==0) and not (v_id=='ego'):
                    right_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),traci.vehicle.getPosition(v_id)[0]])
            
            
            left_vehicles.sort()
            right_vehicles.sort()

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
        traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 60.0)
        distance_to_center = np.abs(traci.vehicle.getPosition('ego')[0]-(empty_space_dictionary_sorted_by_space[chosen_space_number][1][1]+(empty_space_dictionary_sorted_by_space[chosen_space_number][1][0]-empty_space_dictionary_sorted_by_space[chosen_space_number][1][1])/2))
        print('distance_to_center: ',distance_to_center)
        if chosen_space_number == 0:            
            print('+200-distance_to_center')
            return 200 -distance_to_center
        elif chosen_space_number == 1:            
            print('+150-distance_to_center')
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
                print('Forward_SD_negative_reward: ',des_clearance -x_forward)
                return int(des_clearance -x_forward)
                
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

            if((traci.vehicle.getPosition(Leader_id)[0]-ego_x_pos) >= front_SD_LC  and (ego_x_pos -traci.vehicle.getPosition(follower_id)[0])>= back_SD_LC):
                return 0
            elif((traci.vehicle.getPosition(Leader_id)[0]-ego_x_pos) < front_SD_LC  and (ego_x_pos -traci.vehicle.getPosition(follower_id)[0])>= back_SD_LC):
                print('LaneChange_SD_negative_reward-> forward: ',(traci.vehicle.getPosition(Leader_id)[0]-ego_x_pos) - front_SD_LC )
                return int((traci.vehicle.getPosition(Leader_id)[0]-ego_x_pos) - front_SD_LC )
            elif((traci.vehicle.getPosition(Leader_id)[0]-ego_x_pos) >= front_SD_LC  and (ego_x_pos -traci.vehicle.getPosition(follower_id)[0]) < back_SD_LC):
                print('LaneChange_SD_negative_reward-> rear: ',(ego_x_pos -traci.vehicle.getPosition(follower_id)[0]) - back_SD_LC)
                return int((ego_x_pos -traci.vehicle.getPosition(follower_id)[0]) - back_SD_LC)
            else:
                print('LaneChange_SD_negative_reward-> forward and rear: ',-50)
                return-50           
                
        
    def __reward(self, step_num):
        
        reward = 0
        ###negative rewards
        
        #collision
        if len(traci.simulation.getCollidingVehiclesIDList()) !=0:
            print('negative reward')
            log = traci.simulation.getCollisions()
            if str(log[0]).split(',')[1]== ' victim=ego': 
                print('-100')   
                reward -= 100
            # collision caused by ego
            elif str(log[0]).split(',')[0]== 'Collision(collider=ego' :
                print('-200')
                reward -= 200 
        if self.__ego_vehicle_LC_completed():
            reward += self.__LaneChange_SD_negative_reward()
        vehs = traci.vehicle.getIDList()
        
        if('ego' in vehs):
            if traci.vehicle.getLaneIndex('ego')==1:
                reward += self.__Forward_SD_negative_reward()
            

            if type(traci.vehicle.getFollower('ego')) is not None and traci.vehicle.getFollower('ego')[0] != '':
                rear_clearance = traci.vehicle.getPosition('ego')[0] - traci.vehicle.getPosition('accel.rear')[0]
                # print('rear_clearance: ',rear_clearance) 
            if(rear_clearance<=50): # 차로 변경을 위한 가장큰 빈공간의 중앙에 가까워질 수 록 보상을 준다.
                # print(self.__toward_biggest_empty_space_reward('ego'))
                # print('__toward_biggest_empty_space_reward: ',self.__toward_biggest_empty_space_reward('ego'))
                reward += self.__toward_biggest_empty_space_reward('ego')
        else:
            rear_clearance =0
        
        if (self.__ego_vehicle_LC_completed() and rear_clearance<=50):
            print("positive reward")
            reward += self.__chosen_space_size_reward('ego')
        vehs = traci.vehicle.getIDList()
        
        if(rlEnv.ego_LC_success ==False and rear_clearance<=50): #후방 차량 충돌 판단 가능 범위
            reward -= int(step_num) * 0.00001
        if(rear_clearance>50 and self.__ego_vehicle_LC_completed()): #후방 차량 충돌 판단 불가능 범위에서 불필요한 차로 변경
            print('early LC reward: ',-20)
            reward -= 20
        return reward
    
    # def __next_state(self):
    #     next_states =[]

    #     return next_states
    
    def step(self, action): #action-> 0: LK_const_vel, 1: LK_accel, 2: LK_decel
        self.done =False
 
        traci.simulationStep()
        vehs = traci.vehicle.getIDList()
        for veh_id in vehs:
            traci.vehicle.setMinGap(veh_id,'0')
            traci.vehicle.setSpeedMode(veh_id,0b000000)
            traci.vehicle.setLaneChangeMode(veh_id,0b000000000000) # 마음대로 차선 변경 x      
        ##################### test ########################
        # if('ego' in vehs):
        #     print('ego_lane: ',traci.vehicle.getLaneIndex('ego'))
        #    self.state('ego')
           
           
           
        ## agent action ##
        #action-> 0: LK_const_vel, 1: LK_accel, 2: LK_decel, 3: LC_left, 4: LC_right
        if len(rlEnv.ego) !=0:
            ego_v_last = rlEnv.ego[-1][4]
        else:
            ego_v_last = traci.vehicle.getSpeed('ego')
        if('ego' in vehs):   
            if action == 0:
                self.__set_v('ego',ego_v_last)
            elif action == 1:
                self.__set_v('ego',ego_v_last +0.03)
            elif action == 2:
                self.__set_v(veh_id,ego_v_last -0.03)

        
        if('ego' in vehs):
            nextstate = self.state('ego')            
            reward = self.__reward(rlEnv.step_num) # rear vehicle collision : -10, collision caused by ego : -20, LC_succeed_with_biggest_space : +20, LC_succeed_with_smaller_space : +10, step*-0.01
        else:
            nextstate = [0,0,0,0,0,0] ## it might wrong code
            reward = self.__reward(rlEnv.step_num)
            self.done = True  
        
        if len(traci.simulation.getCollidingVehiclesIDList()) !=0:
            log = traci.simulation.getCollisions()
            if str(log[0]).split(',')[1]== ' victim=ego' or str(log[0]).split(',')[0]== 'Collision(collider=ego' :
                print('done')
                self.collision_num +=1
                self.done = True

          
        
        for veh_id in vehs:
            # surrounding vehicles LK for first 2secs.
            if veh_id[0] =='c'and traci.simulation.getTime()<2:
                traci.vehicle.setLaneChangeMode(veh_id,0b000000000000)

 
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
                if veh_id== 'ego' and not (len(rlEnv.ego)==0):
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rlEnv.ego[-1][5])/self.step_length
                if veh_id== 'car.leftrear0' and not (len(rlEnv.leftrear0))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rlEnv.leftrear0[-1][5])/self.step_length
                if veh_id== 'car.leftrear1' and not (len(rlEnv.leftrear1))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rlEnv.leftrear1[-1][5])/self.step_length
                if veh_id== 'accel.rear' and not (len(rlEnv.rear))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rlEnv.rear[-1][5])/self.step_length
                if veh_id== 'car.rightrear0' and not (len(rlEnv.rightrear0))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rlEnv.rightrear0[-1][5])/self.step_length
                if veh_id== 'car.rightrear1' and not (len(rlEnv.rightrear1))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rlEnv.rightrear1[-1][5])/self.step_length
                if veh_id== 'car.leftforward0' and not (len(rlEnv.leftforward0))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rlEnv.leftforward0[-1][5])/self.step_length
                if veh_id== 'car.leftforward1' and not (len(rlEnv.leftforward1))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rlEnv.leftforward1[-1][5])/self.step_length
                if veh_id== 'car.leftforward2' and not (len(rlEnv.leftforward2))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rlEnv.leftforward2[-1][5])/self.step_length
                if veh_id== 'car.forward' and not (len(rlEnv.forward))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rlEnv.forward[-1][5])/self.step_length
                if veh_id== 'car.rightforward0' and not (len(rlEnv.rightforward0))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rlEnv.rightforward0[-1][5])/self.step_length
                if veh_id== 'car.rightforward1' and not (len(rlEnv.rightforward1))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rlEnv.rightforward1[-1][5])/self.step_length
                if veh_id== 'car.rightforward2' and not (len(rlEnv.rightforward2))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rlEnv.rightforward2[-1][5])/self.step_length

                vehicle_state.append(acceleration_y)
                
                
                vehicle_state.append(traci.vehicle.getAngle(veh_id))
            
                
                if(veh_id == 'ego'):
                    rlEnv.ego.append(vehicle_state)
                if(veh_id =='car.leftrear0'):
                    rlEnv.leftrear0.append(vehicle_state)
                if(veh_id=='car.leftrear1'):
                    rlEnv.leftrear1.append(vehicle_state)
                if(veh_id=='accel.rear'):
                    rlEnv.rear.append(vehicle_state)
                if(veh_id=='car.rightrear0'):
                    rlEnv.rightrear0.append(vehicle_state)
                if(veh_id=='car.rightrear1'):
                    rlEnv.rightrear1.append(vehicle_state)
                if(veh_id=='car.leftforward0'):
                    rlEnv.leftforward0.append(vehicle_state)
                if(veh_id=='car.leftforward1'):
                    rlEnv.leftforward1.append(vehicle_state)
                if(veh_id=='car.leftforward2'):
                    rlEnv.leftforward2.append(vehicle_state)
                if(veh_id=='car.forward'):
                    rlEnv.forward.append(vehicle_state)
                if(veh_id=='car.rightforward0'):
                    rlEnv.rightforward0.append(vehicle_state)
                if(veh_id=='car.rightforward1'):
                    rlEnv.rightforward1.append(vehicle_state)
                if(veh_id=='car.rightforward2'):
                    rlEnv.rightforward2.append(vehicle_state)    


            ### ego_vehicle LC -> episode ends.
        
            if veh_id == 'ego':
                if self.__ego_vehicle_LC_start() and traci.simulation.getTime()>=1:
                    print('Lane change start')
                    
                    rlEnv.ego_LC_start =True
                    traci.vehicle.setLaneChangeMode('ego',0)
                if(self.__ego_vehicle_LC_completed ()): # LC completed checking
                    self.LC_succeed_num +=1
                rlEnv.lane_buffer_ego = traci.vehicle.getLaneIndex('ego') # ego_car lane buffer  
                   
                      
            
            if traci.vehicle.getRoadID(veh_id) == 'E5' and veh_id[0] == 'a':
                traci.vehicle.setAccel(veh_id, '1')
                traci.vehicle.setDecel(veh_id, '0.00001')
                # traci.vehicle.setSpeedMode(veh_id,'0')
                self.__setvehiclestate('accel.rear')

            if (traci.vehicle.getRoadID(veh_id) == 'E5' and veh_id[0] == 'c'):
                c0 = 1.98
                # tau = 1.36
                
                if veh_id == 'car.leftrear0':
                    tau = rlEnv.vehicles_tau[0]                    
                elif veh_id =='car.leftrear1':
                    tau = rlEnv.vehicles_tau[1]                    
                elif veh_id =='car.rightrear0':
                    tau = rlEnv.vehicles_tau[2]                    
                elif veh_id =='car.rightrear1':
                    tau = rlEnv.vehicles_tau[3]                    
                elif veh_id =='car.leftforward0':
                    tau = rlEnv.vehicles_tau[4]               
                elif veh_id =='car.leftforward1':
                    tau = rlEnv.vehicles_tau[5]                    
                elif veh_id =='car.leftforward2':
                    tau = rlEnv.vehicles_tau[6]
                elif veh_id =='car.forward':
                    tau = rlEnv.vehicles_tau[7]                    
                elif veh_id =='car.rightforward0':
                    tau = rlEnv.vehicles_tau[8]                    
                elif veh_id =='car.rightforward1':
                    tau = rlEnv.vehicles_tau[9]                    
                elif veh_id =='car.rightforward2':
                    tau = rlEnv.vehicles_tau[10]
                else:
                    tau = 1.36
                
                
                v_controled = traci.vehicle.getSpeed(veh_id)

                if traci.vehicle.getLeader(veh_id) is not None: # 선행 차량이 있을 때 ACC
                    Leader_id,c_front = traci.vehicle.getLeader(veh_id)
                    v_preceding= traci.vehicle.getSpeed(Leader_id)
                    c_desire = c0+tau*v_preceding
                    
                    ##### ACC with speed limit ########
                    speed_limit = 23
                    traci.vehicle.setAcceleration(veh_id,self.__a_desire_with_speed_limit(v_controled,v_preceding,c_desire,c_front,speed_limit),20) #입력 가속도
                    self.__a_desire_with_speed_limit(v_controled,v_preceding,c_desire,c_front,speed_limit) #입력 가속도

                else: # 선두 차량 CC
                    self.__set_v(veh_id,22.22)
        vehs = traci.vehicle.getIDList()
        
        # print(rlEnv.step_num)
        rlEnv.step_num +=1    
        return nextstate, reward
    
                             # acition -> 3: LC_left, 4: LC_right
    def step3(self, action): # continue the simulation step but return finalstate and finalreward as result of Lanechange.
        self.done =False
 
        traci.simulationStep()
        vehs = traci.vehicle.getIDList()
        for veh_id in vehs:
            traci.vehicle.setMinGap(veh_id,'0')
            traci.vehicle.setSpeedMode(veh_id,0b000000)
            traci.vehicle.setLaneChangeMode(veh_id,0b000000000000) # 마음대로 차선 변경 x 
        ## agent action ##
        #action-> 0: LK_const_vel, 1: LK_accel, 2: LK_decel, 3: LC_left, 4: LC_right
        if('ego' in vehs):   
            if action == 3:
                traci.vehicle.changeLaneRelative('ego',1,3)
            

        if('ego' in vehs):
            nextstate = self.state('ego')            
            nextreward = self.__reward(rlEnv.step_num) # rear vehicle collision : -10, collision caused by ego : -20, LC_succeed_with_biggest_space : +20, LC_succeed_with_smaller_space : +10, step*-0.01
        else:
            nextstate = [0,0,0,0,0,0]  ## it might wrong code
            nextreward = self.__reward(rlEnv.step_num)
            self.done = True  
        
        if len(traci.simulation.getCollidingVehiclesIDList()) !=0:
            log = traci.simulation.getCollisions()
            if str(log[0]).split(',')[1]== ' victim=ego' or str(log[0]).split(',')[0]== 'Collision(collider=ego' :
                print('done')
                self.collision_num +=1
                self.done = True

        for veh_id in vehs:
            # surrounding vehicles LK for first 2secs.
            if veh_id[0] =='c'and traci.simulation.getTime()<2:
                traci.vehicle.setLaneChangeMode(veh_id,0b000000000000)

 
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
                if veh_id== 'ego' and not (len(rlEnv.ego)==0):
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rlEnv.ego[-1][5])/self.step_length
                if veh_id== 'car.leftrear0' and not (len(rlEnv.leftrear0))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rlEnv.leftrear0[-1][5])/self.step_length
                if veh_id== 'car.leftrear1' and not (len(rlEnv.leftrear1))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rlEnv.leftrear1[-1][5])/self.step_length
                if veh_id== 'accel.rear' and not (len(rlEnv.rear))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rlEnv.rear[-1][5])/self.step_length
                if veh_id== 'car.rightrear0' and not (len(rlEnv.rightrear0))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rlEnv.rightrear0[-1][5])/self.step_length
                if veh_id== 'car.rightrear1' and not (len(rlEnv.rightrear1))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rlEnv.rightrear1[-1][5])/self.step_length
                if veh_id== 'car.leftforward0' and not (len(rlEnv.leftforward0))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rlEnv.leftforward0[-1][5])/self.step_length
                if veh_id== 'car.leftforward1' and not (len(rlEnv.leftforward1))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rlEnv.leftforward1[-1][5])/self.step_length
                if veh_id== 'car.leftforward2' and not (len(rlEnv.leftforward2))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rlEnv.leftforward2[-1][5])/self.step_length
                if veh_id== 'car.forward' and not (len(rlEnv.forward))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rlEnv.forward[-1][5])/self.step_length
                if veh_id== 'car.rightforward0' and not (len(rlEnv.rightforward0))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rlEnv.rightforward0[-1][5])/self.step_length
                if veh_id== 'car.rightforward1' and not (len(rlEnv.rightforward1))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rlEnv.rightforward1[-1][5])/self.step_length
                if veh_id== 'car.rightforward2' and not (len(rlEnv.rightforward2))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rlEnv.rightforward2[-1][5])/self.step_length

                vehicle_state.append(acceleration_y)
                
                
                vehicle_state.append(traci.vehicle.getAngle(veh_id))
            
                
                if(veh_id == 'ego'):
                    rlEnv.ego.append(vehicle_state)
                if(veh_id =='car.leftrear0'):
                    rlEnv.leftrear0.append(vehicle_state)
                if(veh_id=='car.leftrear1'):
                    rlEnv.leftrear1.append(vehicle_state)
                if(veh_id=='accel.rear'):
                    rlEnv.rear.append(vehicle_state)
                if(veh_id=='car.rightrear0'):
                    rlEnv.rightrear0.append(vehicle_state)
                if(veh_id=='car.rightrear1'):
                    rlEnv.rightrear1.append(vehicle_state)
                if(veh_id=='car.leftforward0'):
                    rlEnv.leftforward0.append(vehicle_state)
                if(veh_id=='car.leftforward1'):
                    rlEnv.leftforward1.append(vehicle_state)
                if(veh_id=='car.leftforward2'):
                    rlEnv.leftforward2.append(vehicle_state)
                if(veh_id=='car.forward'):
                    rlEnv.forward.append(vehicle_state)
                if(veh_id=='car.rightforward0'):
                    rlEnv.rightforward0.append(vehicle_state)
                if(veh_id=='car.rightforward1'):
                    rlEnv.rightforward1.append(vehicle_state)
                if(veh_id=='car.rightforward2'):
                    rlEnv.rightforward2.append(vehicle_state)    


            ### ego_vehicle LC -> episode ends.
        
            if veh_id == 'ego':
                if self.__ego_vehicle_LC_start() and traci.simulation.getTime()>=1:
                    print('Lane change start')
                    self.LC_succeed_num +=1
                    rlEnv.ego_LC_start =True
                    traci.vehicle.setLaneChangeMode('ego',0)
                self.__ego_vehicle_LC_completed () # LC completed checking
                rlEnv.lane_buffer_ego = traci.vehicle.getLaneIndex('ego') # ego_car lane buffer 
                

            if traci.vehicle.getRoadID(veh_id) == 'E5' and veh_id[0] == 'a':
                traci.vehicle.setAccel(veh_id, '1')
                traci.vehicle.setDecel(veh_id, '0.00001')
                # traci.vehicle.setSpeedMode(veh_id,'0')
                self.__setvehiclestate('accel.rear')

            if (traci.vehicle.getRoadID(veh_id) == 'E5' and veh_id[0] == 'c'):
                c0 = 1.98
                # tau = 1.36
                
                if veh_id == 'car.leftrear0':
                    tau = rlEnv.vehicles_tau[0]                    
                elif veh_id =='car.leftrear1':
                    tau = rlEnv.vehicles_tau[1]                    
                elif veh_id =='car.rightrear0':
                    tau = rlEnv.vehicles_tau[2]                    
                elif veh_id =='car.rightrear1':
                    tau = rlEnv.vehicles_tau[3]                    
                elif veh_id =='car.leftforward0':
                    tau = rlEnv.vehicles_tau[4]               
                elif veh_id =='car.leftforward1':
                    tau = rlEnv.vehicles_tau[5]                    
                elif veh_id =='car.leftforward2':
                    tau = rlEnv.vehicles_tau[6]
                elif veh_id =='car.forward':
                    tau = rlEnv.vehicles_tau[7]                    
                elif veh_id =='car.rightforward0':
                    tau = rlEnv.vehicles_tau[8]                    
                elif veh_id =='car.rightforward1':
                    tau = rlEnv.vehicles_tau[9]                    
                elif veh_id =='car.rightforward2':
                    tau = rlEnv.vehicles_tau[10]
                else:
                    tau = 1.36
                
                
                v_controled = traci.vehicle.getSpeed(veh_id)

                if traci.vehicle.getLeader(veh_id) is not None: # 선행 차량이 있을 때 ACC
                    Leader_id,c_front = traci.vehicle.getLeader(veh_id)
                    v_preceding= traci.vehicle.getSpeed(Leader_id)
                    c_desire = c0+tau*v_preceding
                    
                    ##### ACC with speed limit ########
                    speed_limit = 23
                    traci.vehicle.setAcceleration(veh_id,self.__a_desire_with_speed_limit(v_controled,v_preceding,c_desire,c_front,speed_limit),20) #입력 가속도
                    self.__a_desire_with_speed_limit(v_controled,v_preceding,c_desire,c_front,speed_limit) #입력 가속도

                else: # 선두 차량 CC
                    self.__set_v(veh_id,22.22)
        vehs = traci.vehicle.getIDList()
        
        # print(rlEnv.step_num)
        rlEnv.step_num +=1
        
        
        return nextstate,nextreward
    

    def step4(self, action): # continue the simulation step but return finalstate and finalreward as result of Lanechange.
        self.done =False
 
        traci.simulationStep()
        vehs = traci.vehicle.getIDList()
        for veh_id in vehs:
            traci.vehicle.setMinGap(veh_id,'0')
            traci.vehicle.setSpeedMode(veh_id,0b000000)
            traci.vehicle.setLaneChangeMode(veh_id,0b000000000000) # 마음대로 차선 변경 x 
        ## agent action ##
        #action-> 0: LK_const_vel, 1: LK_accel, 2: LK_decel, 3: LC_left, 4: LC_right
        if('ego' in vehs):   
            if action == 4:
                traci.vehicle.changeLaneRelative('ego',-1,3)

        if('ego' in vehs):
            nextstate = self.state('ego')            
            nextreward = self.__reward(rlEnv.step_num) # rear vehicle collision : -10, collision caused by ego : -20, LC_succeed_with_biggest_space : +20, LC_succeed_with_smaller_space : +10, step*-0.01
        else:
            nextstate = [0,0,0,0,0,0]  ## it might wrong code
            nextreward = self.__reward(rlEnv.step_num)
            self.done = True  
        
        if len(traci.simulation.getCollidingVehiclesIDList()) !=0:
            log = traci.simulation.getCollisions()
            if str(log[0]).split(',')[1]== ' victim=ego' or str(log[0]).split(',')[0]== 'Collision(collider=ego' :
                print('done')
                self.collision_num +=1
                self.done = True

        for veh_id in vehs:
            # surrounding vehicles LK for first 2secs.
            if veh_id[0] =='c'and traci.simulation.getTime()<2:
                traci.vehicle.setLaneChangeMode(veh_id,0b000000000000)

 
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
                if veh_id== 'ego' and not (len(rlEnv.ego)==0):
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rlEnv.ego[-1][5])/self.step_length
                if veh_id== 'car.leftrear0' and not (len(rlEnv.leftrear0))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rlEnv.leftrear0[-1][5])/self.step_length
                if veh_id== 'car.leftrear1' and not (len(rlEnv.leftrear1))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rlEnv.leftrear1[-1][5])/self.step_length
                if veh_id== 'accel.rear' and not (len(rlEnv.rear))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rlEnv.rear[-1][5])/self.step_length
                if veh_id== 'car.rightrear0' and not (len(rlEnv.rightrear0))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rlEnv.rightrear0[-1][5])/self.step_length
                if veh_id== 'car.rightrear1' and not (len(rlEnv.rightrear1))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rlEnv.rightrear1[-1][5])/self.step_length
                if veh_id== 'car.leftforward0' and not (len(rlEnv.leftforward0))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rlEnv.leftforward0[-1][5])/self.step_length
                if veh_id== 'car.leftforward1' and not (len(rlEnv.leftforward1))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rlEnv.leftforward1[-1][5])/self.step_length
                if veh_id== 'car.leftforward2' and not (len(rlEnv.leftforward2))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rlEnv.leftforward2[-1][5])/self.step_length
                if veh_id== 'car.forward' and not (len(rlEnv.forward))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rlEnv.forward[-1][5])/self.step_length
                if veh_id== 'car.rightforward0' and not (len(rlEnv.rightforward0))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rlEnv.rightforward0[-1][5])/self.step_length
                if veh_id== 'car.rightforward1' and not (len(rlEnv.rightforward1))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rlEnv.rightforward1[-1][5])/self.step_length
                if veh_id== 'car.rightforward2' and not (len(rlEnv.rightforward2))==0:
                    acceleration_y = (traci.vehicle.getLateralSpeed(veh_id) - rlEnv.rightforward2[-1][5])/self.step_length

                vehicle_state.append(acceleration_y)
                
                
                vehicle_state.append(traci.vehicle.getAngle(veh_id))
            
                
                if(veh_id == 'ego'):
                    rlEnv.ego.append(vehicle_state)
                if(veh_id =='car.leftrear0'):
                    rlEnv.leftrear0.append(vehicle_state)
                if(veh_id=='car.leftrear1'):
                    rlEnv.leftrear1.append(vehicle_state)
                if(veh_id=='accel.rear'):
                    rlEnv.rear.append(vehicle_state)
                if(veh_id=='car.rightrear0'):
                    rlEnv.rightrear0.append(vehicle_state)
                if(veh_id=='car.rightrear1'):
                    rlEnv.rightrear1.append(vehicle_state)
                if(veh_id=='car.leftforward0'):
                    rlEnv.leftforward0.append(vehicle_state)
                if(veh_id=='car.leftforward1'):
                    rlEnv.leftforward1.append(vehicle_state)
                if(veh_id=='car.leftforward2'):
                    rlEnv.leftforward2.append(vehicle_state)
                if(veh_id=='car.forward'):
                    rlEnv.forward.append(vehicle_state)
                if(veh_id=='car.rightforward0'):
                    rlEnv.rightforward0.append(vehicle_state)
                if(veh_id=='car.rightforward1'):
                    rlEnv.rightforward1.append(vehicle_state)
                if(veh_id=='car.rightforward2'):
                    rlEnv.rightforward2.append(vehicle_state)    


            ### ego_vehicle LC -> episode ends.
        
            if veh_id == 'ego':
                if self.__ego_vehicle_LC_start() and traci.simulation.getTime()>=1:
                    print('Lane change start')
                    self.LC_succeed_num +=1
                    rlEnv.ego_LC_start =True
                    traci.vehicle.setLaneChangeMode('ego',0)
                self.__ego_vehicle_LC_completed () # LC completed checking
                rlEnv.lane_buffer_ego = traci.vehicle.getLaneIndex('ego') # ego_car lane buffer 
                

            if traci.vehicle.getRoadID(veh_id) == 'E5' and veh_id[0] == 'a':
                traci.vehicle.setAccel(veh_id, '1')
                traci.vehicle.setDecel(veh_id, '0.00001')
                # traci.vehicle.setSpeedMode(veh_id,'0')
                self.__setvehiclestate('accel.rear')

            if (traci.vehicle.getRoadID(veh_id) == 'E5' and veh_id[0] == 'c'):
                c0 = 1.98
                # tau = 1.36
                
                if veh_id == 'car.leftrear0':
                    tau = rlEnv.vehicles_tau[0]                    
                elif veh_id =='car.leftrear1':
                    tau = rlEnv.vehicles_tau[1]                    
                elif veh_id =='car.rightrear0':
                    tau = rlEnv.vehicles_tau[2]                    
                elif veh_id =='car.rightrear1':
                    tau = rlEnv.vehicles_tau[3]                    
                elif veh_id =='car.leftforward0':
                    tau = rlEnv.vehicles_tau[4]               
                elif veh_id =='car.leftforward1':
                    tau = rlEnv.vehicles_tau[5]                    
                elif veh_id =='car.leftforward2':
                    tau = rlEnv.vehicles_tau[6]
                elif veh_id =='car.forward':
                    tau = rlEnv.vehicles_tau[7]                    
                elif veh_id =='car.rightforward0':
                    tau = rlEnv.vehicles_tau[8]                    
                elif veh_id =='car.rightforward1':
                    tau = rlEnv.vehicles_tau[9]                    
                elif veh_id =='car.rightforward2':
                    tau = rlEnv.vehicles_tau[10]
                else:
                    tau = 1.36
                
                
                v_controled = traci.vehicle.getSpeed(veh_id)

                if traci.vehicle.getLeader(veh_id) is not None: # 선행 차량이 있을 때 ACC
                    Leader_id,c_front = traci.vehicle.getLeader(veh_id)
                    v_preceding= traci.vehicle.getSpeed(Leader_id)
                    c_desire = c0+tau*v_preceding
                    
                    ##### ACC with speed limit ########
                    speed_limit = 23
                    traci.vehicle.setAcceleration(veh_id,self.__a_desire_with_speed_limit(v_controled,v_preceding,c_desire,c_front,speed_limit),20) #입력 가속도
                    self.__a_desire_with_speed_limit(v_controled,v_preceding,c_desire,c_front,speed_limit) #입력 가속도

                else: # 선두 차량 CC
                    self.__set_v(veh_id,22.22)
        vehs = traci.vehicle.getIDList()
        
        # print(rlEnv.step_num)
        rlEnv.step_num +=1
        
        
        return nextstate,nextreward

    
    
    