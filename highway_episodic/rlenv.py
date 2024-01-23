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
    lane_buffer_ego = 0
    ego_LC_success = False
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

    vehicles_tau=[random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27),random.uniform(0.74,2.27)]
    def __init__(self, sumoBinary, net_file: str, cfg_file: str,  veh:str, use_gui: bool = True,
            begin_time: int =0, step_length: int = 0.01):
        
        self.sumoBinary = sumoBinary
        self.net = net_file
        self.sumocfg = cfg_file
        self.use_gui = use_gui
        self.step_length = step_length
        self.veh = veh

        self.episode = 0 # # of run time 
        self.begin_time = begin_time
        self.action_space = [0, 1, 2, 3, 4, 5] # 0: LK_const_vel, 1: LK_accel, 2: LK_decel, 3: LC_left, 4: LC_right
        #self.action_space = [LK_const_vel,LK_accel, LK_decel,LC_left_const_vel, LC_right_const_vel, LC_left_accel, LC_left_decel, LC_right_accel, LC_left_decel]
        self.n_actions = len(self.action_space)

        self.sumo =traci
        
 
    def start_simulation(self):

        sumo_cmd = [self.sumoBinary,
            '-c', self.sumocfg,
            '--start','true',
            '--route-files','/home/mds/Desktop/highway_episodic/route_file/highway_episodic_'+str(self.episode)+'.rou.xml',
            '--gui-settings-file','/home/mds/Desktop/highway_episodic/viewsettings.xml',
            '--lanechange.duration', '2',
            #  '--collision.action', 'warn',
            #  '--collision.stoptime','5',
            '--collision.action', 'remove',
            '--collision.mingap-factor', '0',
            #  '--time-to-teleport','10',
            '--collision-output','colliderSpeed',
            '--step-length', str(self.step_length),
            '--no-step-log',
            '--quit-on-end','true']
        
        self.sumo.start(sumo_cmd)

    def reset(self):
        # if self.episode!=0: 
        #     self.sumo.close()
        #     sys.stdout.flush()
        
        self.start_simulation()
        self.episode+=1
        print('episode : ',self.episode)
        self.__runsim()

    def __ego_vehicle_LC_success(self):
        cur_lane = traci.vehicle.getLaneIndex('ego')

        # print('cur_lane : ',cur_lane)
        last_lane = rlEnv.lane_buffer_ego
        if(last_lane != cur_lane):
            # traci.vehicle.setSpeedMode(id,0b011000)
            # traci.vehicle.setLaneChangeMode(id,0b101010101010)
            traci.vehicle.setLaneChangeMode('ego',0b100010101010) #-> 이거 대신해서
            # ACC 모드로 변경해야함.
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

    
        
    def __state(self,id):
        left_vehicles=[] #[|ego_x_pos - x_pos|, id]
        right_vehicles=[] #[|ego_x_pos - x_pos|, id]
        left_near_vehicles=[] #[x_pos]
        right_near_vehicles =[] #[x_pos]
        
        left_near_spaces=[] #[space_x_pos, space_size]
        right_near_spaces=[] #[space_x_pos, space_size]
        ego_to_left_spaces=[]
        ego_to_right_spaces=[]
        states = [] # ego_pos_x, ego_v, ego_lane_num, ego_to_left_spaces, ego_to_right_spaces, left_space_size, right_space_size
        if id:
            traci.vehicle.subscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 100.0, [tc.VAR_POSITION])
            # print(id,' subscribeContext')
            for v_id in traci.vehicle.getContextSubscriptionResults(str(id)):
                if(traci.vehicle.getLaneIndex(v_id)==2):
                    #left_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),v_id]) -> id check
                    left_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),traci.vehicle.getPosition(v_id)[0]])
                if(traci.vehicle.getLaneIndex(v_id)==0):
                    right_vehicles.append([np.abs(traci.vehicle.getPosition('ego')[0]- traci.vehicle.getPosition(v_id)[0]),traci.vehicle.getPosition(v_id)[0]])
            
            
            left_vehicles.sort()
            right_vehicles.sort()
            for i in range(3): # getting near 3 vehicles x_pos each left and right
                left_near_vehicles.append(left_vehicles[:3][i][1])
                right_near_vehicles.append(right_vehicles[:3][i][1])
                
            left_near_vehicles.sort()
            right_near_vehicles.sort()
            for i in range(2):
                left_near_spaces.append([left_near_vehicles[i]+(left_near_vehicles[i+1]-left_near_vehicles[i])/2,left_near_vehicles[i+1]-left_near_vehicles[i]])
                right_near_spaces.append([right_near_vehicles[i]+(right_near_vehicles[i+1]-right_near_vehicles[i])/2, right_near_vehicles[i+1]-right_near_vehicles[i]])
            for i in range(2):
                ego_to_left_spaces.append(left_near_spaces[i][0]- traci.vehicle.getPosition('ego')[0])
                ego_to_right_spaces.append(right_near_spaces[i][0]-traci.vehicle.getPosition('ego')[0])
            states.append(traci.vehicle.getPosition('ego')[0])
            states.append(traci.vehicle.getSpeed('ego'))
            states.append(int(traci.vehicle.getLaneIndex('ego')))
            for i in range(2):
                states.append(ego_to_left_spaces[i])
            for i in range(2):
                states.append(ego_to_right_spaces[i])
            for i in range(2):
                states.append(left_near_spaces[i][1])
            for i in range(2):
                states.append(right_near_spaces[i][1])
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
            print(states)
            traci.vehicle.unsubscribeContext(str(id), tc.CMD_GET_VEHICLE_VARIABLE, 100.0)
        return states
        
    def __runsim(self):
        rlEnv.ego_LC_success =False
        end_flag =False
        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            vehs = traci.vehicle.getIDList()
            for veh_id in vehs:
                traci.vehicle.setMinGap(veh_id,'0')
                traci.vehicle.setSpeedMode(veh_id,0b000000)
                traci.vehicle.setLaneChangeMode(veh_id,0b000000000000) # 마음대로 차선 변경 x      
            ##################### test ########################
            if('ego' in vehs):
                self.__state('ego')
            
            if len(traci.simulation.getCollidingVehiclesIDList()) !=0:
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
                    
                    if(traci.simulation.getTime()>4 and traci.simulation.getTime()<4.1):
                        traci.vehicle.changeLaneRelative('ego',1,5)
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
                    # print("success? : ",rlEnv.ego_LC_success)
                    if self.__ego_vehicle_LC_success() and traci.simulation.getTime()>=1:
                        print('Lane change success')
                        rlEnv.ego_LC_success =True
                        # print('angle : ',traci.vehicle.getAngle('ego'))
                    if rlEnv.ego_LC_success:
                        if traci.vehicle.getAngle('ego') >= 89.9 and traci.vehicle.getAngle('ego') <= 90.1:
                        # if traci.vehicle.getAngle('ego') == 90:
                            print('close')
                            rlEnv.ego_LC_success=False
                            end_flag = True
                            traci.close()
                            sys.stdout.flush()
                            break

                    if(veh_id =='ego'):
                        rlEnv.lane_buffer_ego = traci.vehicle.getLaneIndex('ego') # ego_car lane buffer    
                    

                if traci.vehicle.getRoadID(veh_id) == 'E5' and veh_id[0] == 'a':
                    traci.vehicle.setAccel(veh_id, '2.5')
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
            if(end_flag):
                break
        if not end_flag:
            traci.close()
            sys.stdout.flush()
    

    # def get_curedge(self,veh):
    #     curlane = self.sumo.vehicle.getLaneID(veh)
    #     curedge = self.sumo.lane.getEdgeID(curlane)
    #     return curedge

    # def get_done(self,curedge):
    #     done = False
    #     if curedge in self.endpoint:
    #         done = True
    #     return done
    
    # def get_reward(self, nextedge):
    #     reward = 0
    #     if nextedge=='E6' or nextedge=='E2' or nextedge=='E13' or nextedge=='-E19':
    #         reward = -100
    #     elif nextedge=='E20':
    #         reward = 500
    #     return reward
    
    # def get_nextedge(self,curedge, action):
    #     nextedge = self.dict_connection[curedge][action]
    #     return nextedge

    # def get_vehiclecount(self):
    #     return self.sumo.vehicle.getIDCount()
  
    def step(self, curedge, nextedge):
        
        beforeedge = curedge #비교해서 변하면 고를려고!

        done = self.get_done(curedge)
        reward = self.get_reward(nextedge)

        if done:
            return reward, done
        
        self.sumo.vehicle.changeTarget(self.veh,nextedge) #차량 움직여!
        
        while self.sumo.simulation.getMinExpectedNumber() > 0:
            curedge = self.get_curedge(self.veh)
            done = self.get_done(curedge)
            if done:
                break  
            self.sumo.simulationStep() 
            if curedge in self.edgelists and curedge !=beforeedge : #변했네!! 그럼 이제 다음 꺼 고르러 가야지
                break

        return reward, done    

    
    
    