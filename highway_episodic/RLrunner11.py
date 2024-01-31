from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import random
import traci
from datetime import datetime
import numpy as np
import matplotlib.pyplot as plt  
import pandas as pd
from xml.etree.ElementTree import parse
from collections import defaultdict

from matplotlib.ticker import MaxNLocator

 
from RL_env11 import rlEnv
# from rlagent import rlAgent

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'],'tools')
    print(tools)
    sys.path.append(tools)
    import sumolib 
else:
    sys.exit('Declare environment variable "SUMO_HOME"')
total_reward =[]
final_result =[]
def save_result():
    col = ['episode','total_reward','TD_error']
    df_result = pd.DataFrame(total_reward,columns=col)

    col2 = ['LC_succeed_num','collision_num']
    df_final_result = pd.DataFrame(final_result,columns=col2)

    path = "/home/mds/Desktop/highway_episodic/result2/"
    os.chdir(path)
    now = datetime.now()
    date = str(now.date()) +str('_')+str(now.time())
    df_result.to_csv('result_'+date+'.csv')
    df_final_result.to_csv('final_result_'+date+'.csv')
def save_q_table(q):
    now = datetime.now()
    date = str(now.date()) +str('_')+str(now.time())
    path = "/home/mds/Desktop/highway_episodic/q_table2/"
    name =path+"q_table "+date
    np.save(name,q)

def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("-N","--episodenum", 
                        default=1000, help="numer of episode to run qlenv")
    optParser.add_option("--nogui", action="store_true",
                        default=False, help="run commandline version of sumo")
    options, args = optParser.parse_args()
    return options 

def update_q_table(env,q,prev_state, action, reward, nextstate, alpha, gamma): ## need to check
     
    qa = max([[q[nextstate[0],nextstate[1],nextstate[2],nextstate[3],nextstate[4],nextstate[5], a]] for a in env.action_space] )
    # print('qa: ',qa[0])
    # print(type(alpha))
    # print(type(reward))
    # print(type(gamma))
    # print(type(qa))
    # print('addition: ',alpha * (reward + gamma*qa[0] - q[prev_state[0],prev_state[1],prev_state[2],prev_state[3],prev_state[4],prev_state[5],prev_state[6],prev_state[7],prev_state[8],prev_state[9],prev_state[10], action]))
    q[prev_state[0],prev_state[1],prev_state[2],prev_state[3],prev_state[4],prev_state[5],action] += alpha * (reward + gamma*qa[0] - q[prev_state[0],prev_state[1],prev_state[2],prev_state[3],prev_state[4],prev_state[5],action])
    td_error = (reward + gamma*qa[0] - q[prev_state[0],prev_state[1],prev_state[2],prev_state[3],prev_state[4],prev_state[5],action])
    print('reward: ',reward)
    print('qa: ',qa[0])
    print('q: ',q[prev_state[0],prev_state[1],prev_state[2],prev_state[3],prev_state[4],prev_state[5],action])
    print('prev_state[0]: ',prev_state[0])
    print('prev_state[1]: ',prev_state[1])
    print('prev_state[2]: ',prev_state[2])
    print('prev_state[3]: ',prev_state[3])
    print('prev_state[4]: ',prev_state[4])
    print('prev_state[5]: ',prev_state[5])
    print('action: ',action)

    print('td_error: ',td_error)
    return td_error

def epsilon_greedy_policy(env, q, state, epsilon): #return action and Israndomchoice  ## 1 : Isradom, 0 : not Israndom
    if random.uniform(0,1) < epsilon:
        # print('env.action_space: ',env.action_space)
        # print('(ramdom): ',random in (env.action_space))
        # print('random: ',random in env.action_space)
        # print('random.choice: ',random.choice(env.action_space))
        random_action = random.choice(env.action_space)
        return random_action, 1
    else:
        return np.argmax(q[state[0],state[1],state[2],state[3],state[4],state[5],:]),0
        # return max(list(range(env.action_space.n)), key = lambda x: q[(state,x)] )
        



def rl_run(sumoBinary, episodenum,net, sumocfg,step_length, veh):  
    env = rlEnv(sumoBinary, net_file = net, cfg_file = sumocfg, veh = veh)

    alpha =0.1
    gamma = 0.999
    epsilon = 0.09 #0~1
    # state: ego_forward_clearance, ego_rear_clearance, ego_lane_num, ego_to_left_spaces, ego_to_right_spaces, left_space_size, right_space_size
    
    q = np.zeros((10,10,21,21,10,10,5))  
    # q  = np.load("/home/mds/Desktop/highway_episodic/q_table/q_table 2024-01-30_05:08:42.090530.npy")
   
    
    
    for episode in range(episodenum):
        result =[]
        print("\n********#{} episode start***********".format(episode))
        #reset environment
        
        state = env.reset()
        print(state)
        action,_ = epsilon_greedy_policy(env, q,state, epsilon) ### 2 ###
        env.done = False
        
        r = 0 

        
        while (env.done) == False:
            # print(r)
            # nextstate, reward = env.step(0)
            # print('nextstate: ',nextstate)
            # print('reward: ',reward)
            
             
        
            # print('action: ',action) 
            # print('Israndom, ',Israndom)
            
            if action == 0 or action ==1 or action ==2:
                nextstate, reward = env.step(action) ### 1 ###
                TD_error = update_q_table(env,q,state, action, reward, nextstate, alpha, gamma)  ### 3 ###
                state = nextstate
                print('update_q_table 0 1 2')
                action, Israndom = epsilon_greedy_policy(env, q, state, epsilon) ### 2 ###
            elif action == 3:
                nextstate, reward = env.step3(action) ### 1 ###
                print('action 3 ')
                if env.done == True:
                    print('update_q_table 3')
                    TD_error = update_q_table(env,q,state, action, reward, nextstate, alpha, gamma)  ### 3 ###
            elif action == 4:
                nextstate, reward = env.step4(action) ### 1 ###
                print('action 4')
                if env.done == True:
                    print('update_q_table 4')
                    TD_error = update_q_table(env,q,state, action, reward, nextstate, alpha, gamma)  ### 3 ###

            r += reward
            # print(reward)
        print('total reward: ',r)
        
        result.append(episode)
        result.append(r)
        result.append(TD_error)
        total_reward.append(result)
        # print(env.done)
        env.end()

        if ((episode+1)%100) == 0 and not episode ==0:
            print('LC_succeed_num: ',env.LC_succeed_num)
            print('collision_num: ',env.collision_num)
            value = []
            value.append(env.LC_succeed_num)
            value.append(env.collision_num)
            final_result.append(value)
            save_result()
            save_q_table(q)
        
        
              


if __name__ == "__main__":
    net = "highway_episodic.net.xml"
    sumocfg = "/home/mds/Desktop/highway_episodic/highway_episodic.sumocfg"
    veh = "ego"
    options = get_options()
    if options.nogui:
        sumoBinary = sumolib.checkBinary('sumo')
    else:
        sumoBinary = sumolib.checkBinary('sumo-gui')


    """Run Simulation"""
    #2) Run in rl environment
    step_length =0.01
    episodenum= int(options.episodenum)
    
    

    rl_run(sumoBinary, episodenum,net, sumocfg, veh, step_length)
    
