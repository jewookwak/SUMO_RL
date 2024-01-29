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


from RL_env4 import rlEnv
# from rlagent import rlAgent

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'],'tools')
    print(tools)
    sys.path.append(tools)
    import sumolib 
else:
    sys.exit('Declare environment variable "SUMO_HOME"')
total_reward =[]

def save_result():
    col = ['episode','total_reward']
    df_result = pd.DataFrame(total_reward,columns=col)

    path = "/home/mds/Desktop/highway_episodic/result/"
    os.chdir(path)
    now = datetime.now()
    date = str(now.date()) +str('_')+str(now.time())
    df_result.to_csv('result_'+date+'.csv')
def save_q_table(q):
    now = datetime.now()
    date = str(now.date()) +str('_')+str(now.time())
    path = "/home/mds/Desktop/highway_episodic/q_table/"
    name =path+"q_table "+date
    np.save(name,q)

def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("-N","--episodenum", 
                        default=3, help="numer of episode to run qlenv")
    optParser.add_option("--nogui", action="store_true",
                        default=False, help="run commandline version of sumo")
    options, args = optParser.parse_args()
    return options 

def update_q_table(env,q,prev_state, action, reward, nextstate, alpha, gamma): ## need to check
     
    qa = max([[q[nextstate[0],nextstate[1],nextstate[2],nextstate[3],nextstate[4],nextstate[5],nextstate[6],nextstate[7],nextstate[8],nextstate[9],nextstate[10], a]] for a in env.action_space] )
    # print('qa: ',qa[0])
    # print(type(alpha))
    # print(type(reward))
    # print(type(gamma))
    # print(type(qa))
    # print('addition: ',alpha * (reward + gamma*qa[0] - q[prev_state[0],prev_state[1],prev_state[2],prev_state[3],prev_state[4],prev_state[5],prev_state[6],prev_state[7],prev_state[8],prev_state[9],prev_state[10], action]))
    q[prev_state[0],prev_state[1],prev_state[2],prev_state[3],prev_state[4],prev_state[5],prev_state[6],prev_state[7],prev_state[8],prev_state[9],prev_state[10],action] += alpha * (reward + gamma*qa[0] - q[prev_state[0],prev_state[1],prev_state[2],prev_state[3],prev_state[4],prev_state[5],prev_state[6],prev_state[7],prev_state[8],prev_state[9],prev_state[10], action])
    
def epsilon_greedy_policy(env, q, state, epsilon):
    if random.uniform(0,1) < epsilon:
        return random in (env.action_space)
    else:
        return np.argmax(q[state[0],state[1],state[2],state[3],state[4],state[5],state[6],state[7],state[8],state[9],state[10],:])
        # return max(list(range(env.action_space.n)), key = lambda x: q[(state,x)] )
        



def rl_run(sumoBinary, episodenum,net, sumocfg,step_length, veh):  
    env = rlEnv(sumoBinary, net_file = net, cfg_file = sumocfg, veh = veh)

    alpha =0.4
    gamma = 0.999
    epsilon = 0.05
    # state: ego_forward_clearance, ego_rear_clearance, ego_lane_num, ego_to_left_spaces, ego_to_right_spaces, left_space_size, right_space_size
    
    q = np.zeros((5,5,3,11,11,11,11,5,5,5,5,5))  
            
    state = env.reset()
    print(state)
    action = epsilon_greedy_policy(env, q,state, epsilon) ### 2 ###
    
    
    for episode in range(episodenum):
        result =[]
        print("\n********#{} episode start***********".format(episode))
        #reset environment
        if episode>0:
            env.start_simulation()
            env.done = False
        
        r = 0 

        
        while (env.done) == False:
            # print(r)
            # nextstate, reward = env.step(0)
            # print('nextstate: ',nextstate)
            # print('reward: ',reward)

            action = epsilon_greedy_policy(env, q, state, epsilon) ### 2 ###
        
            
            nextstate, reward = env.step(action) ### 1 ###
            update_q_table(env,q,state, action, reward, nextstate, alpha, gamma)  ### 3 ###
            
            state = nextstate
            
            r += reward
        print('total reward: ',r)
        
        result.append(episode)
        result.append(r)
        total_reward.append(result)
        # print(env.done)
        env.end()
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
    
