from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import random
import traci
import numpy as np
import matplotlib.pyplot as plt
from xml.etree.ElementTree import parse
from collections import defaultdict

from matplotlib.ticker import MaxNLocator


from RL_env import rlEnv
# from rlagent import rlAgent

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'],'tools')
    print(tools)
    sys.path.append(tools)
    import sumolib 
else:
    sys.exit('Declare environment variable "SUMO_HOME"')



def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("-N","--episodenum", 
                        default=3, help="numer of episode to run qlenv")
    optParser.add_option("--nogui", action="store_true",
                        default=False, help="run commandline version of sumo")
    options, args = optParser.parse_args()
    return options 

def update_q_table(env,q,prev_state, action, reward, nextstate, alpha, gamma): ## need to check
    
    qa = max([[q(nextstate, a)] for a in env.action_space] )
    q[(prev_state,action)] += alpha * (reward + gamma*qa - q[(prev_state, action)])
    
def epsilon_greedy_policy(env, q, state, epsilon):
    if random.uniform(0,1) < epsilon:
        return random in (env.action_space)
    else:
        return np.argmax(q[state,:])
        # return max(list(range(env.action_space.n)), key = lambda x: q[(state,x)] )
        



def rl_run(sumoBinary, episodenum,net, sumocfg,step_length, veh):  
    env = rlEnv(sumoBinary, net_file = net, cfg_file = sumocfg, veh = veh)

    alpha =0.4
    gamma = 0.999
    epsilon = 0.017
    
    q = {}
    for s in range(len(env.observation_space)):
        for a in range(len(env.action_space)):
            q[(s,a)] = 0.0
            
            
    state = env.reset()
    action = epsilon_greedy_policy(env, q,state, epsilon) ### 2 ###
    
    
    for episode in range(episodenum):
        print("\n********#{} episode start***********".format(episode))
        #reset environment
        r = 0
        env.reset()
        print(env.done)
        while (env.done) is not True:
            env.step()
            vehs = traci.vehicle.getIDList()
            if('ego' in vehs):
                print(env.state('ego'))
                
                
            action = epsilon_greedy_policy(env, q, state, epsilon) ### 2 ###
        
            nextstate, reward = env.step(action) ### 1 ###
            
            update_q_table(state, action, reward, nextstate, alpha, gamma)  ### 3 ###
            
            state = nextstate
            
            r += reward
        print('total reward: ',r)
        env.end()
        
        
              


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
    
