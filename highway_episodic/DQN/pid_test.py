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
from pid_test_env import rlEnv

from IPython import display 
import gym
import pylab
from collections import deque
import tensorflow as tf
from tensorflow.keras.layers import Dense
from tensorflow.keras.initializers import RandomUniform


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
    col = ['episode','total_reward']
    df_result = pd.DataFrame(total_reward,columns=col)

    col2 = ['LC_succeed_num','collision_num']
    df_final_result = pd.DataFrame(final_result,columns=col2)

    path = "/home/mds/Desktop/highway_episodic/DQN/result3/"
    os.chdir(path)
    now = datetime.now()
    date = str(now.date()) +str('_')+str(now.time())
    df_result.to_csv('result_'+date+'.csv')
    df_final_result.to_csv('final_result_'+date+'.csv')


def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("-N","--episodenum", 
                        default=1, help="numer of episode to run qlenv")
    optParser.add_option("--nogui", action="store_true",
                        default=False, help="run commandline version of sumo")
    options, args = optParser.parse_args()
    return options 




def rl_run(sumoBinary, episodenum,net, sumocfg,step_length, veh):  
    score_avg = 0    
    
    for episode in range(episodenum):

        score = 0
        result =[]
        env.done = False
        r = 0 
        print("\n********#{} episode start***********".format(episode))
        #reset environment
        
        state = env.reset()
        state = np.reshape(state, [1, state_size])
        # print(state)
        action = 2

        while (env.done) == False:
            
            # print(r)
            # nextstate, reward = env.step(0)
            # print('nextstate: ',nextstate)
            # print('reward: ',reward)
            
             
        
            # print('action: ',action) 
            # print('Israndom, ',Israndom)
            
            if action == 0 or action ==1 or action ==2 or action ==3 or action ==4:
                print('action:::::::::::::::::::::::::::::::::::::::::::::::::',action)
                next_state, reward = env.step(action) ### 1 ###
                score +=reward
                action = 2
            

            if env.done:
                # 각 에피소드마다 타깃 모델을 모델의 가중치로 업데이트
                # if (episode+1)%5 ==0:

                # 에피소드마다 학습 결과 출력
                score_avg = 0.9 * score_avg + 0.1 * score if score_avg != 0 else score
                print("episode: {:3d} | score: {:.3f} ".format(episode, score))


            r += reward
            # print(reward)
        print('total reward: ',r)
        
        env.end()
    
        

# 상태가 입력, 큐함수가 출력인 인공신경망 생성
class DQN(tf.keras.Model):
    def __init__(self, action_size):
        super(DQN, self).__init__()
        self.fc1 = Dense(128, activation='relu')
        self.fc2 = Dense(128, activation='relu')
        self.fc3 = Dense(128, activation='relu')
        self.fc_out = Dense(action_size,
                            kernel_initializer=RandomUniform(-1e-3, 1e-3))

    def call(self, x):
        x = self.fc1(x)
        x = self.fc2(x)
        x = self.fc3(x)
        q = self.fc_out(x)
        return q


# 카트폴 예제에서의 DQN 에이전트
class DQNAgent:
    def __init__(self, state_size, action_size): 
        # 상태와 행동의 크기 정의
        self.state_size = state_size
        self.action_size = action_size

        # 모델과 타깃 모델 생성
        self.model = DQN(action_size)
        self.model.load_weights("/home/mds/Desktop/highway_episodic/DQN/save_model5/trained12/model")

    # 입실론 탐욕 정책으로 행동 선택
    def get_action(self, state):
        q_value = self.model(state)
        return np.argmax(q_value[0])


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





    # CartPole-v1 환경, 최대 타임스텝 수가 500
    # env = gym.make('CartPole-v1')
    # state_size = env.observation_space.shape[0]
    # action_size = env.action_space.n
    # print('state,action')
    # print(state_size)
    # print(action_size)
    # print(type(env.observation_space))
    env = rlEnv(sumoBinary, net_file = net, cfg_file = sumocfg, veh = veh)
    # states = env.reset()
    
    # print(states.shape[0])
    
    state_size = 16 
    action_size = 7
    # DQN 에이전트 생성
    agent = DQNAgent(state_size, action_size)

    scores, episodes, collisions_num = [], [], []
    rl_run(sumoBinary, episodenum,net, sumocfg, veh, step_length)
    env.save_csv()
