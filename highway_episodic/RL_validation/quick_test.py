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
from pathlib import Path
# from env_reward_fix2_space3 import rlEnv
# from env_reward_fix5 import rlEnv
# from env_reward_fix4_random_space_2 import rlEnv
# from env_space_target_point_fix import rlEnv
# from env_total_random import rlEnv
# from env_total_random_with_normalized_new_states_based_on_paper_fix2_100ms import rlEnv
# from env_total_random_with_normalized_new_states_based_on_paper_preLC_target_point_fix import rlEnv
from env_total_random_with_normalized_new_states_based_on_paper_preLC_fix_new_last_SD_visualization_all_actions import rlEnv

from IPython import display  
import pylab 
from collections import deque
import tensorflow as tf
from tensorflow.keras.layers import Dense
from tensorflow.keras.initializers import RandomUniform
from tensorflow.keras.layers import LSTM

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'],'tools')
    print(tools)
    sys.path.append(tools)
    import sumolib 
else:
    sys.exit('Declare environment variable "SUMO_HOME"')

total_reward =[]
final_result =[]
# def save_result():
#     col = ['episode','total_reward']
#     df_result = pd.DataFrame(total_reward,columns=col)

#     col2 = ['LC_succeed_num','collision_num']
#     df_final_result = pd.DataFrame(final_result,columns=col2)

#     path = "/home/mds/Desktop/highway_episodic/DQN/RL_validation/result4/"
#     os.chdir(path)
#     now = datetime.now()
#     date = str(now.date()) +str('_')+str(now.time())
#     df_result.to_csv('result_'+date+'.csv')
#     df_final_result.to_csv('final_result_'+date+'.csv')


def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("-N","--episodenum", 
                         default=100, help="numer of episode to run q lenv")
    optParser.add_option("--nogui", action="store_true",
                        default=False, help="run commandline version of sumo")
    options, args = optParser.parse_args()
    return options 




def rl_run(sumoBinary, episodenum,net, sumocfg,step_length, veh):  
    score_avg = 0    
    
    for episode in range(episodenum):
        count = 0
        score = 0
        result =[]
        env.done = False
        r = 0 
        print("\n********#{} episode start***********".format(episode))
        #reset environment
        
        _ = env.reset()
        action = 0
        if action == 5:
            action = 7
        while (env.done) == False:
            
            if action == 0 or action ==1 or action ==2 or action ==3 or action ==4 or action ==7:
                # print('action:::::::::::::::::::::::::::::::::::::::::::::::::',action)
                if action ==7:
                    print('action:::::::::::::::::::::::::::::                ',action)
                if action ==0:
                    print('action:::::::::::::::::::::::::::::                ',action)
                elif action ==1:
                    print('action:::::::::::::::::::::::::::::            ',action)
                elif action ==2:
                    print('action:::::::::::::::::::::::::::::                    ',action)
                elif action ==3:
                    print('action:::::::::::::::::::::::::::::               ',action)
                elif action ==4:
                    print('action:::::::::::::::::::::::::::::                 ',action)
                _, _ = env.step(action) ### 1 ###
                
                #<reward 설계 확인>
                #각 action을 계속 선택했을 때 reward가 잘 나오는지 확인
                action = 1
                # train 중에 reward가 잘 나오는지 확인 50 step 단위로 random한 action을 선택했을 때 reward 확인
                if traci.simulation.getTime()>=6:
                #     # env.done =True
                    action = 1
                if traci.simulation.getTime()>=7:
                    action = 2
                if traci.simulation.getTime()>=8:
                    action = 0
                if traci.simulation.getTime()>=9:
                    action = 1
                # elif traci.simulation.getTime()<12 and traci.simulation.getTime()>=4.5:
                #     action = 2
                # elif traci.simulation.getTime()<12 and traci.simulation.getTime()>=9:
                #     action = 1
                # else:
                #     env.done =True
                    # action = 2

                # if traci.simulation.getTime()<4 and traci.simulation.getTime()>=1:
                #     action = 2
                # if traci.simulation.getTime()<5 and traci.simulation.getTime()>=4:
                #     action = 7
                # if traci.simulation.getTime()<8 and traci.simulation.getTime()>=5:
                #     action = 2
                # # else:
                #     action = 4
                # else:
            #     if count %100 == 0:
            #         action_size=6
            #         fix_action = random.randrange(1,3)
            #         if fix_action == 5:
            #             fix_action = 7
            #     action = fix_action
            # count+=1

            if env.done:
                # 각 에피소드마다 타깃 모델을 모델의 가중치로 업데이트
                # if (episode+1)%5 ==0:

                # 에피소드마다 학습 결과 출력
                score_avg = 0.9 * score_avg + 0.1 * score if score_avg != 0 else score
                print("episode: {} | score: {:.3f} ".format(episode, score))
                print("episode: {} | collision: {} | space size 1st: {} | space size 2nd: {} | space size 3rd: {} | space size 4th: {}".format(episode, env.collision_num,env.space1_count, env.space2_count,env.space3_count,env.space4_count))
                print("episode: {} | collision: {} | R1 space: {} | R2 space: {} | L1 space: {} | L2 space: {}".format(episode, env.collision_num,env.R1_count, env.R2_count,env.L1_count,env.L2_count))
                # 에피소드마다 학습 결과 그래프로 저장
                scores.append(score_avg)
                original_scores.append(score)
                episodes.append(episode)
                R1.append(env.R1_count)
                R2.append(env.R2_count)
                L1.append(env.L1_count)
                L2.append(env.L2_count)
                space1.append(env.space1_count)
                space2.append(env.space2_count)
                space3.append(env.space3_count)
                space4.append(env.space4_count)
                
                plt.figure(1,figsize=(8,4))            
                pylab.plot(episodes, original_scores, 'cyan',label='score')
                pylab.plot(episodes, scores, 'b',label='average score')
                pylab.xlabel("episode")
                pylab.ylabel("score")
                pylab.savefig("/home/mds/Desktop/highway_episodic/DQN/RL_validation/save_graph/test_reward.png")
                
                plt.figure(2,figsize=(8,4))
                collisions_num.append(env.collision_num)
                pylab.plot(episodes,collisions_num,'r')
                pylab.xlabel("episode")
                pylab.ylabel("collision number")
                pylab.savefig("/home/mds/Desktop/highway_episodic/DQN/RL_validation/save_graph/test_collision.png")

                plt.figure(3,figsize=(8,4))
                pylab.plot(episodes,collisions_num,'r',label='collision')
                plt.plot(episodes, R1, 'orange', label='R1')
                plt.plot(episodes, R2, 'g', label='R2')
                plt.plot(episodes, L1, 'b', label='L1')
                plt.plot(episodes, L2, 'm', label='L2')
                pylab.legend()
                pylab.xlabel("episode")
                pylab.ylabel("succeed space ")
                pylab.savefig("/home/mds/Desktop/highway_episodic/DQN/RL_validation/save_graph/LC_success_spaces.png")
                plt.close()
                plt.figure(4,figsize=(8,4))
                pylab.plot(episodes,collisions_num,'r',label='collision')
                plt.plot(episodes, space1, 'orange', label='1st')
                plt.plot(episodes, space2, 'g', label='2nd')
                plt.plot(episodes, space3, 'b', label='3rd')
                plt.plot(episodes, space4, 'm', label='4th')
                pylab.legend()
                pylab.xlabel("episode")
                pylab.ylabel("succeed space ")
                pylab.savefig("/home/mds/Desktop/highway_episodic/DQN/RL_validation/save_graph/LC_success_spaces_size.png")
                plt.close()
                # # 이동 평균이 70 이상일 때 종료
                # # if score_avg > 70:
                # if episode == episodenum-1:
                #     agent.model.save_weights("/home/mds/Desktop/highway_episodic/DQN/RL_validation/save_model/model", save_format="tf")
                #     sys.exit()


            # print(reward)
        print('total reward: ',r)
        name = str(Path(__file__).name)
        env.save_csv(episode,name)
        env.save_action_csv(episode,name)
        env.end()

        

# 상태가 입력, 큐함수가 출력인 인공신경망 생성
class DQN(tf.keras.Model):
    def __init__(self, action_size,state_size,batch_size,sequence_length):
        super(DQN, self).__init__()
        self.lstm = LSTM(64, batch_input_shape=(batch_size,sequence_length,state_size),return_sequences=False)
        self.fc1 = Dense(128, activation='relu')
        self.fc2 = Dense(128, activation='relu')
        # self.fc3 = Dense(64, activation='relu')
        self.fc_out = Dense(action_size,
                            kernel_initializer=RandomUniform(-1e-3, 1e-3))

    def call(self, x):
        x = self.lstm(x)
        x = self.fc1(x)
        x = self.fc2(x)
        # x = self.fc3(x)
        q = self.fc_out(x)
        return q


# 카트폴 예제에서의 DQN 에이전트
class DQNAgent:
    def __init__(self, state_size, action_size): 
        # 상태와 행동의 크기 정의
        self.state_size = state_size
        self.action_size = action_size
        self.batch_size = 64
        self.sequence_length =50

        # 모델과 타깃 모델 생성
        self.model = DQN(action_size,state_size,self.batch_size,self.sequence_length)
        self.model.load_weights("/home/mds/Desktop/highway_episodic/DQN/RL_validation/save_model/space4_target_model8/best_model")

    # 입실론 탐욕 정책으로 행동 선택
    def get_action(self, state, last_action):
        state = np.expand_dims(state, axis=0)  # state를 3차원 텐서로 변환        
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
    
    state_size = 26 
    action_size = 6
    # DQN 에이전트 생성
    agent = DQNAgent(state_size, action_size)

    scores,original_scores, episodes, collisions_num = [], [], [], []
    R1,R2,L1,L2 = [], [], [], []
    space1,space2,space3,space4 = [], [], [], []
    rl_run(sumoBinary, episodenum,net, sumocfg, veh, step_length)