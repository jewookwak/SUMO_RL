from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import random
import traci
import pickle
from datetime import datetime
import numpy as np
import matplotlib.pyplot as plt  
import pandas as pd
from xml.etree.ElementTree import parse
from collections import defaultdict

from matplotlib.ticker import MaxNLocator

# from env_big_lateral_reward_random_space import rlEnv
# from env_total_random_with_normalized_new_states_based_on_paper_200ms import rlEnv
from env_total_random_with_normalized_new_states_based_on_paper_preLC_fix_new_last_SD_visualization_all_actions import rlEnv


import pylab
from collections import deque
import tensorflow as tf
from tensorflow.keras.layers import Concatenate
from tensorflow.keras.layers import Dense
from tensorflow.keras.optimizers import Adam
from tensorflow.keras.initializers import RandomUniform
from tensorflow.keras.layers import LSTM, Conv1D, MaxPool1D

from pathlib import Path

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'],'tools')
    print(tools)
    sys.path.append(tools)
    import sumolib 
else:
    sys.exit('Declare environment variable "SUMO_HOME"')

total_reward =[]
final_result =[]
Loss = []


def save_results(episodes, original_scores, scores, collisions_num,R1,R2,L1,L2, space1,space2,space3,space4):
    name = str(Path(__file__).name)
    file_path = '/home/mds/Desktop/highway_episodic/DQN/RL_validation/data_log/data/results/'+name
    os.makedirs(file_path, exist_ok=True)
    os.chdir(file_path)
    # col_reward=['episode','reward','weight_reward']
    # col_collision = ['episode','collision']
    # col_space_size= ['episode','space1','space2','space3','space4']
    # col_space_location= ['episode','l1','l2','r1','r2']
    df_episodes = pd.DataFrame(episodes)
    df_original_scores = pd.DataFrame(original_scores)
    df_scores = pd.DataFrame(scores)
    df_collisions_num = pd.DataFrame(collisions_num)
    df_R1 = pd.DataFrame(R1)
    df_R2 = pd.DataFrame(R2)
    df_L1 = pd.DataFrame(L1)
    df_L2 = pd.DataFrame(L2)
    df_space1 = pd.DataFrame(space1)
    df_space2 = pd.DataFrame(space2)
    df_space3 = pd.DataFrame(space3)
    df_space4 = pd.DataFrame(space4)

    df_episodes.to_csv('log_data_episodes')
    df_original_scores.to_csv('log_data_original_scores')
    df_scores.to_csv('log_data_scores')
    df_collisions_num.to_csv('log_data_collisions_num')
    df_R1.to_csv('log_data_R1')
    df_R2.to_csv('log_data_R2')
    df_L1.to_csv('log_data_L1')
    df_L2.to_csv('log_data_L2')
    df_space1.to_csv('log_data_space1')
    df_space2.to_csv('log_data_space2')
    df_space3.to_csv('log_data_space3')
    df_space4.to_csv('log_data_space4')
    
def save_loss(episode):
    name = str(Path(__file__).name)
    file_path = '/home/mds/Desktop/highway_episodic/DQN/RL_validation/data_log/data/'+'/'+name+'/'+str(episode)
    os.makedirs(file_path, exist_ok=True)
    os.chdir(file_path)
    df_loss = pd.DataFrame(Loss)

    df_loss.to_csv('loss_'+str(episode)+'.csv')
# def save_result():
#     col = ['episode','total_reward']
#     df_result = pd.DataFrame(total_reward,columns=col)

#     col2 = ['LC_succeed_num','collision_num']
#     df_final_result = pd.DataFrame(final_result,columns=col2)

#     path = "/home/mds/Desktop/highway_episodic/DQN/RL_validation/result/"
#     os.chdir(path)
#     now = datetime.now()
#     date = str(now.date()) +str('_')+str(now.time())
#     df_result.to_csv('result_'+date+'.csv')
#     df_final_result.to_csv('final_result_'+date+'.csv')
def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("-N","--episodenum", 
                        default=10001, help="numer of episode to run qlenv")
    optParser.add_option("--nogui", action="store_true",
                        default=False , help="run commandline version of sumo")
    options, args = optParser.parse_args()
    return options 

def rl_run(sumoBinary, episodenum,net, sumocfg,step_length, veh):  
    
    
    score_avg = 0    
    max_score_avg = 0
    for episode in range(1,episodenum):

        score = 0
        result =[]
        env.done = False
        reward_memory =0
        episode_total_reward = 0 
        print("\n********#{} episode start***********".format(episode))
        #reset environment
        
        state = env.reset()
        state = np.reshape(state, [1, state_size])
        # print('state: ',state)
        # print('state type: ',type(state))
        # print('state shape: ',state.shape)
        state_memory=np.zeros((agent.sequence_length-1,state_size))
       
        next_state_memory=np.zeros((agent.sequence_length-2,state_size))
        # state_memory = np.append(state_memory,state,axis = 0)
        # print('state_memory: ',state_memory)
        # print('state_memory type: ',type(state_memory))
        # print('state_memory shape: ',state_memory.shape)

        next_state_memory= np.append(next_state_memory,state,axis = 0)

        last_action = None
        while (env.done) == False:
            if (np.round(traci.simulation.getTime(),2)*decision_frequncy)%1 == 0: #판단 주기: 1000ms 1번 판단
       
                    action = agent.get_action(state_memory)
                    if action == 5:
                        action = 7 
                    if action == 0 or action == 1 or action == 2 or action ==3 or action ==4 or action == 7:
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
                        
                        next_state, reward = env.step(action) ### 1 ###
                        reward_memory +=reward
                        # print(next_state)
                        next_state = np.reshape(next_state, [1, state_size])
                        if len(next_state_memory)>agent.sequence_length-1:
                            next_state_memory = np.delete(next_state_memory,0,axis = 0)
                            next_state_memory= np.append(next_state_memory,next_state,axis = 0)
                        else:
                            next_state_memory= np.append(next_state_memory,next_state,axis = 0)
                        #reward clipping
                        if reward_memory>20:
                            reward_memory = 20
                        if reward_memory <-20:
                            reward_memory = -20
                        score +=reward_memory
                        # if score > 150:
                        #     score =150
                        # if score < -150:
                        #     score = -150
                        # 리플레이 메모리에 샘플 <s, a, r, s'> 저장
                        if not np.all(state_memory[0] == 0):
                            if action==0:
                                agent.append_sample0(state_memory, action, reward_memory, next_state_memory, env.done)
                            elif action ==1:
                                agent.append_sample1(state_memory, action, reward_memory, next_state_memory, env.done)
                            elif action ==2:
                                agent.append_sample2(state_memory, action, reward_memory, next_state_memory, env.done)
                            elif action ==3:
                                agent.append_sample3(state_memory, action, reward_memory, next_state_memory, env.done)
                            elif action ==4:
                                agent.append_sample4(state_memory, action, reward_memory, next_state_memory, env.done)
                            elif action ==7:
                                agent.append_sample7(state_memory, action, reward_memory, next_state_memory, env.done)
                            agent.append_sample()
                        # 매 타임스텝마다 학습
                        if len(agent.memory) >= agent.train_start:
                            agent.train_model(episode)

                        state = next_state
                        if len(state_memory)>agent.sequence_length-1:
                            state_memory = np.delete(state_memory,0,axis = 0)
                            state_memory = np.append(state_memory,state,axis = 0)
                        else:
                            state_memory = np.append(state_memory,state,axis = 0)

                        last_action = action
                        episode_total_reward += reward_memory
                        reward_memory = 0
            else:
                next_state,reward = env.step(last_action)
                next_state = np.reshape(next_state, [1, state_size])
                if len(next_state_memory)>agent.sequence_length-1:
                    next_state_memory = np.delete(next_state_memory,0,axis = 0)
                    next_state_memory= np.append(next_state_memory,next_state,axis = 0)
                else:
                    next_state_memory= np.append(next_state_memory,next_state,axis = 0)
                state = next_state
                if len(state_memory)>agent.sequence_length-1:
                    state_memory = np.delete(state_memory,0,axis = 0)
                    state_memory = np.append(state_memory,state,axis = 0)
                else:
                    state_memory = np.append(state_memory,state,axis = 0)

                reward_memory += reward

            # if len(agent.memory) >= agent.train_start and np.round(traci.simulation.getTime(),2)%1 == 0:
            #     print('update target model')
            #     agent.update_target_model()
                


            if env.done:
                # env.save_csv()
                # 각 에피소드마다 타깃 모델을 모델의 가중치로 업데이트
                # if (episode)%5 ==0:
                # agent.update_target_model()
                if len(agent.memory) >= agent.train_start and (episode)%1 == 0:
                    agent.update_target_model()
                # 에피소드마다 학습 결과 출력
                score_avg = 0.9 * score_avg + 0.1 * score if score_avg != 0 else score
                print("episode: {:3d} | score avg: {:3.2f} | memory length: {:4d} | epsilon: {:.4f}".format(episode, score_avg, len(agent.memory), agent.epsilon))
                print("episode: {} | collision: {} | space size 1st: {} | space size 2nd: {} | space size 3rd: {} | space size 4th: {}".format(episode, env.collision_num,env.space1_count, env.space2_count,env.space3_count,env.space4_count))
                print("episode: {} | collision: {} | R1 space: {} | R2 space: {} | L1 space: {} | L2 space: {}".format(episode, env.collision_num,env.R1_count, env.R2_count,env.L1_count,env.L2_count))
                # 에피소드마다 학습 결과 그래프로 저장
                original_scores.append(score)
                scores.append(score_avg)
                episodes.append(episode)
                collisions_num.append(env.collision_num)
                R1.append(env.R1_count)
                R2.append(env.R2_count)
                L1.append(env.L1_count)
                L2.append(env.L2_count)
                space1.append(env.space1_count)
                space2.append(env.space2_count)
                space3.append(env.space3_count)
                space4.append(env.space4_count)
                
                # 이동 평균이 better than last one.
                name = str(Path(__file__).name)
                if score_avg >max_score_avg and len(agent.memory) >= agent.train_start:
                    agent.model.save_weights("/home/mds/Desktop/highway_episodic/DQN/RL_validation/save_model/"+name+"/best_model", save_format="tf")
                    print('max_score_avg: ',max_score_avg)
                    max_score_avg =max([max_score_avg,score_avg])
                if episode == episodenum-1 or (episode)%50 ==0:
                    agent.model.save_weights("/home/mds/Desktop/highway_episodic/DQN/RL_validation/save_model/"+name+"/model_"+str(episode), save_format="tf")
                    # with open("/home/mds/Desktop/highway_episodic/DQN/sav e_model20/memory/6/replaybuffer"+str(episode), "wb") as file:
                    #     pickle.dump(agent.memory, file)
                if episode == episodenum-1:
                    sys.exit()
                

            
            # print(reward)
        print('total reward: ',episode_total_reward)
        
        result.append(episode)
        result.append(episode_total_reward)
        
        total_reward.append(result)
        # print(env.done)
        # name = str(Path(__file__).name)
        env.save_csv(episode,name)
        # env.save_action_csv(episode,name)
        
        # Loss.append(episode)
        save_loss(episode)
        Loss.clear

        env.end()
        

        if ((episode)%10) == 0 and not episode ==0:
            # print('LC_succeed_num: ',env.LC_succeed_num)
            # print('collision_num: ',env.collision_num)
            value = []
            value.append(env.LC_succeed_num)
            value.append(env.collision_num)
            final_result.append(value)
            # save_result()
            save_results(episodes, original_scores, scores, collisions_num,R1,R2,L1,L2, space1,space2,space3,space4)
            
            # plt.figure(1,figsize=(8,4))
            
            # pylab.plot(episodes, original_scores, 'cyan',label='score')
            # pylab.plot(episodes, scores, 'b',label='average score')
            # pylab.xlabel("episode")
            # pylab.ylabel("score")
            # pylab.title('Reward')
            # pylab.savefig("/home/mds/Desktop/highway_episodic/DQN/RL_validation/save_graph/reward.png")

            # plt.figure(2,figsize=(8,4))                
            # pylab.plot(episodes,collisions_num,'r')
            # pylab.xlabel("episode")
            # pylab.ylabel("collision number")
            # pylab.title('Collision number')
            # pylab.savefig("/home/mds/Desktop/highway_episodic/DQN/RL_validation/save_graph/collision.png")

            # plt.figure(3,figsize=(8,4))
            # # pylab.plot(episodes,collisions_num,'r',label='collision')
            # plt.plot(episodes, R1, 'orange', label='R1')
            # plt.plot(episodes, R2, 'g', label='R2')
            # plt.plot(episodes, L1, 'b', label='L1')
            # plt.plot(episodes, L2, 'm', label='L2')
            # pylab.legend()
            # pylab.xlabel("episode")
            # pylab.ylabel("succeed space ")
            # pylab.title('Lane change success space name')
            # pylab.savefig("/home/mds/Desktop/highway_episodic/DQN/RL_validation/save_graph/LC_success_spaces.png")
            # plt.close()
            # plt.figure(4,figsize=(8,4))
            # # pylab.plot(episodes,collisions_num,'r',label='collision')
            # plt.plot(episodes, space1, 'orange', label='1st')
            # plt.plot(episodes, space2, 'g', label='2nd')
            # plt.plot(episodes, space3, 'b', label='3rd')
            # plt.plot(episodes, space4, 'm', label='4th')
            # pylab.legend()
            # pylab.xlabel("episode")
            # pylab.ylabel("succeed space ")
            # pylab.title('Lane change success space size')
            # pylab.savefig("/home/mds/Desktop/highway_episodic/DQN/RL_validation/save_graph/LC_success_spaces_size.png")
            # plt.close()

# 상태가 입력, 큐함수가 출력인 인공신경망 생성
class DQN(tf.keras.Model):
    def __init__(self, action_size,state_size,batch_size,sequence_length):
        super(DQN, self).__init__()
        self.lstm1 = LSTM(64, batch_input_shape=(batch_size,sequence_length,3),return_sequences=False,unroll=True)
        self.lstm2 = LSTM(64, batch_input_shape=(batch_size,sequence_length,3),return_sequences=False,unroll=True)
        self.lstm3 = LSTM(64, batch_input_shape=(batch_size,sequence_length,3),return_sequences=False,unroll=True) #spaces lstm

        self.conv1 = Conv1D(filters=32,kernel_size=1,strides=1,activation='relu')
        self.conv2 = Conv1D(filters=32,kernel_size=1,strides=1,activation='relu')
        self.maxpool = MaxPool1D(pool_size=8,strides=1)
        self.conv3 = Conv1D(filters=32,kernel_size=1,strides=1,activation='relu')
        self.conv4 = Conv1D(filters=32,kernel_size=1,strides=1,activation='relu')
        self.conv5 = Conv1D(filters=32,kernel_size=1,strides=1,activation='relu')
        self.maxpool2 = MaxPool1D(pool_size=5,strides=1)

        self.fc1 = Dense(48, activation='relu')
        # self.fc3 = Dense(64, activation='relu')
        self.fc_out = Dense(action_size,
                            kernel_initializer=RandomUniform(-1e-3, 1e-3))

    def call(self, x):
        # print('x(vehicle): ',x[:,:,0:3].shape)  #batch_size(32) x sequence_length(30) x state(3)
        # print(type(x[:,:,0:3]))
        # print(x[:,:,0:3])
        # print('x2: ',x[:,:,3:6])

        l1_1 = self.lstm1(x[:,:,0:3])
        l1_2 = self.lstm1(x[:,:,3:6])
        l1_3 = self.lstm1(x[:,:,6:9])
        l1_4 = self.lstm1(x[:,:,9:12])
        l1_5 = self.lstm1(x[:,:,12:15])
        l1_6 = self.lstm1(x[:,:,15:18])
        l1_7 = self.lstm1(x[:,:,18:21])
        l1_8 = self.lstm1(x[:,:,21:24])

        l2 = self.lstm2(x[:,:,24:28])      #ego vehicle state

        # print('x(space): ',x[:,:,28:32])
        # print('x(space): ',x[:,:,28:32].shape) #batch_size(32) x sequence_length(30) x state(4)
        l3_1 = self.lstm3(x[:,:,28:32])   
        l3_2 = self.lstm3(x[:,:,32:36]) 
        l3_3 = self.lstm3(x[:,:,36:40]) 
        l3_4 = self.lstm3(x[:,:,40:44]) 
        l3_mid = self.lstm3(x[:,:,44:48]) 

        # print('l2: ',l2.shape) #batch_size(32) x hidden_state_num(64)

        h_v = Concatenate(axis=1)([tf.expand_dims(l1_1, axis=1), tf.expand_dims(l1_2, axis=1), tf.expand_dims(l1_3, axis=1), tf.expand_dims(l1_4, axis=1), tf.expand_dims(l1_5, axis=1), tf.expand_dims(l1_6, axis=1), tf.expand_dims(l1_7, axis=1), tf.expand_dims(l1_8, axis=1)])
        # print('h_v',h_v.shape) #batch_size(32) x vehicle_num(8) x hidden_state_num(64)
        h_v = self.conv1(h_v) 
        # print('h_v: ',h_v.shape)#batch_size(32) x vehicle_num(8) x hidden_state_num(32)
        h_v = self.conv2(h_v) 
        # print('h_v: ',h_v.shape)#batch_size(32) x vehicle_num(8) x hidden_state_num(32)
        h_1 = self.maxpool(h_v)
        # print('h_1: ',h_1.shape) #batch_size(32) x maxPool(1) x  hidden_state_num(32)        
        
        h_EV = tf.expand_dims(l2, axis=1)
        # print('h_EV: ',h_EV.shape) #batch_size(32) x ego(1) x  hidden_state_num(64)
        h_2 = self.conv3(h_EV)
        # print('h_2: ',h_2.shape) #batch_size(32) x ego(1) x  hidden_state_num(32)


        # print('l3_1: ',l3_1.shape) #batch_size(32) x hidden_state_num(64)
        h_s = Concatenate(axis=1)([tf.expand_dims(l3_1, axis=1), tf.expand_dims(l3_2, axis=1), tf.expand_dims(l3_3, axis=1), tf.expand_dims(l3_4, axis=1),tf.expand_dims(l3_mid, axis=1)])
        # print('h_s',h_s.shape) #batch_size(32) x space_num(5) x hidden_state_num(64)
        h_s = self.conv4(h_s) 
        # print('h_s: ',h_s.shape)#batch_size(32) x space_num(5) x hidden_state_num(32)
        h_s = self.conv5(h_s) 
        # print('h_s: ',h_s.shape)#batch_size(32) x space_num(5) x hidden_state_num(32)
        h_3 = self.maxpool2(h_s)
        # print('h_3: ',h_3.shape) #batch_size(32) x maxPool(1) x  hidden_state_num(32)   



        x = Concatenate(axis=2)([h_1,h_2, h_3])
        # print('x: ',x.shape) #batch_size(32) x 1 x total_hidden_state_num(96)
        x =tf.squeeze(x, axis=1)
        # print('x: ',x.shape)#batch_size(32) x total_hidden_state_num(96)
        x = self.fc1(x)
        # print('x: ',x.shape)#batch_size(32) x total_hidden_state_num(48)
        q = self.fc_out(x)
        # print('q: ',q.shape)#batch_size(32) x action_num(6)
        return q


# 카트폴 예제에서의 DQN 에이전트
class DQNAgent:
    def __init__(self, state_size, action_size):
        self.render = False

        # 상태와 행동의 크기 정의
        self.state_size = state_size
        self.action_size = action_size

        # DQN 하이퍼파라미터
        self.discount_factor = 0.99 #가감률 미래의 보상. 작으면 현재의 즉각적인 보상에 더 관심을 갖음.
        self.learning_rate = 0.01
        self.epsilon = 1
        self.epsilon_decay = 0.995
        self.epsilon_min = 0.01
        self.batch_size = 32
        self.train_start = 64
        self.memory = deque(maxlen=5000*6)
        self.memory0 = deque(maxlen= 5000)
        self.memory1 = deque(maxlen= 5000)
        self.memory2 = deque(maxlen= 5000)
        self.memory3 = deque(maxlen= 5000)
        self.memory4 = deque(maxlen= 5000)
        self.memory7 = deque(maxlen= 5000)

        
        # <s,r,a,s'> 10

        # self.sequence_length =5
        self.sequence_length = 100
        

        # 모델과 타깃 모델 생성
        self.model = DQN(action_size,state_size,self.batch_size,self.sequence_length)
        self.target_model = DQN(action_size,state_size,self.batch_size,self.sequence_length)
        # self.model.load_weights("/home/mds/Desktop/highway_episodic/DQN/RL_validation/save_model/model_1000")
        # self.target_model.load_weights("/home/mds/Desktop/highway_episodic/DQN/RL_validation/save_model/best_model")
        self.optimizer = Adam(learning_rate=self.learning_rate)
        # 타깃 모델 초기화
        self.update_target_model()

    # 타깃 모델을 모델의 가중치로 업데이트
    def update_target_model(self):
        self.target_model.set_weights(self.model.get_weights())

    # 입실론 탐욕 정책으로 행동 선택 
    def get_action(self, state):
        if np.all(state[0] == 0): #RL start after state date is accumulated for sequence length
            # print("state가 squence length 만큼 채워지지 않음.")
            # print(state[0])
            return 0
        else:
            state = np.expand_dims(state, axis=0)  # state를 3차원 텐서로 변환
            # print('get_action_state: ',state)
            if np.random.rand() <= self.epsilon:
                print('random action is')    
                return random.randrange(self.action_size)    
                
            else:
                # print('state: ',state)
                # print('state type: ',type(state))
                # print('state shape: ',state.shape)
                q_value = self.model(state)
                # print('Q shape: ',q_value.shape)
                # print('q_value: ',q_value[0])
                print('optimal action is ',np.argmax(q_value[0]))
                return np.argmax(q_value[0])

    # 샘플 <s, a, r, s'>을 리플레이 메모리에 저장
    def append_sample(self):
        # memory_limit = 2
        memory_limit = 100
        if len(self.memory0)>=memory_limit and len(self.memory1)>=memory_limit and len(self.memory2)>=memory_limit and len(self.memory3)>=memory_limit and len(self.memory4)>=memory_limit and len(self.memory7)>=memory_limit:
            # print('memory append!!!!!!!!!!!!!!!!!!!')
            self.memory+=self.memory0
            self.memory+=self.memory1
            self.memory+=self.memory2
            self.memory+=self.memory3
            self.memory+=self.memory4
            self.memory+=self.memory7

            #clear() have to be use in random environment unless it can't be converge to an optimal action
            # self.memory0.clear()
            # self.memory1.clear()
            # self.memory2.clear()
            # self.memory3.clear()
            # self.memory4.clear()
            # self.memory7.clear()

    # 샘플 <s, a, r, s'>을 각 action에 따른 리플레이 메모리에 저장    
    def append_sample0(self, state, action, reward, next_state, done):
        self.memory0.append((state, action, reward, next_state, done))
    def append_sample1(self, state, action, reward, next_state, done):
        self.memory1.append((state, action, reward, next_state, done))
    def append_sample2(self, state, action, reward, next_state, done):
        self.memory2.append((state, action, reward, next_state, done))
    def append_sample3(self, state, action, reward, next_state, done):
        self.memory3.append((state, action, reward, next_state, done))
    def append_sample4(self, state, action, reward, next_state, done):
        self.memory4.append((state, action, reward, next_state, done))
    def append_sample7(self, state, action, reward, next_state, done):
        if action == 7:
            action =5
        self.memory7.append((state, action, reward, next_state, done))
            
    # 리플레이 메모리에서 무작위로 추출한 배치로 모델 학습
    def train_model(self,episode):
        # print(np.round(traci.simulation.getTime(),2))
        # print(np.round(traci.simulation.getTime(),2) == 0.01)
        if self.epsilon > self.epsilon_min and np.round(traci.simulation.getTime(),2) == 1.01 and (episode)%1 == 0:
        # if self.epsilon > self.epsilon_min :
            self.epsilon *= self.epsilon_decay
            print('epsilon decay!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        # if (episode)%250 ==0:
        #     self.epsilon = 0.3
        #     self.epsilon_decay = 0.9
        

        # 메모리에서 배치 크기만큼 무작위로 샘플 추출
        mini_batch = random.sample(self.memory, self.batch_size)
        # print(mini_batch)

        states = np.array([sample[0] for sample in mini_batch])
        actions = np.array([sample[1] for sample in mini_batch])
        rewards = np.array([sample[2] for sample in mini_batch])
        next_states = np.array([sample[3] for sample in mini_batch])
        dones = np.array([sample[4] for sample in mini_batch])
        # print('next_states shape: ',next_states.shape)
        # print('next_states type: ',type(next_states))
        # 학습 파라메터
        model_params = self.model.trainable_variables
        with tf.GradientTape() as tape:
            # 현재 상태에 대한 모델의 큐함수
            predicts = self.model(states)
            one_hot_action = tf.one_hot(actions, self.action_size)
            predicts = tf.reduce_sum(one_hot_action * predicts, axis=1)

            # 다음 상태에 대한 타깃 모델의 큐함수
            target_predicts = self.target_model(next_states)
            target_predicts = tf.stop_gradient(target_predicts)

            # 벨만 최적 방정식을 이용한 업데이트 타깃
            max_q = np.amax(target_predicts, axis=-1)
            targets = rewards + (1 - dones) * self.discount_factor * max_q
            loss = tf.reduce_mean(tf.square(targets - predicts))

        # 오류함수를 줄이는 방향으로 모델 업데이트
        grads = tape.gradient(loss, model_params)
        self.optimizer.apply_gradients(zip(grads, model_params))
        Loss.append(loss.numpy())

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
    decision_frequncy = 1 # 판단 주기: 20 step에 한번(20*10m에 한번 판단 내림)
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
    
    state_size = 28+20 #states based on paper + states of spaces 
    action_size = 6
    # DQN 에이전트 생성
    agent = DQNAgent(state_size, action_size)

    scores,original_scores, episodes, collisions_num = [], [], [], []
    R1,R2,L1,L2 = [], [], [], []
    space1,space2,space3,space4 = [], [], [], []


    rl_run(sumoBinary, episodenum,net, sumocfg, veh, step_length)
