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


from DQN_env_LC_path5_action7_5_LSTM import rlEnv


import gym
import pylab
from collections import deque
import tensorflow as tf
from tensorflow.keras.layers import Dense
from tensorflow.keras.optimizers import Adam
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
def save_result():
    col = ['episode','total_reward']
    df_result = pd.DataFrame(total_reward,columns=col)

    col2 = ['LC_succeed_num','collision_num']
    df_final_result = pd.DataFrame(final_result,columns=col2)

    path = "/home/mds/Desktop/highway_episodic/DQN/result4/"
    os.chdir(path)
    now = datetime.now()
    date = str(now.date()) +str('_')+str(now.time())
    df_result.to_csv('result_'+date+'.csv')
    df_final_result.to_csv('final_result_'+date+'.csv')


def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("-N","--episodenum", 
                        default=10001, help="numer of episode to run qlenv")
    optParser.add_option("--nogui", action="store_true",
                        default=True , help="run commandline version of sumo")
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
        # print('state: ',state)
        # print('state type: ',type(state))
        # print('state shape: ',state.shape)
        state_memory=np.zeros((agent.sequence_length-1,state_size))
        # print('state_memory: ',state_memory)
        # print('state_memory type: ',type(state_memory))
        # print('state_memory shape: ',state_memory.shape)
        next_state_memory=np.zeros((agent.sequence_length-2,state_size))
        state_memory = np.append(state_memory,state,axis = 0)
        next_state_memory= np.append(next_state_memory,state,axis = 0)

        action = agent.get_action(state_memory)
        
        while (env.done) == False:  
            
            if action == 0 or action == 1 or action == 2 or action ==3 or action ==4 or action == 7:
                # print('action:::::::::::::::::::::::::::::::::::::::::::::::::',action)
                # if action ==7:
                #     print('action:::::::::::::::::::::::::::::                ',action)
                # if action ==0:
                #     print('action:::::::::::::::::::::::::::::                ',action)
                # elif action ==1:
                #     print('action:::::::::::::::::::::::::::::            ',action)
                # elif action ==2:
                #     print('action:::::::::::::::::::::::::::::                    ',action)
                # elif action ==3:
                #     print('action:::::::::::::::::::::::::::::               ',action)
                # elif action ==4:
                #     print('action:::::::::::::::::::::::::::::                 ',action)
                next_state, reward = env.step(action) ### 1 ###
                next_state = np.reshape(next_state, [1, state_size])
                if len(next_state_memory)>agent.sequence_length-1:
                    next_state_memory = np.delete(next_state_memory,0,axis = 0)
                    next_state_memory= np.append(next_state_memory,next_state,axis = 0)
                else:
                    next_state_memory= np.append(next_state_memory,next_state,axis = 0)
                score +=reward
                # 리플레이 메모리에 샘플 <s, a, r, s'> 저장
                agent.append_sample(state_memory, action, reward, next_state_memory, env.done)
                # 매 타임스텝마다 학습
                if len(agent.memory) >= agent.train_start:
                    agent.train_model()

                state = next_state
                if len(state_memory)>agent.sequence_length-1:
                    state_memory = np.delete(state_memory,0,axis = 0)
                    state_memory = np.append(state_memory,state,axis = 0)
                else:
                    state_memory = np.append(state_memory,state,axis = 0)
                action = agent.get_action(state_memory)  
                # action =6
                # if traci.simulation.getTime() >= 0.8:
                #     action = 5


            elif action == 5:
                
                next_state, reward = env.step3(action) ### 1 ###
                # print('action:::::::::::::::::::::::::::::           5')
                next_state = np.reshape(next_state, [1, state_size])
                if len(next_state_memory)>agent.sequence_length-1:
                    next_state_memory = np.delete(next_state_memory,0,axis = 0)
                    next_state_memory= np.append(next_state_memory,next_state,axis = 0)
                else:
                    next_state_memory= np.append(next_state_memory,next_state,axis = 0)
                score +=reward
                agent.append_sample(state_memory, action, reward, next_state_memory, env.done)
                if len(agent.memory) >= agent.train_start:
                    agent.train_model()
                state = next_state
                if len(state_memory)>agent.sequence_length-1:
                    state_memory = np.delete(state_memory,0,axis = 0)
                    state_memory = np.append(state_memory,state,axis = 0)
                else:
                    state_memory = np.append(state_memory,state,axis = 0)
                action = agent.get_action(state_memory) 
                # action =6
                # if traci.simulation.getTime() >= 0.8:
                #     action = 5

            elif action == 6:
                
                next_state, reward = env.step4(action) ### 1 ###     
                # print('action:::::::::::::::::::::::::::::                      6') 
                next_state = np.reshape(next_state, [1, state_size])
                if len(next_state_memory)>agent.sequence_length-1:
                    next_state_memory = np.delete(next_state_memory,0,axis = 0)
                    next_state_memory= np.append(next_state_memory,next_state,axis = 0)
                else:
                    next_state_memory= np.append(next_state_memory,next_state,axis = 0)
                score +=reward
                agent.append_sample(state_memory, action, reward, next_state_memory, env.done)
                if len(agent.memory) >= agent.train_start:
                    agent.train_model()
                state = next_state   
                if len(state_memory)>agent.sequence_length-1:
                    state_memory = np.delete(state_memory,0,axis = 0)
                    state_memory = np.append(state_memory,state,axis = 0)
                else:
                    state_memory = np.append(state_memory,state,axis = 0)   
                action = agent.get_action(state_memory)
                # action =6
                # if traci.simulation.getTime() >= 0.8:
                #     action = 5


            if env.done:
                # env.save_csv()
                # 각 에피소드마다 타깃 모델을 모델의 가중치로 업데이트
                # if (episode+1)%5 ==0:
                agent.update_target_model()
                # 에피소드마다 학습 결과 출력
                score_avg = 0.9 * score_avg + 0.1 * score if score_avg != 0 else score
                print("episode: {:3d} | score avg: {:3.2f} | memory length: {:4d} | epsilon: {:.4f}".format(episode, score_avg, len(agent.memory), agent.epsilon))
                # 에피소드마다 학습 결과 그래프로 저장
                scores.append(score_avg)
                episodes.append(episode)
                collisions_num.append(env.collision_num)
                
                # 이동 평균이 70 이상일 때 종료
                # if score_avg > 70:
                if episode == episodenum-1 or (episode+1)%10 ==0:
                    agent.model.save_weights("/home/mds/Desktop/highway_episodic/DQN/DQN_LSTM/save_model/model", save_format="tf")
                    # with open("/home/mds/Desktop/highway_episodic/DQN/save_model20/memory/6/replaybuffer"+str(episode), "wb") as file:
                    #     pickle.dump(agent.memory, file)
                if episode == episodenum-1:
                    sys.exit()


            r += reward
            # print(reward)
        print('total reward: ',r)
        
        result.append(episode)
        result.append(r)
        
        total_reward.append(result)
        # print(env.done)
        # env.save_csv()
        env.end()
        

        if ((episode+1)%10) == 0 and not episode ==0:
            print('LC_succeed_num: ',env.LC_succeed_num)
            print('collision_num: ',env.collision_num)
            value = []
            value.append(env.LC_succeed_num)
            value.append(env.collision_num)
            final_result.append(value)
            save_result()
            plt.figure(1,figsize=(8,4))
            pylab.plot(episodes, scores, 'b')
            pylab.xlabel("episode")
            pylab.ylabel("average score")
            pylab.savefig("/home/mds/Desktop/highway_episodic/DQN/DQN_LSTM/save_graph/train_graph.png")

            plt.figure(2,figsize=(8,4))                
            pylab.plot(episodes,collisions_num,'r')
            pylab.xlabel("episode")
            pylab.ylabel("collision number")
            pylab.savefig("/home/mds/Desktop/highway_episodic/DQN/DQN_LSTM/save_graph/train_collision.png")

# 상태가 입력, 큐함수가 출력인 인공신경망 생성
class DQN(tf.keras.Model):
    def __init__(self, action_size,state_size,batch_size,sequence_length):
        super(DQN, self).__init__()
        self.lstm = LSTM(64, activation='relu', batch_input_shape=(batch_size,sequence_length,state_size),return_sequences=False)
        self.fc1 = Dense(128, activation='relu')
        self.fc2 = Dense(128, activation='relu')
        self.fc_out = Dense(action_size,
                            kernel_initializer=RandomUniform(-1e-3, 1e-3))

    def call(self, x):
        x = self.lstm(x)
        x = self.fc1(x)
        x = self.fc2(x)
        q = self.fc_out(x)
        return q


# 카트폴 예제에서의 DQN 에이전트
class DQNAgent:
    def __init__(self, state_size, action_size):
        self.render = False

        # 상태와 행동의 크기 정의
        self.state_size = state_size
        self.action_size = action_size

        # DQN 하이퍼파라미터
        self.discount_factor = 0.99
        self.learning_rate = 0.0001
        self.epsilon = 1
        self.epsilon_decay = 0.999
        self.epsilon_min = 0.001
        self.batch_size = 64
        self.train_start = 100*8

        # 리플레이 메모리, 최대 크기 100
        self.memory = deque(maxlen=3000*8)
        # <s,r,a,s'> 10

        self.sequence_length =10
        

        # 모델과 타깃 모델 생성
        self.model = DQN(action_size,state_size,self.batch_size,self.sequence_length)
        self.target_model = DQN(action_size,state_size,self.batch_size,self.sequence_length)
        self.optimizer = Adam(lr=self.learning_rate)

        # 타깃 모델 초기화
        self.update_target_model()

    # 타깃 모델을 모델의 가중치로 업데이트
    def update_target_model(self):
        self.target_model.set_weights(self.model.get_weights())

    # 입실론 탐욕 정책으로 행동 선택
    def get_action(self, state):
        state = np.expand_dims(state, axis=0)  # state를 3차원 텐서로 변환
        if np.random.rand() <= self.epsilon:
            return random.randrange(self.action_size)
        else:
            # print('state: ',state)
            # print('state type: ',type(state))
            # print('state shape: ',state.shape)
            q_value = self.model(state)
            return np.argmax(q_value[0])

    # 샘플 <s, a, r, s'>을 리플레이 메모리에 저장
    def append_sample(self,state, action, reward, next_state, done):
        # print('states shape: ',state.shape)
        # print('states type: ',type(state))
        # print('next_states shape: ',next_state.shape)
        # print('next_states type: ',type(next_state))
        self.memory.append((state, action, reward, next_state, done))
            
    # 리플레이 메모리에서 무작위로 추출한 배치로 모델 학습
    def train_model(self):
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

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
    action_size = 8
    # DQN 에이전트 생성
    agent = DQNAgent(state_size, action_size)

    scores, episodes, collisions_num = [], [], []
    


    rl_run(sumoBinary, episodenum,net, sumocfg, veh, step_length)
