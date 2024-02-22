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


from DQN_env5 import rlEnv

import gym
import pylab
from collections import deque
import tensorflow as tf
from tensorflow.keras.layers import Dense
from tensorflow.keras.optimizers import Adam
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
                        default=1801, help="numer of episode to run qlenv")
    optParser.add_option("--nogui", action="store_true",
                        default=True, help="run commandline version of sumo")
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
        action = agent.get_action(state)

        while (env.done) == False:
            
            # print(r)
            # nextstate, reward = env.step(0)
            # print('nextstate: ',nextstate)
            # print('reward: ',reward)
            
             
        
            # print('action: ',action) 
            # print('Israndom, ',Israndom)
            
            if action == 0 or action ==1 or action ==2:
                print('action:::::::::::::::::::::::::::::::::::::::::::::::::',action)
                next_state, reward = env.step(action) ### 1 ###
                next_state = np.reshape(next_state, [1, state_size])
                score +=reward

                # 리플레이 메모리에 샘플 <s, a, r, s'> 저장
                agent.append_sample(state, action, reward, next_state, env.done)
                # 매 타임스텝마다 학습
                if len(agent.memory) >= agent.train_start:
                    agent.train_model()

                state = next_state
                action = agent.get_action(state)
            elif action == 3:
                
                next_state, reward = env.step3(action) ### 1 ###
                # print('.................................................action 3 ')
                if env.done == True:
                    print('action::::::::::::::::::::::::::::::::::::::::::::::::: 3')
                    # print('next_state', next_state)
                    next_state = np.reshape(next_state, [1, state_size])
                    score +=reward

                    # 리플레이 메모리에 샘플 <s, a, r, s'> 저장
                    agent.append_sample(state, action, reward, next_state, env.done)
                    # 매 타임스텝마다 학습
                    if len(agent.memory) >= agent.train_start:
                        agent.train_model()

                    state = next_state
                    action = agent.get_action(state)

            elif action == 4:
                
                next_state, reward = env.step4(action) ### 1 ###
                # print('.................................................action 4')
                if env.done == True:
                    print('action::::::::::::::::::::::::::::::::::::::::::::::::: 4')
                    # print('next_state', next_state)
                    next_state = np.reshape(next_state, [1, state_size])
                    score +=reward

                    # 리플레이 메모리에 샘플 <s, a, r, s'> 저장
                    agent.append_sample(state, action, reward, next_state, env.done)
                    # 매 타임스텝마다 학습
                    if len(agent.memory) >= agent.train_start:
                        agent.train_model()

                    state = next_state
                    action = agent.get_action(state)

            if env.done:
                # 각 에피소드마다 타깃 모델을 모델의 가중치로 업데이트
                # if (episode+1)%5 ==0:
                agent.update_target_model()
                # 에피소드마다 학습 결과 출력
                score_avg = 0.9 * score_avg + 0.1 * score if score_avg != 0 else score
                print("episode: {:3d} | score avg: {:3.2f} | memory length: {:4d} | epsilon: {:.4f}".format(episode, score_avg, len(agent.memory), agent.epsilon))

                # 에피소드마다 학습 결과 그래프로 저장
                scores.append(score_avg)
                episodes.append(episode)
                plt.figure(1,figsize=(8,4))
                pylab.plot(episodes, scores, 'b')
                pylab.xlabel("episode")
                pylab.ylabel("average score")
                pylab.savefig("/home/mds/Desktop/highway_episodic/DQN/save_graph5/graph.png")

                plt.figure(2,figsize=(8,4))
                collisions_num.append(env.collision_num)
                pylab.plot(episodes,collisions_num,'r')
                pylab.xlabel("episode")
                pylab.ylabel("collision number")
                pylab.savefig("/home/mds/Desktop/highway_episodic/DQN/save_graph5/collision.png")
                # 이동 평균이 70 이상일 때 종료
                # if score_avg > 70:
                if episode == episodenum-1:
                    agent.model.save_weights("/home/mds/Desktop/highway_episodic/DQN/save_model5/model", save_format="tf")
                    sys.exit()


            r += reward
            # print(reward)
        print('total reward: ',r)
        
        result.append(episode)
        result.append(r)
        
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

# 상태가 입력, 큐함수가 출력인 인공신경망 생성
class DQN(tf.keras.Model):
    def __init__(self, action_size):
        super(DQN, self).__init__()
        self.fc1 = Dense(128, activation='relu')
        self.fc2 = Dense(128, activation='relu')
        self.fc_out = Dense(action_size,
                            kernel_initializer=RandomUniform(-1e-3, 1e-3))

    def call(self, x):
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
        self.learning_rate = 0.001
        self.epsilon = 1
        self.epsilon_decay = 0.999
        self.epsilon_min = 0.01
        self.batch_size = 64*4
        self.train_start = 1000

        # 리플레이 메모리, 최대 크기 100
        self.memory = deque(maxlen=2000)

        # 모델과 타깃 모델 생성
        self.model = DQN(action_size)
        self.target_model = DQN(action_size)
        self.optimizer = Adam(lr=self.learning_rate)

        # 타깃 모델 초기화
        self.update_target_model()

    # 타깃 모델을 모델의 가중치로 업데이트
    def update_target_model(self):
        self.target_model.set_weights(self.model.get_weights())

    # 입실론 탐욕 정책으로 행동 선택
    def get_action(self, state):
        if np.random.rand() <= self.epsilon:
            return random.randrange(self.action_size)
        else:
            q_value = self.model(state)
            return np.argmax(q_value[0])

    # 샘플 <s, a, r, s'>을 리플레이 메모리에 저장
    def append_sample(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    # 리플레이 메모리에서 무작위로 추출한 배치로 모델 학습
    def train_model(self):
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

        # 메모리에서 배치 크기만큼 무작위로 샘플 추출
        mini_batch = random.sample(self.memory, self.batch_size)

        states = np.array([sample[0][0] for sample in mini_batch])
        actions = np.array([sample[1] for sample in mini_batch])
        rewards = np.array([sample[2] for sample in mini_batch])
        next_states = np.array([sample[3][0] for sample in mini_batch])
        dones = np.array([sample[4] for sample in mini_batch])
        
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
    
    state_size = 16 
    action_size = 5
    # DQN 에이전트 생성
    agent = DQNAgent(state_size, action_size)

    scores, episodes, collisions_num = [], [], []
    


    rl_run(sumoBinary, episodenum,net, sumocfg, veh, step_length)
