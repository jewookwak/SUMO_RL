# main.py
import os
import sys
import argparse
import optparse
import random
import traci 
from datetime import datetime
import numpy as np
import matplotlib.pyplot as plt  
import pandas as pd
from envs.sumo_env import rlEnv
from envs.sumo_rulebase_env import rulebaseEnv
from envs.sumo_simple_env import simpleEnv
from envs.validation.validation_env1 import validationEnv1
from envs.validation.validation_env2 import validationEnv2
from envs.validation.validation_env3 import validationEnv3
from envs.validation.validation_env4 import validationEnv4
from envs.test_env import testEnv
# 기본 구성요소
from train.config import Config
from train.trainer.trainer import DQNTrainer ,DQNIQNTrainer
from train.trainer.LSTM_trainer import LSTMDQNTrainer
from train.trainer.LSTM_prev_nstep_trainer import PREVNSTEPLSTMDQNTrainer
from train.trainer.LSTM_autoencoder_trainer import LSTMAutoencoderTrainer
from train.trainer.LSTM_prev_nstep_autoencoder_trainer import PREVNSTEPLSTMAutoencoderTrainer
from train.trainer.LSTM_Rainbow_trainer import LSTMRainbowTrainer
from train.trainer.LSTM_prev_nstep_Rainbow_trainer import PREVNSTEPLSTMRainbowTrainer

from test.test_config import TestConfig
from test.tester import Tester
from test.evaluator import DQNEvaluator
from test.rulebase import RuleBase

from data_collection.collection import Collection

# 에이전트 클래스들
from train.agent.dqn_per_agent import DQNPERAgent
from train.agent.dqn_nstep_agent import DQNNStepAgent
from train.agent.dqn_per_nstep_agent import DQNPERNStepAgent
from train.agent.dqn_iqn_agent import DQNIQNAgent
from train.agent.dqn_rainbow_agent import DQNRainbowAgent
from train.agent.lstm_dqn_per_agent import LSTMDQNPERAgent
from train.agent.lstm_dqn_nstep_agent import LSTMDQNNStepAgent
from train.agent.lstm_dqn_per_nstep_agent import LSTMDQNPERNStepAgent
from train.agent.lstm_dqn_iqn_agent import LSTMDQNIQNAgent
from train.agent.lstm_dqn_rainbow_agent import LSTMDQNRainbowAgent
from train.agent.lstm_prev_nstep_per_nstep_agent import LSTMPrevNStepPERNStepAgent
from train.agent.lstm_prev_nstep_autoencoder_per_nstep_agent import LSTMPrevNStepAutoencoderPERNStepAgent
from train.agent.lstm_prev_nstep_iqn_agent import LSTMPrevNStepIQNAgent
from train.agent.lstm_prev_nstep_autoencoder_iqn_agent import LSTMPrevNStepAutoencoderIQNAgent
from train.agent.lstm_prev_nstep_rainbow_agent import LSTMPrevNStepRainbowAgent

from pathlib import Path

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'],'tools')
    sys.path.append(tools)
    import sumolib 
else:
    sys.exit('Declare environment variable "SUMO_HOME"')

def parse_args():
    parser = argparse.ArgumentParser(description='DQN for CartPole')
    # 기존 인자들
    parser.add_argument('--eval', action='store_true', help='Run evaluation')
    parser.add_argument('--render', action='store_true', help='Render environment during evaluation')
    parser.add_argument('--episodes', type=int, default=10, help='Number of simulation episodes')
    parser.add_argument('--rulebase', action='store_true', help='Run rulebase Anti-RVC')
    parser.add_argument('--simple_env', action='store_true', help='Run simple evironment')
    parser.add_argument('--validation_env1', action='store_true', help='Run validation1 evironment')
    parser.add_argument('--validation_env2', action='store_true', help='Run validation2 evironment')
    parser.add_argument('--validation_env3', action='store_true', help='Run validation3 evironment')
    parser.add_argument('--validation_env4', action='store_true', help='Run validation4 evironment')
    parser.add_argument('--test_env', action='store_true', help='Run test evironment')
    # Collect data (rear accel vehicle)
    parser.add_argument('--collect_data', action='store_true', help='Collect data')
    # LSTM 테스트 관련 인자 추가
    parser.add_argument('--test', action='store_true', help='Test trained model')
    parser.add_argument('--weight_path', type=str, default='model.h5', help='Filename of trained model weights')
    # 알고리즘 관련 인자 그룹 생성
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--dqn', action='store_true', help='Run basic DQN')
    group.add_argument('--lstm', action='store_true', help='Run LSTM DQN')
    group.add_argument('--per', action='store_true', help='Run DQN with Prioritized Experience Replay')
    group.add_argument('--nstep', action='store_true', help='Run DQN with n-step returns')
    group.add_argument('--per_nstep', action='store_true', help='Run DQN with PER and n-step')
    group.add_argument('--iqn', action='store_true', help='Run DQN with Implicit Quantile Networks (IQN)')
    group.add_argument('--rainbow', action='store_true', help='Run DQN with Rainbow (IQN + PER + n-step + Dueling + Noisy Net)')
    group.add_argument('--lstm_per', action='store_true', help='Run LSTM DQN with PER')
    group.add_argument('--lstm_nstep', action='store_true', help='Run LSTM DQN with n-step')
    group.add_argument('--lstm_per_nstep', action='store_true', help='Run LSTM DQN with PER and n-step')
    group.add_argument('--lstm_iqn', action='store_true', help='Run LSTM DQN with Implicit Quantile Networks (IQN)')    
    group.add_argument('--lstm_rainbow', action='store_true', help='Use LSTM Rainbow DQN')
    group.add_argument('--lstm_autoencoder', action='store_true', help='Run LSTM_Autoencoder DQN')
    group.add_argument('--prev_nstep_lstm', action='store_true', help='Run Prev nstep LSTM DQN')
    group.add_argument('--prev_nstep_lstm_autoencoder', action='store_true', help='Run Prev nstep LSTM_Autoencoder DQN')
    group.add_argument('--prev_nstep_lstm_per_nstep', action='store_true', help='Run Prev N-Step LSTM DQN with PER and n-step')
    group.add_argument('--prev_nstep_lstm_autoencoder_per_nstep', action='store_true', help='Run Prev N-Step LSTM Autoencoder DQN with PER and n-step')
    group.add_argument('--prev_nstep_lstm_iqn', action='store_true', help='Run Prev N-Step LSTM DQN with IQN')
    group.add_argument('--prev_nstep_lstm_autoencoder_iqn', action='store_true', help='Run Prev N-Step LSTM Autoencoder DQN with IQN')
    group.add_argument('--prev_nstep_lstm_rainbow', action='store_true', help='Run Prev N-Step LSTM with Rainbow DQN (IQN + PER + n-step + Dueling + Noisy)')
    # group.add_argument('--prev_nstep_lstm_autoencoder_rainbow', action='store_true', help='Run Prev N-Step LSTM Autoencoder with Rainbow DQN')

    return parser.parse_args()

def main():
    args = parse_args()
    base_dir = os.getcwd()
    # print("base: ",base_dir)
    net = base_dir+"/envs/config/highway_episodic.net.xml"
    sumocfg = base_dir+"/envs/config/highway_episodic.sumocfg"
    veh = "ego"
    
    # 환경 설정 코드는 그대로 유지...
    if args.eval and args.render:
        sumoBinary = sumolib.checkBinary('sumo-gui')
        env = rlEnv(sumoBinary, net_file = net, cfg_file = sumocfg, veh = veh)
        env.gui_on =True
    elif args.rulebase and args.render:
        sumoBinary = sumolib.checkBinary('sumo-gui')
        env = rulebaseEnv(sumoBinary, net_file = net, cfg_file = sumocfg, veh = veh)
        env.gui_on =True
    elif args.rulebase:
        sumoBinary = sumolib.checkBinary('sumo')
        env = rulebaseEnv(sumoBinary, net_file = net, cfg_file = sumocfg, veh = veh)
        env.gui_on =False
    elif args.simple_env and args.render:
        sumoBinary = sumolib.checkBinary('sumo-gui')
        env = simpleEnv(sumoBinary, net_file = net, cfg_file = sumocfg, veh = veh)
        env.gui_on =True
    elif args.simple_env:
        sumoBinary = sumolib.checkBinary('sumo')
        env = simpleEnv(sumoBinary, net_file = net, cfg_file = sumocfg, veh = veh)
        env.gui_on =False
    elif args.validation_env1 and args.render:
        sumoBinary = sumolib.checkBinary('sumo-gui')
        env = validationEnv1(sumoBinary, net_file = net, cfg_file = sumocfg, veh = veh)
        env.gui_on =True
    elif args.validation_env1:
        sumoBinary = sumolib.checkBinary('sumo')
        env = validationEnv1(sumoBinary, net_file = net, cfg_file = sumocfg, veh = veh)
        env.gui_on =False
    elif args.validation_env2 and args.render:
        sumoBinary = sumolib.checkBinary('sumo-gui')
        env = validationEnv2(sumoBinary, net_file = net, cfg_file = sumocfg, veh = veh)  
        env.gui_on =True
    elif args.validation_env2:
        sumoBinary = sumolib.checkBinary('sumo')
        env = validationEnv2(sumoBinary, net_file = net, cfg_file = sumocfg, veh = veh)  
        env.gui_on =False
    elif args.validation_env3 and args.render:
        sumoBinary = sumolib.checkBinary('sumo-gui')
        env = validationEnv3(sumoBinary, net_file = net, cfg_file = sumocfg, veh = veh)  
        env.gui_on =True
    elif args.validation_env3:
        sumoBinary = sumolib.checkBinary('sumo')
        env = validationEnv3(sumoBinary, net_file = net, cfg_file = sumocfg, veh = veh)  
        env.gui_on =False
    elif args.validation_env4 and args.render:
        sumoBinary = sumolib.checkBinary('sumo-gui')
        env = validationEnv4(sumoBinary, net_file = net, cfg_file = sumocfg, veh = veh) 
        env.gui_on =True
    elif args.validation_env4:
        sumoBinary = sumolib.checkBinary('sumo')
        env = validationEnv4(sumoBinary, net_file = net, cfg_file = sumocfg, veh = veh)  
        env.gui_on =False
    elif args.test_env and args.render:
        sumoBinary = sumolib.checkBinary('sumo-gui')
        env = testEnv(sumoBinary, net_file = net, cfg_file = sumocfg, veh = veh)  
        env.gui_on =True
    elif args.test_env:
        sumoBinary = sumolib.checkBinary('sumo')
        env = testEnv(sumoBinary, net_file = net, cfg_file = sumocfg, veh = veh)  
        env.gui_on =False
    else:
        sumoBinary = sumolib.checkBinary('sumo')
        env = rlEnv(sumoBinary, net_file = net, cfg_file = sumocfg, veh = veh)
        env.gui_on = False

    if args.eval and args.render:
        sumoBinary = sumolib.checkBinary('sumo-gui')
        env = rlEnv(sumoBinary, net_file = net, cfg_file = sumocfg, veh = veh)
        env.gui_on = True    
    config = Config()
    test_config = TestConfig()
    # 알고리즘 선택 로직
    if args.lstm:
        trainer = LSTMDQNTrainer(env, config)
    elif args.prev_nstep_lstm:
        trainer = PREVNSTEPLSTMDQNTrainer(env,config)
    elif args.prev_nstep_lstm_per_nstep:
        agent = LSTMPrevNStepPERNStepAgent(len(env.observation_space), len(env.action_space), config)
        trainer = PREVNSTEPLSTMDQNTrainer(env, config, agent=agent, agent_name='prev_nstep_lstm_dqn_per_nstep')
    elif args.prev_nstep_lstm_iqn:
        agent = LSTMPrevNStepIQNAgent(len(env.observation_space), len(env.action_space), config)
        trainer = PREVNSTEPLSTMDQNTrainer(env, config, agent=agent, agent_name='prev_nstep_lstm_dqn_iqn')
    elif args.per:
        agent = DQNPERAgent(len(env.observation_space), len(env.action_space), config)
        trainer = DQNTrainer(env, config, agent=agent, agent_name='dqn_per')
    elif args.nstep:
        agent = DQNNStepAgent(len(env.observation_space), len(env.action_space), config)
        trainer = DQNTrainer(env, config, agent=agent,agent_name='dqn_nstep')
    elif args.per_nstep:
        # 이미 구현되어 있습니다 (dqn_per_agent.py에 n-step 기능이 포함됨)
        agent = DQNPERNStepAgent(len(env.observation_space), len(env.action_space), config)
        trainer = DQNTrainer(env, config, agent=agent,agent_name='dqn_per_nstep')
    elif args.lstm_per:
        agent = LSTMDQNPERAgent(len(env.observation_space), len(env.action_space), config)
        trainer = LSTMDQNTrainer(env, config, agent=agent,agent_name='lstm_dqn_per')
    elif args.lstm_nstep:
        agent = LSTMDQNNStepAgent(len(env.observation_space), len(env.action_space), config)
        trainer = LSTMDQNTrainer(env, config, agent=agent,agent_name='lstm_dqn_nstep')
    elif args.lstm_per_nstep:
        agent = LSTMDQNPERNStepAgent(len(env.observation_space), len(env.action_space), config)
        trainer = LSTMDQNTrainer(env, config, agent=agent,agent_name='lstm_dqn_per_nstep')
    elif args.iqn:
        agent = DQNIQNAgent(len(env.observation_space), len(env.action_space), config)
        trainer = DQNIQNTrainer(env, config, agent=agent,agent_name='dqn_iqn')
    elif args.lstm_iqn:
        agent = LSTMDQNIQNAgent(len(env.observation_space), len(env.action_space), config)
        trainer = LSTMDQNTrainer(env, config, agent=agent,agent_name='iqn')
    elif args.rainbow:
        agent = DQNRainbowAgent(len(env.observation_space), len(env.action_space), config)
        trainer = DQNTrainer(env, config, agent=agent,agent_name='dqn_rainbow')
    elif args.lstm_rainbow:
        agent = LSTMDQNRainbowAgent(len(env.observation_space), len(env.action_space), config)
        trainer = LSTMRainbowTrainer(env, config, agent=agent,agent_name='lstm_dqn_raibow')  
    elif args.lstm_autoencoder:
        trainer = LSTMAutoencoderTrainer(env, config)
    elif args.prev_nstep_lstm_autoencoder:
        trainer = PREVNSTEPLSTMAutoencoderTrainer(env, config)
    elif args.prev_nstep_lstm_autoencoder_per_nstep:
        agent = LSTMPrevNStepAutoencoderPERNStepAgent(len(env.observation_space), len(env.action_space), config)
        trainer = PREVNSTEPLSTMAutoencoderTrainer(env, config, agent=agent,agent_name='prev_nstep_lstm_autoencoder_dqn_per_nstep')
    elif args.prev_nstep_lstm_autoencoder_iqn:
        agent = LSTMPrevNStepAutoencoderIQNAgent(len(env.observation_space), len(env.action_space), config)
        trainer = PREVNSTEPLSTMAutoencoderTrainer(env, config, agent=agent,agent_name='prev_nstep_lstm_autoencoder_dqn_iqn')
    elif args.prev_nstep_lstm_rainbow:
        agent = LSTMPrevNStepRainbowAgent(len(env.observation_space), len(env.action_space), config)
        trainer = PREVNSTEPLSTMRainbowTrainer(env, config, agent=agent, agent_name='prev_nstep_lstm_dqn_rainbow')
    
    else:  # 기본값은 DQN
        trainer = DQNTrainer(env, config)
    
    # 평가 모드
    if args.eval:
        print("Starting evaluation...")
        evaluator = DQNEvaluator(env, trainer, config)
        evaluation_results = evaluator.evaluate(
            num_episodes=args.episodes,
            render=args.render
        )
        
        print("\nFinal Evaluation Results:")
        print(f"Mean Reward: {evaluation_results['mean_reward']:.2f}")
        if evaluation_results['mean_reward'] > config.REWARD_THRESHOLD:
            print("Goal succeeded!")
    elif args.rulebase:
        print("Starting rulebase sumo")
        sumo_test = RuleBase(env)
        sumo_test_results = sumo_test.simulate(
            num_episodes=args.episodes,
            render=args.render
        )
        print("\nFinal simulation Results:")
        print(f"LC success number: {sumo_test_results['num_LC_succeed_number']:.2f}")
    elif args.test:
        print("Start lstm dqn test")
        tester = Tester(env, trainer, test_config, args.weight_path, agent)
        evaluation_results = tester.evaluate(
            num_episodes=args.episodes,
            render=args.render
        )
    elif args.collect_data:
        print("Start collecting data")
        collector = Collection(env)
        collector.run()
        
    else:
        print("Starting training...")
        # 어떤 알고리즘을 사용하는지 출력
        if args.lstm:
            print("Using LSTM DQN")
        elif args.per:
            print("Using DQN with Prioritized Experience Replay")
        elif args.nstep:
            print("Using DQN with n-step returns")
        elif args.per_nstep:
            print("Using DQN with PER and n-step returns")
        elif args.iqn:
            print("Using DQN with Implicit Quantile Networks (IQN)")
        elif args.rainbow:
            print("Using Rainbow DQN (IQN + PER + n-step + Dueling + Noisy Net)")
        elif args.lstm_per:
            print("Using LSTM DQN with PER")
        elif args.lstm_nstep:
            print("Using LSTM DQN with n-step returns")
        elif args.lstm_per_nstep:
            print("Using LSTM DQN with PER and n-step returns")        
        elif args.lstm_iqn:
            print("Using LSTM DQN with Implicit Quantile Networks (IQN)")        
        elif args.lstm_rainbow:
            print("Using LSTM Rainbow DQN (IQN + PER + n-step + Dueling + Noisy Net)")
        elif args.prev_nstep_lstm_per_nstep:
            print("Using Prev N-Step LSTM DQN with PER and n-step returns")
        elif args.prev_nstep_lstm_autoencoder_per_nstep:
            print("Using Prev N-Step LSTM Autoencoder DQN with PER and n-step returns")
        elif args.prev_nstep_lstm_iqn:
            print("Using Prev N-Step LSTM DQN with Implicit Quantile Networks (IQN)")
        elif args.prev_nstep_lstm_autoencoder_iqn:
            print("Using Prev N-Step LSTM Autoencoder DQN with Implicit Quantile Networks (IQN)")
        elif args.prev_nstep_lstm_rainbow:
            print("Using Prev N-Step LSTM with Rainbow DQN (IQN + PER + n-step + Dueling + Noisy Networks)")
        # elif args.prev_nstep_lstm_autoencoder_rainbow:
        #     print("Using Prev N-Step LSTM Autoencoder with Rainbow DQN (IQN + PER + n-step + Dueling + Noisy Networks)")
        else:
            print("Using basic DQN")
            
        trainer.train()

if __name__ == "__main__":
    main()