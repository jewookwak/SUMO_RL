import os
import numpy as np
import pandas as pd
import tensorflow as tf
import random
from datetime import datetime
import traci


class Collection:
    def __init__(self, env):
        self.env = env
        # 데이터 저장 디렉토리 생성
        self.data_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + "/data_collection"
        if not os.path.exists(self.data_dir):
            os.makedirs(self.data_dir)
    
    def run(self):
        for ep in range(100):
            state = self.env.reset()
            state = np.reshape(state, [1, len(self.env.observation_space)])
            
            # 에피소드별 데이터 저장을 위한 리스트
            episode_data = []
            
            done = False
            while not done:
                action = 5
                print('action:::::::::::::', action)
                next_state, _, done = self.env.step(action)
                time = traci.simulation.getTime()
                collision_status = False if self.env.None_collision == True else True
                
                # 데이터를 리스트에 추가 (각 스텝마다)
                step_data = {
                    'time': time,
                    'state': next_state,
                    'collision_status': collision_status,
                    'action': action
                }
                episode_data.append(step_data)
            
            # 에피소드 종료 후 데이터 저장
            self.save_episode_data(episode_data, ep)
            self.env.end()
    
    def save_episode_data(self, episode_data, episode_num):
        """에피소드 데이터를 CSV 파일로 저장"""
        # 현재 날짜/시간으로 파일명 생성
        current_time = datetime.now().strftime("%Y%m_%d%H")
        save_dir = self.data_dir + f'/data_{current_time}'
        filename = f"ep{episode_num}.csv"
        filepath = os.path.join(save_dir, filename)
        
        # 디렉토리 생성 (존재하지 않는 경우)
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
        
        # DataFrame으로 변환
        df_data = []
        for i, step in enumerate(episode_data):
            row = {
                'step': i,
                'time': step['time'],
                'collision_status': step['collision_status'],
                'action': step['action']
            }
            
            # state가 배열인 경우 각 요소를 별도 컬럼으로 추가
            if isinstance(step['state'], (list, np.ndarray)):
                state_array = np.array(step['state']).flatten()
                
                # state 이름 매핑
                state_names = [
                    'rear_rel_x_scaled', 'rear_rel_y_scaled', 'rear_v_scaled',
                    'forward_rel_x_scaled', 'forward_rel_y_scaled', 'forward_v_scaled',
                    'left_leader_rel_x_scaled', 'left_leader_rel_y_scaled', 'left_leader_v_scaled',
                    'left_rel_x_scaled', 'left_rel_y_scaled', 'left_v_scaled',
                    'left_follower_rel_x_scaled', 'left_follower_rel_y_scaled', 'left_follower_v_scaled',
                    'right_leader_rel_x_scaled', 'right_leader_rel_y_scaled', 'right_leader_v_scaled',
                    'right_rel_x_scaled', 'right_rel_y_scaled', 'right_v_scaled',
                    'right_follower_rel_x_scaled', 'right_follower_rel_y_scaled', 'right_follower_v_scaled',
                    'ego_x_scaled', 'ego_y_scaled', 'ego_v_scaled', 'ego_heading_angle_scaled',
                    's1_rel_x_scaled', 's1_rel_y_scaled', 's1_v_scaled', 'space1_size_scaled',
                    's2_rel_x_scaled', 's2_rel_y_scaled', 's2_v_scaled', 'space2_size_scaled',
                    's3_rel_x_scaled', 's3_rel_y_scaled', 's3_v_scaled', 'space3_size_scaled',
                    's4_rel_x_scaled', 's4_rel_y_scaled', 's4_v_scaled', 'space4_size_scaled',
                    'smid_rel_x_scaled', 'smid_rel_y_scaled', 'smid_v_scaled', 'spacemid_size_scaled'
                ]
                
                for j, state_val in enumerate(state_array):
                    if j < len(state_names):
                        row[state_names[j]] = state_val
                    else:
                        row[f'state_{j}'] = state_val  # 예외 처리: 더 많은 상태가 있는 경우
            else:
                row['state'] = step['state']
            
            df_data.append(row)
        
        # DataFrame 생성 및 CSV 저장
        df = pd.DataFrame(df_data)
        df.to_csv(filepath, index=False)
        print(f"Episode {episode_num} data saved to: {filepath}")