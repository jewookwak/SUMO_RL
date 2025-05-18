import numpy as np
import pandas as pd
import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import LSTM, Dense, Dropout
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split
import matplotlib.pyplot as plt
import os
import glob

class RearCollisionLSTM:
    def __init__(self, sequence_length=10):
        """
        후방 차량 충돌 예측 LSTM 모델
        
        Args:
            sequence_length: 시계열 입력 길이 (과거 몇 스텝을 보고 예측할지)
        """
        self.sequence_length = sequence_length
        self.model = None
        self.scaler = StandardScaler()
        
        # 후방 차량 관련 특성들
        self.rear_features = [
            'rear_rel_x_scaled', 'rear_rel_y_scaled', 'rear_v_scaled',
            'ego_v_scaled', 'ego_heading_angle_scaled',
            # 상대적 위치와 속도가 중요하므로 추가
            'ego_x_scaled', 'ego_y_scaled'
        ]
        
        # 타겟 변수
        self.target_column = 'collision_status'
    
    def load_data(self, data_directory):
        """
        데이터 디렉토리에서 모든 CSV 파일을 로드
        
        Args:
            data_directory: CSV 파일들이 있는 디렉토리 경로
        """
        csv_files = glob.glob(os.path.join(data_directory, "*.csv"))
        
        all_data = []
        collision_episodes = 0
        
        for file in csv_files:
            try:
                df = pd.read_csv(file)
                # 에피소드 번호 추가
                episode_name = os.path.basename(file).replace('.csv', '')
                df['episode'] = episode_name
                
                # 충돌 발생 확인 (마지막 행의 collision_status)
                is_collision = df.iloc[-1]['collision_status']
                if is_collision:
                    collision_episodes += 1
                    print(f"충돌 에피소드 발견: {episode_name}")
                
                all_data.append(df)
            except Exception as e:
                print(f"파일 로드 실패 {file}: {e}")
                continue
        
        if not all_data:
            raise ValueError("로드된 데이터가 없습니다. 경로를 확인해주세요.")
        
        self.df = pd.concat(all_data, ignore_index=True)
        print(f"\n=== 데이터 로드 완료 ===")
        print(f"총 {len(all_data)}개 에피소드")
        print(f"총 {len(self.df)}개 샘플")
        print(f"충돌 에피소드: {collision_episodes}개")
        print(f"비충돌 에피소드: {len(all_data) - collision_episodes}개")
        
    def prepare_sequences(self):
        """
        시계열 데이터를 LSTM 입력 형태로 변환
        에피소드 마지막이 충돌인 경우 해당 에피소드 전체를 충돌 예측 학습에 사용
        """
        sequences = []
        targets = []
        
        # 에피소드별로 시퀀스 생성
        for episode in self.df['episode'].unique():
            episode_data = self.df[self.df['episode'] == episode].copy().reset_index(drop=True)
            
            # 에피소드 마지막 충돌 상태 확인
            is_collision_episode = episode_data.iloc[-1][self.target_column]
            
            # 특성 데이터 정규화 (에피소드별로 개별 정규화)
            features = episode_data[self.rear_features].values
            scaler = StandardScaler()
            features_scaled = scaler.fit_transform(features)
            
            # 충돌 에피소드인 경우: 마지막으로 갈수록 충돌 확률 증가
            # 비충돌 에피소드인 경우: 전체적으로 낮은 충돌 확률
            if is_collision_episode:
                # 시간에 따라 충돌 확률이 점진적으로 증가하도록 설정
                total_steps = len(episode_data)
                collision_probs = np.linspace(0.1, 1.0, total_steps)
                # 마지막 몇 스텝은 확실히 충돌으로 설정
                collision_probs[-5:] = 1.0
            else:
                # 비충돌 에피소드는 전체적으로 낮은 확률
                collision_probs = np.random.uniform(0.0, 0.2, len(episode_data))
            
            # 시퀀스 생성
            for i in range(self.sequence_length, len(features_scaled)):
                sequences.append(features_scaled[i-self.sequence_length:i])
                targets.append(collision_probs[i])
            
            print(f"Episode {episode}: {len(episode_data)} steps, collision: {is_collision_episode}")
        
        self.X = np.array(sequences)
        self.y = np.array(targets, dtype=np.float32)
        
        print(f"\n=== 시퀀스 생성 완료 ===")