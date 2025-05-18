import numpy as np
import pandas as pd
import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import LSTM, Dense, Dropout, BatchNormalization
from sklearn.preprocessing import RobustScaler
from sklearn.model_selection import train_test_split
import os
import glob

class RealTimeCollisionLSTM:
    def __init__(self, sequence_length=4):
        """
        실시간 충돌 예측 LSTM 모델
        - 0.05초 데이터를 1초 단위로 집계
        - 4초 시퀀스로 학습
        """
        self.sequence_length = sequence_length  # 4초
        self.time_aggregation = 1.0  # 1초 단위 집계
        self.original_dt = 0.05  # 원본 데이터 간격 (0.05초)
        self.samples_per_second = int(self.time_aggregation / self.original_dt)  # 20개 샘플
        
        self.model = None
        self.scaler = RobustScaler()
        
        # 입력 특성
        self.rear_features = [
            'rear_rel_x_scaled', 'rear_rel_y_scaled', 'rear_v_scaled',
            'ego_v_scaled', 'ego_heading_angle_scaled',
            'ego_x_scaled', 'ego_y_scaled',
            'forward_rel_x_scaled', 'forward_rel_y_scaled', 'forward_v_scaled',
            'left_rel_x_scaled', 'left_rel_y_scaled', 'left_v_scaled',
            'right_rel_x_scaled', 'right_rel_y_scaled', 'right_v_scaled'
        ]
        
        self.target_column = 'collision_status'
    
    def aggregate_to_seconds(self, episode_data):
        """
        0.05초 단위 데이터를 1초 단위로 집계
        """
        # 시간 기준으로 그룹핑 (1초 단위)
        episode_data['time_group'] = (episode_data['time'] // self.time_aggregation).astype(int)
        
        # 각 시간 그룹별로 집계
        aggregated_data = []
        
        for time_group in episode_data['time_group'].unique():
            group_data = episode_data[episode_data['time_group'] == time_group]
            
            if len(group_data) == 0:
                continue
            
            # 집계 방법 정의
            agg_row = {
                'time': time_group * self.time_aggregation,
                'episode': group_data['episode'].iloc[0],
                'collision_status': group_data['collision_status'].iloc[-1]  # 마지막 상태 사용
            }
            
            # 특성별 집계 전략
            for feature in self.rear_features:
                if feature in group_data.columns:
                    if 'v_scaled' in feature:  # 속도: 평균값
                        agg_row[feature] = group_data[feature].mean()
                    elif 'rel_x_scaled' in feature or 'rel_y_scaled' in feature:  # 위치: 마지막 값
                        agg_row[feature] = group_data[feature].iloc[-1]
                    else:  # 기타: 평균값
                        agg_row[feature] = group_data[feature].mean()
                        
            # 추가 통계적 특성 (옵션)
            agg_row['rear_rel_x_change'] = group_data['rear_rel_x_scaled'].iloc[-1] - group_data['rear_rel_x_scaled'].iloc[0]
            agg_row['ego_v_change'] = group_data['ego_v_scaled'].iloc[-1] - group_data['ego_v_scaled'].iloc[0]
            agg_row['rear_v_std'] = group_data['rear_v_scaled'].std() if len(group_data) > 1 else 0
            
            aggregated_data.append(agg_row)
        
        return pd.DataFrame(aggregated_data)
    
    def load_and_aggregate_data(self, data_directory):
        """
        데이터 로드 및 1초 단위 집계
        """
        csv_files = glob.glob(os.path.join(data_directory, "*.csv"))
        
        all_aggregated = []
        collision_episodes = 0
        
        print(f"=== 데이터 집계 시작 ===")
        print(f"원본 데이터: 0.05초 간격 → 집계 후: 1초 간격")
        print(f"시퀀스 길이: {self.sequence_length}초")
        
        for file in csv_files[:10]:  # 테스트를 위해 처음 10개 파일만
            try:
                df = pd.read_csv(file)
                episode_name = os.path.basename(file).replace('.csv', '')
                df['episode'] = episode_name
                
                # 1초 단위로 집계
                aggregated_df = self.aggregate_to_seconds(df)
                
                if len(aggregated_df) >= self.sequence_length:  # 최소 4초 이상의 데이터만 사용
                    # 충돌 발생 확인
                    is_collision = aggregated_df.iloc[-1]['collision_status']
                    if is_collision:
                        collision_episodes += 1
                    
                    all_aggregated.append(aggregated_df)
                    print(f"Episode {episode_name}: {len(df)} → {len(aggregated_df)} 샘플, 충돌: {is_collision}")
                
            except Exception as e:
                print(f"파일 처리 실패 {file}: {e}")
                continue
        
        if not all_aggregated:
            raise ValueError("집계된 데이터가 없습니다.")
        
        self.df = pd.concat(all_aggregated, ignore_index=True)
        
        # 추가된 특성을 특성 리스트에 포함
        self.rear_features.extend(['rear_rel_x_change', 'ego_v_change', 'rear_v_std'])
        
        print(f"\n=== 집계 완료 ===")
        print(f"총 에피소드: {len(all_aggregated)}")
        print(f"충돌 에피소드: {collision_episodes}")
        print(f"총 샘플: {len(self.df)}")
        print(f"평균 에피소드 길이: {len(self.df) / len(all_aggregated):.1f}초")
    
    def prepare_sequences(self):
        """
        1초 단위 시퀀스 생성 (4초 윈도우)
        """
        sequences = []
        targets = []
        
        for episode in self.df['episode'].unique():
            episode_data = self.df[self.df['episode'] == episode].copy().reset_index(drop=True)
            
            if len(episode_data) < self.sequence_length:
                continue
            
            # 충돌 여부 확인
            is_collision_episode = episode_data.iloc[-1][self.target_column]
            
            # 특성 데이터 추출 및 결측치 처리
            features = episode_data[self.rear_features].fillna(0).values
            
            # 정규화
            if not hasattr(self, '_scaler_fitted'):
                all_features = self.df[self.rear_features].fillna(0)
                self.scaler.fit(all_features)
                self._scaler_fitted = True
            
            features_scaled = self.scaler.transform(features)
            
            # 레이블 생성 (1초 단위에 맞게 조정)
            if is_collision_episode:
                total_seconds = len(episode_data)
                # 충돌까지의 시간을 고려한 위험도 곡선
                time_to_collision = np.arange(total_seconds, 0, -1)  # 충돌까지 남은 시간
                
                # 위험도 함수: 시간이 적을수록 급격히 증가
                collision_probs = 1 - np.exp(-3 / (time_to_collision + 1))
                collision_probs[-1] = 1.0  # 마지막은 확실히 충돌
            else:
                # 비충돌: 낮은 랜덤 위험도
                collision_probs = np.random.beta(0.3, 3, len(episode_data))
                collision_probs = np.clip(collision_probs, 0, 0.2)
            
            # 4초 시퀀스 생성
            for i in range(self.sequence_length, len(features_scaled)):
                sequences.append(features_scaled[i-self.sequence_length:i])
                targets.append(collision_probs[i])
        
        self.X = np.array(sequences)
        self.y = np.array(targets, dtype=np.float32)
        
        print(f"\n=== 시퀀스 생성 완료 ===")
        print(f"시퀀스 형태: {self.X.shape}")  # (샘플수, 4초, 특성수)
        print(f"타겟 형태: {self.y.shape}")
        print(f"평균 충돌 확률: {self.y.mean():.4f}")
        print(f"고위험 (>0.7): {np.sum(self.y > 0.7)}")
        print(f"중위험 (0.3-0.7): {np.sum((self.y > 0.3) & (self.y <= 0.7))}")
        print(f"저위험 (<=0.3): {np.sum(self.y <= 0.3)}")
    
    def build_model(self):
        """
        4초 시퀀스에 최적화된 LSTM 모델
        """
        input_shape = (self.sequence_length, len(self.rear_features))
        
        self.model = Sequential([
            # 첫 번째 LSTM - 4초간의 시간적 패턴 추출
            LSTM(64, return_sequences=True, input_shape=input_shape, 
                 dropout=0.2, recurrent_dropout=0.2),
            BatchNormalization(),
            
            # 두 번째 LSTM - 중간 수준 패턴 학습
            LSTM(32, return_sequences=False, 
                 dropout=0.2, recurrent_dropout=0.2),
            BatchNormalization(),
            
            # Dense 레이어 - 충돌 확률 예측
            Dense(16, activation='relu'),
            Dropout(0.3),
            Dense(8, activation='relu'),
            Dense(1, activation='sigmoid')
        ])
        
        # 컴파일
        self.model.compile(
            optimizer=tf.keras.optimizers.Adam(learning_rate=0.001),
            loss='mse',
            metrics=['mae', 'accuracy']
        )
        
        print(f"\n=== 모델 구조 ===")
        print(f"입력 형태: (batch_size, {self.sequence_length}, {len(self.rear_features)})")
        self.model.summary()
    
    def train(self, epochs=100, batch_size=64):
        """
        모델 훈련 (TensorBoard 포함)
        """
        from datetime import datetime
        
        # 데이터 분할
        high_risk_mask = self.y > 0.5
        X_train, X_val, y_train, y_val = train_test_split(
            self.X, self.y, test_size=0.2, random_state=42,
            stratify=high_risk_mask
        )
        
        print(f"\n=== 훈련 데이터 분할 ===")
        print(f"훈련: {len(X_train)} (고위험: {np.sum(y_train > 0.7)})")
        print(f"검증: {len(X_val)} (고위험: {np.sum(y_val > 0.7)})")
        
        # TensorBoard 및 콜백 설정
        log_dir = f"./tensorboard_logs/realtime_lstm_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        
        callbacks = [
            tf.keras.callbacks.TensorBoard(log_dir=log_dir, histogram_freq=1),
            tf.keras.callbacks.EarlyStopping(monitor='val_loss', patience=15, restore_best_weights=True),
            tf.keras.callbacks.ReduceLROnPlateau(monitor='val_loss', factor=0.5, patience=8, min_lr=0.00001),
            tf.keras.callbacks.ModelCheckpoint('best_realtime_model.h5', monitor='val_loss', save_best_only=True)
        ]
        
        # 훈련 실행
        print(f"\n=== 훈련 시작 ===")
        print(f"TensorBoard: tensorboard --logdir={log_dir}")
        
        self.history = self.model.fit(
            X_train, y_train,
            batch_size=batch_size,
            epochs=epochs,
            validation_data=(X_val, y_val),
            callbacks=callbacks,
            verbose=1
        )
        
        # 최종 평가
        self._evaluate_performance(X_val, y_val)
    
    def _evaluate_performance(self, X_val, y_val):
        """
        성능 평가
        """
        y_pred = self.model.predict(X_val, verbose=0)
        
        print(f"\n=== 성능 평가 ===")
        
        # 회귀 메트릭
        mae = np.mean(np.abs(y_val - y_pred.flatten()))
        mse = np.mean((y_val - y_pred.flatten())**2)
        print(f"MAE: {mae:.4f}, MSE: {mse:.4f}")
        
        # 분류 메트릭 (다양한 임계값)
        for threshold in [0.2, 0.5, 0.8]:
            pred_binary = (y_pred.flatten() > threshold).astype(int)
            true_binary = (y_val > threshold).astype(int)
            acc = np.mean(pred_binary == true_binary)
            print(f"임계값 {threshold}: 정확도 {acc:.3f}")
        
        # 조기 경고 성능 (0.3 임계값)
        early_warning_pred = y_pred.flatten() > 0.3
        actual_danger = y_val > 0.7
        
        if np.sum(actual_danger) > 0:
            detection_rate = np.sum(early_warning_pred & actual_danger) / np.sum(actual_danger)
            false_alarm_rate = np.sum(early_warning_pred & ~actual_danger) / np.sum(~actual_danger)
            print(f"조기 경고 성능:")
            print(f"  위험 감지율: {detection_rate:.3f}")
            print(f"  오경보율: {false_alarm_rate:.3f}")
    
    def predict_realtime(self, last_4_seconds_data):
        """
        실시간 예측 (4초간의 1초 단위 데이터 입력)
        """
        # 입력 데이터 정규화
        input_scaled = self.scaler.transform(last_4_seconds_data)
        input_batch = np.expand_dims(input_scaled, axis=0)
        
        # 예측
        collision_prob = self.model.predict(input_batch, verbose=0)[0][0]
        
        # 위험 수준 판단
        if collision_prob > 0.8:
            risk_level = "CRITICAL"
            action = "즉시 자동 브레이크"
        elif collision_prob > 0.5:
            risk_level = "HIGH"
            action = "경고음 + 브레이크 준비"
        elif collision_prob > 0.2:
            risk_level = "MEDIUM"
            action = "시각적 경고"
        else:
            risk_level = "LOW"
            action = "정상 주행"
        
        return {
            'collision_probability': float(collision_prob),
            'risk_level': risk_level,
            'recommended_action': action,
            'confidence': min(1.0, abs(collision_prob - 0.5) * 2)
        }

# 메인 실행
if __name__ == "__main__":
    print("=== 실시간 충돌 예측 시스템 ===")
    print("데이터 변환: 0.05초 → 1초 집계")
    print("시퀀스 길이: 4초")
    
    # 모델 초기화
    predictor = RealTimeCollisionLSTM(sequence_length=4)
    
    # 데이터 로드 및 집계
    data_dirs = ["./data_202505_1811", "./data_202505_1812"]
    for data_dir in data_dirs:
        if os.path.exists(data_dir):
            predictor.load_and_aggregate_data(data_dir)
            break
    
    # 시퀀스 생성 및 모델 구축
    predictor.prepare_sequences()
    predictor.build_model()
    
    # 훈련
    predictor.train(epochs=100, batch_size=32)
    
    print("\n=== 실시간 시스템 준비 완료 ===")
    print("사용법:")
    print("result = predictor.predict_realtime(last_4_seconds_data)")
    print("print(result['collision_probability'], result['risk_level'])")