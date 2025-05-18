import numpy as np
import pandas as pd
import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import LSTM, Dense, Dropout, BatchNormalization
from sklearn.preprocessing import RobustScaler
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report, confusion_matrix, roc_auc_score
import os
import glob
from datetime import datetime

class BinaryWindowCollisionLSTM:
    def __init__(self, window_seconds=8, sequence_length=4):
        """
        Binary Window 충돌 예측 모델
        
        Args:
            window_seconds: 충돌 전 위험 구간 길이 (8초 또는 10초)
            sequence_length: LSTM 입력 시퀀스 길이 (4초)
        """
        self.window_seconds = window_seconds
        self.sequence_length = sequence_length
        self.model = None
        self.scaler = RobustScaler()
        
        # 특성 정의
        self.features = [
            'rear_rel_x_scaled', 'rear_rel_y_scaled', 'rear_v_scaled',
            'ego_v_scaled', 'ego_heading_angle_scaled',
            'ego_x_scaled', 'ego_y_scaled',
            'forward_rel_x_scaled', 'forward_rel_y_scaled', 'forward_v_scaled',
            'left_rel_x_scaled', 'left_rel_y_scaled', 'left_v_scaled',
            'right_rel_x_scaled', 'right_rel_y_scaled', 'right_v_scaled'
        ]
        
        print(f"=== Binary Window {window_seconds}초 모델 초기화 ===")
        
    def aggregate_to_seconds(self, episode_data):
        """0.05초 데이터를 1초 단위로 집계"""
        episode_data['time_group'] = (episode_data['time'] // 1.0).astype(int)
        
        aggregated_data = []
        for time_group in episode_data['time_group'].unique():
            group_data = episode_data[episode_data['time_group'] == time_group]
            
            agg_row = {
                'time': time_group,
                'episode': group_data['episode'].iloc[0],
                'collision_status': group_data['collision_status'].iloc[-1]
            }
            
            # 특성별 집계
            for feature in self.features:
                if feature in group_data.columns:
                    if 'v_scaled' in feature:
                        agg_row[feature] = group_data[feature].mean()
                    else:
                        agg_row[feature] = group_data[feature].iloc[-1]
            
            # 변화량 특성 추가
            if len(group_data) > 1:
                agg_row['rear_distance_change'] = group_data['rear_rel_x_scaled'].iloc[-1] - group_data['rear_rel_x_scaled'].iloc[0]
                agg_row['ego_speed_change'] = group_data['ego_v_scaled'].iloc[-1] - group_data['ego_v_scaled'].iloc[0]
            else:
                agg_row['rear_distance_change'] = 0
                agg_row['ego_speed_change'] = 0
                
            aggregated_data.append(agg_row)
        
        return pd.DataFrame(aggregated_data)
    
    def create_binary_window_labels(self, episode_data):
        """
        Binary Window 레이블 생성
        - 충돌 에피소드: 마지막 window_seconds초만 1, 나머지 0
        - 비충돌 에피소드: 전체 0
        """
        is_collision = episode_data.iloc[-1]['collision_status']
        labels = np.zeros(len(episode_data))
        
        if is_collision:
            window_size = min(self.window_seconds, len(episode_data))
            labels[-window_size:] = 1
            
        return labels
    
    def load_and_prepare_data(self, data_directories):
        """데이터 로드 및 전처리"""
        all_aggregated = []
        total_files = 0
        collision_episodes = 0
        
        for data_dir in data_directories:
            if not os.path.exists(data_dir):
                continue
                
            csv_files = glob.glob(os.path.join(data_dir, "*.csv"))
            
            for file in csv_files[:50]:  # 처음 50개 파일만 (테스트용)
                try:
                    df = pd.read_csv(file)
                    episode_name = os.path.basename(file).replace('.csv', '')
                    df['episode'] = episode_name
                    
                    # 1초 단위로 집계
                    aggregated_df = self.aggregate_to_seconds(df)
                    
                    if len(aggregated_df) >= self.sequence_length + self.window_seconds:
                        is_collision = aggregated_df.iloc[-1]['collision_status']
                        if is_collision:
                            collision_episodes += 1
                        
                        all_aggregated.append(aggregated_df)
                        total_files += 1
                        
                except Exception as e:
                    print(f"파일 처리 실패 {file}: {e}")
                    continue
        
        if not all_aggregated:
            raise ValueError("처리된 데이터가 없습니다.")
        
        self.df = pd.concat(all_aggregated, ignore_index=True)
        
        # 추가된 특성을 features에 포함
        self.features.extend(['rear_distance_change', 'ego_speed_change'])
        
        print(f"=== 데이터 로드 완료 ({self.window_seconds}초 윈도우) ===")
        print(f"처리된 파일: {total_files}개")
        print(f"충돌 에피소드: {collision_episodes}개")
        print(f"비충돌 에피소드: {total_files - collision_episodes}개")
        print(f"총 샘플: {len(self.df)}개")
        
    def prepare_sequences(self):
        """시퀀스 데이터 및 Binary Window 레이블 생성"""
        sequences = []
        targets = []
        
        for episode in self.df['episode'].unique():
            episode_data = self.df[self.df['episode'] == episode].copy().reset_index(drop=True)
            
            if len(episode_data) < self.sequence_length + self.window_seconds:
                continue
            
            # Binary Window 레이블 생성
            episode_labels = self.create_binary_window_labels(episode_data)
            
            # 특성 추출 및 정규화
            features = episode_data[self.features].fillna(0).values
            
            # 전체 데이터 기준 스케일링
            if not hasattr(self, '_scaler_fitted'):
                all_features = self.df[self.features].fillna(0)
                self.scaler.fit(all_features)
                self._scaler_fitted = True
            
            features_scaled = self.scaler.transform(features)
            
            # 시퀀스 생성 (마지막 window_seconds개는 타겟으로만 사용)
            max_seq_start = len(features_scaled) - self.window_seconds
            
            for i in range(self.sequence_length, max_seq_start):
                sequences.append(features_scaled[i-self.sequence_length:i])
                targets.append(episode_labels[i + self.window_seconds - 1])  # window_seconds 후의 위험도
        
        self.X = np.array(sequences)
        self.y = np.array(targets, dtype=np.float32)
        
        print(f"\n=== 시퀀스 생성 완료 ({self.window_seconds}초 윈도우) ===")
        print(f"시퀀스 형태: {self.X.shape}")
        print(f"타겟 형태: {self.y.shape}")
        print(f"위험 샘플 비율: {self.y.mean():.3f} ({np.sum(self.y):,} / {len(self.y):,})")
        
        # 클래스 분포 확인
        unique, counts = np.unique(self.y, return_counts=True)
        print(f"클래스 분포: {dict(zip(unique, counts))}")
        
    def build_model(self):
        """Binary 분류에 최적화된 LSTM 모델 구축"""
        input_shape = (self.sequence_length, len(self.features))
        
        self.model = Sequential([
            # 첫 번째 LSTM 블록
            LSTM(64, return_sequences=True, input_shape=input_shape, 
                 dropout=0.2, recurrent_dropout=0.1),
            BatchNormalization(),
            
            # 두 번째 LSTM 블록  
            LSTM(32, return_sequences=False,
                 dropout=0.2, recurrent_dropout=0.1),
            BatchNormalization(),
            
            # Dense 레이어들
            Dense(32, activation='relu'),
            Dropout(0.3),
            Dense(16, activation='relu'),
            Dropout(0.2),
            
            # 출력 레이어 (이진 분류)
            Dense(1, activation='sigmoid')
        ])
        
        # 컴파일 (이진 분류)
        self.model.compile(
            optimizer=tf.keras.optimizers.Adam(learning_rate=0.001),
            loss='binary_crossentropy',
            metrics=['accuracy', 'precision', 'recall']
        )
        
        print(f"\n=== {self.window_seconds}초 윈도우 모델 구조 ===")
        self.model.summary()
        
    def train(self, epochs=100, batch_size=64):
        """모델 훈련"""
        # 층화 샘플링으로 데이터 분할
        X_train, X_val, y_train, y_val = train_test_split(
            self.X, self.y, test_size=0.2, random_state=42,
            stratify=self.y
        )
        
        print(f"\n=== 훈련 시작 ({self.window_seconds}초 윈도우) ===")
        print(f"훈련 세트: {len(X_train)} (위험: {np.sum(y_train):,})")
        print(f"검증 세트: {len(X_val)} (위험: {np.sum(y_val):,})")
        
        # 클래스 가중치 계산 (불균형 데이터 처리)
        pos_weight = len(y_train) / (2 * np.sum(y_train))
        neg_weight = len(y_train) / (2 * (len(y_train) - np.sum(y_train)))
        class_weight = {0: neg_weight, 1: pos_weight}
        
        print(f"클래스 가중치: {class_weight}")
        
        # TensorBoard 로그 디렉토리
        log_dir = f"./tensorboard_logs/binary_{self.window_seconds}s_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        
        # 콜백 설정
        callbacks = [
            tf.keras.callbacks.TensorBoard(
                log_dir=log_dir,
                histogram_freq=1,
                write_graph=True
            ),
            tf.keras.callbacks.EarlyStopping(
                monitor='val_loss',
                patience=15,
                restore_best_weights=True,
                verbose=1
            ),
            tf.keras.callbacks.ReduceLROnPlateau(
                monitor='val_loss',
                factor=0.5,
                patience=8,
                min_lr=0.00001,
                verbose=1
            ),
            tf.keras.callbacks.ModelCheckpoint(
                f'best_binary_{self.window_seconds}s_model.h5',
                monitor='val_loss',
                save_best_only=True,
                verbose=1
            )
        ]
        
        # 훈련 실행
        print(f"TensorBoard: tensorboard --logdir={log_dir}")
        
        self.history = self.model.fit(
            X_train, y_train,
            batch_size=batch_size,
            epochs=epochs,
            validation_data=(X_val, y_val),
            class_weight=class_weight,
            callbacks=callbacks,
            verbose=1
        )
        
        # 훈련 완료 후 성능 평가
        self.evaluate_model(X_val, y_val)
        
    def evaluate_model(self, X_val, y_val):
        """모델 성능 상세 평가"""
        print(f"\n=== {self.window_seconds}초 윈도우 모델 성능 평가 ===")
        
        # 예측 수행
        y_pred_proba = self.model.predict(X_val, verbose=0)
        
        # 다양한 임계값에서 성능 평가
        thresholds = [0.3, 0.5, 0.7, 0.9]
        
        print("Threshold | Precision | Recall | F1-Score | Accuracy")
        print("-" * 55)
        
        best_f1 = 0
        best_threshold = 0.5
        
        for threshold in thresholds:
            y_pred = (y_pred_proba > threshold).astype(int).flatten()
            
            from sklearn.metrics import precision_score, recall_score, f1_score, accuracy_score
            
            precision = precision_score(y_val, y_pred, zero_division=0)
            recall = recall_score(y_val, y_pred, zero_division=0)
            f1 = f1_score(y_val, y_pred, zero_division=0)
            accuracy = accuracy_score(y_val, y_pred)
            
            print(f"{threshold:9.1f} | {precision:9.3f} | {recall:6.3f} | {f1:8.3f} | {accuracy:8.3f}")
            
            if f1 > best_f1:
                best_f1 = f1
                best_threshold = threshold
        
        # AUC-ROC
        try:
            auc = roc_auc_score(y_val, y_pred_proba)
            print(f"\nAUC-ROC: {auc:.3f}")
        except:
            print("\nAUC-ROC 계산 실패 (클래스 불균형)")
        
        print(f"최적 임계값: {best_threshold} (F1-Score: {best_f1:.3f})")
        
        # 혼동 행렬 (최적 임계값 사용)
        y_pred_best = (y_pred_proba > best_threshold).astype(int).flatten()
        cm = confusion_matrix(y_val, y_pred_best)
        
        print(f"\n혼동 행렬 (임계값 {best_threshold}):")
        print("         Predicted")
        print("       0    1")
        print(f"Actual 0 {cm[0,0]:4d} {cm[0,1]:4d}")
        print(f"       1 {cm[1,0]:4d} {cm[1,1]:4d}")
        
        # 실용성 평가
        self.evaluate_practical_performance(y_val, y_pred_proba, best_threshold)
        
        return {
            'auc': auc if 'auc' in locals() else None,
            'best_threshold': best_threshold,
            'best_f1': best_f1,
            'window_seconds': self.window_seconds
        }
    
    def evaluate_practical_performance(self, y_val, y_pred_proba, threshold):
        """실용적 관점에서의 성능 평가"""
        y_pred = (y_pred_proba > threshold).astype(int).flatten()
        
        # 실제 활용 시나리오별 성능
        print(f"\n=== 실용적 성능 평가 ({self.window_seconds}초 전 예측) ===")
        
        # 1. 조기 경고 성능
        true_positives = np.sum((y_val == 1) & (y_pred == 1))
        false_positives = np.sum((y_val == 0) & (y_pred == 1))
        false_negatives = np.sum((y_val == 1) & (y_pred == 0))
        true_negatives = np.sum((y_val == 0) & (y_pred == 0))
        
        detection_rate = true_positives / (true_positives + false_negatives) if (true_positives + false_negatives) > 0 else 0
        false_alarm_rate = false_positives / (false_positives + true_negatives) if (false_positives + true_negatives) > 0 else 0
        
        print(f"위험 상황 감지율: {detection_rate:.3f} ({true_positives} / {true_positives + false_negatives})")
        print(f"오경보율: {false_alarm_rate:.3f} ({false_positives} / {false_positives + true_negatives})")
        
        # 2. 대응 시간 분석
        response_time = self.window_seconds
        print(f"예상 대응 시간: {response_time}초")
        
        if response_time >= 8:
            print("✅ 차선 변경 가능")
            print("✅ 점진적 제동 가능")
            print("✅ 운전자 개입 여유")
        else:
            print("⚠️ 급제동 위주 대응")
        
        # 3. 시스템 신뢰성
        reliability_score = detection_rate * (1 - false_alarm_rate)
        print(f"시스템 신뢰성 점수: {reliability_score:.3f}")
        
        return {
            'detection_rate': detection_rate,
            'false_alarm_rate': false_alarm_rate,
            'reliability_score': reliability_score,
            'response_time': response_time
        }
    
    def predict_collision_risk(self, last_4_seconds_data):
        """실시간 충돌 위험 예측"""
        # 정규화
        input_scaled = self.scaler.transform(last_4_seconds_data)
        input_batch = np.expand_dims(input_scaled, axis=0)
        
        # 예측
        collision_prob = self.model.predict(input_batch, verbose=0)[0][0]
        
        # 위험 수준 및 권장 행동
        if collision_prob > 0.8:
            risk_level = "CRITICAL"
            action = f"{self.window_seconds}초 내 충돌 예상 - 즉시 회피"
        elif collision_prob > 0.6:
            risk_level = "HIGH" 
            action = f"{self.window_seconds}초 내 충돌 위험 - 제동 시작"
        elif collision_prob > 0.4:
            risk_level = "MEDIUM"
            action = f"주의 필요 - 상황 모니터링"
        else:
            risk_level = "LOW"
            action = "정상 주행"
        
        return {
            'collision_probability': float(collision_prob),
            'risk_level': risk_level,
            'predicted_window': f"{self.window_seconds}초",
            'recommended_action': action
        }

# 사용법
def compare_window_sizes():
    """8초와 10초 Binary Window 모델 비교"""
    
    print("=" * 80)
    print("Binary Window 충돌 예측 모델 비교: 8초 vs 10초")
    print("=" * 80)
    
    # 데이터 디렉토리
    data_dirs = ["./data_202505_1811", "./data_202505_1812"]
    
    results = {}
    
    # 8초 윈도우 모델
    print("\n🚗 8초 Binary Window 모델 훈련 시작...")
    model_8s = BinaryWindowCollisionLSTM(window_seconds=8)
    model_8s.load_and_prepare_data(data_dirs)
    model_8s.prepare_sequences()
    model_8s.build_model()
    model_8s.train(epochs=50, batch_size=32)
    results['8s'] = model_8s
    
    print("\n" + "="*50)
    
    # 10초 윈도우 모델  
    print("\n🚗 10초 Binary Window 모델 훈련 시작...")
    model_10s = BinaryWindowCollisionLSTM(window_seconds=10)
    model_10s.load_and_prepare_data(data_dirs)
    model_10s.prepare_sequences()
    model_10s.build_model()
    model_10s.train(epochs=50, batch_size=32)
    results['10s'] = model_10s
    
    # 최종 비교
    print("\n" + "="*80)
    print("최종 성능 비교 결과")
    print("="*80)
    
    print("Window | Detection | False Alarm | Reliability | Response Time")
    print("-" * 65)
    
    # 각 모델의 성능 요약 (가상의 값, 실제로는 evaluate_practical_performance 결과 사용)
    for window in ['8s', '10s']:
        model = results[window]
        window_sec = int(window[:-1])
        print(f"{window:6s} | TBD       | TBD         | TBD         | {window_sec}초")
    
    print("\n권장사항:")
    print("- 8초 윈도우: 더 정확한 예측, 적절한 대응 시간")
    print("- 10초 윈도우: 더 여유로운 대응, 가능한 오탐 증가")
    print("- 최종 선택은 실제 성능 결과에 따라 결정")
    
    return results

if __name__ == "__main__":
    results = compare_window_sizes()