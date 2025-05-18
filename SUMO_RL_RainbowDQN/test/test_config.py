# test/test_config.py
import os
from datetime import datetime

class TestConfig:
    def __init__(self):
        # 테스트 환경 설정
        self.MAX_EPISODES = 10      # 테스트 에피소드 수
        self.MAX_STEPS = 3600       # 에피소드 당 최대 스텝 수 (시뮬레이션 초)
        self.RENDER = False         # GUI 렌더링 여부
        # self.COLLISION_MODE = 0     # 후방 차량 충돌 모드 (0: 일반, 1: 비주의)
        self.SAVE_RESULTS = True    # 테스트 결과 시각화 및 저장 여부
        
        # 모델 가중치 경로 설정
        base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.WEIGHTS_PATH = os.path.join(base_dir, "save_weights")
        self.MODEL_FILENAME = "lstm_external_model.h5"
        

        # 결과 저장 경로 설정
        current_time = datetime.now().strftime("%Y%m%d_%H%M")
        self.RESULTS_PATH = os.path.join(base_dir, "test_results", current_time)
        
        # LSTM 설정 (학습과 동일하게 유지)
        self.SEQUENCE_LENGTH = 20
        self.PREV_SEQUENCE_LENGTH = 4
        
        # 테스트 시 사용되는 학습 관련 파라미터 (모델 초기화에 필요)
        self.BATCH_SIZE = 32
        self.EPSILON = 0.01         # 테스트에서는 낮은 입실론 사용 (대부분 학습된 정책 따름)
        self.EPSILON_DECAY = 1.0    # 테스트에서는 입실론 감소 없음
        self.EPSILON_MIN = 0.01
        self.DISCOUNT_FACTOR = 0.99
        self.LEARNING_RATE = 0.0005
        self.MEMORY_SIZE = 30000    # 모델 로드 시 필요
        
        # N-step 설정
        self.N_STEP = 6
        
        # IQN 하이퍼파라미터
        self.IQN_EMBEDDING_DIM = 64
        self.IQN_ACTION_QUANTILES = 32
        self.IQN_ACTION_TAU_MIN = 0.0
        self.IQN_ACTION_TAU_MAX = 1.0
        self.IQN_TRAIN_QUANTILES = 32
        self.IQN_TRAIN_TAU_MIN = 0.0
        self.IQN_TRAIN_TAU_MAX = 1.0
        self.IQN_HUBER_THRESHOLD = 1.0
        
        # Rainbow DQN 관련 파라미터
        self.USE_NOISY_NETWORK = True
        self.NOISY_SIGMA_INIT = 0.5
        self.USE_DUELING_NETWORK = True
        self.TARGET_UPDATE_FREQ = 200
        
        # PER 파라미터
        self.PER_ALPHA = 0.6
        self.PER_BETA = 0.4
        self.PER_BETA_INCREMENT = 0.001
        self.PER_EPSILON = 0.01
        
        # 테스트 특화 파라미터
        self.ACTION_NAMES = ['가속', '감속', '우측 차선 변경', '좌측 차선 변경', '속도 유지', '무작동']
        self.LOG_INTERVAL = 10      # 로그 출력 간격 (스텝 단위)
        self.VALIDATION_MODE = False  # 검증 모드 여부 (다른 시나리오 테스트)
        
        # 평가 지표 설정
        self.SAFETY_WEIGHT = 0.7    # 안전성 가중치 (충돌 없음)
        self.EFFICIENCY_WEIGHT = 0.3  # 효율성 가중치 (높은 보상)
        
        # 시각화 설정
        self.PLOT_TRAJECTORY = True   # 궤적 시각화 여부
        self.PLOT_ACTIONS = True      # 액션 시각화 여부
        self.PLOT_REWARDS = True      # 보상 시각화 여부
        self.PLOT_SUMMARY = True      # 종합 결과 시각화 여부
        
        # 디버깅 모드
        self.DEBUG = False            # 디버깅 정보 출력 여부