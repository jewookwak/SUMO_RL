# train/config.py
import os
from datetime import datetime
class Config:
    def __init__(self):
        # DQN hyperparameters
        self.DISCOUNT_FACTOR = 0.99
        self.LEARNING_RATE = 0.0005
        self.EPSILON = 1.0
        self.EPSILON_DECAY = 0.9
        self.EPSILON_MIN = 0.001
        self.BATCH_SIZE = 32
        self.TRAIN_START = 100
        
        # N-step 설정
        self.N_STEP = 6
        
        # LSTM 설정
        self.SEQUENCE_LENGTH = 20
        self.PREV_SEQUENCE_LENGTH = 4
        
        # IQN 하이퍼파라미터
        self.IQN_EMBEDDING_DIM = 64       # 임베딩 차원
        self.IQN_ACTION_QUANTILES = 32    # 행동 선택 시 사용할 분위수 개수
        self.IQN_ACTION_TAU_MIN = 0.0     # 행동 선택 시 최소 tau 값
        self.IQN_ACTION_TAU_MAX = 1.0     # 행동 선택 시 최대 tau 값
        self.IQN_TRAIN_QUANTILES = 32      # 학습 시 사용할 분위수 개수
        self.IQN_TRAIN_TAU_MIN = 0.0      # 학습 시 최소 tau 값
        self.IQN_TRAIN_TAU_MAX = 1.0      # 학습 시 최대 tau 값
        self.IQN_HUBER_THRESHOLD = 1.0 #0.25    # Huber 손실 함수 경계값 
        
        # Rainbow DQN 추가 하이퍼파라미터
        self.USE_NOISY_NETWORK = False     # Noisy Network 활성화 여부
        self.NOISY_SIGMA_INIT = 0.5       # Noisy Network의 시그마 초기값
        self.USE_DUELING_NETWORK = True   # Dueling Network 활성화 여부
        self.TARGET_UPDATE_FREQ = 200     # 타깃 네트워크 업데이트 주기 (스텝 단위)
        
        # PER 파라미터 (Rainbow에서 사용)
        self.PER_ALPHA = 0.6              # PER의 우선순위 지수 alpha
        self.PER_BETA = 0.4               # PER의 중요도 샘플링 지수 beta
        self.PER_BETA_INCREMENT = 0.001   # beta의 점진적 증가량
        self.PER_EPSILON = 0.01           # PER의 최소 우선순위 값

        # Training settings
        self.MAX_EPISODES = 1000
        self.MEMORY_SIZE = 30000
        
        # 절대 경로 사용
        base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        current_time = datetime.now().strftime("%Y%m%d_%H%M")
        self.WEIGHTS_PATH = os.path.join(base_dir,"save_weights/",current_time)
        self.GRAPH_PATH = os.path.join(base_dir, "save_graph/",current_time)
        self.LOG_PATH = os.path.join(base_dir, "logs/")
        
        # Early stopping settings
        self.REWARD_THRESHOLD = 450
        self.SAVE_INTERVAL = 20