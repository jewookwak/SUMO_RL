심층강화학습 기반 후방 접근 차량 추돌 방지 연구(Development of a Rear Approach Vehicle Collision Prevention Algorithm using Deep Reinforcement Learning)  
Introduction  
‒ Fatalities from accidents involving commercial vehicles such as trucks and buses are the highest among vehicle-to-vehicle collisions.  
‒ Proactive avoidance algorithms for inattentive rear vehicles, such as those caused by drowsy driving or lack of attention, could
reduce the number of fatalities in rear-end collisions.  
‒ In this study, we used DQN, which is known for its eﬀectiveness with discrete input and output states, to ﬁnd the optimal empty space
to avoid collisions with unconscious rear vehicles.  
‒ We deﬁned the input state as the 8 surrounding vehicles and the output action as 6 modes, covering longitudinal and lateral control.  
[보고서](석사논문연구%20중간%20보고서.pdf)

## 🔧 설치 방법
### 필요 라이브러리
pip install tensorflow numpy matplotlib gym
### SUMO 설치 방법
https://sumo.dlr.de/docs/Installing/Linux_Build.html  
### 시스템 요구사항
- Python 3.8+  
- CUDA 지원 GPU (권장)  

## 📁 프로젝트 구조

SUMO_RL_RainbowDQN  
├── main.py             # 메인 실행 파일     
├── envs/               # SUMO 시뮬레이션 환경들  
├── train/              # 학습,에이전트,네트워크 파일       
├── test/               # 학습한 모델 테스트      
├── weights/            # 학습한 모델 저장    
└── logs/               # 학습 로그 기록    

Quick Execution    
python main.py --rulebase --render  

Basic Train  
python main.py --validation_env3 --lstm

Quick Test  
python main.py --validation_env4 --prev_nstep_lstm --test --weight_path=20250522_2004/prev_nstep_lstm_dqn_model.h5 --state_num=28 --render



영상(https://youtu.be/M56FDJPp8BQ)
