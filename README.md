심층강화학습 기반 후방 접근 차량 추돌 방지 연구(Development of a Rear Approach Vehicle Collision Prevention Algorithm using Deep Reinforcement Learning)  
Introduction  
‒ Fatalities from accidents involving commercial vehicles such as trucks and buses are the highest among vehicle-to-vehicle collisions.  
‒ Proactive avoidance algorithms for inattentive rear vehicles, such as those caused by drowsy driving or lack of attention, could
reduce the number of fatalities in rear-end collisions.  
‒ In this study, we used DQN, which is known for its eﬀectiveness with discrete input and output states, to ﬁnd the optimal empty space
to avoid collisions with unconscious rear vehicles.  
‒ We deﬁned the input state as the 8 surrounding vehicles and the output action as 6 modes, covering longitudinal and lateral control.  


Quick Execution    
python main.py --rulebase --render  

Basic Train  
python main.py --validation_env3 --lstm

Quick Test  
python main.py --validation_env4 --prev_nstep_lstm --test --weight_path=20250522_2004/prev_nstep_lstm_dqn_model.h5 --state_num=28 --render



영상(https://youtu.be/M56FDJPp8BQ)
