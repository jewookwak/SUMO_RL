# envs/components/reward.py
class Reward:
    def __init__(self):
        pass

    def process(self, reward, done=False, total_steps=0):

        # if not done or total_steps >= 500:  # 최대 스텝에 도달해서 끝난 경우
        #     return 0.1
        # else:  # 중간에 실패한 경우
        #     return -1.0
        return reward