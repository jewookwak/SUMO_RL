import os
import sys
import time
import numpy as np

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'],'tools')
    print(tools)
    sys.path.append(tools)
    try:
        import traci
    except ImportError:
        raise EnvironmentError("Declare SUMO_HOME environment")
else:
    sys.exit('Declare environment variable "SUMO_HOME"')

class Env():
    def __init__(self, sumoBinary,net_file: str,det_file: str, cfg_file: str, episode):
        self.sumoBinary = sumoBinary

        self.net = net_file
        self.det = det_file
        self.sumocfg = cfg_file

        self.episode = episode
        self.sumo = traci

    def start_simulation(self):
        step_length = 0.01
        sumo_cmd = [self.sumoBinary,
            '-c', self.sumocfg,
            '--lanechange.duration', '2',
            #  '--collision.action', 'warn',
            #  '--collision.stoptime','5',
             '--collision.action', 'remove',
             '--collision.mingap-factor', '0',
            #  '--time-to-teleport','10',
             '--collision-output','colliderSpeed',
             '--step-length', str(step_length),
             '--no-step-log']
        
        self.sumo.start(sumo_cmd)
    
    def reset(self):
        if self.episode!=0: 
            self.sumo.close(False)

        self.episode+=1
        self.start_simulation()

        # #vehicle  생성
        # self.sumo.route.add("rou1", ["E19", "E0", "E1","E2"]) #default route
        # self.sumo.vehicle.add(self.veh, "rou1")
        self.sumo.simulationStep()
        
        # curedge = self.get_curedge(self.veh)
        # return curedge