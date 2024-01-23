from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import random
import matplotlib.pyplot as plt
from xml.etree.ElementTree import parse
from collections import defaultdict

from matplotlib.ticker import MaxNLocator


from rlenv import rlEnv
# from rlagent import rlAgent

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'],'tools')
    print(tools)
    sys.path.append(tools)
    import sumolib 
else:
    sys.exit('Declare environment variable "SUMO_HOME"')



def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("-N","--episodenum", 
                        default=3, help="numer of episode to run qlenv")
    optParser.add_option("--nogui", action="store_true",
                        default=False, help="run commandline version of sumo")
    options, args = optParser.parse_args()
    return options

def get_toedges(net, fromedge):
    #calculate reachable nextedges
    tree = parse(net)
    root = tree.getroot()
    toedges = []
    for connection in root.iter("connection"):
        if connection.get("from")==fromedge:
            toedges.append(connection.get("to"))
    return toedges


def plot_result(episodenum, lst_cntSuccess):
    ax = plt.figure().gca()
    ax.yaxis.set_major_locator(MaxNLocator(integer=True))
    plt.xticks(range(episodenum),rotation=45)
    plt.plot(lst_cntSuccess, 'r-')
    
    plt.savefig('./Qlearning/Result/qlresult%i.png' %episodenum )
    
##########@경로 탐색@##########
#qlearning routing : routing by applying qlearning algorithm (using qlEnv & alAgent)
def rl_run(sumoBinary, episodenum,net, sumocfg,step_length, veh):  
    env = rlEnv(sumoBinary, net_file = net, cfg_file = sumocfg, veh = veh)

    
    # qtable = agent.set_qtable()
    # cntSuccess=0
    # lst_cntSuccess=[]
    # idxSuccess=-1
    for episode in range(episodenum):
        print("\n********#{} episode start***********".format(episode))
        #reset environment
        isSuccess=True
        routes = []
        # agent.set_episilon()
        env.reset()
    #     routes.append(curedge)
    #     print('%s -> ' %curedge, end=' ')
    #     done = False
    #     cnt=0
    #     while not done: 
            
    #         block = True
    #         while block: #막힌 도로를 고름
    #             if curedge in endpoint:
    #                 break
    #             curedge = env.get_curedge(veh)
    #             action = agent.get_action(curedge) #현재 edge에서 가능한 (0,1,2) 중 선택 :e-greedy 방식으로
                
    #             nextedge = env.get_nextedge(curedge, action) #next edge 계산해서 env에 보냄.

    #             if nextedge!="" : break

    #             agent.learn_block(curedge, action) #막힌 도로 선택시, blockreward 부여(blockreward - -100)
            
    #         print('%s -> ' %nextedge, end=' ')
    #         if nextedge in badpoint: isSuccess=False
    #         routes.append(nextedge)

    #         reward, done = env.step(curedge, nextedge) #changeTarget to nextedge
    #         #print("env step check: nextedge {}/ reward {}/ done {}".format(nextedge, reward, done))
    #         agent.learn(curedge, action, reward, nextedge)
            

    #         if done:
    #             if nextedge==endpoint[0]:
    #                 print('Arrived:) ')
    #             else:
    #                 isSuccess = False
    #                 print('Bad Arrived:( ')
    #             break
            
    #         curedge = nextedge
    #         cnt+=1

    #     #Consecutive Success 계산
    #     if isSuccess:
    #         if idxSuccess==-1: idxSuccess = episode
    #         cntSuccess+=1
    #     else:
    #         cntSuccess = 0
    #         idxSuccess=-1
    #     lst_cntSuccess.append(cntSuccess)
    #     print('Routing #{} => Consecutive Success: {} from episode #{}'.format(episode, cntSuccess,idxSuccess))
    #     '''  
    #     print('Qtable result after #%i episode' %episode)            
    #     for i,v in agent.qtable.items():
    #         if i in ['-E5','E1','E18','E4','-E0']:
    #             print(i,v)   
    #     '''  
    # plot_result(episodenum,lst_cntSuccess)
    # sys.stdout.flush()
            


if __name__ == "__main__":
    net = "highway_episodic.net.xml"
    sumocfg = "/home/mds/Desktop/highway_episodic/highway_episodic.sumocfg"
    veh = "ego"
    options = get_options()
    if options.nogui:
        sumoBinary = sumolib.checkBinary('sumo')
    else:
        sumoBinary = sumolib.checkBinary('sumo-gui')


    """Run Simulation"""
    #2) Run in rl environment
    step_length =0.01
    episodenum= int(options.episodenum)
    rl_run(sumoBinary, episodenum,net, sumocfg, veh, step_length)
    
