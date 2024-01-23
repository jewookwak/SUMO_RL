#!/usr/bin/env python
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
# Copyright (C) 2008-2023 German Aerospace Center (DLR) and others.
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# https://www.eclipse.org/legal/epl-2.0/
# This Source Code may also be made available under the following Secondary
# Licenses when the conditions for such availability set forth in the Eclipse
# Public License 2.0 are satisfied: GNU General Public License, version 2
# or later which is available at
# https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
# SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later

# @file    runner.py
# @author  Michael Behrisch
# @date    2012-12-09

from __future__ import absolute_import
from __future__ import print_function


import os
import subprocess
import sys
import shutil
import time

import sumolib
import optparse
from sumolib import checkBinary  # noqa
import traci  # noqa
import traci.constants as tc
import pandas as pd
import numpy as np
import random
import matplotlib.pyplot as plt
import pickle
import joblib
from enum import Enum
import math
sys.path.append(
    os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', "tools"))
sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
    os.path.dirname(__file__), "..", "..", "..")), "tools"))
from sumolib import checkBinary  # noqa
import sumolib


def generate_routefile(episode):
    #collisionAvoidanceOverride="50"
    ## """+"\""+str(num)+"\""+"""
    file_path = f"route_file/highway_episodic_{episode}.rou.xml"
    num = 28
    rand=[0,0,0,0,0,0,0,0,0,0]
    rand_shift_left =random.uniform(-15,11.5)
    rand_shift_right =random.uniform(-15,11.5)
    for i in range(10):
        # rand[i] = random.uniform(-3.5,3.5)
        rand[i] = random.uniform(0,3.5)
        print(rand[i])
    with open(file_path, "w") as routes:
        print("""<routes>
        <vType accel="3" decel="4.5" departSpeed = "22" id="car" length="5" maxSpeed="22.22" minGap="0.0" sigma="1" lcSpeedGain ="0"/>
        <vType accel="2.5" decel="4.5" departSpeed = "33" emergencyDecel="9.0" id="truck" length="12" maxSpeed="33.3333" minGap="0.0" sigma="1" collisionMinGapFactor="0" tau="0.0001" lcSpeedGain ="0" vClass="truck"/>
        <vType accel="2.5" decel="4.5" departSpeed = "33" id="accel_truck" length="12" maxSpeed="33.3333" minGap="0.0" sigma="1" collisionMinGapFactor="0" tau="0.0001" lcSpeedGain ="0" vClass="truck"/>
        <route id="route0" color="1,1,0" edges="E5"/>
        
        <vehicle id="ego" type="car" route="route0" depart="0.00" departSpeed="avg" departPos="95" departLane="1" color="0,1,0"/>   
                
        <vehicle id="car.leftrear0" type="car" route="route0" depart="0.00" departSpeed="avg" departPos="""+"\""+str(40+rand[0]+rand_shift_left)+"\""+""" departLane="2" color="1,1,0"/>
        <vehicle id="car.leftrear1" type="car" route="route0" depart="0.00" departSpeed="avg" departPos="""+"\""+str(70+rand[1]+rand_shift_left)+"\""+""" departLane="2" color="1,1,0"/>
        <vehicle id="accel.rear" type="accel_truck" route="route0" depart="0.00" departSpeed="max" departPos="0" departLane="1" color="1,0,0"/>
        <vehicle id="car.rightrear0" type="car" route="route0" depart="0.00" departSpeed="avg" departPos="""+"\""+str(40+rand[2]+rand_shift_right)+"\""+""" departLane="0" color="1,1,0"/> 
        <vehicle id="car.rightrear1" type="car" route="route0" depart="0.00" departSpeed="avg" departPos="""+"\""+str(70+rand[3]+rand_shift_right)+"\""+""" departLane="0" color="1,1,0"/> 
        <vehicle id="car.leftforward0" type="car" route="route0" depart="0.00" departSpeed="avg" departPos="""+"\""+str(110+rand[4]+rand_shift_left)+"\""+""" departLane="2" color="1,1,0"/>
        <vehicle id="car.leftforward1" type="car" route="route0" depart="0.00" departSpeed="avg" departPos="""+"\""+str(145+rand[5]+rand_shift_left)+"\""+""" departLane="2" color="1,1,0"/>
        <vehicle id="car.leftforward2" type="car" route="route0" depart="0.00" departSpeed="avg" departPos="""+"\""+str(175+rand[6]+rand_shift_left)+"\""+""" departLane="2" color="1,1,0"/>
        <vehicle id="car.forward" type="car" route="route0" depart="0.00" departSpeed="avg" departPos="195" departLane="1" color="1,1,0"/>
        <vehicle id="car.rightforward0" type="car" route="route0" depart="0.00" departSpeed="avg" departPos="""+"\""+str(110+rand[7]+rand_shift_right)+"\""+""" departLane="0" color="1,1,0"/> 
        <vehicle id="car.rightforward1" type="car" route="route0" depart="0.00" departSpeed="avg" departPos="""+"\""+str(145+rand[8]+rand_shift_right)+"\""+""" departLane="0" color="1,1,0"/>
        <vehicle id="car.rightforward2" type="car" route="route0" depart="0.00" departSpeed="avg" departPos="""+"\""+str(175+rand[9]+rand_shift_right)+"\""+""" departLane="0" color="1,1,0"/> 
        
                    
        """, file=routes)
        print("</routes>", file=routes)



if __name__ == "__main__":
    
    
    
    for episode in range(5,10000):
        generate_routefile(episode)
        # first, generate the route file for this simulation
          
        print('episode : ',episode)  
        