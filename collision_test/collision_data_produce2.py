#!/usr/bin/env python
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
# Copyright (C) 2008-2023 German Aerospace Center (DLR) and others.
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# https://www.eclipse.org/legal/epl-2.0/

"""
SUMO Highway Simulation with Vehicle Tracking and Collision Detection
"""

from __future__ import absolute_import
from __future__ import print_function

import os
import subprocess
import sys
import shutil
import argparse
import random
import pathlib
from datetime import datetime
from typing import Dict, List, Tuple, Optional, Any, Union

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Add SUMO tools to path
if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
else:
    sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'tools'))
    sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'tools'))

import traci
import traci.constants as tc
from sumolib import checkBinary


class HighwaySimulation:
    def __init__(self, args):
        """Initialize the simulation with arguments"""
        self.args = args
        
        # Set default directories
        self.log_dir = args.log_dir if args.log_dir else os.path.join(
            os.getcwd(), 'log_data', datetime.now().strftime("%Y%m%d"))
        
        # Create log directory if it doesn't exist
        pathlib.Path(self.log_dir).mkdir(parents=True, exist_ok=True)
        
        # Initialize vehicle data storage
        self.vehicle_data = {
            'accel0': [[] for _ in range(5)],
            'accel1': [[] for _ in range(5)],
            'accel2': [[] for _ in range(5)],
            'accel3': [[] for _ in range(5)],
            'truck0': [[] for _ in range(5)],
            'truck1': [[] for _ in range(5)],
            'truck2': [[] for _ in range(5)],
            'truck3': [[] for _ in range(5)],
            'car0': [[] for _ in range(10)],
            'car1': [[] for _ in range(10)],
            'car2': [[] for _ in range(10)],
            'car3': [[] for _ in range(10)],
            'car4': [[] for _ in range(10)],
            'car5': [[] for _ in range(10)],
            'car6': [[] for _ in range(10)],
            'car7': [[] for _ in range(10)]
        }
        
        # Collision data
        self.collision_data = []
        
        # Column names for the CSV files
        self.data_columns = [
            'time', 'vehicle_id', 'x', 'y', 'v_x', 'v_y', 'a_x', 'a_y', 'theta_h',
            'x_for_id', 'x_for', 'v_for', 'x_bac_id', 'x_bac', 'v_bac',
            'left_leader', 'left_follower', 'right_leader', 'right_follower'
        ]
        
        # Random speeds for accelerating vehicles
        self.random_speeds = None

    def generate_route_file(self):
        """Generate the SUMO route file for the simulation"""
        with open("highway.rou.xml", "w") as routes:
            print("""<routes>
            <vType accel="3" decel="4.5" departSpeed="22" id="car" length="5" maxSpeed="27.77" minGap="0.0" sigma="1" lcSpeedGain="0" />
            <vType accel="2.5" decel="4.5" departSpeed="22" emergencyDecel="9.0" id="truck" length="12" maxSpeed="33.3333" minGap="0.0" sigma="1" collisionMinGapFactor="0" tau="0.0001" lcSpeedGain="0" vClass="truck" color="0,0,1"/>
            <vType accel="2.5" decel="4.5" departSpeed="22" id="accel_truck" length="12" maxSpeed="33.3333" minGap="0.0" sigma="1" collisionMinGapFactor="0" tau="0.0001" lcSpeedGain="0" vClass="truck" color="1,0,0"/>
            
            <flow id="car0" departPos="free" begin="0.00" from="E4" to="E6" end="500.00" number="10" type="car"/>
            <flow id="car1" begin="5.00" from="E4" to="E8" end="500.00" number="10" type="car"/>
            <flow id="car2" begin="10.00" from="E7" to="E6" end="500.00" number="10" type="car" color="0,1,0"/>
            <flow id="car3" begin="15.00" from="E7" to="E8" end="500.00" number="10" type="car" color="0,1,0"/>
            <flow id="truck0" departPos="free" begin="20.00" from="E4" to="E6" end="500.00" number="5" type="truck" color="0,0,1"/>
            <flow id="truck1" departPos="free" begin="25.00" from="E7" to="E6" end="500.00" number="5" type="truck" color="0,0,1"/>
            <flow id="car4" departPos="free" begin="30.00" from="E4" to="E6" end="500.00" number="10" type="car"/>
            <flow id="car5" begin="35.00" from="E4" to="E8" end="500.00" number="10" type="car"/>
            <flow id="car6" begin="40.00" from="E7" to="E6" end="500.00" number="10" type="car" color="0,1,0"/>
            <flow id="car7" begin="45.00" from="E7" to="E8" end="500.00" number="10" type="car" color="0,1,0"/>
            <flow id="truck2" departPos="free" begin="50.00" from="E4" to="E6" end="500.00" number="5" type="truck" color="0,0,1"/>
            <flow id="truck3" departPos="free" begin="55.00" from="E7" to="E6" end="500.00" number="5" type="truck" color="0,0,1"/>
            </routes>""", file=routes)

    def save_data_to_csv(self):
        """Save all collected data to CSV files"""
        try:
            # Process data for each vehicle type
            dfs = {}
            
            # Helper function to convert vehicle data to DataFrame
            def data_to_df(vehicle_type, indices):
                if not self.vehicle_data[vehicle_type] or not any(self.vehicle_data[vehicle_type]):
                    return pd.DataFrame(columns=self.data_columns)
                
                all_data = []
                for idx in indices:
                    if idx < len(self.vehicle_data[vehicle_type]) and self.vehicle_data[vehicle_type][idx]:
                        all_data.extend(self.vehicle_data[vehicle_type][idx])
                
                return pd.DataFrame(all_data, columns=self.data_columns)
            
            # Create DataFrames for each vehicle type
            df_accel = pd.concat([
                data_to_df('accel0', range(5)),
                data_to_df('accel1', range(5)),
                data_to_df('accel0', range(5)),
                data_to_df('accel1', range(5))
            ], ignore_index=True)
            
            df_truck = pd.concat([
                data_to_df('truck0', range(5)),
                data_to_df('truck1', range(5)),
                data_to_df('truck0', range(5)),
                data_to_df('truck1', range(5))
            ], ignore_index=True)
            
            df_car = pd.concat([
                data_to_df('car0', range(10)),
                data_to_df('car1', range(10)),
                data_to_df('car2', range(10)),
                data_to_df('car3', range(10)),
                data_to_df('car0', range(10)),
                data_to_df('car1', range(10)),
                data_to_df('car2', range(10)),
                data_to_df('car3', range(10))
            ], ignore_index=True)
            
            # Create collision DataFrame
            df_collision = pd.DataFrame(self.collision_data, columns=['victim', 'collider'])
            
            # Save DataFrames to CSV
            df_accel.to_csv(os.path.join(self.log_dir, 'log_data_accel.csv'), index=False)
            df_truck.to_csv(os.path.join(self.log_dir, 'log_data_truck.csv'), index=False)
            df_car.to_csv(os.path.join(self.log_dir, 'log_data_car.csv'), index=False)
            df_collision.to_csv(os.path.join(self.log_dir, 'log_data_collision.csv'), index=False)
            
            print(f"Data saved to {self.log_dir}")
            
        except Exception as e:
            print(f"Error saving data to CSV: {e}")

    def calculate_acceleration(self, veh_id: str, current_speed: float, current_lateral_speed: float) -> Tuple[float, float]:
        """
        Calculate longitudinal and lateral acceleration based on previous data
        
        Args:
            veh_id: Vehicle ID
            current_speed: Current longitudinal speed
            current_lateral_speed: Current lateral speed
            
        Returns:
            Tuple of (longitudinal_acceleration, lateral_acceleration)
        """
        # Parse vehicle ID to get type and index
        parts = veh_id.split('.')
        if len(parts) != 2:
            return 0.0, 0.0
            
        veh_type, idx = parts[0], int(parts[1])
        
        # Check for valid vehicle type and index
        if veh_type not in self.vehicle_data or idx >= len(self.vehicle_data[veh_type]) or not self.vehicle_data[veh_type][idx]:
            return 0.0, 0.0
            
        # Calculate accelerations based on previous speeds
        prev_data = self.vehicle_data[veh_type][idx][-1]
        accel_x = current_speed - prev_data[4]  # v_x is at index 4
        accel_y = current_lateral_speed - prev_data[5]  # v_y is at index 5
        
        return accel_x, accel_y

    def process_vehicle(self, veh_id: str):
        """
        Process a single vehicle's data for the current simulation step
        
        Args:
            veh_id: Vehicle ID to process
        """
        # Only process vehicles on road E5
        if traci.vehicle.getRoadID(veh_id) != 'E5':
            return
            
        vehicle_state = []
        
        # Store timestamp
        vehicle_state.append(traci.simulation.getCurrentTime() / 1000)
        vehicle_state.append(veh_id)
        
        # Get position
        x, y = traci.vehicle.getPosition(veh_id)
        vehicle_state.append(x)
        vehicle_state.append(y)
        
        # Get speeds
        v_x = traci.vehicle.getSpeed(veh_id)
        v_y = traci.vehicle.getLateralSpeed(veh_id)
        vehicle_state.append(v_x)
        vehicle_state.append(v_y)
        
        # Calculate accelerations
        a_x, a_y = self.calculate_acceleration(veh_id, v_x, v_y)
        vehicle_state.append(a_x)
        vehicle_state.append(a_y)
        
        # Store heading angle
        vehicle_state.append(traci.vehicle.getAngle(veh_id))
        
        # Process leader vehicle
        leader = traci.vehicle.getLeader(veh_id)
        if leader is not None:
            leader_id, x_for = leader
            v_for = traci.vehicle.getSpeed(leader_id) - v_x
            vehicle_state.extend([leader_id, x_for, v_for])
        else:
            vehicle_state.extend(['None', 'None', 'None'])
        
        # Process follower vehicle
        follower = traci.vehicle.getFollower(veh_id)
        if follower is not None and follower[0] != '':
            follower_id, x_bac = follower
            v_bac = traci.vehicle.getSpeed(follower_id) - v_x
            vehicle_state.extend([follower_id, x_bac, v_bac])
        else:
            vehicle_state.extend(['None', 'None', 'None'])
        
        # Process adjacent lane vehicles
        left_leaders = traci.vehicle.getLeftLeaders(veh_id)
        vehicle_state.append(left_leaders[0][0] if left_leaders and len(left_leaders) > 0 else 'None')
        
        left_followers = traci.vehicle.getLeftFollowers(veh_id)
        vehicle_state.append(left_followers[0][0] if left_followers and len(left_followers) > 0 else 'None')
        
        right_leaders = traci.vehicle.getRightLeaders(veh_id)
        vehicle_state.append(right_leaders[0][0] if right_leaders and len(right_leaders) > 0 else 'None')
        
        right_followers = traci.vehicle.getRightFollowers(veh_id)
        vehicle_state.append(right_followers[0][0] if right_followers and len(right_followers) > 0 else 'None')
        
        # Store the vehicle data in the appropriate list
        parts = veh_id.split('.')
        if len(parts) == 2:
            veh_type, idx = parts[0], int(parts[1])
            if veh_type in self.vehicle_data and idx < len(self.vehicle_data[veh_type]):
                self.vehicle_data[veh_type][idx].append(vehicle_state)
                
        # Control accelerating vehicles
        if veh_id[0] == 'a' and traci.vehicle.getRoadID(veh_id) == 'E5':
            idx = int(veh_id.split('.')[1])
            if idx < len(self.random_speeds):
                traci.vehicle.setSpeed(veh_id, self.random_speeds[idx])
                traci.vehicle.setAccel(veh_id, '50')
                traci.vehicle.setDecel(veh_id, '0.00001')
                traci.vehicle.setSpeedMode(veh_id, '0')

    def run_simulation(self):
        """Run the main simulation loop"""
        # Generate random speeds for accelerating vehicles (only once at the beginning)
        self.random_speeds = [float(random.randint(5000, 5550)) / 100 for _ in range(200)]
        
        # Main simulation loop
        step = 0
        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            step += 1
            
            # Record collisions
            colliding_vehicles = traci.simulation.getCollidingVehiclesIDList()
            if colliding_vehicles:
                self.collision_data.append(colliding_vehicles)
            
            # Process all vehicles
            for veh_id in traci.vehicle.getIDList():
                self.process_vehicle(veh_id)
                
            # Optional: Print progress
            if step % 1000 == 0:
                print(f"Simulation step: {step}, vehicles: {len(traci.vehicle.getIDList())}")
                
        # Close TRACI and flush output
        traci.close()
        sys.stdout.flush()
        
        # Save collected data
        self.save_data_to_csv()

    def setup_and_run(self):
        """Setup the simulation environment and run it"""
        try:
            # Check and build network if needed
            netconvert_binary = checkBinary('netconvert')
            sumo_binary = checkBinary('sumo-gui' if not self.args.nogui else 'sumo')
            
            # Build/check network
            retcode = subprocess.call(
                [netconvert_binary, "-c", "highway.netgcfg"], 
                stdout=sys.stdout, stderr=sys.stderr
            )
            
            try:
                shutil.copy("highway_episodic.net.xml", "net.net.xml")
            except IOError:
                print("Warning: Missing 'highway_episodic.net.xml'")
            
            # Generate route file
            self.generate_route_file()
            
            # Start SUMO with the appropriate options
            traci.start([
                sumo_binary,
                "-c", "highway.sumocfg",
                '--lanechange.duration', str(self.args.lanechange_duration),
                '--collision.action', self.args.collision_action,
                '--collision.mingap-factor', '0',
                '--collision-output', os.path.join(self.log_dir, 'collisions.xml'),
                '--step-length', str(self.args.step_length),
                '--no-step-log'
            ])
            
            # Run the simulation
            self.run_simulation()
            
        except Exception as e:
            print(f"Error in simulation: {e}")
            if traci.isConnected():
                traci.close()


def parse_arguments():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description='SUMO Highway Simulation')
    parser.add_argument('--nogui', action='store_true', default=False,
                        help='Run the commandline version of SUMO')
    parser.add_argument('--log-dir', type=str, 
                        help='Directory to store log files')
    parser.add_argument('--collision-action', type=str, default='remove',
                        choices=['remove', 'warn', 'none'],
                        help='Action to take when vehicles collide')
    parser.add_argument('--lanechange-duration', type=int, default=4,
                        help='Duration of lane change maneuvers in seconds')
    parser.add_argument('--step-length', type=float, default=0.01,
                        help='Simulation step length in seconds')
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_arguments()
    simulation = HighwaySimulation(args)
    simulation.setup_and_run()