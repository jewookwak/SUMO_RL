#!/usr/bin/env python
# Eclipse SUMO Traffic Collision Simulator

import os
import datetime
import subprocess
import sys
import shutil
import random
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sumolib import checkBinary
import traci
import traci.constants as tc

# Global configuration
USE_GUI = True
SIM_DURATION = 3000  # simulation end time
STEP_LENGTH = 0.01  # simulation step length
SEED = 42  # random seed for reproducibility
DATE_STR = datetime.datetime.now().strftime("%Y%m%d")

# Set random seed for reproducibility
random.seed(SEED)
np.random.seed(SEED)

# Path setup
if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
else:
    sys.path.append(os.path.join(os.path.dirname(__file__), "..", "..", "..", "tools"))

# Initialize data storage for different vehicle types
vehicle_data = {
    'truck': [[] for _ in range(30)],  # More capacity for different trucks
    'car': [[] for _ in range(240)],    # More capacity for different cars
    'aggr': [[] for _ in range(30)]    # Aggressive vehicles
}

# Collision tracking
collisions = []

def check_files_exist():
    """Check if required files exist in the current directory."""
    current_dir = os.path.dirname(os.path.abspath(__file__))
    files_to_check = ["highway.sumocfg", "highway.net.xml"]
    
    for file in files_to_check:
        file_path = os.path.join(current_dir, file)
        if not os.path.exists(file_path):
            print(f"Missing file: {file_path}")
            return False
    return True

def get_options():
    """Parse command line options."""
    import optparse
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true", default=False, help="Run without GUI")
    options, _ = optParser.parse_args()
    return options

def generate_routefile():
    """Generate route file with different vehicle configurations."""
    with open("highway.rou.xml", "w") as routes:
        print("""<routes>
        <!-- Regular cars: moderate parameters -->
        <vType id="car" 
            accel="2.9" 
            decel="4.5" 
            departSpeed="22" 
            length="5" 
            maxSpeed="33.3" 
            minGap="2.0" 
            sigma="0.5" 
            lcSpeedGain="1.0"
            carFollowModel="Krauss" 
            laneChangeModel="LC2013"
            color="0,1,0" />
        
        <!-- Trucks: larger, slower vehicles -->
        <vType id="truck" 
            accel="1.5" 
            decel="3.5" 
            departSpeed="33" 
            emergencyDecel="7.0" 
            length="12" 
            maxSpeed="25.0" 
            minGap="2.5" 
            sigma="0.8" 
            lcStrategic="0.5" 
            lcSpeedGain="0.5" 
            vClass="truck" 
            carFollowModel="ACC" 
            laneChangeModel="LC2013"
            color="0,0,1" />
            
        <!-- Aggressive vehicles: minimal gap, high accel/decel -->
        <vType id="aggressive" 
            accel="4.0" 
            decel="9.0" 
            departSpeed="33" 
            length="12" 
            maxSpeed="40.0" 
            minGap="0.1" 
            sigma="0.9" 
            emergencyDecel="10.0"
            lcSpeedGain="1.5"
            vClass="truck" 
            lcPushy="1.0" 
            lcAssertive="1.0"
            lcImpatience="1.0"
            carFollowModel="Wiedemann" 
            laneChangeModel="LC2013"
            color="1,0,0" />

        <!-- Car flows (regular pattern) -->
        <flow id="car0" departPos="free" departLane="2" begin="0.00" from="E5" to="E1" end="500.00" number="30" type="car"/>
        <flow id="car1" departPos="free" departLane="1" begin="5.00" from="E5" to="E1" end="500.00" number="30" type="car"/>
        <flow id="car2" departPos="free" departLane="0" begin="10.00" from="E5" to="E1" end="500.00" number="30" type="car"/>
        <flow id="car3" departPos="free" departLane="2" begin="15.00" from="E5" to="E1" end="500.00" number="30" type="car"/>
        <flow id="car4" departPos="free" departLane="1" begin="20.00" from="E5" to="E1" end="500.00" number="30" type="car"/>
        <flow id="car5" departPos="free" departLane="0" begin="25.00" from="E5" to="E1" end="500.00" number="30" type="car"/>
        <flow id="car6" departPos="free" departLane="2" begin="30.00" from="E5" to="E1" end="500.00" number="30" type="car"/>
        <flow id="car7" departPos="free" departLane="1" begin="35.00" from="E5" to="E1" end="500.00" number="30" type="car"/>
        
        <!-- Truck flows (slower vehicles) -->
        <flow id="truck0" departPos="free" departLane="2" begin="50.00" from="E5" to="E1" end="500.00" number="10" type="truck"/>
        <!-- Aggressive truck flows (high collision probability) -->
        <flow id="aggr0" departPos="free" departLane="1" begin="75.00" from="E5" to="E1" end="500.00" number="10" type="aggressive"/>
        <!-- Truck flows (slower vehicles) -->
        <flow id="truck1" departPos="free" departLane="1" begin="100.00" from="E5" to="E1" end="500.00" number="10" type="truck"/>
        <!-- Aggressive truck flows (high collision probability) -->
        <flow id="aggr1" departPos="free" departLane="2" begin="125.00" from="E5" to="E1" end="500.00" number="10" type="aggressive"/>
        <!-- Truck flows (slower vehicles) -->
        <flow id="truck2" departPos="free" departLane="2" begin="150.00" from="E5" to="E1" end="500.00" number="10" type="truck"/>
        <!-- Aggressive truck flows (high collision probability) -->
        <flow id="aggr2" departPos="free" departLane="1" begin="175.00" from="E5" to="E1" end="500.00" number="10" type="aggressive"/>
        <!-- Truck flows (slower vehicles) -->

        
        
        
        """, file=routes)
        print("</routes>", file=routes)

def save_csv_data():
    """Save simulation data to CSV files."""
    # Prepare columns
    columns = ['time', 'vehicle_id', 'x', 'y', 'v_x', 'v_y', 'a_x', 'a_y', 'theta_h', 
               'leader_id', 'leader_dist', 'leader_speed', 'lane']
    
    # Truck data
    truck_data_frames = []
    for i, data_list in enumerate(vehicle_data['truck']):
        if data_list:  # Skip empty lists
            df = pd.DataFrame(data_list, columns=columns)
            truck_data_frames.append(df)
    
    if truck_data_frames:
        truck_df = pd.concat(truck_data_frames, ignore_index=True)
    else:
        truck_df = pd.DataFrame(columns=columns)
    
    # Car data
    car_data_frames = []
    for i, data_list in enumerate(vehicle_data['car']):
        if data_list:  # Skip empty lists
            df = pd.DataFrame(data_list, columns=columns)
            car_data_frames.append(df)
    
    if car_data_frames:
        car_df = pd.concat(car_data_frames, ignore_index=True)
    else:
        car_df = pd.DataFrame(columns=columns)
    
    # Aggressive vehicle data
    aggr_data_frames = []
    for i, data_list in enumerate(vehicle_data['aggr']):
        if data_list:  # Skip empty lists
            df = pd.DataFrame(data_list, columns=columns)
            aggr_data_frames.append(df)
    
    if aggr_data_frames:
        aggr_df = pd.concat(aggr_data_frames, ignore_index=True)
    else:
        aggr_df = pd.DataFrame(columns=columns)
        
    # Collision data
    collision_df = pd.DataFrame(collisions, columns=['time', 'victim', 'collider'])
    
    # Create log directory
    log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "log_data", DATE_STR)
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    
    # Save data
    truck_df.to_csv(os.path.join(log_dir, 'log_data_truck.csv'), index=False)
    car_df.to_csv(os.path.join(log_dir, 'log_data_car.csv'), index=False)
    aggr_df.to_csv(os.path.join(log_dir, 'log_data_aggressive.csv'), index=False)
    collision_df.to_csv(os.path.join(log_dir, 'log_data_collision.csv'), index=False)
    
    # Create statistics summary
    stats = {
        'total_vehicles': len(truck_df['vehicle_id'].unique()) + len(car_df['vehicle_id'].unique()) + len(aggr_df['vehicle_id'].unique()),
        'total_collisions': len(collision_df),
        'truck_count': len(truck_df['vehicle_id'].unique()),
        'car_count': len(car_df['vehicle_id'].unique()),
        'aggressive_count': len(aggr_df['vehicle_id'].unique()),
        'avg_truck_speed': truck_df['v_x'].mean() if not truck_df.empty else 0,
        'avg_car_speed': car_df['v_x'].mean() if not car_df.empty else 0,
        'avg_aggressive_speed': aggr_df['v_x'].mean() if not aggr_df.empty else 0,
    }
    
    # Save statistics
    with open(os.path.join(log_dir, 'simulation_stats.txt'), 'w') as f:
        for key, value in stats.items():
            f.write(f"{key}: {value}\n")
    
    print(f"Data saved to {log_dir}")
    return log_dir

def visualize_data(log_dir):
    """Create visualizations from the simulation data."""
    # Load data
    truck_df = pd.read_csv(os.path.join(log_dir, 'log_data_truck.csv'))
    car_df = pd.read_csv(os.path.join(log_dir, 'log_data_car.csv'))
    aggr_df = pd.read_csv(os.path.join(log_dir, 'log_data_aggressive.csv'))
    collision_df = pd.read_csv(os.path.join(log_dir, 'log_data_collision.csv'))
    
    # 1. Speed distributions
    plt.figure(figsize=(12, 8))
    plt.subplot(3, 1, 1)
    truck_df['v_x'].hist(bins=20, alpha=0.7)
    plt.title('Truck Speed Distribution')
    plt.xlabel('Speed (m/s)')
    plt.ylabel('Frequency')
    
    plt.subplot(3, 1, 2)
    car_df['v_x'].hist(bins=20, alpha=0.7)
    plt.title('Car Speed Distribution')
    plt.xlabel('Speed (m/s)')
    plt.ylabel('Frequency')
    
    plt.subplot(3, 1, 3)
    aggr_df['v_x'].hist(bins=20, alpha=0.7)
    plt.title('Aggressive Vehicle Speed Distribution')
    plt.xlabel('Speed (m/s)')
    plt.ylabel('Frequency')
    
    plt.tight_layout()
    plt.savefig(os.path.join(log_dir, 'speed_distributions.png'))
    
    # 2. Collision Timeline
    if not collision_df.empty:
        plt.figure(figsize=(10, 6))
        collision_times = collision_df['time'].sort_values()
        plt.plot(range(len(collision_times)), collision_times, 'ro-')
        plt.title('Collision Timeline')
        plt.xlabel('Collision Number')
        plt.ylabel('Simulation Time (s)')
        plt.grid(True)
        plt.savefig(os.path.join(log_dir, 'collision_timeline.png'))
    
    # 3. Vehicle density over time
    time_bins = np.linspace(0, SIM_DURATION, 50)
    
    vehicle_counts = {}
    for df, vehicle_type in zip([truck_df, car_df, aggr_df], ['Truck', 'Car', 'Aggressive']):
        if not df.empty:
            counts = []
            for t in time_bins:
                count = len(df[df['time'] <= t]['vehicle_id'].unique())
                counts.append(count)
            vehicle_counts[vehicle_type] = counts
    
    plt.figure(figsize=(10, 6))
    for vehicle_type, counts in vehicle_counts.items():
        plt.plot(time_bins, counts, '-', label=vehicle_type)
    
    plt.title('Vehicle Count Over Time')
    plt.xlabel('Simulation Time (s)')
    plt.ylabel('Number of Vehicles')
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join(log_dir, 'vehicle_counts.png'))
    
    # 4. Acceleration distributions
    plt.figure(figsize=(12, 8))
    plt.subplot(3, 1, 1)
    truck_df['a_x'].hist(bins=20, alpha=0.7)
    plt.title('Truck Acceleration Distribution')
    plt.xlabel('Acceleration (m/s²)')
    plt.ylabel('Frequency')
    
    plt.subplot(3, 1, 2)
    car_df['a_x'].hist(bins=20, alpha=0.7)
    plt.title('Car Acceleration Distribution')
    plt.xlabel('Acceleration (m/s²)')
    plt.ylabel('Frequency')
    
    plt.subplot(3, 1, 3)
    aggr_df['a_x'].hist(bins=20, alpha=0.7)
    plt.title('Aggressive Vehicle Acceleration Distribution')
    plt.xlabel('Acceleration (m/s²)')
    plt.ylabel('Frequency')
    
    plt.tight_layout()
    plt.savefig(os.path.join(log_dir, 'acceleration_distributions.png'))

def run_simulation():
    """Run the TraCI control loop with randomly varying vehicle parameters."""
    # Parameter ranges for randomization
    param_ranges = {
        'car': {
            'accel': (1.5, 3.0),
            'decel': (3.0, 5.0),
            'maxSpeed': (25.0, 35.0),
            'minGap': (0.5, 3.0),
            'tau': (0.5, 2.0),
            'lcSpeedGain': (0.0, 1.0)
        },
        'truck': {
            'accel': (0.8, 2.0),
            'decel': (2.0, 4.0),
            'maxSpeed': (20.0, 30.0),
            'minGap': (0.2, 4.0),
            'tau': (1.0, 3.0),
            'lcSpeedGain': (0.0, 0.5)
        },
        'aggressive': {
            'accel': (3.0, 5.0),
            'decel': (5.0, 10.0),
            'maxSpeed': (30.0, 45.0),
            'minGap': (0.0, 1.0),
            'tau': (0.1, 1.0),
            'lcSpeedGain': (1.0, 2.0)
        }
    }
    
    # Vehicle parameter storage (to track previously set values)
    vehicle_params = {}
    
    # Store vehicle IDs by type
    vehicle_ids_by_type = {'car': [], 'truck': [], 'aggressive': []}
    
    # Continue as long as there are vehicles in the simulation
    step = 0
    while traci.simulation.getMinExpectedNumber() > 0 and step < SIM_DURATION / STEP_LENGTH:
        traci.simulationStep()
        step += 1
        
        # Current simulation time
        sim_time = traci.simulation.getTime()
        
        # Check for collisions
        if len(traci.simulation.getCollidingVehiclesIDList()) != 0:
            collision_pair = traci.simulation.getCollidingVehiclesIDList()
            collisions.append([sim_time, collision_pair[0], collision_pair[1]])
            print(f"Collision at time {sim_time}: {collision_pair}")
        
        # Process all vehicles currently on edge E1 (main road)
        e1_vehicles = traci.edge.getLastStepVehicleIDs('E1')
        for veh_id in e1_vehicles:
            # Get vehicle type
            veh_type = traci.vehicle.getTypeID(veh_id)
            
            # Classify by base type
            if veh_type.startswith('car'):
                type_key = 'car'
            elif veh_type.startswith('truck'):
                type_key = 'truck'
            elif veh_type.startswith('aggressive'):
                type_key = 'aggressive'
            else:
                type_key = 'car'  # Default
            
            # Track vehicle IDs by type
            if veh_id not in vehicle_ids_by_type[type_key]:
                vehicle_ids_by_type[type_key].append(veh_id)
                
                # Index for data storage
                index = len(vehicle_ids_by_type[type_key]) - 1
                if index >= len(vehicle_data[type_key]):
                    # Expand storage if needed
                    vehicle_data[type_key].append([])
            else:
                index = vehicle_ids_by_type[type_key].index(veh_id)
            
            # Apply random parameters if not already set for this vehicle
            if veh_id not in vehicle_params:
                # Generate and store random parameters
                ranges = param_ranges[type_key]
                params = {
                    'accel': round(random.uniform(ranges['accel'][0], ranges['accel'][1]), 2),
                    'decel': round(random.uniform(ranges['decel'][0], ranges['decel'][1]), 2),
                    'maxSpeed': round(random.uniform(ranges['maxSpeed'][0], ranges['maxSpeed'][1]), 2),
                    'minGap': round(random.uniform(ranges['minGap'][0], ranges['minGap'][1]), 2),
                    'tau': round(random.uniform(ranges['tau'][0], ranges['tau'][1]), 2),
                    'lcSpeedGain': round(random.uniform(ranges['lcSpeedGain'][0], ranges['lcSpeedGain'][1]), 2)
                }
                vehicle_params[veh_id] = params
                
                # Apply to vehicle
                traci.vehicle.setAccel(veh_id, params['accel'])
                traci.vehicle.setDecel(veh_id, params['decel'])
                traci.vehicle.setMaxSpeed(veh_id, params['maxSpeed'])
                traci.vehicle.setMinGap(veh_id, params['minGap'])
                traci.vehicle.setTau(veh_id, params['tau'])
                
                # Special handling for aggressive vehicles
                if type_key == 'aggressive':
                    # 50% chance of disabling safety checks for truly aggressive behavior
                    if random.random() < 0.5:
                        traci.vehicle.setSpeedMode(veh_id, 0)  # Disable all safety checks
                        # Increase lane change aggressiveness
                        traci.vehicle.setLaneChangeMode(veh_id, 0b111111111)  # Enable all lane change options
                    else:
                        # Semi-aggressive: still obey some rules
                        traci.vehicle.setSpeedMode(veh_id, 31)  # Keep some safety checks
                
                # Random lane change behavior for all vehicles
                # if random.random() < 0.3:  # 30% chance
                #     # Initiate a lane change if possible
                #     current_lane = traci.vehicle.getLaneIndex(veh_id)
                #     lane_count = traci.lane.getLinks(f"E1_{current_lane}") 
                    
                #     if current_lane > 0 and random.random() < 0.5:
                #         # Change left
                #         traci.vehicle.changeLaneRelative(veh_id, -1, 5.0)
                #     elif current_lane < 2:  # Assuming 3 lanes (0, 1, 2)
                #         # Change right
                #         traci.vehicle.changeLaneRelative(veh_id, 1, 5.0)
            
            # Collect vehicle state data
            leader_info = traci.vehicle.getLeader(veh_id, 100)  # Get leader within 100m
            
            # Calculate acceleration
            v_prev = 0
            if index < len(vehicle_data[type_key]) and len(vehicle_data[type_key][index]) > 0:
                v_prev = vehicle_data[type_key][index][-1][4]  # Previous speed
            
            v_current = traci.vehicle.getSpeed(veh_id)
            a_x = v_current - v_prev
            
            # lateral acceleration (from lateral speed change)
            v_y_prev = 0
            if index < len(vehicle_data[type_key]) and len(vehicle_data[type_key][index]) > 0:
                v_y_prev = vehicle_data[type_key][index][-1][5]  # Previous lateral speed
            
            v_y_current = traci.vehicle.getLateralSpeed(veh_id)
            a_y = v_y_current - v_y_prev
            
            vehicle_state = [
                sim_time,
                veh_id,
                traci.vehicle.getPosition(veh_id)[0],  # x position
                traci.vehicle.getPosition(veh_id)[1],  # y position
                v_current,                             # x velocity
                v_y_current,                           # y velocity
                a_x,                                   # x acceleration
                a_y,                                   # y acceleration
                traci.vehicle.getAngle(veh_id),        # heading angle
                leader_info[0] if leader_info else "None",  # leader ID
                leader_info[1] if leader_info else 999,     # distance to leader
                traci.vehicle.getSpeed(leader_info[0]) if leader_info else 0,  # leader speed
                traci.vehicle.getLaneIndex(veh_id)     # current lane
            ]
            
            # Store data
            if index < len(vehicle_data[type_key]):
                vehicle_data[type_key][index].append(vehicle_state)
        
        # Process vehicles on edge E5 (entry road) - for possible direct interference
        e5_vehicles = traci.edge.getLastStepVehicleIDs('E5')
        for veh_id in e5_vehicles:
            # 잠재적 충돌 시나리오를 만들기 위한 간헐적 차선 변경
            if random.random() < 0.1:  # 10% 확률
                current_lane = traci.vehicle.getLaneIndex(veh_id)
                # lane 0으로의 이동 방지
                if current_lane < 2:  # 차선 0에 있는 차량은 항상 오른쪽으로 이동하도록
                    traci.vehicle.changeLaneRelative(veh_id, -1, 3.0)
        for veh_id in e1_vehicles:
            # 잠재적 충돌 시나리오를 만들기 위한 간헐적 차선 변경
            if random.random() < 0.05:  # 5% 확률
                current_lane = traci.vehicle.getLaneIndex(veh_id)
                # lane 0으로의 이동 방지
                if current_lane < 2:  # 차선 0에 있는 차량은 항상 오른쪽으로 이동하도록
                    traci.vehicle.changeLaneRelative(veh_id, -1, 3.0)
                
    # Close TraCI connection
    traci.close()
    print("Simulation completed.")

if __name__ == "__main__":
    # Verify that required files exist
    if not check_files_exist():
        sys.exit(1)
    
    # Set working directory to script location
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    
    # Parse command line options
    options = get_options()
    
    # Choose SUMO binary
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')
    
    # Generate route file
    generate_routefile()
    
    # Start SUMO
    traci.start([
        sumoBinary,
        "-c", "highway.sumocfg",
        '--lanechange.duration', '3',
        '--collision.action', 'remove',
        '--collision.mingap-factor', '0',
        '--collision-output', 'collisions.xml',
        '--step-length', str(STEP_LENGTH),
        '--no-step-log'
    ])
    
    # Run simulation
    run_simulation()
    
    # Save data to CSV
    log_dir = save_csv_data()
    
    # Create visualizations
    visualize_data(log_dir)
    
    print(f"Simulation results saved to {log_dir}")