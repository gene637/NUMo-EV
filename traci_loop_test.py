#!/usr/bin/env python

# @file    I-TSC.py
# @author  David Oswald
# @date    August 2021

# Code is to recreate I-TSC from the paper:
# A Coupled Vehicle-Signal Control Method at Signalized Intersections in Mixed Traffic Environment
# written by Yu Du, Wei ShangGuan, and Linguo Chai

# function: generate_routefile is taken from runner.py from the traci_tls example code.
# function generate a route file that randomly routes vehicles.

from __future__ import absolute_import
from __future__ import print_function

from sklearn.ensemble import RandomForestRegressor
import os
import sys
import optparse
import random
import math
import datetime
import time
import pickle
# from EAD_regressor import *
from timeit import default_timer as timer

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa
from scipy.optimize import Bounds
from scipy.optimize import minimize
import numpy as np


def generate_routefile():
    global RandSeed
    random.seed(RandSeed)  # make tests reproducible
    N = 500  # number of time steps; usually is 3600
    Pr = PEN_RATE   # CAV penetration rate

def calculate_pass_time(veh_id, tlsid):
    """
    Calculate the time interval during which a vehicle can pass through a traffic light.

    :param veh_id: The ID of the vehicle.
    :param tlsid: The ID of the traffic light system.
    :return: A tuple (Tstart, Tend) representing the start and end time intervals during which the vehicle can pass.
    """
    
    # Get the index of the traffic light and the current state of the vehicle's lane
    tls_index = traci.vehicle.getNextTLS(veh_id)[0][1]  # Traffic light index for the vehicle
    vehicle_state = traci.vehicle.getNextTLS(veh_id)[0][3]  # Current state for the vehicle's lane

    # Get all the traffic light logic
    tl_log = traci.trafficlight.getAllProgramLogics(tlsid)
    print(tl_log)
    # Get the current phase and state of the traffic light
    current_phase = traci.trafficlight.getPhase(tlsid)
    current_state = traci.trafficlight.getRedYellowGreenState(tlsid)
    next_switch = traci.trafficlight.getNextSwitch(tlsid)
    
    # Retrieve phase information from tl_log
    phases = tl_log[0].phases
    print(phases)
    current_duration = phases[current_phase].duration
    phase_states = [phase.state for phase in phases]
    
    # Calculate the current time and time to the next signal change
    current_time = traci.simulation.getTime()
    Cycle= sum(phase.duration for phase in phases)
    
    # Initialize Tstart and Tend
    Tstart = None
    Tend = None
    TGy = None

    if vehicle_state in 'GgYy':
        # If the current state is green or yellow
        Tstart = 0
        time_to_next_red = 0
        time_to_last_red = 0
        remaining_time_in_current_phase = next_switch - current_time
        
        # Find the time to the next red light
        for i in range(current_phase + 1, len(phases) + current_phase + 1):
            next_phase_index = i % len(phases)
            next_phase_state = phases[next_phase_index].state
            
            if 'r' in next_phase_state[tls_index]:
                break
            else:
                time_to_next_red += phases[next_phase_index].duration
        
        Tend = time_to_next_red + remaining_time_in_current_phase

        # Find the time of Green and yellow duration
        for i in range(current_phase - 1, current_phase - 1 - len(phases)):
            last_phase_index = i % len(phases)
            last_phase_state = phases[last_phase_index].state
            
            if 'r' in last_phase_state[tls_index]:
                break
            else:
                time_to_last_red += phases[last_phase_index].duration
        TGy = time_to_last_red + current_duration + time_to_next_red
    
    elif vehicle_state == 'r':
        # If the current state is red
        time_to_next_green = 0
        remaining_time_in_current_phase = next_switch - current_time
        
        # Find the time to the next green light
        for i in range(current_phase + 1, len(phases) + current_phase + 1):
            next_phase_index = i % len(phases)
            next_phase_state = phases[next_phase_index].state
            
            if 'G' in next_phase_state[tls_index] or 'g' in next_phase_state[tls_index]:
                Tstart = time_to_next_green + remaining_time_in_current_phase
                time_to_next_red2 = phases[next_phase_index].duration
                # Find the time to the next red light after green light
                for j in range(i + 1, len(phases) + current_phase + 1):
                    next_phase_index2 = j % len(phases)
                    next_phase_state2 = phases[next_phase_index2].state                    
                    if 'r' in next_phase_state2[tls_index]:
                        Tend = Tstart + time_to_next_red2
                        break
                    else:
                        time_to_next_red2 += phases[next_phase_index2].duration
            else:
                time_to_next_green += phases[next_phase_index].duration

        TGy = Tend - Tstart
    
    return Tstart, Tend, Cycle, TGy

def run():
    """execute the TraCI control loop"""

    step = 0
    # CAV penetration rate
    pen_rate = PEN_RATE
    Qab = []
    EAD_ID_list = []
    veh_vel = []
    last_speed = []
    total_co2_emission = 0.0
    total_fuel_consumption = 0.0
    total_co2_emission_cav = 0.0
    total_fuel_consumption_cav = 0.0
    total_electricity = 0.0
    total_electricity_caev = 0.0

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        if step>7199:

            tlsid = 'cluster_619627545_658976990'
            tl_log = traci.trafficlight.getAllProgramLogics(tlsid)
            program = traci.trafficlight.getProgram(tlsid)
            period_num = int(program[5:])+1
            for i in range(len(tl_log)):
                if tl_log[i].programID == program:
                    phases = tl_log[i].phases
            print(period_num)
            print(phases)
            print(tl_log)
            # time = traci.simulation.getTime()
            # vehicle_ids = traci.vehicle.getIDList()
            # for veh_id in vehicle_ids:
            #     if veh_id == '47':
            #         tlsid = traci.vehicle.getNextTLS(veh_id)[0][0]
            #         if tlsid == 'cluster_619627545_658976990':
            #             next_switch = traci.trafficlight.getNextSwitch(tlsid)
            #             print('duration:', next_switch-time)
            #             Tstart, Tend, Cycle, TGy = calculate_pass_time(veh_id, tlsid)
            #             print(f'Tstart{Tstart}, Tend{Tend}, Cycle{Cycle}, TGy{TGy}')




        step += 1
    
    if EV == 'off':
        print(f"Total CO2 emission: {total_co2_emission} kg")
        print(f"Total fuel consumption: {total_fuel_consumption} kg")
    else:
        print(f"Total elecctricity consumption: {total_electricity} Wh")
        print(f"cav elecctricity consumption: {total_electricity_caev} Wh")
        print(f"other ev elecctricity consumption: {total_electricity - total_electricity_caev} Wh")

    traci.close()
    sys.stdout.flush()


def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=True, help="run the commandline version of sumo")
    optParser.add_option('--PR', type='int', default=100, help='CAV penetration rate')
    optParser.add_option('--evPR', type='int', default=100, help='EV penetration rate')
    optParser.add_option('--RS', type='int', default=42, help='Random Seed number')
    optParser.add_option('--vcRat', type='int', default=10, help='VC ratio: 10, 67, 52, ...')
    optParser.add_option('--Hour', type='int', default=0, help='traffic hour: 0, 7, 10, 14, ...')
    options, args = optParser.parse_args()
    return options


###########################################
# Global Constants
options = get_options()
PEN_RATE = (options.PR/100)
RandSeed = options.RS
EAD = 'off'
EV = 'off'
###########################################


# this is the main entry point of this script
if __name__ == "__main__":
    options = get_options()
    print("options:")
    print(options)
    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # first, generate the route file for this simulation
    # generate_routefile()

    # save_folder = '/home/gene/Downloads/EAD code test/peak_hour_0.1caep/'

    # if not os.path.exists(save_folder):
    #     os.makedirs(save_folder)

    traci.start([sumoBinary, "-c", "/home/gene/numo/nagoya.sumocfg", "--step-length", "1", "--device.rerouting.threads",
                     "64", "--device.rerouting.synchronize", "true", "--time-to-teleport", "100"])
                    #  "--tripinfo-output", save_folder+"tripinfo.xml", "--summary", save_folder+"summary_100.xml", "--amitran-output", save_folder+"TrajectoryOutput.xml",
                    #  "--queue-output", save_folder+"QueueOutput.xml", "--statistic-output", save_folder+"StatOutput.xml", "--fcd-output", save_folder+"FCDOutput.xml", "--emission-output", save_folder+"EmissionOutput.xml"])
    run()
