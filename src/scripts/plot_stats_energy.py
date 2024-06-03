from os import listdir, environ
from os.path import isdir, join, splitext, dirname, normpath, basename

# Plotting
import numpy as np
import matplotlib
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from matplotlib.patches import Rectangle
from matplotlib.font_manager import FontProperties
from matplotlib.collections import LineCollection
import seaborn as sns
import pandas as pd

# Utility
import sys
import math
import time
from collections import defaultdict
from itertools import combinations
import pprint
from copy import deepcopy

# Parse simulation log
sys.path.append(join(dirname(__file__), "..", "protos", "generated")) # Path to compiled proto files
import time_step_pb2
from load_data import SimData

import networkx as nx


# Path to simulation logs
# RESULTS_DIR = join(environ['HOME'], 'GIT/swarm-energy-replenishment_prep/results/work_and_charge_ants(main)')
# RESULTS_DIR = join(environ['HOME'], 'GIT/swarm-energy-replenishment_prep/results/work_and_charge_ants(work_rate)')
RESULTS_DIR = join(environ['HOME'], 'GIT/swarm-energy-replenishment_prep/results/work_and_charge_ants(recharge_rate)')
# RESULTS_DIR = join(environ['HOME'], 'GIT/swarm-energy-replenishment_prep/results/work_and_charge_ants(transfer_loss)')

BINARY_FILENAME = 'log_data.pb'
SUMMARY_FILENAME = 'summary.csv'
COMMANDS_FILENAME = 'commands.csv'

# Parameters
ROBOT_RADIUS = 0.035
HEADING_LENGTH = 0.05
# COMM_RANGE = 0.8 # simulation
COMM_RANGE = 0.6 # swarmhack

SWARMHACK_X = 0.4 # Fixed distance in the x-direction to subtract from swarmhack log
SWARMHACK_Y = 0.2 # Fixed distance in the y-direction
SWARMHACK_FIXY = 1.09 # Fixed distance to add from swarmhack log after flipping sign
SWARMHACK_ARENA_X = 1.8 # arena height
SWARMHACK_ARENA_Y = 0.9 # arena width

def quaternion_to_euler_angle_vectorized1(w, x, y, z):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = np.arctan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = np.where(t2>+1.0,+1.0,t2)
    #t2 = +1.0 if t2 > +1.0 else t2

    t2 = np.where(t2<-1.0, -1.0, t2)
    #t2 = -1.0 if t2 < -1.0 else t2
    Y = np.arcsin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = np.arctan2(t3, t4)

    return X, Y, Z # in radians


def load_log(path):

    # Load log and commands file
    log_file = join(path, BINARY_FILENAME) # path: RESULTS_DIR/scenario/BINARY_FILE
    # commands_file = join(RESULTS_DIR, scenario, COMMANDS_FILENAME) # path: RESULTS_DIR/scenario/COMMANDS_FILE
    summary_file = join(path, SUMMARY_FILENAME) # path: RESULTS_DIR/scenario/SUMMARY_FILE
    s = SimData(log_file, summary_file)
    # s = SimData(log_file) # Real robot experiments

    return s


def load_log_with_checks(path, print_result=False):
    s = load_log(path)

    # Store points scored
    trial_points = s.totalPoints

    if print_result:
        print(f'### Points scored ###')
        print(f'      scenario: {basename(normpath(path))}, seed: {s.seed}, points: {trial_points}')
        # print(f'### GLOBAL Connectivity ###')
        # print(f'      scenario: {basename(normpath(path))}, seed: {s.seed}, connectivity: {trial_connectivity}')
        # print(f'### TEAM Connectivity ###')
        # print(f'      scenario: {basename(normpath(path))}, seed: {s.seed}, connectivity: {trial_team_connectivity}')
    return s, trial_points


def get_network_length_and_connectors(s, plot=False):
    """Get the network length of produced by the robots and the solver result."""

    # print(s.data[s.totalTime]['log'])

    if plot:
        # Font
        font = FontProperties()
        font.set_family('serif')
        font.set_name('Times New Roman')
        font.set_size(20)

        fig, ax = plt.subplots(nrows=1, ncols=2, figsize=(16, 8))

        # z-order
        z_arena = 0
        z_task = 1
        z_team_trajectory = 2
        z_connection = 3
        z_team_center = 4
        z_robot_position = 5
        z_robot_orientation = 6

    # sns.scatterplot(data=df, ax = ax, x="PositionX", y="PositionY", hue="State", zorder=2)

    # Store position of robots and sort followers by team
    robot_pos_by_time = {}
    team_pos_by_time = {}

    for time in range(1,s.totalTime+1):

        # Store data every 10 timesteps, the first and final timesteps
        if time % 10 == 0 or time == 1 or time == s.totalTime:

            robot_pos = {}
            leader_follower_pos = defaultdict(list)
            team_pos = {}

            # Store robot positions
            for robot in s.data[time]['log'].robots:
                if time == s.totalTime:
                    robot_pos[robot.name] = robot.position

                if robot.teamID != 255 and robot.teamID != 100000: # Ignore connectors and travellers
                    leader_follower_pos[robot.teamID].append(robot.position)

            robot_pos_by_time[time] = robot_pos

            # Store team positions
            for team, positions in leader_follower_pos.items():
                num_robots = len(positions)

                x = y = 0
                for pos in positions:
                    x += pos.x
                    y += pos.y

                team_pos[team] = {
                    'x': x / num_robots,
                    'y': y / num_robots
                }

            team_pos_by_time[time] = team_pos
            print(leader_follower_pos)
    
    num_connectors = 0
    connections = {}

    label_leader = label_follower = label_connector = label_traveler = None

    # init Networkx graph
    G = nx.Graph()
    for team in team_pos_by_time[s.totalTime].keys():
        G.add_node(team)

    # Plot robots
    for robot in s.data[s.totalTime]['log'].robots:

        pos = robot.position
        orientation = robot.orientation

        if plot:
            color = 'white'

            if robot.state == time_step_pb2.Robot.CONNECTOR:
                color = 'cyan'
            elif robot.state == time_step_pb2.Robot.FOLLOWER:
                color = 'limegreen'
            elif robot.state == time_step_pb2.Robot.LEADER:
                color = 'red'
            elif robot.state == time_step_pb2.Robot.TRAVELER:
                color = 'magenta'

            # Draw position
            circle_robot = plt.Circle(xy=(pos.x, pos.y), radius=ROBOT_RADIUS, facecolor=color, fill=True, linewidth=1, edgecolor='k', zorder=z_robot_position)
            
            if robot.state == time_step_pb2.Robot.CONNECTOR:
                label_connector = plt.Line2D([], [], marker='o', linestyle='None', color='k', markerfacecolor=color, markersize=10)
            elif robot.state == time_step_pb2.Robot.FOLLOWER:
                label_follower = plt.Line2D([], [], marker='o', linestyle='None', color='k', markerfacecolor=color, markersize=10)
            elif robot.state == time_step_pb2.Robot.LEADER:
                label_leader = plt.Line2D([], [], marker='o', linestyle='None', color='k', markerfacecolor=color, markersize=10)

            ax[0].add_patch(circle_robot)

            # Draw orientation
            rad = quaternion_to_euler_angle_vectorized1(orientation.w, orientation.x, orientation.y, orientation.z)[2]
            x_values = [pos.x, pos.x + HEADING_LENGTH * math.cos(rad)]
            y_values = [pos.y, pos.y + HEADING_LENGTH * math.sin(rad)]
            ax[0].plot(x_values, y_values, 'k', zorder=z_robot_orientation)

        # Save the connections
        if robot.state == time_step_pb2.Robot.CONNECTOR:
            # print(f'--- {robot.name} --- pos=({robot.position.x}, {robot.position.y})')

            my_id = robot.name

            num_connectors += 1

            for hop in robot.hopCount:
                
                if hop.count > 1:
                    
                    # Connection between two connectors

                    other_id = hop.neighbor
                    key = ''
                    if int(my_id[1:]) < int(other_id[1:]):
                        key += my_id + other_id
                    else:
                        key += other_id + my_id

                    robot_pos = robot_pos_by_time[s.totalTime]

                    if not key in connections:                        

                        dist = math.dist([robot.position.x, robot.position.y], [robot_pos[hop.neighbor].x, robot_pos[hop.neighbor].y])
                        
                        if dist <= COMM_RANGE:
                            connections[key] = {
                                'x_values': [robot.position.x, robot_pos[hop.neighbor].x],
                                'y_values': [robot.position.y, robot_pos[hop.neighbor].y]
                            }
                            G.add_edge(my_id, other_id, length=dist)
                else:
                    
                    # Connection between a tail connector and a team
                    
                    key = my_id + str(hop.teamID)

                    team_pos = team_pos_by_time[s.totalTime]

                    dist = math.dist([robot.position.x, robot.position.y], [team_pos[hop.teamID]['x'], team_pos[hop.teamID]['y']])
                    
                    if dist <= COMM_RANGE:
                        connections[key] = {
                            'x_values': [robot.position.x, team_pos[hop.teamID]['x']],
                            'y_values': [robot.position.y, team_pos[hop.teamID]['y']]
                        }
                        G.add_edge(my_id, hop.teamID, length=dist)
                    else:
                        # If the connector's distance to the team center exceeds the communication range,
                        # Find the follower in that team that is closest to the team center and is within
                        # the connector's communication range.
                        # If that follower is still not in range of the team center, repeat the process.

                        print(f'Connector {my_id} is not close enough ({dist:.2f}) to the team ({hop.teamID}) center...')

                        # current robot
                        # current distance
                        current_robot = deepcopy(robot)
                        prev_robot = deepcopy(robot)
                        current_min_dist = dist
                        isDisconnected = False

                        # while team center not reached by current robot
                        while (current_min_dist != None and current_min_dist > COMM_RANGE) and not isDisconnected:

                            current_min_dist = None # reset

                            # For all robots
                            for other_robot in s.data[s.totalTime]['log'].robots:

                                # If it belongs to the target team
                                if other_robot.teamID == hop.teamID:
                                    dist_between = math.dist([current_robot.position.x, current_robot.position.y], [other_robot.position.x, other_robot.position.y])
                                    
                                    # If the distance from this robot is within the communication range
                                    if dist_between <= COMM_RANGE:
                                        other_dist = math.dist([other_robot.position.x, other_robot.position.y], [team_pos[hop.teamID]['x'], team_pos[hop.teamID]['y']])
                                        
                                        # If distance to team center is smaller than the current smallest distance
                                        if not current_min_dist or other_dist < current_min_dist:
                                            print(f'{other_robot.name} is close to the team center ({other_dist:.2f})')

                                            # Save this robot and distance
                                            current_min_dist = other_dist
                                            current_robot = deepcopy(other_robot)

                            if current_robot.name != prev_robot.name:
                                print(f'Adding {current_robot.name} to the graph')

                                # Add the robot to the graph
                                key = ''
                                if int(current_robot.name[1:]) < int(prev_robot.name[1:]):
                                    key += current_robot.name + prev_robot.name
                                else:
                                    key += prev_robot.name + current_robot.name
                                connections[key] = {
                                    'x_values': [current_robot.position.x, prev_robot.position.x],
                                    'y_values': [current_robot.position.y, prev_robot.position.y]
                                }
                                G.add_edge(current_robot.name, prev_robot.name, length=current_min_dist)

                                prev_robot = current_robot

                            else:
                                print(f'NO ROBOT FOUND TO BE CLOSE TO THE TEAM CENTER! robot: {current_robot.name} @ ({current_robot.position.x:.2f},{current_robot.position.y:.2f}) team: {hop.teamID}')
                                isDisconnected = True
                        
                        if not isDisconnected:
                            # Add the connection between the robot and the team center
                            key = current_robot.name + str(hop.teamID)
                            connections[key] = {
                                'x_values': [current_robot.position.x, team_pos[hop.teamID]['x']],
                                'y_values': [current_robot.position.y, team_pos[hop.teamID]['y']]
                            }
                            G.add_edge(current_robot.name, hop.teamID, length=dist)

    if plot:
        # Draw the center of mass of each team
        for pos in team_pos_by_time[s.totalTime].values():
            ax[0].plot(pos['x'], pos['y'], marker='*', color='orange', markersize=10, zorder=z_team_center)
            label_team = plt.Line2D([], [], marker='*', linestyle='None', color='orange', markersize=10)

        # Draw connections as lines
        for line in connections.values():
            ax[0].plot(line['x_values'], line['y_values'], 'cyan', zorder=z_connection)

        # Plot tasks
        for task in s.data[s.totalTime]['log'].tasks:
            pos = task.position
            circle_task = plt.Circle(xy=(pos.x, pos.y), radius=task.radius, facecolor='whitesmoke', fill=True, linewidth=1, edgecolor='grey', zorder=z_task)
            ax[0].add_patch(circle_task)

        # Reorganize team positions by teams instead of timesteps
        team_trajectory = defaultdict()
        for team in team_pos_by_time[1].keys():
            x_values = []
            y_values = []
            for time, team_pos in team_pos_by_time.items():
                x_values.append(team_pos[team]['x'])
                y_values.append(team_pos[team]['y'])

            team_trajectory[team] = {
                'x': x_values,
                'y': y_values
            }

        # Draw team trajectory
        for team, team_pos in team_trajectory.items():
            ax[0].plot(team_pos['x'], team_pos['y'], linestyle='dashed', zorder=z_team_trajectory)

    # # Run the Steiner tree MSP solver
    # positions = {}
    # for team, pos in team_pos_by_time[s.totalTime].items():
    #     positions[team] = (pos['x'], pos['y'])

    # smt_solver = SMT_MSP(positions, COMM_RANGE)
    # smt_solver.solve(mode='release') # mode = {'release'|'debug'|'plot'}, default is 'release'
    # smt_pos = smt_solver.pos
    
    # # Run the Two tiered relay node placement solver
    # positions = {}
    # for team, pos in team_pos_by_time[s.totalTime].items():
    #     positions[team] = (pos['x'], pos['y'])

    # ttrnp_solver = TwoTieredRNP(positions, COMM_RANGE)
    # ttrnp_solver.solve(mode='release') # mode = {'release'|'debug'|'plot'}, default is 'release'
    # ttrnp_pos = ttrnp_solver.pos

    # Run the Two tiered relay node placement solver
    positions = {}
    for team, pos in team_pos_by_time[s.totalTime].items():
        positions[team] = (pos['x'], pos['y'])

    print(positions)
    print(robot_pos_by_time)

    esmt_solver = ESMT(positions, COMM_RANGE)
    esmt_solver.solve(mode='release') # mode = {'release'|'debug'|'plot'}, default is 'release'
    esmt_pos = esmt_solver.pos

    if plot:
        for node in esmt_solver.G.nodes:
            if str(node)[0] == 'S':
                circle_robot = plt.Circle(xy=(esmt_pos[node][0], esmt_pos[node][1]), radius=ROBOT_RADIUS, facecolor='cyan', fill=True, linewidth=1, edgecolor='k', zorder=z_robot_position)
                ax[1].add_patch(circle_robot)
            else:
                ax[1].plot(esmt_pos[node][0], esmt_pos[node][1], marker='*', color='orange', markersize=10, zorder=z_team_center)

        for edge in esmt_solver.G.edges:
            ax[1].plot([esmt_pos[edge[0]][0], esmt_pos[edge[1]][0]], [esmt_pos[edge[0]][1], esmt_pos[edge[1]][1]], 'cyan', zorder=z_connection)

        ### EXPERIMENT INFO ###
        print('\n------ Experiment result -------')
        print(f'Connectors: {num_connectors}')
        print(f'Network is connected: {nx.is_connected(G)}')
        print(f'Total connection distance: {G.size(weight="length")}')
        print(f'Nodes: {G.nodes}')
        print(f'Edges: {G.edges}')

        print('\n------ ESMT Solver result -------')
        print(f'Connectors: {esmt_solver.added_points}')
        print(f'Network is connected: {nx.is_connected(esmt_solver.G)}')
        print(f'Total connection distance: {esmt_solver.length}')
        print(f'Nodes: {esmt_solver.G.nodes}')
        print(f'Edges: {esmt_solver.G.edges}')

        for i, axis in enumerate(ax):

            # Plot arena 
            circle_arena = plt.Circle(xy=(0, 0), radius=s.arenaRadius, color='black', fill=False, zorder=z_arena)
            axis.add_patch(circle_arena)
        
            if i == 0:
                legend_robot = axis.legend([label_leader, label_follower, label_connector, label_team], ['Leader', 'Follower', 'Connector', 'Team Center'], loc=1)
                axis.set_title(f'Experiment (Connectors = {num_connectors}, Length = {round(G.size(weight="length"), 4)}, Connected = {nx.is_connected(G)})')
            else:
                legend_robot = axis.legend([label_connector, label_team], ['Connector', 'Team Center'], loc=1)
                axis.set_title(f'SMT-MSP Solver (Connectors = {esmt_solver.added_points}, Length = {round(esmt_solver.length, 4)}, Connected = {nx.is_connected(esmt_solver.G)})')

            axis.add_artist(legend_robot)

            # Fit to arena size
            plt.setp(axis, xlim=[-s.arenaRadius,s.arenaRadius])
            plt.setp(axis, ylim=[-s.arenaRadius,s.arenaRadius])
            axis.set_aspect('equal')
            axis.set_xlabel('X (m)', fontproperties=font)
            axis.set_ylabel('Y (m)', fontproperties=font)
            for label in axis.get_xticklabels():
                label.set_fontproperties(font)
            for label in axis.get_yticklabels():
                label.set_fontproperties(font)

        plt.show()

    return {'res': G, 'esmt': esmt_solver.G}, \
           {'res': G.size(weight="length"), 'esmt': esmt_solver.length}, \
           {'res': num_connectors, 'esmt': esmt_solver.added_points} 


def get_team_distance(G):
    """Calculate the sum of the distances between every team"""
    team_nodes = [n for n in G.nodes if str(n)[0] != 'S' and type(n) != str]
    pairs = combinations(team_nodes, 2)

    dist = 0
    if nx.is_connected(G):
        for v1, v2 in pairs:
            dist += nx.shortest_path_length(G, v1, v2, 'length')
    else:
        print('not connected')
    return dist


def get_team_distances(G):
    """Calculate the min, max and mean distances between teams along the network as well as the difference between the min and max"""
    team_nodes = [n for n in G.nodes if str(n)[0] != 'S' and type(n) != str]
    pairs = combinations(team_nodes, 2)

    min = max = None
    sum = mean = 0
    count = 0

    if nx.is_connected(G):
        for v1, v2 in pairs:
            dist = nx.shortest_path_length(G, v1, v2, 'length')

            if not min or dist < min:
                min = dist
            
            if not max or dist > max:
                max = dist

            sum += dist
            count += 1
    else:
        print('not connected')
        return {}
    
    if count > 0:
        mean = sum / count

    return {'min': min, 'max': max, 'mean': mean, 'diff': max - min}


def plot_all_trials():
    
    df = pd.DataFrame({
        'Teams':                        pd.Series(dtype='int'), 
        'Workers Per Team':             pd.Series(dtype='int'),
        'Network Length':               pd.Series(dtype='float'),
        'Network Length (ESMT)':        pd.Series(dtype='float'),
        'Connectors':                   pd.Series(dtype='int'),
        'Connectors (ESMT)':            pd.Series(dtype='int'),
        'Average Path Length':          pd.Series(dtype='float'),
        'Average Path Length (ESMT)':   pd.Series(dtype='float'),          
    })
    
    # Get trial names
    team_num_dirs = [f for f in listdir(RESULTS_DIR) if isdir(join(RESULTS_DIR, f))]
    team_num_dirs.sort()

    for team_num in team_num_dirs:

        trial_dirs = [f for f in listdir(join(RESULTS_DIR, team_num)) if isdir(join(RESULTS_DIR, team_num, f))]
        trial_dirs.sort()

        trial_no_points = {}
        trial_no_connectivity = {}
        trial_no_team_connectivity = {}

        start_time = time.time()

        count = 0
        # for scenario in trial_dirs:
        while count < len(trial_dirs):
            # if count >= len(trial_dirs):
            #     break

            scenario = trial_dirs[count]

            start_time_single = time.time()

            # Check if the trial was successful
            s, final_points, final_connectivity, final_team_connectivity = load_log_with_checks(join(RESULTS_DIR, team_num, scenario))
            if final_points == 0:
                trial_no_points[scenario] = {
                    'seed': s.seed,
                    'data': final_points
                }

            if final_team_connectivity:
                trial_no_team_connectivity[scenario] = {
                    'seed': s.seed,
                    'data': final_team_connectivity
                }

            # if final_points > 0 and final_connectivity:

            # Find the total network length and connectors for the result obtained and the solver output
            graphs, lengths, connectors = get_network_length_and_connectors(s, plot=False)
            if not nx.is_connected(graphs['res']):
                trial_no_connectivity[scenario] = {
                    'seed': s.seed,
                    'data': False
                }

            # Find the total distance between every team along the network
            # get_team_distance(graphs['res'])

            res_path_length = get_team_distances(graphs["res"])
            esmt_path_length = get_team_distances(graphs["esmt"])

            # Add data of the successful trial
            if not scenario in trial_no_points and not scenario in trial_no_team_connectivity and not scenario in trial_no_connectivity:

                d = pd.DataFrame({
                    'Teams': [s.numLeaders], 
                    'Workers Per Team': [(int)(s.numWorkers/s.numLeaders)], 
                    'Network Length': [lengths['res']], 
                    'Network Length (ESMT)': [lengths['esmt']],
                    'Connectors': [connectors['res']],
                    'Connectors (ESMT)': [connectors['esmt']],
                    'Average Path Length': [res_path_length['mean']],
                    'Average Path Length (ESMT)': [esmt_path_length['mean']],
                })
                df = pd.concat([df, d], ignore_index=True, axis=0)

            duration_single = round(time.time() - start_time_single, 3)
            duration_total = round(time.time() - start_time, 3)
            print("Loaded -- '{0}' --\tin {1} s ({2} s)".format(scenario, duration_single, duration_total))

            # DEBUG: For limiting the number of data to plot
            count += 1
            num_to_use = 50
            if (count % 10) % num_to_use == 0:
                count += 50 - num_to_use
                # print(count)
                # break

        duration = round(time.time() - start_time, 3)
        print('Finished in {0} seconds'.format(duration))

        print(f'### No points scored ###')
        for scenario, trial in trial_no_points.items():
            print(f'      scenario: {scenario}, seed: {trial["seed"]}, points: {trial["data"]}')
        print(f'    Trials with no points scored: ({len(trial_no_points)}/{len(trial_dirs)})')

        print(f'### Lost global connectivity ###')
        for scenario, trial in trial_no_connectivity.items():
            print(f'      scenario: {scenario}, seed: {trial["seed"]}, connectivity: {trial["data"]}')
        print(f'    Trials that lost global connectivity: ({len(trial_no_connectivity)}/{len(trial_dirs)})')

        print(f'### Lost team connectivity ###')
        for scenario, trial in trial_no_team_connectivity.items():
            print(f'      scenario: {scenario}, seed: {trial["seed"]}, followers lost: {len(trial["data"])}')
        print(f'    Trials that lost team connectivity: ({len(trial_no_team_connectivity)}/{len(trial_dirs)})')


    # Plot data
    fig1 = plt.figure(figsize=(6, 5))
    axes1 = fig1.gca()
    fig2 = plt.figure(figsize=(6, 5))
    axes2 = fig2.gca()
    fig3 = plt.figure(figsize=(6, 5))
    axes3 = fig3.gca()
    fig4 = plt.figure(figsize=(6, 5))
    axes4 = fig4.gca()
    fig5 = plt.figure(figsize=(6, 5))
    axes5 = fig5.gca()
    fig6 = plt.figure(figsize=(6, 5))
    axes6 = fig6.gca()
    # fig7 = plt.figure(figsize=(9, 5))
    # axes7 = fig7.gca()
    # fig8 = plt.figure(figsize=(9, 5))
    # axes8 = fig8.gca()

    fig9, axd9 = plt.subplot_mosaic([['ul', 'r'],
                                   ['ml', 'r'],
                                   ['bl', 'r']],
                                   figsize=(9,12),
                                   gridspec_kw={'width_ratios': [30, 1]})

    axes = [axes1, axes2, axes3, axes4, axes5, axes6]

    order_num_team = list(df['Teams'].unique())
    order_num_team.sort()
    order_team_size = list(df['Workers Per Team'].unique())
    order_team_size.sort()

    ### LINE PLOT

    # Get subset of dataframe
    df_connectors = df[['Teams', 'Workers Per Team', 'Connectors', 'Connectors (ESMT)']]
    df_connectors = df_connectors.rename(columns={'Connectors': 'Trials', 'Connectors (ESMT)': 'Optimal Trees'})
    df_connectors = df_connectors.melt(id_vars=['Teams', 'Workers Per Team'])#.sort_values(['Workers Per Team', 'Teams', 'variable'], ascending = [True, True, True])
    df_connectors.rename(columns={'variable': 'Type'}, inplace=True)
    # print(df_connectors)
    df_network_length = df[['Teams', 'Workers Per Team', 'Network Length', 'Network Length (ESMT)']]
    df_network_length = df_network_length.rename(columns={'Network Length': 'Trials', 'Network Length (ESMT)': 'Optimal Trees'})
    df_network_length = df_network_length.melt(id_vars=['Teams', 'Workers Per Team'])#.sort_values(['Workers Per Team', 'Teams', 'variable'], ascending = [True, True, True])
    df_network_length.rename(columns={'variable': 'Type'}, inplace=True)
    # print(df_network_length)
    df_team_length = df[['Teams', 'Workers Per Team', 'Average Path Length', 'Average Path Length (ESMT)']]
    df_team_length = df_team_length.rename(columns={'Average Path Length': 'Trials', 'Average Path Length (ESMT)': 'Optimal Trees'})
    df_team_length = df_team_length.melt(id_vars=['Teams', 'Workers Per Team'])#.sort_values(['Workers Per Team', 'Teams', 'variable'], ascending = [True, True, True])
    df_team_length.rename(columns={'variable': 'Type'}, inplace=True)
    # print(df_team_length)

    # Build hue label
    # hue1 = df_network_length[['Workers Per Team', 'Type']].apply(lambda row: f"{row['Workers Per Team']}, {row['Type']}", axis=1)
    # hue2 = df_network_length[['Teams', 'Type']].apply(lambda row: f"{row['Teams']}, {row['Type']}", axis=1)

    sns.lineplot(
        data=df_connectors, ax=axes1, 
        x='Teams', y='value', 
        hue='Workers Per Team',
        err_style='bars', style='Type', dashes=True, palette=['blue','green','orange','red']
    )

    sns.lineplot(
        data=df_connectors, ax=axes2, 
        x='Workers Per Team', y='value', 
        hue='Teams',
        err_style='bars', style='Type', dashes=True, palette=['purple','blue','green','orange','red']
    )

    sns.lineplot(
        data=df_network_length, ax=axes3, 
        x='Teams', y='value', 
        hue='Workers Per Team',
        err_style='bars', style='Type', dashes=True, palette=['blue','green','orange','red']
    )

    sns.lineplot(
        data=df_network_length, ax=axes4, 
        x='Workers Per Team', y='value', 
        hue='Teams',
        err_style='bars', style='Type', dashes=True, palette=['purple','blue','green','orange','red']
    )

    sns.lineplot(
        data=df_team_length, ax=axes5, 
        x='Teams', y='value', 
        hue='Workers Per Team',
        err_style='bars', style='Type', dashes=True, palette=['blue','green','orange','red']
    )

    sns.lineplot(
        data=df_team_length, ax=axes6, 
        x='Workers Per Team', y='value', 
        hue='Teams',
        err_style='bars', style='Type', dashes=True, palette=['purple','blue','green','orange','red']
    )

    ### HEAT MAP

    mean_connectors = df_connectors.groupby(['Teams', 'Workers Per Team', 'Type']).mean().reset_index()
    # print(mean_connectors)
    mean_network_length = df_network_length.groupby(['Teams', 'Workers Per Team', 'Type']).mean().reset_index()
    # print(mean_network_length)
    mean_team_length = df_team_length.groupby(['Teams', 'Workers Per Team', 'Type']).mean().reset_index()

    def calculate_ratio(group, type_col, type1, type2, val_col):
        ratio = group.loc[group[type_col] == type1, val_col].iloc[0] / group.loc[group[type_col] == type2, val_col].iloc[0]
        return pd.Series({'ratio': ratio})

    # group DataFrame by id1 and id2 and apply function to calculate ratio
    ratio_df_connectors = mean_connectors.groupby(['Teams', 'Workers Per Team']).apply(calculate_ratio, type_col='Type', type1='Trials', type2='Optimal Trees', val_col='value')
    print(ratio_df_connectors)
    ratio_df_network_length = mean_network_length.groupby(['Teams', 'Workers Per Team']).apply(calculate_ratio, type_col='Type', type1='Trials', type2='Optimal Trees', val_col='value')
    print(ratio_df_network_length)
    ratio_df_team_length = mean_team_length.groupby(['Teams', 'Workers Per Team']).apply(calculate_ratio, type_col='Type', type1='Trials', type2='Optimal Trees', val_col='value')
    print(ratio_df_team_length)

    pivot_table_connectors = ratio_df_connectors.pivot_table(index='Workers Per Team', columns='Teams', values='ratio')
    pivot_table_network_length = ratio_df_network_length.pivot_table(index='Workers Per Team', columns='Teams', values='ratio')
    pivot_table_team_length = ratio_df_team_length.pivot_table(index='Workers Per Team', columns='Teams', values='ratio')

    # create a list of the desired order of the index values
    index_order = df.sort_values('Workers Per Team', ascending=False)['Workers Per Team'].unique().tolist()

    # reindex the pivot table using the desired order of the index values
    pivot_table_connectors = pivot_table_connectors.reindex(index_order)
    pivot_table_network_length = pivot_table_network_length.reindex(index_order)
    pivot_table_team_length = pivot_table_team_length.reindex(index_order)

    # plot the pivot table as a heatmap using seaborn
    max_disp = max((1 - ratio_df_connectors['ratio']).abs().max(), (1 - ratio_df_network_length['ratio']).abs().max(), (1 - ratio_df_team_length['ratio']).abs().max())
    # sns.heatmap(pivot_table_network_length, ax=axes7, annot=True, fmt='.2f', cmap='vlag', center=1, vmin=1-max_disp, vmax=1+max_disp, cbar_kws={'label': 'Ratio'})
    # sns.heatmap(pivot_table_connectors, ax=axes8, annot=True, fmt='.2f', cmap='vlag', center=1, vmin=1-max_disp, vmax=1+max_disp, cbar_kws={'label': 'Ratio'})

    sns.heatmap(pivot_table_connectors, ax=axd9['ml'], annot=True, fmt='.2f', cmap='vlag', center=1, vmin=1-max_disp, vmax=1+max_disp,
                cbar=True, cbar_ax=axd9['r'])

    sns.heatmap(pivot_table_network_length, ax=axd9['ul'], annot=True, fmt='.2f', cmap='vlag', center=1, vmin=1-max_disp, vmax=1+max_disp,
                cbar=False, cbar_ax=None)
    
    sns.heatmap(pivot_table_team_length, ax=axd9['bl'], annot=True, fmt='.2f', cmap='vlag', center=1, vmin=1-max_disp, vmax=1+max_disp,
                cbar=False, cbar_ax=None)

    # Font
    font = FontProperties()
    # font.set_family('serif')
    # font.set_name('Times New Roman')
    font.set_size(16)

    font2 = FontProperties()
    # font2.set_family('serif')
    # font2.set_name('Times New Roman')
    font2.set_size(14)

    # Set axis labels
    # axes.set_xlabel("Average waiting time per task (s)")
    axes1.set_xlabel("Teams", fontproperties=font)
    axes1.set_ylabel("Connectors", fontproperties=font)
    axes2.set_xlabel("Workers Per Team", fontproperties=font)
    axes2.set_ylabel("Connectors", fontproperties=font)
    axes3.set_xlabel("Teams", fontproperties=font)
    axes3.set_ylabel("Network Length (m)", fontproperties=font)
    axes4.set_xlabel("Workers Per Team", fontproperties=font)
    axes4.set_ylabel("Network Length (m)", fontproperties=font)
    axes5.set_xlabel("Teams", fontproperties=font)
    axes5.set_ylabel("Average Path Length (m)", fontproperties=font)
    axes6.set_xlabel("Workers Per Team", fontproperties=font)
    axes6.set_ylabel("Average Path Length (m)", fontproperties=font)
    # axes7.set_xlabel("Teams", fontproperties=font)
    # axes7.set_ylabel("Workers Per Team", fontproperties=font)
    # axes8.set_xlabel("Teams", fontproperties=font)
    # axes8.set_ylabel("Workers Per Team", fontproperties=font)

    for key, ax in axd9.items():
        if key != 'r':
            ax.set_xlabel("Teams", fontproperties=font)
            ax.set_ylabel("Workers Per Team", fontproperties=font)

        for label in ax.get_xticklabels():
            label.set_fontproperties(font2)
        for label in ax.get_yticklabels():
            label.set_fontproperties(font2)

    for ax in axes:
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.spines['left'].set_visible(True)
        ax.spines['bottom'].set_visible(True)
        ax.spines['left'].set_linewidth(1)
        ax.spines['bottom'].set_linewidth(1)
        ax.tick_params(width=1)

        for label in ax.get_xticklabels():
            label.set_fontproperties(font2)
        for label in ax.get_yticklabels():
            label.set_fontproperties(font2)

    # border and ticks
    ticks = np.arange(2, 6+1, 1)
    axes1.set_xticks(ticks)
    axes1.set_ylim([-1,df_connectors['value'].max()+1])

    ticks = np.arange(6, 24+1, 6)
    axes2.set_xticks(ticks)
    axes2.set_ylim([-1,df_connectors['value'].max()+1])

    ticks = np.arange(2, 6+1, 1)
    axes3.set_xticks(ticks)
    axes3.set_ylim([-1,df_network_length['value'].max()+1])

    ticks = np.arange(6, 24+1, 6)
    axes4.set_xticks(ticks)
    axes4.set_ylim([-1,df_network_length['value'].max()+1])

    ticks = np.arange(2, 6+1, 1)
    axes5.set_xticks(ticks)
    axes5.set_ylim([-1,df_team_length['value'].max()+1])

    ticks = np.arange(6, 24+1, 6)
    axes6.set_xticks(ticks)
    axes6.set_ylim([-1,df_team_length['value'].max()+1])

    # legend
    # legend_labels1 = ['w = 6', 'w = 6 (ESMT)', 'w = 12', 'w = 12 (ESMT)', 'w = 18', 'w = 18 (ESMT)', 'w = 24', 'w = 24 (ESMT)']
    # legend_labels2 = ['t = 2', 't = 2 (ESMT)', 't = 3', 't = 3 (ESMT)', 't = 4', 't = 4 (ESMT)', 't = 5', 't = 5 (ESMT)', 't = 6', 't = 6 (ESMT)']

    # axes1.legend(legend_labels1, bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)
    # axes2.legend(legend_labels2, bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)
    # axes3.legend(legend_labels1, bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)
    # axes4.legend(legend_labels2, bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)
    # axes5.legend(legend_labels1, bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)
    # axes6.legend(legend_labels2, bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)

    # Adjust legends
    for ax in axes:
        leg = ax.legend(loc='upper left')
        for text_obj in leg.get_texts():
            text = text_obj.get_text()
            if text == 'Workers Per Team' or text == 'Teams' or text == 'Type':
                text_obj.set_fontweight('bold')
                text_obj.set_position((-20,0))

    # set font properties of colorbar label
    # cbar7 = axes7.collections[0].colorbar
    # cbar7.set_label("Ratio", fontdict={'size': 14})
    # cbar7.ax.tick_params(labelsize=14)
    # cbar8 = axes8.collections[0].colorbar
    # cbar8.set_label("Ratio", fontdict={'size': 14})
    # cbar8.ax.tick_params(labelsize=14)

    # cbar8 = axd9['r'].collections[0].colorbar
    axd9['r'].set_ylabel("Ratio", fontdict={'size': 14})
    axd9['r'].tick_params(labelsize=14)

    for key, ax in axd9.items():
        if key == 'r':
            ax.spines['top'].set_visible(True)
            ax.spines['right'].set_visible(True)
        else:
            ax.spines['top'].set_visible(False)
            ax.spines['right'].set_visible(False)
        ax.spines['left'].set_visible(True)
        ax.spines['bottom'].set_visible(True)
        ax.spines['left'].set_linewidth(1)
        ax.spines['bottom'].set_linewidth(1)
        ax.tick_params(width=1)

    axd9['r'].set_ylabel('Ratio')

    fig1.tight_layout()
    fig2.tight_layout()
    fig3.tight_layout()
    fig4.tight_layout()
    fig5.tight_layout()
    fig6.tight_layout()
    # fig7.tight_layout()
    # fig8.tight_layout()

    fig9.tight_layout()
    fig9.subplots_adjust(wspace=0.1)

    # plt.show()
    fig1.savefig(join(RESULTS_DIR, 'connectors(num_teams).pdf'))
    fig2.savefig(join(RESULTS_DIR, 'connectors(num_robots).pdf'))
    fig3.savefig(join(RESULTS_DIR, 'network_length(num_teams).pdf'))
    fig4.savefig(join(RESULTS_DIR, 'network_length(num_robots).pdf'))
    fig5.savefig(join(RESULTS_DIR, 'average_path_length(num_teams).pdf'))
    fig6.savefig(join(RESULTS_DIR, 'average_path_length(num_robots).pdf'))               
    # fig7.savefig(join(RESULTS_DIR, 'heatmap_network_length.pdf'))               
    # fig8.savefig(join(RESULTS_DIR, 'heatmap_connectors.pdf'))    
    fig9.savefig(join(RESULTS_DIR, 'heatmap.pdf'))    


def get_network_length_and_connectors_at_time(s, time=0, plot=False):
    """Get the network length of produced by the robots, the number of connectors and the solver result at the specified timestep."""

    if plot:
        # Font
        font = FontProperties()
        # font.set_family('serif')
        # font.set_name('Times New Roman')
        font.set_size(18)

        fig, ax = plt.subplots(nrows=1, ncols=2, figsize=(20, 6))

        # z-order
        z_arena = 0
        z_task = 1
        z_team_trajectory = 2
        z_connection = 3
        z_team_center = 4
        z_robot_position = 5
        z_robot_orientation = 6

    # sns.scatterplot(data=df, ax = ax, x="PositionX", y="PositionY", hue="State", zorder=2)

    # Store position of robots and sort followers by team
    robot_pos_by_time = {}
    team_pos_by_time = {}

    robot_pos = {}
    leader_follower_pos = defaultdict(list)
    team_pos = {}

    # Store robot positions
    for robot in s.data[time]['log'].robots:
        ### Simulation
        robot_pos[robot.name] = robot.position
        # if robot.teamID != 255 and robot.teamID != 100000: # Ignore connectors and travellers
        #     leader_follower_pos[robot.teamID].append(robot.position)

        ### Swarmhack fix y-axis
        robot_pos[robot.name].x -= SWARMHACK_X
        robot_pos[robot.name].y = -robot_pos[robot.name].y + SWARMHACK_FIXY

        if robot.teamID != 255 and robot.teamID != 100000: # Ignore connectors and travellers, Ignore swarmhack template team
            leader_follower_pos[robot.teamID].append(robot.position)

    robot_pos_by_time[time] = robot_pos

    # Store team positions
    for team, positions in leader_follower_pos.items():
        num_robots = len(positions)

        x = y = 0
        for pos in positions:
            x += pos.x
            y += pos.y

        team_pos[team] = {
            'x': x / num_robots,
            'y': y / num_robots
        }

    team_pos_by_time[time] = team_pos
    
    num_connectors = 0
    connections = {}

    label_leader = label_follower = label_connector = label_traveler = None

    # init Networkx graph
    G = nx.Graph()
    for team in team_pos_by_time[time].keys():
        # if team != 100000: # Ignore template 100000 connection, which means null
        G.add_node(team)

    # Plot robots
    for robot in s.data[time]['log'].robots:

        pos = robot.position
        orientation = robot.orientation

        if plot:
            color = 'white'

            if robot.state == time_step_pb2.Robot.CONNECTOR:
                color = 'cyan'
            elif robot.state == time_step_pb2.Robot.FOLLOWER:
                color = 'limegreen'
            elif robot.state == time_step_pb2.Robot.LEADER:
                color = 'red'
            elif robot.state == time_step_pb2.Robot.TRAVELER:
                color = 'magenta'

            # Draw position
            circle_robot = plt.Circle(xy=(pos.x, pos.y), radius=ROBOT_RADIUS, facecolor=color, fill=True, linewidth=0.75, edgecolor='k', zorder=z_robot_position)
            
            if robot.state == time_step_pb2.Robot.CONNECTOR:
                label_connector = plt.Line2D([], [], marker='o', linestyle='None', color='k', markerfacecolor=color, markersize=10)
            elif robot.state == time_step_pb2.Robot.FOLLOWER:
                label_follower = plt.Line2D([], [], marker='o', linestyle='None', color='k', markerfacecolor=color, markersize=10)
            elif robot.state == time_step_pb2.Robot.LEADER:
                label_leader = plt.Line2D([], [], marker='o', linestyle='None', color='k', markerfacecolor=color, markersize=10)
            elif robot.state == time_step_pb2.Robot.TRAVELER:
                label_traveler = plt.Line2D([], [], marker='o', linestyle='None', color='k', markerfacecolor=color, markersize=10)

            ax[0].add_patch(circle_robot)

            # Draw orientation
            rad = quaternion_to_euler_angle_vectorized1(orientation.w, orientation.x, orientation.y, orientation.z)[2]
            # print(f'robot: {robot.name}, rot: {rad}')
            x_values = [pos.x, pos.x + HEADING_LENGTH * math.cos(rad)]
            y_values = [pos.y, pos.y + HEADING_LENGTH * math.sin(rad)]
            ax[0].plot(x_values, y_values, 'k', zorder=z_robot_orientation, linewidth=1)

        # Save the connections
        if robot.state == time_step_pb2.Robot.CONNECTOR:
            # print(f'--- {robot.name} --- pos=({robot.position.x}, {robot.position.y})')

            my_id = robot.name

            num_connectors += 1

            for hop in robot.hopCount:
                
                tmp_hop = deepcopy(hop)

                if tmp_hop.count > 1:
                    
                    # Connection between two connectors

                    other_id = tmp_hop.neighbor
                    key = ''
                    if int(my_id[1:]) < int(other_id[1:]):
                        key += my_id + other_id
                    else:
                        key += other_id + my_id

                    robot_pos = robot_pos_by_time[time]

                    if not key in connections:                        

                        # Check if whether it is still in the process of changing connection
                        usePrevHop = False

                        if tmp_hop.count == 2 or robot.prevHops:
                            for prevHop in robot.prevHops:
                                # print(f'prevHop: team: {prevHop.teamID}, id: {prevHop.neighbor}, count: {prevHop.count}')
                                for other_robot in s.data[time]['log'].robots:
                                    if other_robot.name == other_id and other_robot.state != time_step_pb2.Robot.CONNECTOR:
                                        print(f'Connection does not exist! reverting back to {prevHop.neighbor}')
                                        tmp_hop = deepcopy(prevHop)
                                        usePrevHop = True

                        if not usePrevHop:
                            dist = math.dist([robot.position.x, robot.position.y], [robot_pos[tmp_hop.neighbor].x, robot_pos[tmp_hop.neighbor].y])
                            
                            if dist <= COMM_RANGE:
                                connections[key] = {
                                    'x_values': [robot.position.x, robot_pos[tmp_hop.neighbor].x],
                                    'y_values': [robot.position.y, robot_pos[tmp_hop.neighbor].y]
                                }
                                G.add_edge(my_id, other_id, length=dist)

                if tmp_hop.count == 1:
                    
                    # Connection between a tail connector and a team
                    
                    key = my_id + str(tmp_hop.teamID)

                    team_pos = team_pos_by_time[time]

                    dist = math.dist([robot.position.x, robot.position.y], [team_pos[tmp_hop.teamID]['x'], team_pos[tmp_hop.teamID]['y']])
                    
                    if dist <= COMM_RANGE:
                        connections[key] = {
                            'x_values': [robot.position.x, team_pos[tmp_hop.teamID]['x']],
                            'y_values': [robot.position.y, team_pos[tmp_hop.teamID]['y']]
                        }
                        G.add_edge(my_id, tmp_hop.teamID, length=dist)
                    else:
                        # If the connector's distance to the team center exceeds the communication range,
                        # Find the follower in that team that is closest to the team center and is within
                        # the connector's communication range.
                        # If that follower is still not in range of the team center, repeat the process.

                        print(f'Connector {my_id} is not close enough ({dist:.2f}) to the team ({tmp_hop.teamID}) center...')

                        # current robot
                        # current distance
                        current_robot = deepcopy(robot)
                        prev_robot = deepcopy(robot)
                        current_min_dist = dist
                        isDisconnected = False

                        # while team center not reached by current robot
                        while (current_min_dist != None and current_min_dist > COMM_RANGE) and not isDisconnected:

                            current_min_dist = None # reset

                            # For all robots
                            for other_robot in s.data[time]['log'].robots:

                                # If it belongs to the target team
                                if other_robot.teamID == tmp_hop.teamID:
                                    dist_between = math.dist([current_robot.position.x, current_robot.position.y], [other_robot.position.x, other_robot.position.y])
                                    
                                    # If the distance from this robot is within the communication range
                                    if dist_between <= COMM_RANGE:
                                        other_dist = math.dist([other_robot.position.x, other_robot.position.y], [team_pos[tmp_hop.teamID]['x'], team_pos[tmp_hop.teamID]['y']])
                                        
                                        # If distance to team center is smaller than the current smallest distance
                                        if not current_min_dist or other_dist < current_min_dist:
                                            print(f'{other_robot.name} is close to the team center ({other_dist:.2f})')

                                            # Save this robot and distance
                                            current_min_dist = other_dist
                                            current_robot = deepcopy(other_robot)

                            if current_robot.name != prev_robot.name:
                                print(f'Adding {current_robot.name} to the graph')

                                # Add the robot to the graph
                                key = ''
                                if int(current_robot.name[1:]) < int(prev_robot.name[1:]):
                                    key += current_robot.name + prev_robot.name
                                else:
                                    key += prev_robot.name + current_robot.name
                                connections[key] = {
                                    'x_values': [current_robot.position.x, prev_robot.position.x],
                                    'y_values': [current_robot.position.y, prev_robot.position.y]
                                }
                                G.add_edge(current_robot.name, prev_robot.name, length=current_min_dist)

                                prev_robot = current_robot

                            else:
                                print(f'NO ROBOT FOUND TO BE CLOSE TO THE TEAM CENTER! robot: {current_robot.name} @ ({current_robot.position.x:.2f},{current_robot.position.y:.2f}) team: {hop.teamID}')
                                isDisconnected = True
                        
                        if not isDisconnected:
                            # Add the connection between the robot and the team center
                            key = current_robot.name + str(tmp_hop.teamID)
                            connections[key] = {
                                'x_values': [current_robot.position.x, team_pos[tmp_hop.teamID]['x']],
                                'y_values': [current_robot.position.y, team_pos[tmp_hop.teamID]['y']]
                            }
                            G.add_edge(current_robot.name, tmp_hop.teamID, length=dist)

    if plot:
        # Draw the center of mass of each team
        for pos in team_pos_by_time[time].values():
            ax[0].plot(pos['x'], pos['y'], marker='*', color='orange', markersize=20, zorder=z_team_center)
            label_team = plt.Line2D([], [], marker='*', linestyle='None', color='orange', markersize=10)

        # Draw connections as lines
        for line in connections.values():
            ax[0].plot(line['x_values'], line['y_values'], 'cyan', zorder=z_connection)

        # Plot tasks
        for task in s.data[time]['log'].tasks:
            ### Simulation
            pos = task.position
            ### Swarmhack
            pos.x -= SWARMHACK_X
            pos.y = -pos.y + SWARMHACK_FIXY
            circle_task = plt.Circle(xy=(pos.x, pos.y), radius=task.radius, facecolor='whitesmoke', fill=True, linewidth=1, edgecolor='grey', zorder=z_task)
            ax[0].add_patch(circle_task)

        # Reorganize team positions by teams instead of timesteps
        team_trajectory = defaultdict()
        for team in team_pos_by_time[time].keys():
            x_values = []
            y_values = []
            for time, team_pos in team_pos_by_time.items():
                x_values.append(team_pos[team]['x'])
                y_values.append(team_pos[team]['y'])

            team_trajectory[team] = {
                'x': x_values,
                'y': y_values
            }

        # Draw team trajectory
        for team, team_pos in team_trajectory.items():
            ax[0].plot(team_pos['x'], team_pos['y'], linestyle='dashed', zorder=z_team_trajectory)

    ### Solvers

    # # 1) Run the SMT_MSP solver
    # positions = {}
    # for team, pos in team_pos_by_time[time].items():
    #     # if team != 100000: # Ignore template 100000 connection, which means null
    #     positions[team] = (pos['x'], pos['y'])

    # # print(f'positions {positions}')

    # smt_solver = SMT_MSP(positions, COMM_RANGE)
    # smt_solver.solve(mode='release') # mode = {'release'|'debug'|'plot'}, default is 'release'
    # smt_pos = smt_solver.pos
    
    # # 2) Run the TwoTieredRNP solver
    # positions = {}
    # for team, pos in team_pos_by_time[time].items():
    #     # if team != 100000: # Ignore template 100000 connection, which means null
    #     positions[team] = (pos['x'], pos['y'])

    # # print(f'positions {positions}')

    # ttrnp_solver = TwoTieredRNP(positions, COMM_RANGE)
    # ttrnp_solver.solve(mode='release') # mode = {'release'|'debug'|'plot'}, default is 'release'
    # ttrnp_pos = ttrnp_solver.pos

    # Run the ESMT solver
    positions = {}
    for team, pos in team_pos_by_time[time].items():
        # if team != 100000: # Ignore template 100000 connection, which means null
        positions[team] = (pos['x'], pos['y'])

    print(f'positions {positions}')
    print(robot_pos_by_time)

    esmt_solver = ESMT(positions, COMM_RANGE)
    esmt_solver.solve(mode='release') # mode = {'release'|'debug'|'plot'}, default is 'release'
    esmt_pos = esmt_solver.pos

    if plot:
        # SMT
        # for node in smt_solver.G.nodes:
        #     if str(node)[0] == 'S':
        #         circle_robot = plt.Circle(xy=(smt_pos[node][0], smt_pos[node][1]), radius=ROBOT_RADIUS, facecolor='cyan', fill=True, linewidth=0.75, edgecolor='k', zorder=z_robot_position)
        #         ax[1].add_patch(circle_robot)
        #     else:
        #         ax[1].plot(smt_pos[node][0], smt_pos[node][1], marker='*', color='orange', markersize=10, zorder=z_team_center)

        # for edge in smt_solver.G.edges:
        #     ax[1].plot([smt_pos[edge[0]][0], smt_pos[edge[1]][0]], [smt_pos[edge[0]][1], smt_pos[edge[1]][1]], 'cyan', zorder=z_connection)

        # ESMT
        for node in esmt_solver.G.nodes:
            if str(node)[0] == 'S':
                overlap = False
                for other_node in esmt_solver.G.nodes:
                    if str(other_node)[0] != 'S' and esmt_pos[node] == esmt_pos[other_node]:
                        overlap = True
                        
                if overlap:
                    circle_robot = plt.Circle(xy=(esmt_pos[node][0], esmt_pos[node][1]), radius=ROBOT_RADIUS, facecolor='cyan', fill=True, alpha=0.5, linewidth=0.75, edgecolor='k', zorder=z_robot_position)
                else:
                    circle_robot = plt.Circle(xy=(esmt_pos[node][0], esmt_pos[node][1]), radius=ROBOT_RADIUS, facecolor='cyan', fill=True, linewidth=0.75, edgecolor='k', zorder=z_robot_position)
                ax[1].add_patch(circle_robot)
            else:
                ax[1].plot(esmt_pos[node][0], esmt_pos[node][1], marker='*', color='orange', markersize=20, zorder=z_team_center)
                # if esmt_solver.G.degree[node] > 1:
                #     circle_robot = plt.Circle(xy=(esmt_pos[node][0], esmt_pos[node][1]), radius=ROBOT_RADIUS, facecolor='cyan', fill=True, alpha=0.3, linewidth=0.75, edgecolor='k', zorder=z_robot_position)
                #     ax[1].add_patch(circle_robot)

        for edge in esmt_solver.G.edges:
            ax[1].plot([esmt_pos[edge[0]][0], esmt_pos[edge[1]][0]], [esmt_pos[edge[0]][1], esmt_pos[edge[1]][1]], 'cyan', zorder=z_connection)

        ### EXPERIMENT INFO ###
        print('\n------ Experiment result -------')
        print(f'Connectors: {num_connectors}')
        print(f'Network is connected: {nx.is_connected(G)}')
        print(f'Total connection distance: {G.size(weight="length")}')
        print(f'Nodes: {G.nodes}')
        print(f'Edges: {G.edges}')

        # print('\n------ SMT-MSP Solver result -------')
        # print(f'Connectors: {smt_solver.added_points}')
        # print(f'Network is connected: {nx.is_connected(smt_solver.G)}')
        # print(f'Total connection distance: {smt_solver.length}')
        # print(f'Nodes: {smt_solver.G.nodes}')
        # print(f'Edges: {smt_solver.G.edges}')

        # print('\n------ TwoTieredRNP Solver result -------')
        # print(f'Connectors: {ttrnp_solver.added_points}')
        # print(f'Network is connected: {nx.is_connected(ttrnp_solver.G)}')
        # print(f'Total connection distance: {ttrnp_solver.length}')
        # print(f'Nodes: {ttrnp_solver.G.nodes}')
        # print(f'Edges: {ttrnp_solver.G.edges}')

        print('\n------ ESMT Solver result -------')
        print(f'Connectors: {esmt_solver.added_points}')
        print(f'Network is connected: {nx.is_connected(esmt_solver.G)}')
        print(f'Total connection distance: {esmt_solver.length}')
        print(f'Nodes: {esmt_solver.G.nodes}')
        print(f'Edges: {esmt_solver.G.edges}')

        for i, axis in enumerate(ax):

            # Plot arena 
            circle_arena = plt.Circle(xy=(0, 0), radius=s.arenaRadius, color='black', fill=False, zorder=z_arena)
            axis.add_patch(circle_arena)
        
            if i == 0:
                ### With traveler
                legend_robot = axis.legend([label_leader, label_follower, label_connector, label_traveler, label_team], ['Leader', 'Follower', 'Connector', 'Traveler', 'Team Center'], loc=1, fontsize='16')
                ### No traveler
                # legend_robot = axis.legend([label_leader, label_follower, label_connector, label_team], ['Leader', 'Follower', 'Connector', 'Team Center'], loc=1, fontsize='14')
                # axis.set_title(f'Experiment (Connectors = {num_connectors}, Length = {round(G.size(weight="length"), 4)}, Connected = {nx.is_connected(G)})')
            else:
                # # SMT
                # legend_robot = axis.legend([label_connector, label_team], ['Connector', 'Team Center'], loc=1)
                # axis.set_title(f'STP-MSPBEL (Connectors = {smt_solver.added_points}, Length = {round(smt_solver.length, 4)}, Connected = {nx.is_connected(smt_solver.G)})')

                # 2tRNP
                legend_robot = axis.legend([label_connector, label_team], ['Connector', 'Team Center'], loc=1, fontsize='16')
                # axis.set_title(f'2tRNP (Connectors = {ttrnp_solver.added_points}, Length = {round(ttrnp_solver.length, 4)}, Connected = {nx.is_connected(ttrnp_solver.G)})')

            axis.add_artist(legend_robot)

            # Fit to arena size
            ### Draw simulation arena
            # plt.setp(axis, xlim=[-s.arenaRadius,s.arenaRadius])
            # plt.setp(axis, ylim=[-s.arenaRadius,s.arenaRadius])
            ### Draw swarmhack arena
            # axis.add_patch(Rectangle((SWARMHACK_X, SWARMHACK_Y), SWARMHACK_ARENA_X, SWARMHACK_ARENA_Y, facecolor='none', ec='k', lw=2))
            axis.add_patch(Rectangle((0, -SWARMHACK_Y+SWARMHACK_FIXY-SWARMHACK_ARENA_Y), SWARMHACK_ARENA_X, SWARMHACK_ARENA_Y, facecolor='none', ec='k', lw=2))
            axis.set_xlim([-0.1,SWARMHACK_ARENA_X+0.1])
            axis.set_ylim([-0.1,SWARMHACK_ARENA_Y+0.1])
            axis.set_xticks(np.arange(0, SWARMHACK_ARENA_X+0.1, 0.2))
            axis.set_yticks(np.arange(0, SWARMHACK_ARENA_Y+0.1, 0.2))
            axis.set_aspect('equal')
            axis.set_xlabel('X (m)', fontproperties=font)
            axis.set_ylabel('Y (m)', fontproperties=font)
            for label in axis.get_xticklabels():
                label.set_fontproperties(font)
            for label in axis.get_yticklabels():
                label.set_fontproperties(font)

        fig.tight_layout()

        # plt.show()
        fig.savefig(join(RESULTS_DIR, 'final_timestep_real_robots.pdf'))

    return {'res': G, 'esmt': esmt_solver.G}, \
           {'res': G.size(weight="length"), 'esmt': esmt_solver.length}, \
           {'res': num_connectors, 'esmt': esmt_solver.added_points} 


def plot_single_trial(path, plot=False):
    s, _, _, _ = load_log_with_checks(path, print_result=plot)

    # df = pd.DataFrame({
    #     'Time':                         pd.Series(dtype='int'),
    #     'Teams':                        pd.Series(dtype='int'), 
    #     # 'Workers Per Team':              pd.Series(dtype='int'),
    #     'Network Length':               pd.Series(dtype='float'),
    #     'Network Length (Solver)':      pd.Series(dtype='float'),
    #     'Connectors':                   pd.Series(dtype='int'),
    #     'Steiner Points (Solver)':      pd.Series(dtype='int'),
    #     'Total Team Length':            pd.Series(dtype='float'),
    #     'Total Team Length (Solver)':   pd.Series(dtype='float')       
    # })

    df1 = pd.DataFrame({
        'Time':                         pd.Series(dtype='int'),
        'Network Length':               pd.Series(dtype='float'),  
        'Type':                         pd.Series(dtype='str'),
    })

    df2 = pd.DataFrame({
        'Time':                         pd.Series(dtype='int'),
        'Connectors':                   pd.Series(dtype='int'),
        'Type':                         pd.Series(dtype='str'),    
    })

    df3 = pd.DataFrame({
        'Time':                         pd.Series(dtype='int'),
        'Connectors':                   pd.Series(dtype='int'),    
        'Type':                         pd.Series(dtype='str'),
    })

    for time in range(7,s.totalTime+1):
        # print(f'time {time}')
        graphs, lengths, connectors = get_network_length_and_connectors_at_time(s, time=time, plot=False)
        # print(f'length {lengths["res"]}')
        res_path_length = get_team_distance(graphs["res"])
        esmt_path_length = get_team_distance(graphs["esmt"])
        # ttrnp_path_length = get_team_distance(graphs["2trnp"])

        # print(f'res_path: {res_path_length}')
        # print(f'approx_path: {approx_path_length}')

            # # Add data of the successful trial
            # if final_points > 0 and final_connectivity:

            # # Find the total network length and connectors for the result obtained and the solver output
            # graphs, lengths, connectors = get_network_length_and_connectors(s, plot=False)
            # if not nx.is_connected(graphs['res']):
            #     trial_no_connectivity[scenario] = {
            #         'seed': s.seed,
            #         'data': False
            #     }

            # Find the total distance between every team along the network
            # get_team_distance(graphs['res'])

            # res_path_length = get_team_distance(graphs["res"])
            # approx_path_length = get_team_distance(graphs["approx"])

        # d = pd.DataFrame({
        #     'Time': [time],
        #     'Teams': [s.numLeaders], 
        #     # 'Workers Per Team': [(int)(s.numWorkers/s.numLeaders)], 
        #     'Network Length': [lengths['res']], 
        #     'Network Length (Solver)': [lengths['approx']],
        #     'Connectors': [connectors['res']],
        #     'Steiner Points (Solver)': [connectors['approx']],
        #     'Total Team Length': [res_path_length],
        #     'Total Team Length (Solver)': [approx_path_length]  
        # })
        # df = pd.concat([df, d], ignore_index=True, axis=0)

        d = pd.DataFrame({'Time': [time], 'Network Length': [lengths['res']], 'Type': ['res']})
        df1 = pd.concat([df1, d], ignore_index=True, axis=0)
        d = pd.DataFrame({'Time': [time], 'Network Length': [lengths['esmt']], 'Type': ['smt']})
        df1 = pd.concat([df1, d], ignore_index=True, axis=0)
        # d = pd.DataFrame({'Time': [time], 'Network Length': [lengths['2trnp']],'Type': ['2trnp']})
        # df1 = pd.concat([df1, d], ignore_index=True, axis=0)

        d = pd.DataFrame({'Time': [time], 'Connectors': [connectors['res']], 'Type': ['res']})
        df2 = pd.concat([df2, d], ignore_index=True, axis=0)
        d = pd.DataFrame({'Time': [time], 'Connectors': [connectors['esmt']], 'Type': ['smt']})
        df2 = pd.concat([df2, d], ignore_index=True, axis=0)
        # d = pd.DataFrame({'Time': [time], 'Connectors': [connectors['2trnp']], 'Type': ['2trnp']})
        # df2 = pd.concat([df2, d], ignore_index=True, axis=0)

        d = pd.DataFrame({'Time': [time], 'Total Team Length': [res_path_length], 'Type': ['res']})
        df3 = pd.concat([df3, d], ignore_index=True, axis=0)
        d = pd.DataFrame({'Time': [time], 'Total Team Length': [esmt_path_length], 'Type': ['esmt']})
        df3 = pd.concat([df3, d], ignore_index=True, axis=0)
        # d = pd.DataFrame({'Time': [time], 'Total Team Length': [ttrnp_path_length], 'Type': ['2trnp']})
        # df3 = pd.concat([df3, d], ignore_index=True, axis=0)

    # Font
    font = FontProperties()
    # font.set_family('serif')
    # font.set_name('Times New Roman')
    font.set_size(20)

    font2 = FontProperties()
    # font2.set_family('serif')
    # font2.set_name('Times New Roman')
    font2.set_size(16)

    # Plot data
    fig = plt.figure(figsize=(9, 5))
    axes = fig.gca()
    fig2 = plt.figure(figsize=(9, 5))
    axes2 = fig2.gca()
    fig3 = plt.figure(figsize=(9, 5))
    axes3 = fig3.gca()

    # order_num_team = list(df['Teams'].unique())
    # order_num_team.sort()
    # order_team_size = list(df['Workers Per Team'].unique())
    # order_team_size.sort()

    # Get 

    sns.lineplot(data=df1, ax=axes, x='Time', y='Network Length', hue='Type', palette='Set2')

    sns.lineplot(data=df2, ax=axes2, x='Time', y='Connectors', hue='Type', palette='Set2')

    sns.lineplot(data=df3, ax=axes3, x='Time', y='Total Team Length', hue='Type', palette='Set2')

    # X = np.repeat(np.atleast_2d(np.arange(len(team_num_dirs))),len(team_num_dirs),axis=0) + np.array([[-.3],[-.1],[.1],[.3]])
    # X = np.repeat(np.atleast_2d(np.arange(len(team_num_dirs))),len(team_num_dirs),axis=0) + np.array([[0]])
    # X = X.flatten()
    # X.sort()
    # approx_length_means = []
    # approx_steiner_means = []
    # approx_team_length_means = []
    # for team in order_num_team:
    #     for size in order_team_size:
    #         df_team = df[df['Teams'] == team]
    #         df_team_and_size = df_team[df_team['Workers Per Team'] == size]
    #         print(f'{team} --- {size}')
    #         print(f'Mean length:\t\t{df_team_and_size["Network Length"].mean()}\tSolver:\t{df_team_and_size["Network Length (Solver)"].mean()}')
    #         print(f'Mean connectors:\t{df_team_and_size["Connectors"].mean()}\tSolver:\t{df_team_and_size["Steiner Points (Solver)"].mean()}')
    #         print(f'Mean team distance:\t\t{df_team_and_size["Total Team Length"].mean()}\tSolver:\t{df_team_and_size["Total Team Length (Solver)"].mean()}')
    #         approx_length_means.append(df_team_and_size["Network Length (Solver)"].mean())
    #         approx_steiner_means.append(df_team_and_size["Steiner Points (Solver)"].mean())
    #         approx_team_length_means.append(df_team_and_size["Total Team Length (Solver)"].mean())

    # print(X)
    # print(approx_length_means)

    # axes.plot(X, approx_length_means, 'r<', zorder=4) # Network Length
    # axes2.plot(X, approx_steiner_means, 'r<', zorder=4)  # Connectors
    # axes3.plot(X, approx_team_length_means, 'r<', zorder=4) # Total Team Length

    # colors = ['.25', '.25']
    # sns.stripplot(data=df, ax=axes, x='Teams', y='Network Length', order=[2], hue='Workers Per Team', hue_order=[6, 12, 18, 24], size=6, palette=colors, linewidth=0, dodge=True, jitter=False)
    
    axes.set_xlabel('Timestep', fontproperties=font)
    axes.set_ylabel('Network Length (m)', fontproperties=font)

    axes2.set_xlabel('Timestep', fontproperties=font)
    axes2.set_ylabel('Connectors', fontproperties=font)

    axes3.set_xlabel('Timestep', fontproperties=font)
    axes3.set_ylabel('Total Team Path (m)', fontproperties=font)

    # border
    axes.spines['top'].set_visible(False)
    axes.spines['right'].set_visible(False)
    axes.spines['bottom'].set_linewidth(1)
    axes.tick_params(width=1)

    axes2.spines['top'].set_visible(False)
    axes2.spines['right'].set_visible(False)
    axes2.spines['bottom'].set_linewidth(1)
    axes2.tick_params(width=1)

    axes3.spines['top'].set_visible(False)
    axes3.spines['right'].set_visible(False)
    axes3.spines['bottom'].set_linewidth(1)
    axes3.tick_params(width=1)

    # axes.set_ylim([0,4])
    
    fig.tight_layout()
    fig2.tight_layout()
    fig3.tight_layout()

    plt.show()


def plot_single_trial_at_time(path, time=0, plot=False):
    s, _, _, _ = load_log_with_checks(path, print_result=True)
    graphs, lengths, connectors = get_network_length_and_connectors_at_time(s, time=time, plot=plot)
    print(graphs['res'].edges)
    print(graphs['esmt'].edges)
    res_path_length = get_team_distance(graphs['res'])
    esmt_path_length = get_team_distance(graphs['esmt'])
    print(f'res_path: {res_path_length}')
    print(f'esmt_path: {esmt_path_length}')


def plot_all_trials_congestion(var, max_t=0):
    
    df = pd.DataFrame({
        'Time':             pd.Series(dtype='float'), 
        'Delay':            pd.Series(dtype='float'),
        'Travelers':        pd.Series(dtype='int'),
        'Average Speed':    pd.Series(dtype='float'),  
        'Minimum Speed':    pd.Series(dtype='float'),
        'Total Distance':   pd.Series(dtype='float'),
        'Robots Collided':  pd.Series(dtype='int'),
    })
    
    # Get trial names
    variation_dirs = [f for f in listdir(RESULTS_DIR) if isdir(join(RESULTS_DIR, f))]
    variation_dirs.sort()

    num_failed_runs = {}
    
    max_time = max_t

    for variation in variation_dirs:
        if variation == var:
            trial_dirs = [f for f in listdir(join(RESULTS_DIR, variation)) if isdir(join(RESULTS_DIR, variation, f))]
            trial_dirs.sort()

            trial_no_points = {}
            trial_no_connectivity = {}
            trial_no_team_connectivity = {}
            trial_timeout = {}

            start_time = time.time()

            count = 0
            # for scenario in trial_dirs:
            while count < len(trial_dirs):
                # if count >= len(trial_dirs):
                #     break

                scenario = trial_dirs[count]
                delay = int(scenario.split('_')[3][:-1]) # Get send delay

                start_time_single = time.time()

                # Check if the trial was successful
                s, final_points, final_connectivity, final_team_connectivity = load_log_with_checks(join(RESULTS_DIR, variation, scenario))
                if final_points == 0:
                    trial_no_points[scenario] = {
                        'seed': s.seed,
                        'data': final_points
                    }

                if final_team_connectivity:
                    trial_no_team_connectivity[scenario] = {
                        'seed': s.seed,
                        'data': final_team_connectivity
                    }

                # Find the total network length and connectors for the result obtained and the solver output
                graphs, lengths, connectors = get_network_length_and_connectors(s, plot=False)
                if not nx.is_connected(graphs['res']):
                    trial_no_connectivity[scenario] = {
                        'seed': s.seed,
                        'data': False
                    }

                if s.totalTime == 6000:
                    trial_timeout[scenario] = {
                        'seed': s.seed,
                        'data': True
                    }

                # Find the number of Travelers and the average speed of travelers at every timestep
                data, avg_travel_time = get_traveler_info(s)

                # print(f'AVG TRAVEL TIME: {avg_travel_time}')

                # Add data of the successful trial
                if not scenario in trial_no_connectivity and not scenario in trial_timeout:
                    
                    last_time = 0
                    last_entry = None
                    count_entry = 0
                    
                    for t, log in data.items():
                        d = pd.DataFrame({
                            'Time': [int(t/10)], 
                            'Delay': [delay/10], 
                            'Travelers': [log['num_traveler']], 
                            'Average Speed': [log['avg_speed'] if log['avg_speed'] != None else np.nan], # Replace None with np.nan
                            'Minimum Speed': [log['min_speed'] if log['min_speed'] != None else np.nan],
                            'Total Distance': [log['total_dist']],
                            'Robots Collided': [s.robotsCollided],
                        })
                        last_entry = d
                        last_time = t
                        count_entry += 1
                        df = pd.concat([df, d], ignore_index=True, axis=0)
                        
                        # # FOR FINDING THE MAX_TIME OF ALL TRIALS
                        # if t > max_time:
                        #     max_time = t
                                                    
                    # Repeat last timestep entry
                    # count_t = last_time
                    count_t = int(last_time/10)*10
                    while count_t < max_time:
                        count_t += 10
                        last_entry['Time'] = count_t/10
                        df = pd.concat([df, d], ignore_index=True, axis=0)
                        count_entry += 1
                        
                    # print(f'lines: {count_entry}')
                    
                else:
                    # Record failed trial
                    delay_val = int(scenario.split('_')[3][:-1])
                    trial_num = int(scenario.split('_')[4])
                    if not delay_val in num_failed_runs:
                        num_failed_runs[delay_val] = set()
                    num_failed_runs[delay_val].add(trial_num)

                duration_single = round(time.time() - start_time_single, 3)
                duration_total = round(time.time() - start_time, 3)
                print("Loaded -- '{0}' --\tin {1} s ({2} s)".format(scenario, duration_single, duration_total))

                # DEBUG: For limiting the number of data to plot
                count += 1
                num_to_use = 50
                if (count % 10) % num_to_use == 0:
                    count += 50 - num_to_use
                    # print(count)
                    # break


            print(f'### Lost global connectivity ###')
            for scenario, trial in trial_no_connectivity.items():
                print(f'      scenario: {scenario}, seed: {trial["seed"]}, connectivity: {trial["data"]}')
            print(f'    Trials that lost global connectivity: ({len(trial_no_connectivity)}/{len(trial_dirs)})')

            print(f'### Timeout ###')
            for scenario, trial in trial_timeout.items():
                print(f'      scenario: {scenario}, seed: {trial["seed"]}, timeout: {trial["data"]}')
            print(f'    Trials that timed out: ({len(trial_timeout)}/{len(trial_dirs)})')

            # print(df['Delay'].unique())
            
    print(f'Failed Runs: {num_failed_runs}')
    print(f'max timestep: {max_time}')

    # Font
    font = FontProperties()
    # font.set_family('serif')
    # font.set_name('Times New Roman')
    font.set_size(14)

    font2 = FontProperties()
    # font2.set_family('serif')
    # font2.set_name('Times New Roman')
    font2.set_size(12)

    # Plot data
    fig1 = plt.figure(figsize=(5, 4))
    axes1 = fig1.gca()
    # fig2 = plt.figure(figsize=(9, 5))
    # axes2 = fig2.gca()
    # fig3 = plt.figure(figsize=(9, 5))
    # axes3 = fig3.gca()
    # fig4 = plt.figure(figsize=(9, 5))
    # axes4 = fig4.gca()
    fig5 = plt.figure(figsize=(5, 4))
    axes5 = fig5.gca()
    axes = [axes1, axes5]

    print(df)

    ### LINE PLOT
    hue_order = np.sort(df['Delay'].unique())
    print(f'hue_order {hue_order}')
    sns.lineplot(data=df, ax=axes1, x='Time', y='Travelers', hue='Delay', hue_order=hue_order, legend='full')
    # sns.lineplot(data=df, ax=axes2, x='Time', y='Average Speed', hue='Delay', hue_order=hue_order)
    # sns.lineplot(data=df, ax=axes3, x='Time', y='Minimum Speed', hue='Delay', hue_order=hue_order)
    # sns.lineplot(data=df, ax=axes4, x='Time', y='Total Distance', hue='Delay', hue_order=hue_order)

    sns.boxplot(data=df, ax=axes5, x='Delay', y='Robots Collided', order=hue_order, palette='Set2')
    # colors = ['.25', '.25']
    # sns.stripplot(data=df, ax=axes5, x='Delay', y='Robots Collided', order=hue_order, size=6, palette=colors, linewidth=0, dodge=True, jitter=False)

    axes1.set_xlabel('Time (s)', fontproperties=font)
    axes1.set_ylabel('Travellers', fontproperties=font)

    # legend

    # axes1.legend([str(int(label)) for label in hue_order], bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)
    handles, labels = axes1.get_legend_handles_labels()
    # axes1.legend(handles=handles[1:], labels=[str(int(label)) for label in hue_order], title='Delay (s)')
    axes1.legend(handles=handles, labels=[str(int(float(label))) for label in labels], title='Delay (s)')
    axes1.set_ylim([0,40+1])


    max_val = df['Robots Collided'].max()
    # axes5.set_yticks(np.arange(0, max_val+2, 2))
    axes5.set_xticks(range(len(labels)), labels=[str(int(float(label))) for label in labels])
    axes5.set_xlabel('Delay (s)', fontproperties=font)
    axes5.set_ylabel('Robots Collided', fontproperties=font)
    axes5.set_ylim([0,26+3])
    print(labels)
    # axes5.set_xticks(range(len(labels)))

    # Define the text to be displayed above each box plot
    text_values = []
    for label in labels:
        delay_val = int(float(label)) * 10
        if delay_val in num_failed_runs:
            text_values.append(f'x{len(num_failed_runs[delay_val])}')
        else:
            text_values.append('x0')

    print(text_values)

    # Calculate the x-coordinate for each text based on the number of box plots (data points)
    num_boxes = len(text_values)
    text_positions = range(num_boxes)

    # Add text above each box plot
    for i, t in zip(text_positions, text_values):
        axes5.text(i, 26 + 2, t, ha='center', color='red', fontproperties=font2)

    for ax in axes:
        for label in ax.get_xticklabels():
            label.set_fontproperties(font2)
        for label in ax.get_yticklabels():
            label.set_fontproperties(font2)

        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.spines['bottom'].set_linewidth(1)
        ax.tick_params(width=1)

    fig1.tight_layout()
    fig5.tight_layout()

    # plt.show()
    fig1.savefig(join(RESULTS_DIR, f'congestion_{var}_travellers.pdf'))
    fig5.savefig(join(RESULTS_DIR, f'congestion_{var}_collisions.pdf'))  


def plot_energy_consumption(plot=False):

    # print('### START FUNC ###')

    # scnario_labels = ['travel', 'charger(9R1C)', 'charger(8R2C)', 'charger(7R3C)', 'charger(6R4C)', 'charger(5R5C)']
    # scnario_labels = ['travel', 'charger(9R1C)']
    scenario_labels = []

    df = pd.DataFrame({
        'Scenario':             pd.Series(dtype='str'),
        'Work performed':             pd.Series(dtype='int'), 
        'Total energy':       pd.Series(dtype='float'),
        'Work Energy': pd.Series(dtype='float'),
        'Work Energy / Total energy':  pd.Series(dtype='float'),
        'Connector Energy': pd.Series(dtype='float'),
        'Connector Energy / Total energy':  pd.Series(dtype='float'),
        'Energy Shared': pd.Series(dtype='float'),
        'Workers depleted': pd.Series(dtype='int'),
        'Connectors Depleted': pd.Series(dtype='int'),
    })

    # loop all directories in RESULTS_DIR
    for variation in [f for f in listdir(RESULTS_DIR) if isdir(join(RESULTS_DIR, f))]:
        
        # Split variation by '_'
        var = variation.split('_')

            
        # if variation != '4T_charger(7R3CCC)':
        #     continue

        # scenario_type = var[1]
        scenario_type = variation
        scenario_labels.append(scenario_type)
        
        count = 0

        trial_dirs = [f for f in listdir(join(RESULTS_DIR, variation)) if isdir(join(RESULTS_DIR, variation, f))]
        trial_dirs.sort()

        # loop all directories in variation
        for scenario in [f for f in trial_dirs if isdir(join(RESULTS_DIR, variation, f))]:

            count += 1
            # if count > 10:
            #     break
            # print(scenario)
            
            # if scenario[0] == 'X' :
            #     print('skipping', scenario)
            #     continue

            # build path using RESULTS_DIR, variation and scenario
            path = join(RESULTS_DIR, variation, scenario)
            print(path)

            s, _, _, _ = load_log_with_checks(path, print_result=plot)
            
            # # Counters
            # total_work = 0
            # total_energy_consumed = 0
            # total_energy_consumed_by_connectors = 0
            # total_energy_consumed_by_connectors_moving = 0

            # # Get the total amount of work performed
            # for task in s.data[s.totalTime]['log'].tasks:
            #     total_work += task.demand
            #     # print(task.name, task.demand)

            # # print('total work:', total_work)

            # # Dict to record every robot's energy consumption
            # initial_energy = 100
            # energy_idle_per_step = 0.005
            # energy_moving_per_step = 0.025
            # energy_working_per_step = 0.055
            # current_robot_energy = {}
            # energy_consumed_by_robot = {}
            # energy_consumed_by_connector = {}
            # energy_consumed_by_connector_moving = {}
            # energy_transfered = 0

            # # Init dict for every robot
            # for robot in s.data[1]['log'].robots:
            #     if robot.state is not time_step_pb2.Robot.State.LEADER:
            #         current_robot_energy[robot.name] = initial_energy
            #         energy_consumed_by_robot[robot.name] = 0
            #         energy_consumed_by_connector[robot.name] = 0
            #         energy_consumed_by_connector_moving[robot.name] = 0

            # # print(current_robot_energy)
            # # print(energy_consumed_by_robot)

            # # set of robots that ran out of energy
            # robots_out_of_energy = set()
            # connectors_out_of_energy = set()

            # # Loop every time step
            # for time in range(1, s.totalTime+1):
            # # for time in range(1, 100):

            #     # Energy shared between robots
            #     energy_transfered += s.data[time]['log'].energyShared
            #     # print('energy shared: ', s.data[time]['log'].energy_shared)

            #     # Loop robots
            #     for robot in s.data[time]['log'].robots:
            #         if robot.state is not time_step_pb2.Robot.State.LEADER:

            #             # Energy consumed
            #             if robot.energyLevel < current_robot_energy[robot.name]: 

            #                 # Robot was not charging
            #                 energy_consumed_by_robot[robot.name] += current_robot_energy[robot.name] - robot.energyLevel

            #                 if robot.state == time_step_pb2.Robot.State.CONNECTOR:
            #                     energy_consumed_by_connector[robot.name] += current_robot_energy[robot.name] - robot.energyLevel

            #                     if robot.isMoving:
            #                         energy_consumed_by_connector_moving[robot.name] += current_robot_energy[robot.name] - robot.energyLevel

            #             elif robot.isMoving: 
                            
            #                 # Robot was charing and moving
            #                 energy_consumed_by_robot[robot.name] += energy_moving_per_step

            #                 if robot.state == time_step_pb2.Robot.State.CONNECTOR:
            #                     energy_consumed_by_connector[robot.name] += energy_moving_per_step
            #                     energy_consumed_by_connector_moving[robot.name] += energy_moving_per_step
            #             else:
            #                 # Robot was charging and idle
            #                 energy_consumed_by_robot[robot.name] += energy_idle_per_step

            #                 if robot.state == time_step_pb2.Robot.State.CONNECTOR:
            #                     energy_consumed_by_connector[robot.name] += energy_idle_per_step

            #             # Update robot energy 
            #             current_robot_energy[robot.name] = robot.energyLevel

            #             if robot.energyLevel <= 0:
            #                 robots_out_of_energy.add(robot.name)

            #                 if robot.state == time_step_pb2.Robot.State.CONNECTOR:
            #                     connectors_out_of_energy.add(robot.name)

            # print(energy_consumed_by_robot)

            # Add all values in energy_consumed_by_robot
            # total_energy_consumed = sum(energy_consumed_by_robot.values())

            print('###### SCENARIO:', scenario, '######')

            # print(f'Total work performed:\t{total_work}')
            # print(f'Total energy consumed by all robots:\t{total_energy_consumed}')
            # print(f'Total energy consumed by connectors:\t{sum(energy_consumed_by_connector.values())}')
            # print(f'Total energy consumed by connectors moving:\t{sum(energy_consumed_by_connector_moving.values())}')
            # print(f'Total energy shared:\t{energy_transfered}')

            # print(f'work / total energy:', total_work / total_energy_consumed)
            # print(f'connector / total energy:', sum(energy_consumed_by_connector.values()) / total_energy_consumed)
            # print(f'connector moving / total energy:', sum(energy_consumed_by_connector_moving.values()) / total_energy_consumed)

            # print(f'Max energy consumed by a robot:\t{max(energy_consumed_by_robot.values())}')
            # print(f'Min energy consumed by a robot:\t{min(energy_consumed_by_robot.values())}')

            # df = pd.DataFrame({
            #     'Work performed':             pd.Series(dtype='int'), 
            #     'Total energy':       pd.Series(dtype='float'),
            #     'Work performed / Total energy':  pd.Series(dtype='float'),
            #     'Total energy (Connector)': pd.Series(dtype='float'),
            #     'Total energy (Connector) / Total energy':   pd.Series(dtype='float'),
            #     'Total energy (Connector Moving)': pd.Series(dtype='float'),
            # })


            # for robot, energy in current_robot_energy.items():
            #     if energy <= 0:
            #         robots_out_of_energy.add(robot)

            #         if robot.state == time_step_pb2.Robot.State.CONNECTOR:
            #             connectors_out_of_energy.add(robot)
            
            # print(f'Robots out of energy: {len(robots_out_of_energy)}')
            # print(f'Connectors out of energy: {len(connectors_out_of_energy)}')

            d = pd.DataFrame({
                'Scenario': [scenario_type],
                'Work performed': [s.data[s.totalTime]['log'].points], 
                'Total energy':       [s.data[s.totalTime]['log'].totalEnergy],
                'Work Energy': [s.data[s.totalTime]['log'].workEnergy],
                'Work Energy / Total energy':  [s.data[s.totalTime]['log'].workEnergy / s.data[s.totalTime]['log'].totalEnergy],
                'Connector Energy': [s.data[s.totalTime]['log'].connectorEnergy],
                'Connector Energy / Total energy':   [s.data[s.totalTime]['log'].connectorEnergy / s.data[s.totalTime]['log'].totalEnergy],
                'Energy Shared': [s.data[s.totalTime]['log'].energyShared],
                'Workers depleted': [s.data[s.totalTime]['log'].workersDepleted],
                'Connectors Depleted': [s.data[s.totalTime]['log'].connectorsDepleted],
                # 'Total energy (Connector Moving)': [sum(energy_consumed_by_connector_moving.values())],
                # 'Total energy (Connector Moving) / Total energy':   [sum(energy_consumed_by_connector_moving.values()) / total_energy_consumed],
            })
            df = pd.concat([df, d], ignore_index=True, axis=0)

            # print()

    print(df)

    ## loop scenario_labels
    for label in scenario_labels:

        print('##### RESULTS FOR SCENARIO:', label, '#####')

        # Print summary for 'travel'
        df_scenario = df[df['Scenario'] == label]

        avg = df_scenario['Work performed'].mean()
        print(f'Average Work performed: {avg}')
        avg = df_scenario['Total energy'].mean()
        print(f'Average Total energy: {avg}')
        avg = df_scenario['Work Energy'].mean()
        print(f'Average Work Energy: {avg}')
        avg = df_scenario['Work Energy / Total energy'].mean()
        print(f'Average Work Energy / Total energy: {avg}')
        avg = df_scenario['Connector Energy'].mean()
        print(f'Average Connector Energy: {avg}')
        avg = df_scenario['Connector Energy / Total energy'].mean()
        print(f'Average Connector Energy / Total energy: {avg}')
        avg = df_scenario['Energy Shared'].mean()
        print(f'Average Energy Shared: {avg}')
        avg = df_scenario['Workers depleted'].mean()
        print(f'Average Workers depleted: {avg}')
        avg = df_scenario['Connectors Depleted'].mean()
        print(f'Average Connectors Depleted: {avg}')

        # save df_scenario and the average of each column to a csv
        df_scenario.to_csv(join(RESULTS_DIR, f'energy_{label}.csv'), index=False)
        

        print()

    # Font
    font = FontProperties()
    # font.set_family('serif')
    # font.set_name('Times New Roman')
    font.set_size(14)

    font2 = FontProperties()
    # font2.set_family('serif')
    # font2.set_name('Times New Roman')
    font2.set_size(12)



    exit(0)


    # Plot data
    fig1 = plt.figure(figsize=(5, 4))
    axes1 = fig1.gca()
    # fig2 = plt.figure(figsize=(9, 5))
    # axes2 = fig2.gca()
    # fig3 = plt.figure(figsize=(9, 5))
    # axes3 = fig3.gca()
    # fig4 = plt.figure(figsize=(9, 5))
    # axes4 = fig4.gca()
    # fig5 = plt.figure(figsize=(5, 4))
    # axes5 = fig5.gca()
    # axes = [axes1, axes5]

    print(df)

    ### LINE PLOT
    # hue_order = np.sort(df['Delay'].unique())
    # print(f'hue_order {hue_order}')
    # sns.lineplot(data=df, ax=axes1, x='Time', y='Travelers', hue='Delay', hue_order=hue_order, legend='full')
    # # sns.lineplot(data=df, ax=axes2, x='Time', y='Average Speed', hue='Delay', hue_order=hue_order)
    # # sns.lineplot(data=df, ax=axes3, x='Time', y='Minimum Speed', hue='Delay', hue_order=hue_order)
    # # sns.lineplot(data=df, ax=axes4, x='Time', y='Total Distance', hue='Delay', hue_order=hue_order)

    # sns.boxplot(data=df, ax=axes5, x='Delay', y='Robots Collided', order=hue_order, palette='Set2')
    # colors = ['.25', '.25']
    # sns.stripplot(data=df, ax=axes5, x='Delay', y='Robots Collided', order=hue_order, size=6, palette=colors, linewidth=0, dodge=True, jitter=False)

    # plot an area chart where the x axis is the time and the y axis is the number of 'Connectors', 'Workers working', 'Workers charging', 'Workers moving'
    sns.lineplot(data=df, ax=axes1, x='Time', y='Connectors', color='blue')
    sns.lineplot(data=df, ax=axes1, x='Time', y='Workers working', color='green')
    sns.lineplot(data=df, ax=axes1, x='Time', y='Workers charging', color='orange')
    sns.lineplot(data=df, ax=axes1, x='Time', y='Workers moving', color='red')


    axes1.set_xlabel('Time (s)', fontproperties=font)
    axes1.set_ylabel('Number of robots', fontproperties=font)

    # legend

    # axes1.legend([str(int(label)) for label in hue_order], bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)
    handles, labels = axes1.get_legend_handles_labels()
    # axes1.legend(handles=handles[1:], labels=[str(int(label)) for label in hue_order], title='Delay (s)')
    # axes1.legend(handles=handles, labels=[str(int(float(label))) for label in labels], title='Delay (s)')
    # axes1.set_ylim([0,40+1])


    # max_val = df['Robots Collided'].max()
    # # axes5.set_yticks(np.arange(0, max_val+2, 2))
    # axes5.set_xticks(range(len(labels)), labels=[str(int(float(label))) for label in labels])
    # axes5.set_xlabel('Delay (s)', fontproperties=font)
    # axes5.set_ylabel('Robots Collided', fontproperties=font)
    # axes5.set_ylim([0,26+3])
    # print(labels)
    # axes5.set_xticks(range(len(labels)))

    # Define the text to be displayed above each box plot
    # text_values = []
    # for label in labels:
    #     delay_val = int(float(label)) * 10
    #     if delay_val in num_failed_runs:
    #         text_values.append(f'x{len(num_failed_runs[delay_val])}')
    #     else:
    #         text_values.append('x0')

    # print(text_values)

    # # Calculate the x-coordinate for each text based on the number of box plots (data points)
    # num_boxes = len(text_values)
    # text_positions = range(num_boxes)

    # # Add text above each box plot
    # for i, t in zip(text_positions, text_values):
    #     axes5.text(i, 26 + 2, t, ha='center', color='red', fontproperties=font2)

    # for ax in axes:
    #     for label in ax.get_xticklabels():
    #         label.set_fontproperties(font2)
    #     for label in ax.get_yticklabels():
    #         label.set_fontproperties(font2)

    #     ax.spines['top'].set_visible(False)
    #     ax.spines['right'].set_visible(False)
    #     ax.spines['bottom'].set_linewidth(1)
    #     ax.tick_params(width=1)

    fig1.tight_layout()
    # fig5.tight_layout()

    plt.show()
    # fig1.savefig(join(RESULTS_DIR, f'congestion_{var}_travellers.pdf'))
    # fig5.savefig(join(RESULTS_DIR, f'congestion_{var}_collisions.pdf'))  


def plot_energy_boxplot(chosen_num_chargers, chosen_capacity, plot=False):

    # print('### START FUNC ###')

    # scnario_labels = ['travel', 'charger(9R1C)', 'charger(8R2C)', 'charger(7R3C)', 'charger(6R4C)', 'charger(5R5C)']
    # scnario_labels = ['travel', 'charger(9R1C)']
    scenario_labels = []

    df = pd.DataFrame({
        'Strategy':             pd.Series(dtype='str'),
        'Num Chargers':             pd.Series(dtype='int'),
        'Capacity':             pd.Series(dtype='str'),
        'Work Rate':            pd.Series(dtype='float'),
        'Work performed':             pd.Series(dtype='int'), 
        'Total energy':       pd.Series(dtype='float'),
        'Work Energy': pd.Series(dtype='float'),
        'Work Energy / Total energy':  pd.Series(dtype='float'),
        'Workers depleted': pd.Series(dtype='int'),
        'Chargers depleted': pd.Series(dtype='int'),
    })

    count = 0

    # loop all directories in RESULTS_DIR
    for variation in [f for f in listdir(join(RESULTS_DIR, 'data')) if isdir(join(RESULTS_DIR, 'data', f))]:
        
        # Split variation by '_'
        var = variation.split('_')
        strategy = var[0]
        work_rate = var[1]
        num_chargers = var[2]
        capacity = var[3]
        recharge_rate = var[4]
        loss_rate = var[5]
        trial = var[6]

        # print(var)

        # convert work_rate to number and drop 'W'
        work_rate = float(work_rate[:-1])
        # print('work_rate', work_rate)

        # convert num_chargers to number and drop 'C'
        num_chargers = int(num_chargers[:-1])
        # print('num_chargers', num_chargers)

        # convert capacity to number and drop 'E'
        capacity = int(capacity[:-1])
        # print('capacity', capacity)

        # convert recharge_rate to number and drop 'R'
        recharge_rate = float(recharge_rate[:-1])

        # convert loss_rate to number and drop 'L'
        loss_rate = float(loss_rate[:-1])

        # convert trial to number
        trial = int(trial[1:])
        # print('trial', trial)

        # print('strategy', strategy, 'mobile', strategy == 'mobile')
        # print('num_chargers', num_chargers, chosen_num_chargers, num_chargers == chosen_num_chargers)
        # print('capacity', capacity, chosen_capacity, capacity == chosen_capacity)

        # print('\tsum', strategy == 'mobile' and num_chargers == chosen_num_chargers and capacity == chosen_capacity)
        
        if (strategy == 'fixed') or (strategy == 'mobile' and num_chargers == chosen_num_chargers and capacity == chosen_capacity):

            count += 1

            trial_dirs = [f for f in listdir(join(RESULTS_DIR, 'data', variation)) if isdir(join(RESULTS_DIR, 'data', variation, f))]
            trial_dirs.sort()

            # # loop all directories in variation
            # for scenario in [f for f in trial_dirs if isdir(join(RESULTS_DIR, variation, f))]:

                # count += 1
                # if count > 10:
                #     break
                # print(scenario)
                
            # build path using RESULTS_DIR, variation and scenario
            path = join(RESULTS_DIR, 'data', variation)
            print(count, path)

            s, _ = load_log_with_checks(path, print_result=plot)

            # print('###### SCENARIO:', variation, '######')

            d = pd.DataFrame({
                'Strategy': [strategy],
                'Num Chargers': [num_chargers],
                'Capacity': [f'{int(capacity/100)}x'],
                'Work Rate': [work_rate],
                'Work performed': [s.data[s.totalTime]['log'].points], 
                'Total energy':       [s.data[s.totalTime]['log'].totalEnergy],
                'Work Energy': [s.data[s.totalTime]['log'].workEnergy],
                'Work Energy / Total energy':  [s.data[s.totalTime]['log'].workEnergy / s.data[s.totalTime]['log'].totalEnergy],
                # 'Energy Shared': [s.data[s.totalTime]['log'].energyShared],
                'Workers depleted': [s.data[s.totalTime]['log'].workersDepleted],
                'Chargers depleted': [s.data[s.totalTime]['log'].chargersDepleted],
            })
            df = pd.concat([df, d], ignore_index=True, axis=0)

    print(df)

    fig1 = plt.figure(figsize=(6, 3.5))
    fig2 = plt.figure(figsize=(6, 3.5))
    fig3 = plt.figure(figsize=(6, 3.5))
    fig4 = plt.figure(figsize=(6, 3.5))
    fig5 = plt.figure(figsize=(6, 3.5))
    ax1 = fig1.gca()
    ax2 = fig2.gca()
    ax3 = fig3.gca()
    ax4 = fig4.gca()
    ax5 = fig5.gca()

    axes = [ax1, ax2, ax3, ax4, ax5]

    # Font
    font = FontProperties()
    # font.set_family('serif')
    # font.set_name('Times New Roman')
    font.set_size(14)

    font2 = FontProperties()
    # font2.set_family('serif')
    # font2.set_name('Times New Roman')
    font2.set_size(12)

    plt.rcParams['text.usetex'] = True

    hue_order = ['fixed', 'mobile']
    legend_labels = ['Fixed', 'Mobile']

    # Sort and convert 'Work Rate' to string
    df = df.sort_values(by='Work Rate')
    df['Work Rate'] = df['Work Rate'].astype(str)

    # plot boxplot for 'Work performed'
    sns.boxplot(data=df, ax=ax1, x='Work Rate', y='Work performed', hue='Strategy', hue_order=hue_order, palette='Set2')
    # sns.lineplot(data=df, ax=ax1, x='Work Rate', y='Work performed', hue='Strategy', hue_order=hue_order, palette='Set2')

    # Calculate means by group
    means = df.groupby(['Work Rate', 'Strategy'])['Work performed'].mean().reset_index()
    # Overlay lines connecting means
    sns.lineplot(data=means, ax=ax1, x='Work Rate', y='Work performed', hue='Strategy', hue_order=hue_order, marker='D', dashes=False, palette='Set2', legend=False)

    # Set axis labels
    ax1.set_xlabel('Energy cost of working vs moving')
    ax1.set_ylabel('Work performed')
    handles, labels = ax1.get_legend_handles_labels()
    ax1.legend(handles=handles, labels=legend_labels, title='Strategy',  bbox_to_anchor=([1.0, 1, 0, 0]), ncol=1, frameon=True)    
    # ax1.set_title('Work performed')

    # lineplot

    # plot boxplot for 'Work Energy / Total energy'
    sns.boxplot(data=df, ax=ax2, x='Work Rate', y='Work Energy / Total energy', hue='Strategy', hue_order=hue_order, palette='Set2')
    means = df.groupby(['Work Rate', 'Strategy'])['Work Energy / Total energy'].mean().reset_index()
    sns.lineplot(data=means, ax=ax2, x='Work Rate', y='Work Energy / Total energy', hue='Strategy', hue_order=hue_order, marker='D', dashes=False, palette='Set2', legend=False)
    ax2.set_xlabel('Energy cost of working vs moving')
    ax2.set_ylabel('Energy efficiency')
    # ax2.legend(title='Strategy', bbox_to_anchor=([1.0, 1, 0, 0]), ncol=1, frameon=True)
    # add legend to the bottom right corner
    handles, labels = ax2.get_legend_handles_labels()
    ax2.legend(handles=handles, labels=legend_labels, title='Strategy',  bbox_to_anchor=(1, 0), loc='lower right', ncol=1, frameon=True)    
    # ax2.set_title('Energy efficiency')

    # plot boxplot for 'Total energy'
    sns.boxplot(data=df, ax=ax3, x='Work Rate', y='Total energy', hue='Strategy', hue_order=hue_order, palette='Set2')
    means = df.groupby(['Work Rate', 'Strategy'])['Total energy'].mean().reset_index()
    sns.lineplot(data=means, ax=ax3, x='Work Rate', y='Total energy', hue='Strategy', hue_order=hue_order, marker='D', dashes=False, palette='Set2', legend=False)
    ax2.set_xlabel('Energy cost of working vs moving')
    ax3.set_ylabel('Total energy')
    handles, labels = ax3.get_legend_handles_labels()
    ax3.legend(handles=handles, labels=legend_labels, title='Strategy',  bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)
    # ax3.set_title('Total energy')

    # plot boxplot for 'Workers depleted'
    sns.boxplot(data=df, ax=ax4, x='Work Rate', y='Workers depleted', hue='Strategy', hue_order=hue_order, palette='Set2')
    means = df.groupby(['Work Rate', 'Strategy'])['Workers depleted'].mean().reset_index()
    sns.lineplot(data=means, ax=ax4, x='Work Rate', y='Workers depleted', hue='Strategy', hue_order=hue_order, marker='D', dashes=False, palette='Set2', legend=False)
    ax2.set_xlabel('Energy cost of working vs moving')
    ax4.set_ylabel('Workers depleted')
    handles, labels = ax4.get_legend_handles_labels()
    ax4.legend(handles=handles, labels=legend_labels, title='Strategy',  bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)
    # ax4.set_title('Workers depleted')

    # plot boxplot for 'Chargers depleted'
    sns.boxplot(data=df, ax=ax5, x='Work Rate', y='Chargers depleted', hue='Strategy', hue_order=hue_order, palette='Set2')
    means = df.groupby(['Work Rate', 'Strategy'])['Chargers depleted'].mean().reset_index()
    sns.lineplot(data=means, ax=ax5, x='Work Rate', y='Chargers depleted', hue='Strategy', hue_order=hue_order, marker='D', dashes=False, palette='Set2', legend=False)
    ax2.set_xlabel('Energy cost of working vs moving')
    ax5.set_ylabel('Chargers depleted')
    # ax5.set_title('Chargers depleted')
    handles, labels = ax5.get_legend_handles_labels()
    ax5.legend(handles=handles, labels=legend_labels, title='Strategy',  bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)


    for ax in axes:
        # set font size
        for label in ax.get_xticklabels():
            label.set_fontproperties(font2)
        for label in ax.get_yticklabels():
            label.set_fontproperties(font2)
        
        # set font size of axis label
        ax.set_xlabel(ax.get_xlabel(), fontproperties=font)
        ax.set_ylabel(ax.get_ylabel(), fontproperties=font)

        # set font size of legend
        ax.get_legend().set_title(ax.get_legend().get_title().get_text(), prop=font2)
        for label in ax.get_legend().get_texts():
            label.set_fontproperties(font2)

        # set font size of title
        ax.set_title(ax.get_title(), fontproperties=font)

        # hide top and right spines
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)

        # rename legend labels
                

    fig1.tight_layout()
    fig2.tight_layout()
    fig3.tight_layout()
    fig4.tight_layout()
    fig5.tight_layout()

    fig1.savefig(join(RESULTS_DIR, f'work_{chosen_num_chargers}C_{chosen_capacity}E_boxplot.pdf'))
    fig2.savefig(join(RESULTS_DIR, f'efficiency_{chosen_num_chargers}C_{chosen_capacity}E_boxplot.pdf'))
    fig3.savefig(join(RESULTS_DIR, f'total_energy_{chosen_num_chargers}C_{chosen_capacity}E_boxplot.pdf'))
    fig4.savefig(join(RESULTS_DIR, f'depleted_workers_{chosen_num_chargers}C_{chosen_capacity}E_boxplot.pdf'))
    fig5.savefig(join(RESULTS_DIR, f'depleted_chargers_{chosen_num_chargers}C_{chosen_capacity}E_boxplot.pdf'))


def plot_recharge_rate(chosen_num_chargers, chosen_capacity, plot=False):

    # print('### START FUNC ###')

    # scnario_labels = ['travel', 'charger(9R1C)', 'charger(8R2C)', 'charger(7R3C)', 'charger(6R4C)', 'charger(5R5C)']
    # scnario_labels = ['travel', 'charger(9R1C)']
    scenario_labels = []

    df = pd.DataFrame({
        'Strategy':             pd.Series(dtype='str'),
        'Num Chargers':             pd.Series(dtype='int'),
        'Capacity':             pd.Series(dtype='str'),
        'Work Rate':            pd.Series(dtype='float'),
        'Recharge rate':         pd.Series(dtype='float'),
        'Work performed':             pd.Series(dtype='int'), 
        'Total energy':       pd.Series(dtype='float'),
        'Work Energy': pd.Series(dtype='float'),
        'Work Energy / Total energy':  pd.Series(dtype='float'),
        'Workers depleted': pd.Series(dtype='int'),
        'Chargers depleted': pd.Series(dtype='int'),
    })

    count = 0

    # loop all directories in RESULTS_DIR
    for variation in [f for f in listdir(join(RESULTS_DIR, 'data')) if isdir(join(RESULTS_DIR, 'data', f))]:
        
        # Split variation by '_'
        var = variation.split('_')
        strategy = var[0]
        work_rate = var[1]
        num_chargers = var[2]
        capacity = var[3]
        recharge_rate = var[4]
        loss_rate = var[5]
        trial = var[6]

        # print(var)

        # convert work_rate to number and drop 'W'
        work_rate = float(work_rate[:-1])
        # print('work_rate', work_rate)

        # convert num_chargers to number and drop 'C'
        num_chargers = int(num_chargers[:-1])
        # print('num_chargers', num_chargers)

        # convert capacity to number and drop 'E'
        capacity = int(capacity[:-1])
        # print('capacity', capacity)

        # convert recharge rate to number and drop 'R'
        recharge_rate = float(recharge_rate[:-1])

        # convert loss_rate to number and drop 'L'
        loss_rate = float(loss_rate[:-1])

        # convert trial to number
        trial = int(trial[1:])
        # print('trial', trial)

        # print('strategy', strategy, 'mobile', strategy == 'mobile')
        # print('num_chargers', num_chargers, chosen_num_chargers, num_chargers == chosen_num_chargers)
        # print('capacity', capacity, chosen_capacity, capacity == chosen_capacity)

        # print('\tsum', strategy == 'mobile' and num_chargers == chosen_num_chargers and capacity == chosen_capacity)
        
        if (strategy == 'fixed') or (strategy == 'mobile' and num_chargers == chosen_num_chargers and capacity == chosen_capacity):

            count += 1

            trial_dirs = [f for f in listdir(join(RESULTS_DIR, 'data', variation)) if isdir(join(RESULTS_DIR, 'data', variation, f))]
            trial_dirs.sort()

            # # loop all directories in variation
            # for scenario in [f for f in trial_dirs if isdir(join(RESULTS_DIR, variation, f))]:

                # count += 1
                # if count > 10:
                #     break
                # print(scenario)
                
            # build path using RESULTS_DIR, variation and scenario
            path = join(RESULTS_DIR, 'data', variation)
            print(count, path)

            s, _ = load_log_with_checks(path, print_result=plot)

            # print('###### SCENARIO:', variation, '######')

            d = pd.DataFrame({
                'Strategy': [strategy],
                # 'Num Chargers': [num_chargers],
                # 'Capacity': [f'{int(capacity/100)}x'],
                # 'Work Rate': [work_rate],
                'Recharge rate': [recharge_rate],
                'Work performed': [s.data[s.totalTime]['log'].points], 
                'Total energy':       [s.data[s.totalTime]['log'].totalEnergy],
                # 'Work Energy': [s.data[s.totalTime]['log'].workEnergy],
                'Work Energy / Total energy':  [s.data[s.totalTime]['log'].workEnergy / s.data[s.totalTime]['log'].totalEnergy],
                # 'Energy Shared': [s.data[s.totalTime]['log'].energyShared],
                'Workers depleted': [s.data[s.totalTime]['log'].workersDepleted],
                'Chargers depleted': [s.data[s.totalTime]['log'].chargersDepleted],
            })
            df = pd.concat([df, d], ignore_index=True, axis=0)

    print(df)

    fig1 = plt.figure(figsize=(6, 3.5))
    fig2 = plt.figure(figsize=(6, 3.5))
    fig3 = plt.figure(figsize=(6, 3.5))
    fig4 = plt.figure(figsize=(6, 3.5))
    fig5 = plt.figure(figsize=(6, 3.5))
    ax1 = fig1.gca()
    ax2 = fig2.gca()
    ax3 = fig3.gca()
    ax4 = fig4.gca()
    ax5 = fig5.gca()

    axes = [ax1, ax2, ax3, ax4, ax5]

    # Font
    font = FontProperties()
    # font.set_family('serif')
    # font.set_name('Times New Roman')
    font.set_size(14)

    font2 = FontProperties()
    # font2.set_family('serif')
    # font2.set_name('Times New Roman')
    font2.set_size(12)

    plt.rcParams['text.usetex'] = True

    hue_order = ['fixed', 'mobile']
    legend_labels = ['Fixed', 'Mobile']

    # Sort and convert 'Recharge rate' to string
    df = df.sort_values(by='Recharge rate')
    df['Recharge rate'] = df['Recharge rate'].astype(str)

    # plot boxplot for 'Work performed'
    sns.boxplot(data=df, ax=ax1, x='Recharge rate', y='Work performed', hue='Strategy', hue_order=hue_order, palette='Set2')
    # sns.lineplot(data=df, ax=ax1, x='Recharge rate', y='Work performed', hue='Strategy', hue_order=hue_order, palette='Set2')

    # Calculate means by group
    means = df.groupby(['Recharge rate', 'Strategy'])['Work performed'].mean().reset_index()
    # Overlay lines connecting means
    sns.lineplot(data=means, ax=ax1, x='Recharge rate', y='Work performed', hue='Strategy', hue_order=hue_order, marker='D', dashes=False, palette='Set2', legend=False)

    # Set axis labels
    ax1.set_xlabel('Charging and transfer rate')
    ax1.set_ylabel('Work performed')
    handles, labels = ax1.get_legend_handles_labels()
    ax1.legend(handles=handles, labels=legend_labels, title='Strategy', bbox_to_anchor=(1, 0), loc='lower right', ncol=1, frameon=True)    
    # ax1.set_title('Work performed')

    # lineplot

    # plot boxplot for 'Work Energy / Total energy'
    sns.boxplot(data=df, ax=ax2, x='Recharge rate', y='Work Energy / Total energy', hue='Strategy', hue_order=hue_order, palette='Set2')
    means = df.groupby(['Recharge rate', 'Strategy'])['Work Energy / Total energy'].mean().reset_index()
    sns.lineplot(data=means, ax=ax2, x='Recharge rate', y='Work Energy / Total energy', hue='Strategy', hue_order=hue_order, marker='D', dashes=False, palette='Set2', legend=False)
    ax2.set_xlabel('Charging and transfer rate')
    ax2.set_ylabel('Energy efficiency')
    # ax2.legend(title='Strategy', bbox_to_anchor=([1.0, 1, 0, 0]), ncol=1, frameon=True)
    # add legend to the bottom right corner
    handles, labels = ax2.get_legend_handles_labels()
    ax2.legend(handles=handles, labels=legend_labels, title='Strategy',  bbox_to_anchor=(1, 0), loc='lower right', ncol=1, frameon=True)    
    # ax2.set_title('Energy efficiency')

    # plot boxplot for 'Total energy'
    sns.boxplot(data=df, ax=ax3, x='Recharge rate', y='Total energy', hue='Strategy', hue_order=hue_order, palette='Set2')
    means = df.groupby(['Recharge rate', 'Strategy'])['Total energy'].mean().reset_index()
    sns.lineplot(data=means, ax=ax3, x='Recharge rate', y='Total energy', hue='Strategy', hue_order=hue_order, marker='D', dashes=False, palette='Set2', legend=False)
    ax2.set_xlabel('Charging and transfer rate')
    ax3.set_ylabel('Total energy')
    handles, labels = ax3.get_legend_handles_labels()
    ax3.legend(handles=handles, labels=legend_labels, title='Strategy',  bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)
    # ax3.set_title('Total energy')

    # plot boxplot for 'Workers depleted'
    sns.boxplot(data=df, ax=ax4, x='Recharge rate', y='Workers depleted', hue='Strategy', hue_order=hue_order, palette='Set2')
    means = df.groupby(['Recharge rate', 'Strategy'])['Workers depleted'].mean().reset_index()
    sns.lineplot(data=means, ax=ax4, x='Recharge rate', y='Workers depleted', hue='Strategy', hue_order=hue_order, marker='D', dashes=False, palette='Set2', legend=False)
    ax2.set_xlabel('Charging and transfer rate')
    ax4.set_ylabel('Workers depleted')
    handles, labels = ax4.get_legend_handles_labels()
    ax4.legend(handles=handles, labels=legend_labels, title='Strategy',  bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)
    # ax4.set_title('Workers depleted')

    # plot boxplot for 'Chargers depleted'
    sns.boxplot(data=df, ax=ax5, x='Recharge rate', y='Chargers depleted', hue='Strategy', hue_order=hue_order, palette='Set2')
    means = df.groupby(['Recharge rate', 'Strategy'])['Chargers depleted'].mean().reset_index()
    sns.lineplot(data=means, ax=ax5, x='Recharge rate', y='Chargers depleted', hue='Strategy', hue_order=hue_order, marker='D', dashes=False, palette='Set2', legend=False)
    ax2.set_xlabel('Charging and transfer rate')
    ax5.set_ylabel('Chargers depleted')
    # ax5.set_title('Chargers depleted')
    handles, labels = ax5.get_legend_handles_labels()
    ax5.legend(handles=handles, labels=legend_labels, title='Strategy',  bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)


    for ax in axes:
        # set font size
        for label in ax.get_xticklabels():
            label.set_fontproperties(font2)
        for label in ax.get_yticklabels():
            label.set_fontproperties(font2)
        
        # set font size of axis label
        ax.set_xlabel(ax.get_xlabel(), fontproperties=font)
        ax.set_ylabel(ax.get_ylabel(), fontproperties=font)

        # set font size of legend
        ax.get_legend().set_title(ax.get_legend().get_title().get_text(), prop=font2)
        for label in ax.get_legend().get_texts():
            label.set_fontproperties(font2)

        # set font size of title
        ax.set_title(ax.get_title(), fontproperties=font)

        # hide top and right spines
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)

        # rename legend labels
                

    fig1.tight_layout()
    fig2.tight_layout()
    fig3.tight_layout()
    fig4.tight_layout()
    fig5.tight_layout()

    fig1.savefig(join(RESULTS_DIR, f'work_recharge_rate_{chosen_num_chargers}C_{chosen_capacity}E_boxplot.pdf'))
    fig2.savefig(join(RESULTS_DIR, f'efficiency_recharge_rate_{chosen_num_chargers}C_{chosen_capacity}E_boxplot.pdf'))
    fig3.savefig(join(RESULTS_DIR, f'total_energy_recharge_rate_{chosen_num_chargers}C_{chosen_capacity}E_boxplot.pdf'))
    fig4.savefig(join(RESULTS_DIR, f'depleted_workers_recharge_rate_{chosen_num_chargers}C_{chosen_capacity}E_boxplot.pdf'))
    fig5.savefig(join(RESULTS_DIR, f'depleted_chargers_recharge_rate_{chosen_num_chargers}C_{chosen_capacity}E_boxplot.pdf'))


def plot_transfer_loss(chosen_num_chargers, chosen_capacity, plot=False):

    # print('### START FUNC ###')

    # scnario_labels = ['travel', 'charger(9R1C)', 'charger(8R2C)', 'charger(7R3C)', 'charger(6R4C)', 'charger(5R5C)']
    # scnario_labels = ['travel', 'charger(9R1C)']
    scenario_labels = []

    df = pd.DataFrame({
        'Strategy':             pd.Series(dtype='str'),
        'Num Chargers':             pd.Series(dtype='int'),
        'Capacity':             pd.Series(dtype='str'),
        'Work Rate':            pd.Series(dtype='float'),
        'Recharge rate':         pd.Series(dtype='float'),
        'Transfer loss':         pd.Series(dtype='float'),
        'Work performed':             pd.Series(dtype='int'), 
        'Total energy':       pd.Series(dtype='float'),
        'Work Energy': pd.Series(dtype='float'),
        'Work Energy / Total energy':  pd.Series(dtype='float'),
        'Workers depleted': pd.Series(dtype='int'),
        'Chargers depleted': pd.Series(dtype='int'),
    })

    count = 0

    # loop all directories in RESULTS_DIR
    for variation in [f for f in listdir(join(RESULTS_DIR, 'data')) if isdir(join(RESULTS_DIR, 'data', f))]:
        
        # Split variation by '_'
        var = variation.split('_')
        strategy = var[0]
        work_rate = var[1]
        num_chargers = var[2]
        capacity = var[3]
        recharge_rate = var[4]
        loss_rate = var[5]
        trial = var[6]

        # print(var)

        # convert work_rate to number and drop 'W'
        work_rate = float(work_rate[:-1])
        # print('work_rate', work_rate)

        # convert num_chargers to number and drop 'C'
        num_chargers = int(num_chargers[:-1])
        # print('num_chargers', num_chargers)

        # convert capacity to number and drop 'E'
        capacity = int(capacity[:-1])
        # print('capacity', capacity)

        # convert recharge rate to number and drop 'R'
        recharge_rate = float(recharge_rate[:-1])

        # convert loss_rate to number and drop 'L'
        loss_rate = float(loss_rate[:-1])

        # convert trial to number
        trial = int(trial[1:])
        # print('trial', trial)

        # print('strategy', strategy, 'mobile', strategy == 'mobile')
        # print('num_chargers', num_chargers, chosen_num_chargers, num_chargers == chosen_num_chargers)
        # print('capacity', capacity, chosen_capacity, capacity == chosen_capacity)

        # print('\tsum', strategy == 'mobile' and num_chargers == chosen_num_chargers and capacity == chosen_capacity)
        
        if (strategy == 'fixed') or (strategy == 'mobile' and num_chargers == chosen_num_chargers and capacity == chosen_capacity):

            count += 1

            trial_dirs = [f for f in listdir(join(RESULTS_DIR, 'data', variation)) if isdir(join(RESULTS_DIR, 'data', variation, f))]
            trial_dirs.sort()

            # # loop all directories in variation
            # for scenario in [f for f in trial_dirs if isdir(join(RESULTS_DIR, variation, f))]:

                # count += 1
                # if count > 10:
                #     break
                # print(scenario)
                
            # build path using RESULTS_DIR, variation and scenario
            path = join(RESULTS_DIR, 'data', variation)
            print(count, path)

            s, _ = load_log_with_checks(path, print_result=plot)

            total_energy = s.data[s.totalTime]['log'].totalEnergy + s.data[s.totalTime]['log'].energyLost

            # print('###### SCENARIO:', variation, '######')

            d = pd.DataFrame({
                'Strategy': [strategy],
                # 'Num Chargers': [num_chargers],
                # 'Capacity': [f'{int(capacity/100)}x'],
                # 'Work Rate': [work_rate],
                # 'Recharge rate': [recharge_rate],
                'Transfer loss': [loss_rate],
                'Work performed': [s.data[s.totalTime]['log'].points], 
                'Total energy':       [total_energy],
                # 'Work Energy': [s.data[s.totalTime]['log'].workEnergy],
                'Work Energy / Total energy':  [s.data[s.totalTime]['log'].workEnergy / total_energy],
                # 'Energy Shared': [s.data[s.totalTime]['log'].energyShared],
                'Workers depleted': [s.data[s.totalTime]['log'].workersDepleted],
                'Chargers depleted': [s.data[s.totalTime]['log'].chargersDepleted],
            })
            df = pd.concat([df, d], ignore_index=True, axis=0)

    print(df)

    fig1 = plt.figure(figsize=(6, 3.5))
    fig2 = plt.figure(figsize=(6, 3.5))
    fig3 = plt.figure(figsize=(6, 3.5))
    fig4 = plt.figure(figsize=(6, 3.5))
    fig5 = plt.figure(figsize=(6, 3.5))
    ax1 = fig1.gca()
    ax2 = fig2.gca()
    ax3 = fig3.gca()
    ax4 = fig4.gca()
    ax5 = fig5.gca()

    axes = [ax1, ax2, ax3, ax4, ax5]

    # Font
    font = FontProperties()
    # font.set_family('serif')
    # font.set_name('Times New Roman')
    font.set_size(14)

    font2 = FontProperties()
    # font2.set_family('serif')
    # font2.set_name('Times New Roman')
    font2.set_size(12)

    plt.rcParams['text.usetex'] = True

    hue_order = ['fixed', 'mobile']
    legend_labels = ['Fixed', 'Mobile']

    # Sort and convert 'Transfer loss' to string
    df = df.sort_values(by='Transfer loss')
    df['Transfer loss'] = df['Transfer loss'].astype(str)

    # Splitting data into two parts based on 'Strategy' categories
    data_single_x = df[df['Strategy'] == 'fixed']
    data_multiple_x = df[df['Strategy'] == 'mobile']

    # Create boxplot for the category with multiple x values
    sns.boxplot(data=data_multiple_x, ax=ax1, x='Transfer loss', y='Work performed', hue='Strategy', hue_order=hue_order, palette='Set2')

    # Calculate and plot mean line for the category with single x value
    mean_value = data_single_x['Work performed'].mean()
    ax1.axhline(mean_value, color='#66c2a5', linestyle='-', label='fixed')

    # plot boxplot for 'Work performed'
    # sns.boxplot(data=df, ax=ax1, x='Transfer loss', y='Work performed', hue='Strategy', hue_order=hue_order, palette='Set2')

    # Calculate means by group
    filtered_df = df[df['Strategy'] == 'mobile']
    means = filtered_df.groupby(['Transfer loss', 'Strategy'])['Work performed'].mean().reset_index()
    # Overlay lines connecting means for data_multiple_x
    sns.lineplot(data=means, ax=ax1, x='Transfer loss', y='Work performed', markers=True, marker='D', dashes=False, color='#fc8d62', legend=False)

    # Set axis labels
    ax1.set_xlabel('Transfer loss')
    ax1.set_ylabel('Work performed')
    handles, labels = ax1.get_legend_handles_labels()
    ax1.legend(handles=handles, labels=legend_labels, title='Strategy', bbox_to_anchor=(0, 0), loc='lower left', ncol=1, frameon=True)    
    # ax1.set_title('Work performed')

    # lineplot

    # plot boxplot for 'Work Energy / Total energy'
    sns.boxplot(data=data_multiple_x, ax=ax2, x='Transfer loss', y='Work Energy / Total energy', hue='Strategy', hue_order=hue_order, palette='Set2')
    mean_value = data_single_x['Work Energy / Total energy'].mean()
    ax2.axhline(mean_value, color='#66c2a5', linestyle='-', label='fixed')
    means = filtered_df.groupby(['Transfer loss', 'Strategy'])['Work Energy / Total energy'].mean().reset_index()
    sns.lineplot(data=means, ax=ax2, x='Transfer loss', y='Work Energy / Total energy', markers=True, marker='D', dashes=False, color='#fc8d62', legend=False)
    # sns.boxplot(data=df, ax=ax2, x='Transfer loss', y='Work Energy / Total energy', hue='Strategy', hue_order=hue_order, palette='Set2')
    # means = df.groupby(['Transfer loss', 'Strategy'])['Work Energy / Total energy'].mean().reset_index()
    # sns.lineplot(data=means, ax=ax2, x='Transfer loss', y='Work Energy / Total energy', hue='Strategy', hue_order=hue_order, markers=True, dashes=False, palette='Set2', legend=False)
    ax2.set_xlabel('Transfer loss')
    ax2.set_ylabel('Energy efficiency')
    # ax2.legend(title='Strategy', bbox_to_anchor=([1.0, 1, 0, 0]), ncol=1, frameon=True)
    # add legend to the bottom right corner
    handles, labels = ax2.get_legend_handles_labels()
    ax2.legend(handles=handles, labels=legend_labels, title='Strategy',  bbox_to_anchor=(0, 0), loc='lower left', ncol=1, frameon=True)    
    # ax2.set_title('Energy efficiency')

    # plot boxplot for 'Total energy'
    sns.boxplot(data=df, ax=ax3, x='Transfer loss', y='Total energy', hue='Strategy', hue_order=hue_order, palette='Set2')
    means = df.groupby(['Transfer loss', 'Strategy'])['Total energy'].mean().reset_index()
    sns.lineplot(data=means, ax=ax3, x='Transfer loss', y='Total energy', hue='Strategy', hue_order=hue_order, markers=True, dashes=False, palette='Set2', legend=False)
    ax2.set_xlabel('Transfer loss')
    ax3.set_ylabel('Total energy')
    handles, labels = ax3.get_legend_handles_labels()
    ax3.legend(handles=handles, labels=legend_labels, title='Strategy',  bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)
    # ax3.set_title('Total energy')

    # plot boxplot for 'Workers depleted'
    sns.boxplot(data=df, ax=ax4, x='Transfer loss', y='Workers depleted', hue='Strategy', hue_order=hue_order, palette='Set2')
    means = df.groupby(['Transfer loss', 'Strategy'])['Workers depleted'].mean().reset_index()
    sns.lineplot(data=means, ax=ax4, x='Transfer loss', y='Workers depleted', hue='Strategy', hue_order=hue_order, markers=True, dashes=False, palette='Set2', legend=False)
    ax2.set_xlabel('Transfer loss')
    ax4.set_ylabel('Workers depleted')
    handles, labels = ax4.get_legend_handles_labels()
    ax4.legend(handles=handles, labels=legend_labels, title='Strategy',  bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)
    # ax4.set_title('Workers depleted')

    # plot boxplot for 'Chargers depleted'
    sns.boxplot(data=df, ax=ax5, x='Transfer loss', y='Chargers depleted', hue='Strategy', hue_order=hue_order, palette='Set2')
    means = df.groupby(['Transfer loss', 'Strategy'])['Chargers depleted'].mean().reset_index()
    sns.lineplot(data=means, ax=ax5, x='Transfer loss', y='Chargers depleted', hue='Strategy', hue_order=hue_order, markers=True, dashes=False, palette='Set2', legend=False)
    ax2.set_xlabel('Transfer loss')
    ax5.set_ylabel('Chargers depleted')
    # ax5.set_title('Chargers depleted')
    handles, labels = ax5.get_legend_handles_labels()
    ax5.legend(handles=handles, labels=legend_labels, title='Strategy',  bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)


    for ax in axes:
        # set font size
        for label in ax.get_xticklabels():
            label.set_fontproperties(font2)
        for label in ax.get_yticklabels():
            label.set_fontproperties(font2)
        
        # set font size of axis label
        ax.set_xlabel(ax.get_xlabel(), fontproperties=font)
        ax.set_ylabel(ax.get_ylabel(), fontproperties=font)

        # set font size of legend
        ax.get_legend().set_title(ax.get_legend().get_title().get_text(), prop=font2)
        for label in ax.get_legend().get_texts():
            label.set_fontproperties(font2)

        # set font size of title
        ax.set_title(ax.get_title(), fontproperties=font)

        # hide top and right spines
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)

        # rename legend labels
                

    fig1.tight_layout()
    fig2.tight_layout()
    fig3.tight_layout()
    fig4.tight_layout()
    fig5.tight_layout()

    fig1.savefig(join(RESULTS_DIR, f'work_transfer_loss_{chosen_num_chargers}C_{chosen_capacity}E_boxplot.pdf'))
    fig2.savefig(join(RESULTS_DIR, f'efficiency_transfer_loss_{chosen_num_chargers}C_{chosen_capacity}E_boxplot.pdf'))
    fig3.savefig(join(RESULTS_DIR, f'total_energy_transfer_loss_{chosen_num_chargers}C_{chosen_capacity}E_boxplot.pdf'))
    fig4.savefig(join(RESULTS_DIR, f'depleted_workers_transfer_loss_{chosen_num_chargers}C_{chosen_capacity}E_boxplot.pdf'))
    fig5.savefig(join(RESULTS_DIR, f'depleted_chargers_transfer_loss_{chosen_num_chargers}C_{chosen_capacity}E_boxplot.pdf'))


def plot_energy_heatmaps(chosen_work_rate, plot=False):

    # print('### START FUNC ###')

    # scnario_labels = ['travel', 'charger(9R1C)', 'charger(8R2C)', 'charger(7R3C)', 'charger(6R4C)', 'charger(5R5C)']
    # scnario_labels = ['travel', 'charger(9R1C)']
    scenario_labels = []

    df = pd.DataFrame({
        'Strategy':             pd.Series(dtype='str'),
        'Num Chargers':             pd.Series(dtype='int'),
        'Capacity':             pd.Series(dtype='str'),
        'Work performed':             pd.Series(dtype='int'), 
        'Total energy':       pd.Series(dtype='float'),
        'Work Energy': pd.Series(dtype='float'),
        'Work Energy / Total energy':  pd.Series(dtype='float'),
        'Workers depleted': pd.Series(dtype='int'),
        'Chargers depleted': pd.Series(dtype='int'),
    })

    count = 0

    # loop all directories in RESULTS_DIR
    for variation in [f for f in listdir(join(RESULTS_DIR, 'data')) if isdir(join(RESULTS_DIR, 'data', f))]:
        
        count += 1

        # Split variation by '_'
        var = variation.split('_')
        strategy = var[0]
        work_rate = var[1]
        num_chargers = var[2]
        capacity = var[3]
        # recharge_rate = var[4]
        # loss_rate = var[5]
        trial = var[4]

        # print(var)

        # if strategy == 'fixed':
        #     continue

        # convert work_rate to number and drop 'W'
        work_rate = float(work_rate[:-1])
        # print('work_rate', work_rate)

        # convert num_chargers to number and drop 'C'
        num_chargers = int(num_chargers[:-1])
        # print('num_chargers', num_chargers)

        # convert capacity to number and drop 'E'
        capacity = float(capacity[:-1])
        # print('capacity', capacity)

        # # convert recharge rate to number and drop 'R'
        # recharge_rate = float(recharge_rate[:-1])

        # # convert loss_rate to number and drop 'L'
        # loss_rate = float(loss_rate[:-1])

        # convert trial to number
        trial = int(trial[1:])
        # print('trial', trial)

        # scenario_type = var[1]
        # scenario_type = variation
        # scenario_labels.append(scenario_type)
        
        if work_rate == chosen_work_rate:

            trial_dirs = [f for f in listdir(join(RESULTS_DIR, 'data', variation)) if isdir(join(RESULTS_DIR, 'data', variation, f))]
            trial_dirs.sort()

            # # loop all directories in variation
            # for scenario in [f for f in trial_dirs if isdir(join(RESULTS_DIR, variation, f))]:

                # count += 1
                # if count > 10:
                #     break
                # print(scenario)
                
            # build path using RESULTS_DIR, variation and scenario
            path = join(RESULTS_DIR, 'data', variation)
            print(count, path)

            s, _ = load_log_with_checks(path, print_result=plot)

            # print('###### SCENARIO:', variation, '######')

            d = pd.DataFrame({
                'Strategy': [strategy],
                'Num Chargers': [num_chargers],
                'Capacity': [f'{float(capacity/100)}x'],
                'Work performed': [s.data[s.totalTime]['log'].points], 
                'Total energy':       [s.data[s.totalTime]['log'].totalEnergy],
                'Work Energy': [s.data[s.totalTime]['log'].workEnergy],
                'Work Energy / Total energy':  [s.data[s.totalTime]['log'].workEnergy / s.data[s.totalTime]['log'].totalEnergy],
                # 'Energy Shared': [s.data[s.totalTime]['log'].energyShared],
                'Workers depleted': [s.data[s.totalTime]['log'].workersDepleted],
                'Chargers depleted': [s.data[s.totalTime]['log'].chargersDepleted],
            })
            df = pd.concat([df, d], ignore_index=True, axis=0)

    print(df)

    fig1 = plt.figure(figsize=(6, 4))
    fig2 = plt.figure(figsize=(6, 4))
    fig3 = plt.figure(figsize=(6, 4))
    fig4 = plt.figure(figsize=(6, 4))
    fig5 = plt.figure(figsize=(6, 4))
    ax1 = fig1.gca()
    ax2 = fig2.gca()
    ax3 = fig3.gca()
    ax4 = fig4.gca()
    ax5 = fig5.gca()

    axes = [ax1, ax2, ax3, ax4, ax5]

    # Font
    font = FontProperties()
    # font.set_family('serif')
    # font.set_name('Times New Roman')
    font.set_size(14)

    font2 = FontProperties()
    # font2.set_family('serif')
    # font2.set_name('Times New Roman')
    font2.set_size(12)

    # Select only the columns of interest for aggregation
    agg_work_df = df[['Num Chargers', 'Capacity', 'Work performed']]
    agg_efficiency_df = df[['Num Chargers', 'Capacity', 'Work Energy / Total energy']]
    agg_energy_df = df[['Num Chargers', 'Capacity', 'Total energy']]
    agg_depleted_workers_df = df[['Num Chargers', 'Capacity', 'Workers depleted']]
    agg_depleted_chargers_df = df[['Num Chargers', 'Capacity', 'Chargers depleted']]

    # Aggregate duplicate entries (e.g., using mean)
    grouped_work_df = agg_work_df.groupby(['Num Chargers', 'Capacity']).mean().reset_index()
    grouped_efficiency_df = agg_efficiency_df.groupby(['Num Chargers', 'Capacity']).mean().reset_index()
    grouped_energy_df = agg_energy_df.groupby(['Num Chargers', 'Capacity']).mean().reset_index()
    grouped_depleted_workers_df = agg_depleted_workers_df.groupby(['Num Chargers', 'Capacity']).mean().reset_index()
    grouped_depleted_chargers_df = agg_depleted_chargers_df.groupby(['Num Chargers', 'Capacity']).mean().reset_index()

    # plot a heatmap where the x axis is the number of chargers, the y axis is the capacity and the value is the work performed
    pivot_work_df = grouped_work_df.pivot(index="Capacity", columns="Num Chargers", values="Work performed")
    pivot_efficiency_df = grouped_efficiency_df.pivot(index="Capacity", columns="Num Chargers", values="Work Energy / Total energy")
    pivot_energy_df = grouped_energy_df.pivot(index="Capacity", columns="Num Chargers", values="Total energy")
    pivot_depleted_workers_df = grouped_depleted_workers_df.pivot(index="Capacity", columns="Num Chargers", values="Workers depleted")
    pivot_depleted_chargers_df = grouped_depleted_chargers_df.pivot(index="Capacity", columns="Num Chargers", values="Chargers depleted")

    # Reverse the order of the index
    pivot_work_df = pivot_work_df.iloc[::-1]
    pivot_efficiency_df = pivot_efficiency_df.iloc[::-1]
    pivot_energy_df = pivot_energy_df.iloc[::-1]
    pivot_depleted_workers_df = pivot_depleted_workers_df.iloc[::-1]
    pivot_depleted_chargers_df = pivot_depleted_chargers_df.iloc[::-1]

    sns.heatmap(data=pivot_work_df, ax=ax1, annot=True, cmap="YlGnBu", fmt='.0f')
    # ax1.set_title('Work performed')
    ax1.set_xlabel('Number of chargers')
    ax1.set_ylabel('Charger capacity')
    old_y_labels = [label.get_text() for label in ax1.get_yticklabels()]
    # replace 'x' with '\times'
    modified_y_labels = [string.replace('x', r'$\times$') for string in old_y_labels]
    # new_y_labels = [r'{label}' + "_new" for label in modified_y_labels]
    ax1.set_yticklabels(modified_y_labels)
    # output pivot_work_df to csv
    pivot_work_df.to_csv(join(RESULTS_DIR, f'work_heatmap.csv'))

    sns.heatmap(data=pivot_efficiency_df, ax=ax2, annot=True, cmap="YlGnBu", fmt='.3f')
    # ax2.set_title('Energy efficiency')
    ax2.set_xlabel('Number of chargers')
    ax2.set_ylabel('Charger capacity')
    ax2.set_yticklabels(modified_y_labels)
    pivot_efficiency_df.to_csv(join(RESULTS_DIR, f'efficiency_heatmap.csv'))

    sns.heatmap(data=pivot_energy_df, ax=ax3, annot=True, cmap="YlGnBu", fmt='.0f')
    # ax3.set_title('Total energy Consumed')
    ax3.set_xlabel('Number of chargers')
    ax3.set_ylabel('Charger capacity')
    ax3.set_yticklabels(modified_y_labels)
    pivot_energy_df.to_csv(join(RESULTS_DIR, f'total_energy_heatmap.csv'))

    sns.heatmap(data=pivot_depleted_workers_df, ax=ax4, annot=True, cmap="YlGnBu", fmt='.1f')
    # ax4.set_title('Number of Workers depleted')
    ax4.set_xlabel('Number of chargers')
    ax4.set_ylabel('Charger capacity')
    ax4.set_yticklabels(modified_y_labels)
    pivot_depleted_workers_df.to_csv(join(RESULTS_DIR, f'depleted_workers_heatmap.csv'))

    sns.heatmap(data=pivot_depleted_chargers_df, ax=ax5, annot=True, cmap="YlGnBu", fmt='.1f')
    # ax5.set_title('Number of chargers depleted')
    ax5.set_xlabel('Number of chargers')
    ax5.set_ylabel('Charger capacity')
    ax5.set_yticklabels(modified_y_labels)
    pivot_depleted_chargers_df.to_csv(join(RESULTS_DIR, f'depleted_chargers_heatmap.csv'))

    for ax in axes:
        # set font size
        for label in ax.get_xticklabels():
            label.set_fontproperties(font2)
        for label in ax.get_yticklabels():
            label.set_fontproperties(font2)
        
        # set font size of axis label
        ax.set_xlabel(ax.get_xlabel(), fontproperties=font)
        ax.set_ylabel(ax.get_ylabel(), fontproperties=font)

        # set font size of title
        ax.set_title(ax.get_title(), fontproperties=font)

        # set font size of value in cell
        for text in ax.texts:
            text.set_fontsize(8)
        
        # hide top and right spines
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)  
        ax.spines['bottom'].set_visible(True)
        ax.spines['left'].set_visible(True)
        ax.spines['bottom'].set_linewidth(1)
        ax.spines['left'].set_linewidth(1)      

    fig1.tight_layout()
    fig2.tight_layout()
    fig3.tight_layout()
    fig4.tight_layout()
    fig5.tight_layout()

    fig1.savefig(join(RESULTS_DIR, f'work_heatmap.pdf'))
    fig2.savefig(join(RESULTS_DIR, f'efficiency_heatmap.pdf'))
    fig3.savefig(join(RESULTS_DIR, f'total_energy_heatmap.pdf'))
    fig4.savefig(join(RESULTS_DIR, f'depleted_workers_heatmap.pdf'))
    fig5.savefig(join(RESULTS_DIR, f'depleted_chargers_heatmap.pdf'))


def plot_connector_energy_over_time_single_trial(path, plot=False):

    # print('### START FUNC ###')

    scnario_labels = ['travel', 'charger(9R1C)', 'charger(8R2C)', 'charger(7R3C)', 'charger(6R4C)', 'charger(5R5C)']
    # scnario_labels = ['travel', 'charger(9R1C)']

    df = pd.DataFrame({
        'Time':             pd.Series(dtype='int'),
        'Average Energy':             pd.Series(dtype='float'), 
        'Minimum Energy':       pd.Series(dtype='float'),
        'Maximum Energy':  pd.Series(dtype='float'),
    })

    # # loop all directories in RESULTS_DIR
    # for variation in [f for f in listdir(RESULTS_DIR) if isdir(join(RESULTS_DIR, f))]:
        
    #     # Split variation by '_'
    #     var = variation.split('_')

    #     if len(var) > 1:
            
    #         # if var[-1] != '30min':
    #         #     continue

    #         scenario_type = var[1]
            
    #         count = 0

    #         # sort listdir(join(RESULTS_DIR, variation)
    #         trial_dirs = [f for f in listdir(join(RESULTS_DIR, variation)) if isdir(join(RESULTS_DIR, variation, f))]
    #         trial_dirs.sort()

    # # loop all directories in variation
    # for scenario in [f for f in trial_dirs if isdir(join(RESULTS_DIR, variation, f))]:

    #     count += 1
    #     if count > 10:
    #         break
    #     print(scenario)

    # build path using RESULTS_DIR, variation and scenario
    # path = join(RESULTS_DIR, variation, scenario)
    print(path)

    s, _, _, _ = load_log_with_checks(path, print_result=plot)
    
    # Loop every time step
    for time in range(1, s.totalTime+1):

        if(time % 10 != 0):
            # print(time)
            continue

        connector_energies = []

        # Init dict for every robot
        for robot in s.data[time]['log'].robots:
            if robot.state is time_step_pb2.Robot.State.CONNECTOR:
                connector_energies.append(robot.energyLevel)

        # print('time', time, 'connectors', len(connector_energies))

        # average energy
        if time < 6: # before init
            avg_energy = 100
            min_energy = avg_energy
            max_energy = avg_energy
        else:
            avg_energy = sum(connector_energies) / len(connector_energies)
            min_energy = min(connector_energies)
            max_energy = max(connector_energies)                

        # print('###### SCENARIO:', scenario, '######')


        if time == 10:
            d = pd.DataFrame({
                'Time': [0],
                'Average Energy': [100], 
                'Minimum Energy':       [100],
                'Maximum Energy':  [100],
            })
            df = pd.concat([df, d], ignore_index=True, axis=0)

        d = pd.DataFrame({
            'Time': [time/10],
            'Average Energy': [avg_energy], 
            'Minimum Energy':       [min_energy],
            'Maximum Energy':  [max_energy],
        })
        df = pd.concat([df, d], ignore_index=True, axis=0)

        # print()
    # exit(0)
                    
    # Font
    font = FontProperties()
    # font.set_family('serif')
    # font.set_name('Times New Roman')
    font.set_size(14)

    font2 = FontProperties()
    # font2.set_family('serif')
    # font2.set_name('Times New Roman')
    font2.set_size(12)


    # plot line plot using seaborn, where the x axis is the time and the y axis is the energy level.
    # Make the min and max energy levels as the confidence interval
    fig1 = plt.figure(figsize=(5, 4))
    axes1 = fig1.gca()

    print(df)

    sns.lineplot(data=df, ax=axes1, x='Time', y='Average Energy', color='green', label='Average Energy')
    sns.lineplot(data=df, ax=axes1, x='Time', y='Minimum Energy', color='orange', label='Minimum Energy')
    sns.lineplot(data=df, ax=axes1, x='Time', y='Maximum Energy', color='blue', label='Maximum Energy')

    # make legend for the plot
    # axes1.legend([str(int(label)) for label in hue_order], bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)
    
    # add legend
    # handles, labels = axes1.get_legend_handles_labels()
    # axes1.legend(handles=handles, labels=['Average Energy', 'Minimum Energy', 'Maximum Energy'], title='Energy Level')

    # legend
    # legend_labels1 = ['w = 6', 'w = 6 (ESMT)', 'w = 12', 'w = 12 (ESMT)', 'w = 18', 'w = 18 (ESMT)', 'w = 24', 'w = 24 (ESMT)']
    # legend_labels2 = ['t = 2', 't = 2 (ESMT)', 't = 3', 't = 3 (ESMT)', 't = 4', 't = 4 (ESMT)', 't = 5', 't = 5 (ESMT)', 't = 6', 't = 6 (ESMT)']

    # axes1.legend(legend_labels1, bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)

    axes1.legend()

    # change the x axis label
    axes1.set_ylabel('Energy (%)', fontproperties=font)
    axes1.set_xlabel('Time (s)', fontproperties=font)
    

    axes1.set_ylim([0,100+3])

    fig1.tight_layout()

    fig1.savefig(join(RESULTS_DIR, f'connector_energy.pdf'))


def plot_connector_energy_over_time(plot=False):

    # print('### START FUNC ###')

    scnario_labels = ['travel', 'charger(9R1C)', 'charger(8R2C)', 'charger(7R3C)', 'charger(6R4C)', 'charger(5R5C)', 'charger(8R1C)', 'charger(7R1C)']
    # scnario_labels = ['travel', 'charger(9R1C)']

    # loop all directories in RESULTS_DIR
    for variation in [f for f in listdir(RESULTS_DIR) if isdir(join(RESULTS_DIR, f))]:
        
        # Split variation by '_'
        var = variation.split('_')

        if len(var) > 1:
            
            df = pd.DataFrame({
                'Time':             pd.Series(dtype='int'),
                'Average Energy':             pd.Series(dtype='float'), 
                'Minimum Energy':       pd.Series(dtype='float'),
                'Maximum Energy':  pd.Series(dtype='float'),
            })

            # if var[-1] != 'travel':
            #     continue

            scenario_type = var[1]
            
            count = 0

            # sort listdir(join(RESULTS_DIR, variation)
            trial_dirs = [f for f in listdir(join(RESULTS_DIR, variation)) if isdir(join(RESULTS_DIR, variation, f))]
            trial_dirs.sort()

            # loop all directories in variation
            for scenario in [f for f in trial_dirs if isdir(join(RESULTS_DIR, variation, f))]:

                if scenario[0] == 'X' :
                    print('skipping', scenario)
                    continue

                count += 1
                if count > 10:
                    break
                print(scenario)

                # build path using RESULTS_DIR, variation and scenario
                path = join(RESULTS_DIR, variation, scenario)
                print(path)

                s, _, _, _ = load_log_with_checks(path, print_result=plot)
                
                # Loop every time step
                for time in range(1, s.totalTime+1):

                    if(time % 100 != 0):
                        # print(time)
                        continue

                    connector_energies = []

                    # Init dict for every robot
                    for robot in s.data[time]['log'].robots:
                        if robot.state is time_step_pb2.Robot.State.CONNECTOR:
                            connector_energies.append(robot.energyLevel)

                    # print('time', time, 'connectors', len(connector_energies))

                    # average energy
                    if time < 6: # before init
                        avg_energy = 100
                        min_energy = avg_energy
                        max_energy = avg_energy
                    else:
                        avg_energy = sum(connector_energies) / len(connector_energies)
                        min_energy = min(connector_energies)
                        max_energy = max(connector_energies)                

                    # print('###### SCENARIO:', scenario, '######')

                    if time == 100:
                        d = pd.DataFrame({
                            'Time': [0],
                            'Average Energy': [100], 
                            'Minimum Energy':       [100],
                            'Maximum Energy':  [100],
                        })
                        df = pd.concat([df, d], ignore_index=True, axis=0)

                    d = pd.DataFrame({
                        'Time': [time/10],
                        'Average Energy': [avg_energy], 
                        'Minimum Energy':       [min_energy],
                        'Maximum Energy':  [max_energy],
                    })
                    df = pd.concat([df, d], ignore_index=True, axis=0)

                    # print()

                print('Number of connectors flat battery:', s[s.totalTime]['log'].connectorsDepleted)

            # Font
            font = FontProperties()
            # font.set_family('serif')
            # font.set_name('Times New Roman')
            font.set_size(14)

            font2 = FontProperties()
            # font2.set_family('serif')
            # font2.set_name('Times New Roman')
            font2.set_size(12)

            # plot line plot using seaborn, where the x axis is the time and the y axis is the energy level.
            # Make the min and max energy levels as the confidence interval
            fig1 = plt.figure(figsize=(5, 4))
            axes1 = fig1.gca()

            print(df)

            sns.lineplot(data=df, ax=axes1, x='Time', y='Average Energy', color='green', label='Average Energy')
            sns.lineplot(data=df, ax=axes1, x='Time', y='Minimum Energy', color='orange', label='Minimum Energy')
            sns.lineplot(data=df, ax=axes1, x='Time', y='Maximum Energy', color='blue', label='Maximum Energy')

            # make legend for the plot
            # axes1.legend([str(int(label)) for label in hue_order], bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)
            
            # add legend
            # handles, labels = axes1.get_legend_handles_labels()
            # axes1.legend(handles=handles, labels=['Average Energy', 'Minimum Energy', 'Maximum Energy'], title='Energy Level')

            # legend
            # legend_labels1 = ['w = 6', 'w = 6 (ESMT)', 'w = 12', 'w = 12 (ESMT)', 'w = 18', 'w = 18 (ESMT)', 'w = 24', 'w = 24 (ESMT)']
            # legend_labels2 = ['t = 2', 't = 2 (ESMT)', 't = 3', 't = 3 (ESMT)', 't = 4', 't = 4 (ESMT)', 't = 5', 't = 5 (ESMT)', 't = 6', 't = 6 (ESMT)']

            # axes1.legend(legend_labels1, bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)

            axes1.legend()
                        # place the legend on the top of the plot and place each item horizontally
            # legend = axes1.legend(loc='upper center', bbox_to_anchor=(0.5, 1.15), ncol=3)

            # change the x axis label
            axes1.set_ylabel('Energy (%)', fontproperties=font)
            axes1.set_xlabel('Time (s)', fontproperties=font)
            
            axes1.set_ylim([0,100+3])

            fig1.tight_layout()

            fig1.savefig(join(RESULTS_DIR, f'connector_energy_{scenario_type}.pdf'))
            

def get_traveler_info(s):
    """Find the number of travelers, the average and minimum speed of travelers in each timestep"""

    data = {}

    prev_traveler_pos = {}
    traveler_time = {}

    sim_step = 0.1
    ex_timestep = 1 # Duration between timesteps being read (s)
    total_dist = 0

    for t in range(1,s.totalTime+1,int(ex_timestep/sim_step)):
        
        d = {}
        d['num_traveler'] = 0
        d['avg_speed'] = None
        d['min_speed'] = None
        t_dist = 0
        min_dist = None

        for robot in s[t]['log'].robots:
            if robot.state == time_step_pb2.Robot.TRAVELER:
                d['num_traveler'] += 1
                current_pos = (robot.position.x, robot.position.y)

                # Calculate the distance traveled from the previous timestep
                if robot.name in prev_traveler_pos:
                    prev_pos = (prev_traveler_pos[robot.name][0], prev_traveler_pos[robot.name][1])
                    dist = math.dist(prev_pos, current_pos)
                    t_dist += dist

                    if min_dist == None or dist < min_dist:
                        min_dist = dist

                # Add/Update traveler position
                prev_traveler_pos[robot.name] = current_pos

                # Add/Update time as traveler
                if robot.name in traveler_time:
                    traveler_time[robot.name] += ex_timestep/sim_step
                else:
                    traveler_time[robot.name] = 0

        # Calculate the average speed of travelers during this timestep
        if d['num_traveler'] > 0:
            d['avg_speed'] = t_dist / ex_timestep / d['num_traveler']
        if min_dist != None:
            d['min_speed'] = min_dist / ex_timestep

        total_dist += t_dist
        d['total_dist'] = total_dist

        data[t] = d

    # If the final timestep is not added, add it
    if not s.totalTime in data:
        t = s.totalTime
        d = {}
        d['num_traveler'] = 0
        d['avg_speed'] = None
        d['min_speed'] = None
        t_dist = 0
        min_dist = None

        for robot in s[t]['log'].robots:
            if robot.state == time_step_pb2.Robot.TRAVELER:
                d['num_traveler'] += 1
                current_pos = (robot.position.x, robot.position.y)

                # Calculate the distance traveled from the previous timestep
                if robot.name in prev_traveler_pos:
                    prev_pos = (prev_traveler_pos[robot.name][0], prev_traveler_pos[robot.name][1])
                    dist = math.dist(prev_pos, current_pos)
                    t_dist += dist

                    if min_dist == None or dist < min_dist:
                        min_dist = dist

                # Add/Update traveler position
                prev_traveler_pos[robot.name] = current_pos

                # Add/Update time as traveler
                if robot.name in traveler_time:
                    traveler_time[robot.name] += t - max(list(data.keys()))
                else:
                    traveler_time[robot.name] = 0

        # Calculate the average speed of travelers during this timestep
        if d['num_traveler'] > 0:
            d['avg_speed'] = t_dist / ex_timestep / d['num_traveler']
        if min_dist != None:
            d['min_speed'] = min_dist / ex_timestep

        total_dist += t_dist
        d['total_dist'] = total_dist
        
        data[t] = d

    # Average duration a robot spent as a traveler (for those which did travel)
    total_travel_time = sum(list(traveler_time.values()))
    avg_travel_time = total_travel_time / len(traveler_time)

    return data, avg_travel_time

def plot_ex0_scatter():
    
    df = pd.DataFrame({
        'Teams':                            pd.Series(dtype='int'), 
        'Workers Per Team':                  pd.Series(dtype='int'),
        'Network Length':                   pd.Series(dtype='float'),
        'Network Length (ESMT)':   pd.Series(dtype='float'),
        # 'Network Length (2tRNP)':           pd.Series(dtype='float'),
        'Connectors':                       pd.Series(dtype='int'),
        'Connectors (ESMT)':   pd.Series(dtype='float'),
        # 'Steiner Points (2tRNP)':           pd.Series(dtype='int'),
        # 'Total Team Length':                pd.Series(dtype='float'),
        # 'Total Team Length (CDWX-2008)':    pd.Series(dtype='float'),       
        # 'Total Team Length (CDWX-2008-EX)': pd.Series(dtype='float'),
        # 'Total Team Length (2tRNP)':        pd.Series(dtype='float')       
    })
    
    # Get trial names
    team_num_dirs = [f for f in listdir(RESULTS_DIR) if isdir(join(RESULTS_DIR, f))]
    team_num_dirs.sort()

    for team_num in team_num_dirs:

        trial_dirs = [f for f in listdir(join(RESULTS_DIR, team_num)) if isdir(join(RESULTS_DIR, team_num, f))]
        trial_dirs.sort()

        trial_no_points = {}
        trial_no_connectivity = {}
        trial_no_team_connectivity = {}

        start_time = time.time()

        count = 0
        # for scenario in trial_dirs:
        while count < len(trial_dirs):
            # if count >= len(trial_dirs):
            #     break

            scenario = trial_dirs[count]

            start_time_single = time.time()

            # Check if the trial was successful
            s, final_points, final_connectivity, final_team_connectivity = load_log_with_checks(join(RESULTS_DIR, team_num, scenario))
            if final_points == 0:
                trial_no_points[scenario] = {
                    'seed': s.seed,
                    'data': final_points
                }

            if final_team_connectivity:
                trial_no_team_connectivity[scenario] = {
                    'seed': s.seed,
                    'data': final_team_connectivity
                }

            # if final_points > 0 and final_connectivity:

            # Find the total network length and connectors for the result obtained and the solver output
            graphs, lengths, connectors = get_network_length_and_connectors(s, plot=False)
            if not nx.is_connected(graphs['res']):
                trial_no_connectivity[scenario] = {
                    'seed': s.seed,
                    'data': False
                }

            # Find the total distance between every team along the network
            # get_team_distance(graphs['res'])

            res_path_length = get_team_distance(graphs["res"])
            esmt_path_length = get_team_distance(graphs["esmt"])
            # ttrnp_path_length = get_team_distance(graphs["2trnp"])

            # Add data of the successful trial
            if not scenario in trial_no_points and not scenario in trial_no_connectivity:

                d = pd.DataFrame({
                    'Teams': [s.numLeaders], 
                    'Workers Per Team': [(int)(s.numWorkers/s.numLeaders)], 
                    'Network Length': [lengths['res']], 
                    'Network Length (ESMT)': [lengths['esmt']],
                    # 'Network Length (2tRNP)': [lengths['2trnp']],
                    'Connectors': [connectors['res']],
                    'Connectors (ESMT)': [connectors['esmt']],
                    # 'Connectors (2tRNP)': [connectors['2trnp']],
                    'Total Team Length': [res_path_length],
                    'Total Team Length (ESMT)': [esmt_path_length],
                    # 'Total Team Length (2tRNP)': [ttrnp_path_length]  
                })
                df = pd.concat([df, d], ignore_index=True, axis=0)
            else:
                print('skipping...')

            duration_single = round(time.time() - start_time_single, 3)
            duration_total = round(time.time() - start_time, 3)
            print("Loaded -- '{0}' --\tin {1} s ({2} s)".format(scenario, duration_single, duration_total))

            # DEBUG: For limiting the number of data to plot
            count += 1
            # num_to_use = 50
            # if (count % 10) % num_to_use == 0:
            #     count += 50 - num_to_use
            #     # print(count)
            #     # break

        duration = round(time.time() - start_time, 3)
        print('Finished in {0} seconds'.format(duration))

        print(f'### No points scored ###')
        for scenario, trial in trial_no_points.items():
            print(f'      scenario: {scenario}, seed: {trial["seed"]}, points: {trial["data"]}')
        print(f'    Trials with no points scored: ({len(trial_no_points)}/{len(trial_dirs)})')

        print(f'### Lost global connectivity ###')
        for scenario, trial in trial_no_connectivity.items():
            print(f'      scenario: {scenario}, seed: {trial["seed"]}, connectivity: {trial["data"]}')
        print(f'    Trials that lost global connectivity: ({len(trial_no_connectivity)}/{len(trial_dirs)})')

        print(f'### Lost team connectivity ###')
        for scenario, trial in trial_no_team_connectivity.items():
            print(f'      scenario: {scenario}, seed: {trial["seed"]}, followers lost: {len(trial["data"])}')
        print(f'    Trials that lost team connectivity: ({len(trial_no_team_connectivity)}/{len(trial_dirs)})')


    # Plot data
    fig1 = plt.figure(figsize=(6, 6))
    axes1 = fig1.gca()
    fig2 = plt.figure(figsize=(6, 6))
    axes2 = fig2.gca()

    axes = [axes1, axes2]

    order_num_team = list(df['Teams'].unique())
    order_num_team.sort()
    order_team_size = list(df['Workers Per Team'].unique())
    order_team_size.sort()

    ### SCATTER PLOT
    x = list(range(16))
    axes1.plot(x,x, color='black', linestyle='dashed')
    frequency = df.groupby(['Connectors (ESMT)', 'Connectors']).size().reset_index(name='Frequency')
    sns.scatterplot(
        data=frequency, ax=axes1,
        x='Connectors (ESMT)', y='Connectors', hue='Frequency', size='Frequency', sizes=(250,1000),
        # edgecolor='gray',
        legend=False
    )
    
    for i, row in frequency.iterrows():
        count = int(row['Frequency'])
        if count > 10:
            color = 'white'
        else:
            color = 'black'
        axes1.text(row['Connectors (ESMT)'], row['Connectors'], str(count), ha='center', va='center', color=color)

    axes2.plot(x,x, color='black', linestyle='dashed')
    sns.scatterplot(
        data=df, ax=axes2,
        x='Network Length (ESMT)', y='Network Length'
    )

    # Font
    font = FontProperties()
    # font.set_family('serif')
    # font.set_name('Times New Roman')
    font.set_size(14)

    font2 = FontProperties()
    # font2.set_family('serif')
    # font2.set_name('Times New Roman')
    font2.set_size(12)

    # Set axis labels
    # axes.set_xlabel("Average waiting time per task (s)")
    axes1.set_xlabel("added points: ESMT, n", fontproperties=font)
    axes1.set_ylabel("connectors: simulation, n", fontproperties=font)
    axes2.set_xlabel("network length: ESMT (m)", fontproperties=font)
    axes2.set_ylabel("network length: simulation (m)", fontproperties=font)

    # for key, ax in axd9.items():
    #     if key != 'r':
    #         ax.set_xlabel("Teams", fontproperties=font)
    #         ax.set_ylabel("Workers Per Team", fontproperties=font)

    #     for label in ax.get_xticklabels():
    #         label.set_fontproperties(font2)
    #     for label in ax.get_yticklabels():
    #         label.set_fontproperties(font2)

    for ax in axes:
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.spines['left'].set_visible(True)
        ax.spines['bottom'].set_visible(True)
        ax.spines['left'].set_linewidth(1)
        ax.spines['bottom'].set_linewidth(1)
        ax.tick_params(width=1)

        for label in ax.get_xticklabels():
            label.set_fontproperties(font2)
        for label in ax.get_yticklabels():
            label.set_fontproperties(font2)



    # border and ticks
    max_val = max(df['Connectors'].max(),df['Connectors (ESMT)'].max())
    axes1.set_xticks(np.arange(0, max_val+2, 1))
    axes1.set_yticks(np.arange(0, max_val+2, 1))
    axes1.set_xlim([0,max_val+2])
    axes1.set_ylim([0,max_val+2])

    max_val = max(df['Network Length'].max(),df['Network Length (ESMT)'].max())
    axes2.set_xticks(np.arange(0, max_val+2, 1))
    axes2.set_yticks(np.arange(0, max_val+2, 1))
    axes2.set_xlim([0,max_val+2])
    axes2.set_ylim([0,max_val+2])

    print(df.to_string())
    print(f'Mean Connectors: {df["Connectors"].mean()}')
    print(f'Mean Connectors (ESMT): {df["Connectors (ESMT)"].mean()}')
    print(f'Mean Network Length: {df["Network Length"].mean()}')
    print(f'Mean Network Length (ESMT): {df["Network Length (ESMT)"].mean()}')

    # legend
    # legend_labels1 = ['w = 6', 'w = 6 (2tRNP)', 'w = 12', 'w = 12 (2tRNP)', 'w = 18', 'w = 18 (2tRNP)', 'w = 24', 'w = 24 (2tRNP)']
    # legend_labels2 = ['t = 2', 't = 2 (2tRNP)', 't = 3', 't = 3 (2tRNP)', 't = 4', 't = 4 (2tRNP)', 't = 5', 't = 5 (2tRNP)', 't = 6', 't = 6 (2tRNP)']

    # axes1.legend(legend_labels1, bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)
    # axes2.legend(legend_labels2, bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)

    # set font properties of colorbar label
    # cbar7 = axes7.collections[0].colorbar
    # cbar7.set_label("Ratio", fontdict={'size': 14})
    # cbar7.ax.tick_params(labelsize=14)
    # cbar8 = axes8.collections[0].colorbar
    # cbar8.set_label("Ratio", fontdict={'size': 14})
    # cbar8.ax.tick_params(labelsize=14)

    # cbar8 = axd9['r'].collections[0].colorbar
    # axd9['r'].set_ylabel("Ratio", fontdict={'size': 14})
    # axd9['r'].tick_params(labelsize=14)

    # for key, ax in axd9.items():
    #     if key == 'r':
    #         ax.spines['top'].set_visible(True)
    #         ax.spines['right'].set_visible(True)
    #     else:
    #         ax.spines['top'].set_visible(False)
    #         ax.spines['right'].set_visible(False)
    #     ax.spines['left'].set_visible(True)
    #     ax.spines['bottom'].set_visible(True)
    #     ax.spines['left'].set_linewidth(1)
    #     ax.spines['bottom'].set_linewidth(1)
    #     ax.tick_params(width=1)

    fig1.tight_layout()
    fig2.tight_layout()
    # axes1.axis('equal')
    # axes2.axis('equal')

    # plt.show()
    fig1.savefig(join(RESULTS_DIR, 'connectors_ex0.pdf'))
    fig2.savefig(join(RESULTS_DIR, 'network_length_ex0.pdf'))  


def plot_scatter_real_robots():
    
    df = pd.DataFrame({
        'Teams':                            pd.Series(dtype='int'), 
        'Workers Per Team':                  pd.Series(dtype='int'),
        'Network Length':                   pd.Series(dtype='float'),
        'Network Length (ESMT)':   pd.Series(dtype='float'),
        # 'Network Length (2tRNP)':           pd.Series(dtype='float'),
        'Connectors':                       pd.Series(dtype='int'),
        'Connectors (ESMT)':   pd.Series(dtype='float'),
        # 'Steiner Points (2tRNP)':           pd.Series(dtype='int'),
        # 'Total Team Length':                pd.Series(dtype='float'),
        # 'Total Team Length (CDWX-2008)':    pd.Series(dtype='float'),       
        # 'Total Team Length (CDWX-2008-EX)': pd.Series(dtype='float'),
        # 'Total Team Length (2tRNP)':        pd.Series(dtype='float')       
    })
    
    # Get trial names
    team_num_dirs = [f for f in listdir(RESULTS_DIR) if isdir(join(RESULTS_DIR, f))]
    team_num_dirs.sort()

    for team_num in team_num_dirs:

        trial_dirs = [f for f in listdir(join(RESULTS_DIR, team_num)) if isdir(join(RESULTS_DIR, team_num, f))]
        trial_dirs.sort()

        trial_no_points = {}
        trial_no_connectivity = {}
        trial_no_team_connectivity = {}

        start_time = time.time()

        count = 0
        # for scenario in trial_dirs:
        while count < len(trial_dirs):
            # if count >= len(trial_dirs):
            #     break

            scenario = trial_dirs[count]

            start_time_single = time.time()

            # Check if the trial was successful
            s, final_points, final_connectivity, final_team_connectivity = load_log_with_checks(join(RESULTS_DIR, team_num, scenario))
            if final_points == 0:
                trial_no_points[scenario] = {
                    'seed': s.seed,
                    'data': final_points
                }

            if final_team_connectivity:
                trial_no_team_connectivity[scenario] = {
                    'seed': s.seed,
                    'data': final_team_connectivity
                }

            # if final_points > 0 and final_connectivity:

            # Find the total network length and connectors for the result obtained and the solver output
            graphs, lengths, connectors = get_network_length_and_connectors(s, plot=False)
            if not nx.is_connected(graphs['res']):
                trial_no_connectivity[scenario] = {
                    'seed': s.seed,
                    'data': False
                }

            # Find the total distance between every team along the network
            # get_team_distance(graphs['res'])

            res_path_length = get_team_distance(graphs["res"])
            esmt_path_length = get_team_distance(graphs["esmt"])
            # ttrnp_path_length = get_team_distance(graphs["2trnp"])

            # Add data of the successful trial
            if not scenario in trial_no_points and not scenario in trial_no_connectivity:

                d = pd.DataFrame({
                    'Teams': [s.numLeaders], 
                    'Workers Per Team': [(int)(s.numWorkers/s.numLeaders)], 
                    'Network Length': [lengths['res']], 
                    'Network Length (ESMT)': [lengths['esmt']],
                    # 'Network Length (2tRNP)': [lengths['2trnp']],
                    'Connectors': [connectors['res']],
                    'Connectors (ESMT)': [connectors['esmt']],
                    # 'Connectors (2tRNP)': [connectors['2trnp']],
                    'Total Team Length': [res_path_length],
                    'Total Team Length (ESMT)': [esmt_path_length],
                    # 'Total Team Length (2tRNP)': [ttrnp_path_length]  
                })
                df = pd.concat([df, d], ignore_index=True, axis=0)
            else:
                print('skipping...')

            duration_single = round(time.time() - start_time_single, 3)
            duration_total = round(time.time() - start_time, 3)
            print("Loaded -- '{0}' --\tin {1} s ({2} s)".format(scenario, duration_single, duration_total))

            # DEBUG: For limiting the number of data to plot
            count += 1
            # num_to_use = 50
            # if (count % 10) % num_to_use == 0:
            #     count += 50 - num_to_use
            #     # print(count)
            #     # break

        duration = round(time.time() - start_time, 3)
        print('Finished in {0} seconds'.format(duration))

        print(f'### No points scored ###')
        for scenario, trial in trial_no_points.items():
            print(f'      scenario: {scenario}, seed: {trial["seed"]}, points: {trial["data"]}')
        print(f'    Trials with no points scored: ({len(trial_no_points)}/{len(trial_dirs)})')

        print(f'### Lost global connectivity ###')
        for scenario, trial in trial_no_connectivity.items():
            print(f'      scenario: {scenario}, seed: {trial["seed"]}, connectivity: {trial["data"]}')
        print(f'    Trials that lost global connectivity: ({len(trial_no_connectivity)}/{len(trial_dirs)})')

        print(f'### Lost team connectivity ###')
        for scenario, trial in trial_no_team_connectivity.items():
            print(f'      scenario: {scenario}, seed: {trial["seed"]}, followers lost: {len(trial["data"])}')
        print(f'    Trials that lost team connectivity: ({len(trial_no_team_connectivity)}/{len(trial_dirs)})')


    # Plot data
    fig1 = plt.figure(figsize=(6, 6))
    axes1 = fig1.gca()
    fig2 = plt.figure(figsize=(6, 6))
    axes2 = fig2.gca()

    axes = [axes1, axes2]

    order_num_team = list(df['Teams'].unique())
    order_num_team.sort()
    order_team_size = list(df['Workers Per Team'].unique())
    order_team_size.sort()

    ### SCATTER PLOT
    x = list(range(16))
    axes1.plot(x,x, color='black', linestyle='dashed')
    frequency = df.groupby(['Connectors (ESMT)', 'Connectors']).size().reset_index(name='Frequency')
    sns.scatterplot(
        data=frequency, ax=axes1,
        x='Connectors (ESMT)', y='Connectors', hue='Frequency', size='Frequency', sizes=(1000,2000),
        # edgecolor='gray',
        legend=False
    )
    
    for i, row in frequency.iterrows():
        count = int(row['Frequency'])
        if count > 3:
            color = 'white'
        else:
            color = 'black'
        axes1.text(row['Connectors (ESMT)'], row['Connectors'], str(count), ha='center', va='center', color=color)

    axes2.plot(x,x, color='black', linestyle='dashed')
    sns.scatterplot(
        data=df, ax=axes2, s=100,
        x='Network Length (ESMT)', y='Network Length'
    )

    # Font
    font = FontProperties()
    # font.set_family('serif')
    # font.set_name('Times New Roman')
    font.set_size(16)

    font2 = FontProperties()
    # font2.set_family('serif')
    # font2.set_name('Times New Roman')
    font2.set_size(14)

    # Set axis labels
    # axes.set_xlabel("Average waiting time per task (s)")
    axes1.set_xlabel("added points: ESMT, n", fontproperties=font)
    axes1.set_ylabel("connectors: experiment, n", fontproperties=font)
    axes2.set_xlabel("network length: ESMT (m)", fontproperties=font)
    axes2.set_ylabel("network length: experiment (m)", fontproperties=font)

    # for key, ax in axd9.items():
    #     if key != 'r':
    #         ax.set_xlabel("Teams", fontproperties=font)
    #         ax.set_ylabel("Workers Per Team", fontproperties=font)

    #     for label in ax.get_xticklabels():
    #         label.set_fontproperties(font2)
    #     for label in ax.get_yticklabels():
    #         label.set_fontproperties(font2)

    for ax in axes:
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.spines['left'].set_visible(True)
        ax.spines['bottom'].set_visible(True)
        ax.spines['left'].set_linewidth(1)
        ax.spines['bottom'].set_linewidth(1)
        ax.tick_params(width=1)

        for label in ax.get_xticklabels():
            label.set_fontproperties(font2)
        for label in ax.get_yticklabels():
            label.set_fontproperties(font2)



    # border and ticks
    max_val = max(df['Connectors'].max(),df['Connectors (ESMT)'].max())
    axes1.set_xticks(np.arange(0, max_val+2, 1))
    axes1.set_yticks(np.arange(0, max_val+2, 1))
    axes1.set_xlim([0,max_val+1])
    axes1.set_ylim([0,max_val+1])

    max_val = max(df['Network Length'].max(),df['Network Length (ESMT)'].max())
    axes2.set_xticks(np.arange(0, max_val+2, 0.5))
    axes2.set_yticks(np.arange(0, max_val+2, 0.5))
    axes2.set_xlim([0,max_val+0.5])
    axes2.set_ylim([0,max_val+0.5])

    print(df.to_string())
    print(f'Mean Connectors: {df["Connectors"].mean()}')
    print(f'Mean Connectors (ESMT): {df["Connectors (ESMT)"].mean()}')
    print(f'Mean Network Length: {df["Network Length"].mean()}')
    print(f'Mean Network Length (ESMT): {df["Network Length (ESMT)"].mean()}')

    # legend
    # legend_labels1 = ['w = 6', 'w = 6 (2tRNP)', 'w = 12', 'w = 12 (2tRNP)', 'w = 18', 'w = 18 (2tRNP)', 'w = 24', 'w = 24 (2tRNP)']
    # legend_labels2 = ['t = 2', 't = 2 (2tRNP)', 't = 3', 't = 3 (2tRNP)', 't = 4', 't = 4 (2tRNP)', 't = 5', 't = 5 (2tRNP)', 't = 6', 't = 6 (2tRNP)']

    # axes1.legend(legend_labels1, bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)
    # axes2.legend(legend_labels2, bbox_to_anchor=([1.05, 1, 0, 0]), ncol=1, frameon=True)

    # set font properties of colorbar label
    # cbar7 = axes7.collections[0].colorbar
    # cbar7.set_label("Ratio", fontdict={'size': 14})
    # cbar7.ax.tick_params(labelsize=14)
    # cbar8 = axes8.collections[0].colorbar
    # cbar8.set_label("Ratio", fontdict={'size': 14})
    # cbar8.ax.tick_params(labelsize=14)

    # cbar8 = axd9['r'].collections[0].colorbar
    # axd9['r'].set_ylabel("Ratio", fontdict={'size': 14})
    # axd9['r'].tick_params(labelsize=14)

    # for key, ax in axd9.items():
    #     if key == 'r':
    #         ax.spines['top'].set_visible(True)
    #         ax.spines['right'].set_visible(True)
    #     else:
    #         ax.spines['top'].set_visible(False)
    #         ax.spines['right'].set_visible(False)
    #     ax.spines['left'].set_visible(True)
    #     ax.spines['bottom'].set_visible(True)
    #     ax.spines['left'].set_linewidth(1)
    #     ax.spines['bottom'].set_linewidth(1)
    #     ax.tick_params(width=1)

    fig1.tight_layout()
    fig2.tight_layout()
    # axes1.axis('equal')
    # axes2.axis('equal')

    # plt.show()
    fig1.savefig(join(RESULTS_DIR, 'connectors_real_robots.pdf'))
    fig2.savefig(join(RESULTS_DIR, 'network_length_real_robots.pdf'))


def find_team_distances_average(plot=False):

    df = pd.DataFrame({
        'min':          pd.Series(dtype='float'),  
        'max':          pd.Series(dtype='float'),
        'mean':         pd.Series(dtype='float'),
        'diff':         pd.Series(dtype='float'),
        'min esmt':    pd.Series(dtype='float'),  
        'max esmt':    pd.Series(dtype='float'),
        'mean esmt':   pd.Series(dtype='float'),
        'diff esmt':   pd.Series(dtype='float'),
    })
    
    # Get trial names
    variation_dirs = [f for f in listdir(RESULTS_DIR) if isdir(join(RESULTS_DIR, f))]
    variation_dirs.sort()

    for variation in variation_dirs:

        trial_dirs = [f for f in listdir(join(RESULTS_DIR, variation)) if isdir(join(RESULTS_DIR, variation, f))]
        trial_dirs.sort()

        trial_no_points = {}
        trial_no_connectivity = {}
        trial_no_team_connectivity = {}

        start_time = time.time()

        count = 0
        # for scenario in trial_dirs:
        while count < len(trial_dirs):
            # if count >= len(trial_dirs):
            #     break

            scenario = trial_dirs[count]
            delay = int(scenario.split('_')[3][:-1]) # Get send delay

            start_time_single = time.time()

            # Check if the trial was successful
            s, final_points, final_connectivity, final_team_connectivity = load_log_with_checks(join(RESULTS_DIR, variation, scenario))
            if final_points == 0:
                trial_no_points[scenario] = {
                    'seed': s.seed,
                    'data': final_points
                }

            if final_team_connectivity:
                trial_no_team_connectivity[scenario] = {
                    'seed': s.seed,
                    'data': final_team_connectivity
                }

            # Find the total network length and connectors for the result obtained and the solver output
            graphs, lengths, connectors = get_network_length_and_connectors(s, plot=plot)
            if not nx.is_connected(graphs['res']):
                trial_no_connectivity[scenario] = {
                    'seed': s.seed,
                    'data': False
                }

            res_dist = get_team_distances(graphs['res'])
            esmt_dist = get_team_distances(graphs['esmt'])

            # Add data of the successful trial
            if not scenario in trial_no_connectivity and s.totalTime != 6000:
                
                d = pd.DataFrame({
                    'min': [res_dist['min']], 
                    'max': [res_dist['max']], 
                    'mean': [res_dist['mean']],
                    'diff': [res_dist['diff']],
                    'min esmt': [esmt_dist['min']], 
                    'max esmt': [esmt_dist['max']],
                    'mean esmt': [esmt_dist['mean']], 
                    'diff esmt': [esmt_dist['diff']],
                })
                df = pd.concat([df, d], ignore_index=True, axis=0)

            else:
                print(f'fail? {scenario}')

            duration_single = round(time.time() - start_time_single, 3)
            duration_total = round(time.time() - start_time, 3)
            print("Loaded -- '{0}' --\tin {1} s ({2} s)".format(scenario, duration_single, duration_total))

            # print(df)

            # DEBUG: For limiting the number of data to plot
            count += 1
            num_to_use = 50
            if (count % 10) % num_to_use == 0:
                count += 50 - num_to_use
                # print(count)
                # break

    df_melted = pd.melt(df, value_vars=['min', 'max', 'min esmt', 'max esmt'], var_name='column')

    # Font
    font = FontProperties()
    # font.set_family('serif')
    # font.set_name('Times New Roman')
    font.set_size(16)

    font2 = FontProperties()
    # font.set_family('serif')
    # font.set_name('Times New Roman')
    font2.set_size(12)

    fig1 = plt.figure(figsize=(7,5))
    axis1 = fig1.gca()

    # Set1
    color1 = (0.8941176470588236, 0.10196078431372549, 0.10980392156862745)
    color2 = (0.21568627450980393, 0.49411764705882355, 0.7215686274509804)
    # Set2
    # color1 = (0.4, 0.7607843137254902, 0.6470588235294118)
    # color2 = (0.9882352941176471, 0.5529411764705883, 0.3843137254901961)
    strip1 = sns.stripplot(data=df_melted[df_melted['column'].str.contains('min')], ax=axis1, x='column', y='value', color=color2)
    strip2 = sns.stripplot(data=df_melted[df_melted['column'].str.contains('max')], ax=axis1, x='column', y='value', color=color1, marker=(4,0,0))

    filtered_df = df.filter(['mean', 'mean esmt'])
    df_melted = filtered_df.melt(var_name='Columns', value_name='Values')

    sns.boxplot(data=df_melted, x='Columns', y='Values', ax=axis1,
                showmeans=True,
                meanline=True,
                meanprops={'color': 'k', 'ls': '-', 'lw': 1},
                medianprops={'visible': False},
                whiskerprops={'visible': False},
                zorder=10,
                showfliers=False,
                showbox=False,
                showcaps=False)

    print(df)
    print('max,max_esmt')
    for i, n in enumerate(df['max']):
        print(f'{df["max"][i]},{df["max esmt"][i]}')
    # print('mean esmt')
    # for i in df['mean esmt']:
    #     print(i)
    # print()
    print(df['mean'].mean())
    print(df['mean esmt'].mean())

    strip1.set_xticklabels([])
    strip2.set_xticklabels([])
    plt.xticks([0, 1], ['trial results', 'ESMT'])

    axis1.set_xlabel('', fontproperties=font)
    axis1.set_ylabel('Path length (m)', fontproperties=font)

    axis1.spines['top'].set_visible(False)
    axis1.spines['right'].set_visible(False)
    axis1.spines['left'].set_visible(True)
    axis1.spines['bottom'].set_visible(True)
    axis1.spines['left'].set_linewidth(1)
    axis1.spines['bottom'].set_linewidth(1)
    axis1.tick_params(width=1)

    for label in axis1.get_xticklabels():
        label.set_fontproperties(font)
    for label in axis1.get_yticklabels():
        label.set_fontproperties(font2)

    # border and ticks
    # axes1.set_yticks(np.arange(2, 6+1, 1))
    axis1.set_ylim([0,df['max esmt'].max()+0.1])

    fig1.tight_layout()

    # plt.show()
    fig1.savefig(join(RESULTS_DIR, 'path_length.pdf'))


def find_team_distances_average_real_robots(plot=False):

    df = pd.DataFrame({
        'min':          pd.Series(dtype='float'),  
        'max':          pd.Series(dtype='float'),
        'mean':         pd.Series(dtype='float'),
        'diff':         pd.Series(dtype='float'),
        'min esmt':    pd.Series(dtype='float'),  
        'max esmt':    pd.Series(dtype='float'),
        'mean esmt':   pd.Series(dtype='float'),
        'diff esmt':   pd.Series(dtype='float'),
    })
    
    # Get trial names
    variation_dirs = [f for f in listdir(RESULTS_DIR) if isdir(join(RESULTS_DIR, f))]
    variation_dirs.sort()

    for variation in variation_dirs:

        trial_dirs = [f for f in listdir(join(RESULTS_DIR, variation)) if isdir(join(RESULTS_DIR, variation, f))]
        trial_dirs.sort()

        trial_no_points = {}
        trial_no_connectivity = {}
        trial_no_team_connectivity = {}

        start_time = time.time()

        count = 0
        # for scenario in trial_dirs:
        while count < len(trial_dirs):
            # if count >= len(trial_dirs):
            #     break

            scenario = trial_dirs[count]
            # delay = int(scenario.split('_')[3][:-1]) # Get send delay

            start_time_single = time.time()

            # Check if the trial was successful
            s, final_points, final_connectivity, final_team_connectivity = load_log_with_checks(join(RESULTS_DIR, variation, scenario))
            if final_points == 0:
                trial_no_points[scenario] = {
                    'seed': s.seed,
                    'data': final_points
                }

            if final_team_connectivity:
                trial_no_team_connectivity[scenario] = {
                    'seed': s.seed,
                    'data': final_team_connectivity
                }

            # Find the total network length and connectors for the result obtained and the solver output
            graphs, lengths, connectors = get_network_length_and_connectors(s, plot=plot)
            if not nx.is_connected(graphs['res']):
                trial_no_connectivity[scenario] = {
                    'seed': s.seed,
                    'data': False
                }

            res_dist = get_team_distances(graphs['res'])
            esmt_dist = get_team_distances(graphs['esmt'])

            # Add data of the successful trial
            if not scenario in trial_no_connectivity and s.totalTime != 6000:
                
                d = pd.DataFrame({
                    'min': [res_dist['min']], 
                    'max': [res_dist['max']], 
                    'mean': [res_dist['mean']],
                    'diff': [res_dist['diff']],
                    'min esmt': [esmt_dist['min']], 
                    'max esmt': [esmt_dist['max']],
                    'mean esmt': [esmt_dist['mean']], 
                    'diff esmt': [esmt_dist['diff']],
                })
                df = pd.concat([df, d], ignore_index=True, axis=0)

            else:
                print(f'fail? {scenario}')

            duration_single = round(time.time() - start_time_single, 3)
            duration_total = round(time.time() - start_time, 3)
            print("Loaded -- '{0}' --\tin {1} s ({2} s)".format(scenario, duration_single, duration_total))

            # print(df)

            # DEBUG: For limiting the number of data to plot
            count += 1
            num_to_use = 50
            if (count % 10) % num_to_use == 0:
                count += 50 - num_to_use
                # print(count)
                # break

    df_melted = pd.melt(df, value_vars=['min', 'max', 'min esmt', 'max esmt'], var_name='column')

    # Font
    font = FontProperties()
    # font.set_family('serif')
    # font.set_name('Times New Roman')
    font.set_size(16)

    font2 = FontProperties()
    # font.set_family('serif')
    # font.set_name('Times New Roman')
    font2.set_size(12)

    fig1 = plt.figure(figsize=(7,5))
    axis1 = fig1.gca()

    # Set1
    color1 = (0.8941176470588236, 0.10196078431372549, 0.10980392156862745)
    color2 = (0.21568627450980393, 0.49411764705882355, 0.7215686274509804)
    # Set2
    # color1 = (0.4, 0.7607843137254902, 0.6470588235294118)
    # color2 = (0.9882352941176471, 0.5529411764705883, 0.3843137254901961)
    strip1 = sns.stripplot(data=df_melted[df_melted['column'].str.contains('min')], ax=axis1, x='column', y='value', color=color2, s=8)
    strip2 = sns.stripplot(data=df_melted[df_melted['column'].str.contains('max')], ax=axis1, x='column', y='value', color=color1, marker=(4,0,0), s=8)

    filtered_df = df.filter(['mean', 'mean esmt'])
    df_melted = filtered_df.melt(var_name='Columns', value_name='Values')

    sns.boxplot(data=df_melted, x='Columns', y='Values', ax=axis1,
                showmeans=True,
                meanline=True,
                meanprops={'color': 'k', 'ls': '-', 'lw': 1},
                medianprops={'visible': False},
                whiskerprops={'visible': False},
                zorder=10,
                showfliers=False,
                showbox=False,
                showcaps=False)

    print(df)
    print('max,max_esmt')
    for i, n in enumerate(df['max']):
        print(f'{df["max"][i]},{df["max esmt"][i]}')
    # print('mean esmt')
    # for i in df['mean esmt']:
    #     print(i)
    # print()
    print(df['mean'].mean())
    print(df['mean esmt'].mean())

    strip1.set_xticklabels([])
    strip2.set_xticklabels([])
    plt.xticks([0, 1], ['trial results', 'ESMT'])

    axis1.set_xlabel('', fontproperties=font)
    axis1.set_ylabel('Path length (m)', fontproperties=font)

    axis1.spines['top'].set_visible(False)
    axis1.spines['right'].set_visible(False)
    axis1.spines['left'].set_visible(True)
    axis1.spines['bottom'].set_visible(True)
    axis1.spines['left'].set_linewidth(1)
    axis1.spines['bottom'].set_linewidth(1)
    axis1.tick_params(width=1)

    for label in axis1.get_xticklabels():
        label.set_fontproperties(font)
    for label in axis1.get_yticklabels():
        label.set_fontproperties(font2)

    # border and ticks
    # axes1.set_yticks(np.arange(2, 6+1, 1))
    axis1.set_ylim([0,df['max esmt'].max()+0.1])

    fig1.tight_layout()

    # plt.show()
    fig1.savefig(join(RESULTS_DIR, 'path_length_real_robots.pdf'))


def main():

    # Load a single trial
    strategy = 'travel'
    num_run = '001'
    scenario = f'energy_sharing({strategy})_4T_10R_0C_{num_run}'
    path = join(environ['HOME'], f'GIT/swarm-energy-replenishment_prep/results/energy_sharing/', scenario)
    # print(scenario)


    # plot_energy_consumption(plot=False)
    # plot_connector_energy_over_time(plot=True)
    # path = '/home/genki/GIT/swarm-energy-replenishment_prep/results/iros3/4T_charger(5R5C)/energy_sharing(charger)_4T_5R_5C_003'
    # plot_connector_energy_over_time_single_trial(path, plot=True)

    # plot_energy_heatmaps(chosen_work_rate=1.0,
    #                      plot=False)
    # plot_energy_boxplot(chosen_num_chargers=6,
    #                     chosen_capacity=200,
    #                     plot=False)
    plot_recharge_rate(chosen_num_chargers=6,
                       chosen_capacity=200,
                       plot=False)
    # plot_transfer_loss(chosen_num_chargers=6,
    #                    chosen_capacity=200,
    #                    plot=False)

if __name__ == "__main__":
    main()
