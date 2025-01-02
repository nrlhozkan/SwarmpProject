import json
import math
import os
import random
from datetime import datetime

import numpy as np
from tabulate import tabulate

from helper import position_helper
from helper.charlie_helper import send_message, set_up_charlie
from helper.position_helper import Position
from utils import config
from utils.logging_utils import log, LOG_TYPE_DEBUG, LOG_TYPE_INFO

# To get target positions from the simulation, we need to import the necessary libraries
import zmq
import json
import time

DURATION_PER_LOOP_ITERATION_S = 60  # assume 60s per loop iteration
# TARGET_SPEED_MS = 1.2

MAX_VIS_DISTANCE = 100

def generate_initial_prompt(latitude_range, longitude_range, drone_pos):
    number_drones = len(drone_pos)

    message = (
        f'You are tasked with controlling a drone swarm of {number_drones} drone{"s" if number_drones > 1 else ""} to search for targets in'
        f' a rectangular search area spanning from GPS point ({latitude_range[0]} | {longitude_range[0]}) to ({latitude_range[1]} | {longitude_range[1]}),'
        f' given as (latitude | longitude). The targets can be both stationary and non-stationary.\n'
        f' The drones are currently positioned as follows:\n'
    )

    # target data
    for i, drone_pos in enumerate(drone_pos):
        message += f'drone {i + 1}: ({drone_pos.latitude}, {drone_pos.longitude})\n'

    message += '\n'

    return message

def keep_searching(context_and_pos_prompt, col_avoidance_already_hinted, curr_session, c_id, debug) -> None:
    message = context_and_pos_prompt

    message += (
        f'Move the drones {"again " if col_avoidance_already_hinted else ""}using the script simulation_drone_control_new to keep searching for targets.'
        ' Choose freely where to move the drones next in the previously specified search area. The search should be efficient and cover the entire search area.'
        ' Avoid covering the same spots with different drones at short time intervals.'
        ' Ensure that the entire search area is covered efficiently without overlapping search paths for different drones.'
        # ' The unassigned drones have to move to continue searching for new targets in the specified area.'
    )

    if col_avoidance_already_hinted:
        message += '\nRemember to avoid collisions by applying the mentioned techniques.'
    else:
        message += '\nNote that drones fly at the same altitude, so you have to make sure that drones avoid collisions during flight.'
        message += '\nTo avoid the collisions, you can calculate the speed of the drones and set these speeds to the drones and/or use the sleep function to pause the flight of the drones for a certain period of time, but make sure that the drones reach their destination as soon as possible (max-speed is 5m/s).'

        
    # To avoid the collisions, you can calculate the speed of the drones and set these speeds to the drones and/or use the sleep function to pause the flight of the drones for a certain period of time, but make sure that the drones reach their destination as soon as possible (max-speed is 5m/s).
    
    log(LOG_TYPE_DEBUG, 'Search prompt:')
    print(message)

    res = send_message(curr_session, message, c_id, debug)
    
    print(f'\n Keep searching prompt: {message}')
    print(f'\n Keep searching response: {res}')

def generate_drone_assignment_prompt(distance_table):
    num_drones, num_targets = distance_table.shape

    prompt = (
        f'This is the updated distance adjacency matrix (meters) after moving the drones.'
        f' Note that cells has -1 mean the the drone can not see the target.\n'
        f'Drone ID: {[f"Target {i}" for i in range(1, num_targets + 1)]}\n'
    )

    for i in range(num_drones):
        distance_row = [distance_table[i, j].item() for j in range(num_targets)]
        prompt += f'Drone {i + 1}: {distance_row}\n'

    prompt += (
        '\nDetermine the most efficient drone-to-target assignment to minimize the overall distance between the assigned pairs.' 
        ' A trade-off must be made between assigning multiple drones to the same target and leaving some drones unassigned for use in the search for further targets later, in case there are more drones than visible targets.' 
        ' Show the assigned drones and unassigned drones in two separate lists similar to Assigned drones = [drone1, drone2, ...], Unassigned drones = [drone3, drone4, ...].'
    )

    return prompt


def generate_movement_prompt(col_avoidance_already_hinted, visibility_table, target_positions):
    # initial instruction
    prompt = 'You are given a table specifying the positions (latitude, longitude) of currently visible targets.\n'

    # target data
    for i, target in enumerate(target_positions):

        # only print target positions for targets that are visible
        if np.sum(visibility_table[:, i]) > 0:
            prompt += f'target {target["id"]}: ({target["position"].latitude}, {target["position"].longitude})\n'

    prompt += 'Each drone in the swarm must be moved according to the previous step.If they are assigned to any targets, they must move as per rule 1. If they are not assigned to any targets, they must move as per rule 2.The rules are as follows:\n'
    prompt += '\nRule 1: The assigned drones have to move towards the corresponding target positions in a straight-line path.'
    prompt += '\nRule 2: The unassigned drones must continue searching for new targets in the specified area until all drones in the swarm are assigned to any targets. They must always move for searching.' 
    # prompt += '\nNote that if more than one drone is assigned to the same target, the drones must not collide with each other. The drones must maintain a minimum distance of 25m between each other at all times.'
    
    # prompt += '\nDrones never collide with each other even they assigned to the same target.'

    if col_avoidance_already_hinted:
        prompt += '\nRemember to avoid collisions by applying the mentioned techniques.'
    else:
        prompt += '\nNote that drones fly at the same altitude, so you have to make sure that drones avoid collisions during flight.'
        prompt += '\nTo avoid the collisions, you can calculate the speed of the drones and set these speeds to the drones and/or use the sleep function to pause the flight of the drones for a certain period of time, but make sure that the drones reach their destination as soon as possible (max-speed is 5m/s).'

    return prompt

def generate_target_positions(n, lat_from_to, lon_from_to) -> list:  # it is used to generate initial target positions 
    target_positions = list()

    from_lat, to_lat = lat_from_to
    from_lon, to_lon = lon_from_to

    for i in range(n):
        target_positions.append(
            {
                'id': i + 1,
                'position': Position(
                    np.random.uniform(from_lat, to_lat),
                    np.random.uniform(from_lon, to_lon)
                )
            }
        )

    return target_positions


def print_target_positions(target_data):
    print(target_data)

    headers = ['Target', 'Latitude', 'Longitude']
    positions = [[target['id'], target['position'].latitude, target['position'].longitude]
                 for target in target_data]

    print(tabulate(positions, headers=headers, tablefmt="fancy_grid"))


def get_visibility_scores(ref, drone_pos: list, target_positions: list):
    vis_mtrx = []

    for drone in drone_pos:
        drone_position = Position(drone["latitude"], drone["longitude"])
        drone_visibility_row = []

        for target in target_positions:
            target_pos = Position(target["position"].latitude, target["position"].longitude)

            # Calculate the distance between drone and target
            distance = position_helper.calculate_distance_meters(
                ref, 
                drone_position,
                target_pos
            )

            # Check if target is within visibility range
            if distance < MAX_VIS_DISTANCE:
                visibility = visibility_score(distance, max_distance=MAX_VIS_DISTANCE)
                drone_visibility_row.append(visibility)
            else:
                drone_visibility_row.append(0)

        vis_mtrx.append(drone_visibility_row)

    return np.array(vis_mtrx)

def get_distances(ref, drone_pos: list, target_positions: list):
    dist_mtrx = []

    for drone in drone_pos:
        drone_position = Position(drone['latitude'], drone['longitude'])
        drone_distance_row = []

        for target in target_positions:
            target_position = Position(target['position'].latitude, target['position'].longitude)

            # Calculate the distance between drone and target
            distance = position_helper.calculate_distance_meters(
                ref,
                drone_position,
                target_position
            )

            # Check if target is within visibility range
            if distance < MAX_VIS_DISTANCE:
                drone_distance_row.append(distance)
            else:
                drone_distance_row.append(-1)

        dist_mtrx.append(drone_distance_row)

    return np.array(dist_mtrx)


def filter_visibility_matrix(vis_mtrx: np.ndarray):
    non_zero_columns = ~np.all(vis_mtrx == 0, axis=0)

    # filter
    filtered_matrix = vis_mtrx[:, non_zero_columns]

    # None if no target is visible, otherwise return the filtered matrix
    return filtered_matrix if filtered_matrix.shape[1] > 0 else None


def visibility_score(distance, max_distance=70):
    if distance >= max_distance:
        return 0

    # using exponential decay to provide a non-linear relation
    return np.exp(-distance / max_distance)


def print_adj_matrix(visibility_table, matrix_type):
    # round
    rounded_array = np.round(visibility_table, 3)

    headers = ["Target " + str(i + 1) for i in range(rounded_array.shape[1])]
    drones = [["Drone " + str(i + 1)] + list(row) for i, row in enumerate(rounded_array)]

    print(f'\nAdjacency matrix containing the {matrix_type}')
    print(tabulate(drones, headers=headers, tablefmt="fancy_grid"))


def update_target_positions(target_positions, directory, file_name):
    path = os.path.join(directory, file_name)

    target_positions = [
        {
            'id': pos['id'],
            'latitude': pos['position'].latitude,
            'longitude': pos['position'].longitude
        }
        for pos in target_positions
    ]

    try:
        if os.path.exists(path):
            with open(path, 'r') as target_file:
                previous_positions = json.load(target_file)
        else:
            previous_positions = []

        # append and write
        pos_data = target_positions.copy()
        pos_data.insert(0, {'movement_id': str(len(previous_positions) + 1)})

        previous_positions.append(pos_data)
        with open(path, 'w') as target_file:
            json.dump(previous_positions, target_file, indent=4)

    except PermissionError:
        print(f'Permission denied when trying to open file {path}')
    except Exception as e:
        print(f'An error occurred: {e}')


def update_position_history_and_get_positions(current_position_data, directory, file_name):
    path = os.path.join(directory, file_name)

    try:
        if os.path.exists(path):
            with open(path, 'r') as pos_file:
                previous_positions = json.load(pos_file)
        else:
            previous_positions = []

        # append and write
        pos_data = current_position_data.copy()
        pos_data.insert(0, {'movement_id': str(len(previous_positions) + 1)})

        previous_positions.append(pos_data)
        with open(path, 'w') as pos_file:
            json.dump(previous_positions, pos_file, indent=4)

        return current_position_data
    except PermissionError:
        print('Permission denied when trying to open the file.')
    except Exception as e:
        print(f'An error occurred: {e}')


def update_distance_file(dist_matrx, directory, file_name):
    path = os.path.join(directory, file_name)

    try:
        if os.path.exists(path):
            with open(path, 'r') as distance_file:
                previous_distances = json.load(distance_file)
        else:
            previous_distances = []

        dist_matrx = [
            {
                f'target{j + 1}': dist
                for j, dist in enumerate(row)
            } for i, row in enumerate(dist_matrx.astype(float).tolist())
        ]

        # add ids
        dist_matrx = [
            {'id': i + 1, **entry} for i, entry in enumerate(dist_matrx)
        ]

        # append and write
        distance_data = dist_matrx.copy()
        distance_data.insert(0, {'movement_id': len(previous_distances) + 1})

        previous_distances.append(distance_data)
        with open(path, 'w') as distance_file:
            json.dump(previous_distances, distance_file, indent=4)

    except PermissionError:
        print('Permission denied when trying to open the file.')
    except Exception as e:
        print(f'An error occurred: {e}')


def write_initial_drone_pos(initial_positions, target_file):
    with open(target_file, 'w') as initial_pos_file:
        json.dump(
            [
                {
                    'latitude': pos.latitude,
                    'longitude': pos.longitude
                }
                for pos in initial_positions
            ],
            initial_pos_file, indent=4
        )

def calculate_new_target_positions(targets):
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    pub_address = "tcp://localhost:5557"  # Publisher address
    socket.connect(pub_address)
    socket.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all topics

    print(f"Connected to simulation publisher at {pub_address}")
    
    for target in targets:
        
        target_id = target.get("id")
        
        message = socket.recv_string()
        try:
            data = json.loads(message)
        except json.JSONDecodeError as e:
            print(f"JSON decode error: {e}")
            continue

        Targets = data.get("targets", [])
        
        target_info = next((t for t in Targets if t.get("target_id") == target_id), None)
        
        target['position'].latitude = target_info.get('latitude')
        target['position'].longitude = target_info.get('longitude')

        print(f"\n[Target {target['id']}]:")
        print(f"  Latitude: {target['position'].latitude}")
        print(f"  Longitude: {target['position'].longitude}")
        
    return targets


def pause_until_input(go_char):
    while input(f'Enter {go_char} to continue: ') != go_char:
        pass


def read_json_file(file_path):
    with open(file_path, 'r') as f:
        return json.load(f)


def get_positions_from_dictionary_list(pos_dict):
    positions = list()

    for pos in pos_dict:
        positions.append(
            Position(
                latitude=pos['latitude'],
                longitude=pos['longitude']
            )
        )

    return positions


def merge_position_coordinates(position_dict):
    merged_positions = list()

    for pos in position_dict:
        merged_positions.append({
            'id': pos['id'],
            'position': Position(
                latitude=pos['latitude'],
                longitude=pos['longitude']
            )
        })

    return merged_positions

############### My Edition 4 start ################

def write_target_positions(target_positions, target_pos_file):

    with open(target_pos_file, 'w') as pos_file:
        json.dump(
            [
                {
                    'id': pos['id'],
                    'latitude': pos['position'].latitude,
                    'longitude': pos['position'].longitude
                }
                for pos in target_positions
            ],
            pos_file, indent=4
        )

############### My Edition 4 end ################

if __name__ == '__main__':
    
    # prompting  parameters
    debug_mode = False
    move_targets = True

    chat_id = 'SR3'
    chat_name = 'S&R loop'
    drone_chat_json = {'username': 'admin', 'chat_id': chat_id, 'chat_name': chat_name}

    # parameters
    repeat_previous_experiment = False  # work in progress: do not set to True yet (minor updates need to be made to the file reading below to make it work)
    large_search_area = True  # 250m x 250m (~6ha) or 500m x 500m (25ha)

    n_drones = int(input('Enter the number of drones: ')) # simulating N drones
    n_targets = int(input('Enter the number of targets: ')) # simulating N targets
    dist_between_drones = 30

    if large_search_area:
        # 500m x 500m
        lat_range = (48, 48.0045)  # from lat to lat
        lon_range = (14, 14.00671)  # from lon to lon
    else:
        # ~250m x ~250m
        lat_range = (48, 48.0025)  # from lat to lat
        lon_range = (14, 14.00371)  # from lon to lon
        
    # Coordinate Converter reference position  
    reference_position = [lat_range[0], lon_range[0]]
    print('Reference Position: ', reference_position)
    
    # get path to experiment folder base path + folder name for current experiment
    experiment_folder = os.path.join(config.EXPERIMENT_BASE_FOLDER, datetime.now().strftime(config.EXPERIMENT_FOLDER_FORMAT))
    # experiment_folder = os.path.join(EXPERIMENT_BASE_FOLDER, 'data_01_12_2024_17_35_43')

    log(LOG_TYPE_INFO, f'Generating {n_drones} initial positions (distance {dist_between_drones})')

    if repeat_previous_experiment:
        initial_drone_pos = get_positions_from_dictionary_list(read_json_file(config.INITIAL_DRONE_POS_FILE))
    else:
        initial_drone_pos = position_helper.generate_initial_positions(
            n_drones, dist_between_drones, lat_range, lon_range
        )
    print('initial_drone_pos', initial_drone_pos)

    log(LOG_TYPE_DEBUG, 'Initial positions generated and stored in file')

    if repeat_previous_experiment:
        target_pos = merge_position_coordinates(read_json_file(config.ALL_TARGET_POS_FILE))
    else:
        target_pos = generate_target_positions(n_targets, lat_range, lon_range) # 
 
    # at this point all preparatory work, initialization and data loading is done
    # setting up Charlie and begin to prompt
    
    ############### My Edition 1 start ################
    write_target_positions(target_pos, config.TARGET_POS_FILE)
    ############### My Edition 1 end ################
    
    print_target_positions(target_pos)

    if not debug_mode:
        pause_until_input('g')
        session = set_up_charlie(drone_chat_json, chat_id, reset_data=True)
    else:
        session = None

    log(LOG_TYPE_INFO, 'Writing and preparing files...')
    os.makedirs(experiment_folder, exist_ok=True)

    # make initial positions available to charlie
    if not repeat_previous_experiment:
        write_initial_drone_pos(initial_drone_pos, config.INITIAL_DRONE_POS_FILE)

    # first entry in all files is: initial drone and target positions with no visibility to any of the targets
    update_target_positions(target_pos, experiment_folder, config.ALL_TARGET_POS_FILE)
    
    ############### My Edition 2 start ################
    write_target_positions(target_pos, config.TARGET_POS_FILE)
    ############### My Edition 2 end ##################
    
    log(LOG_TYPE_DEBUG, 'Writing initial positions to history file...')
    update_position_history_and_get_positions(
        [
            {
                'id': i + 1,
                'latitude': pos.latitude,
                'longitude': pos.longitude
            } for i, pos in enumerate(initial_drone_pos)
        ],
        experiment_folder,
        config.ALL_DRONE_POS_FILE
    )
    update_distance_file(np.full((n_drones, n_targets), -1), experiment_folder, config.ALL_DISTANCES_FILE)

    # begin prompting
    # Step1: initial formation
    # Step2: perform first movement and calculate visibility values
    initial_prompt = generate_initial_prompt(lat_range, lon_range, initial_drone_pos)

    collision_avoidance_hinted = [False, False]

    # calculated after each movement/loop
    filtered_vis_mtrx = None
    visibility_matrix = None
    distance_matrix = None

    while True:

        print('\n')
        pause_until_input('g')

        # 2 scenarios:
        #  * visibility matrix = None ... keep searching
        #  * visibility matrix not empty ... fly over target

        if filtered_vis_mtrx is None:
            log(LOG_TYPE_INFO, 'No target visible...continuing the search')
            
            # Note: Initial prompt is only shown in the first loop iteration
            keep_searching(initial_prompt, collision_avoidance_hinted[0], session, chat_id, debug_mode)

            initial_prompt = ''  # only prompt in the first loop iteration
            collision_avoidance_hinted[0] = True

        else:
            log(LOG_TYPE_INFO, 'At least one target is visible...assigning the drone(s)...')

            # drone-target assignment
            log(LOG_TYPE_INFO, 'Generating drone assignment prompt...\n')

            assignment_prompt = generate_drone_assignment_prompt(distance_matrix)
            log(LOG_TYPE_DEBUG, 'Assignment prompt:')

            res = send_message(session, assignment_prompt, chat_id, debug_mode)
            
            # I added this line to print the assignment prompt and response
            print('\n Assignment Prompt : ', assignment_prompt)
            print('\n Assignment Response : ', res)

            if not debug_mode:
                print('\nAssignment completed')
                pause_until_input('g')

            # move drones
            log(LOG_TYPE_INFO, 'Generating drone movement prompt...\n')
            
            ############### My Edition 3 start ################
            
            # For plotting assignments 
            
            target_pos = calculate_new_target_positions(target_pos)
            
            write_target_positions(target_pos, config.TARGET_POS_FILE)
            ############### My Edition 3 end ################
            
            movement_prompt = generate_movement_prompt(
                collision_avoidance_hinted[1],
                visibility_matrix,
                target_pos
            )
            collision_avoidance_hinted[1] = True

            log(LOG_TYPE_DEBUG, 'Movement prompt:')

            # I added this line to print the movement prompt and response
            res_1 = send_message(session, movement_prompt, chat_id, debug_mode)
            
            print('\n Movement Prompt : ', movement_prompt)
            print('\n Movement Response : ', res_1)

        drone_positions = update_position_history_and_get_positions(
            read_json_file(config.DRONE_POS_FILE),
            experiment_folder,
            config.ALL_DRONE_POS_FILE
        )

        # update target location
        if move_targets:
            target_pos = calculate_new_target_positions(target_pos)
            
            update_target_positions(target_pos, experiment_folder, config.ALL_TARGET_POS_FILE)
        
        print('\nDrone positions: ', drone_positions)
        print('\nTarget positions: ', target_pos)
        
        print('\nFinished one loop iteration. Updated visibility')

        # update visibility matrix
        visibility_matrix = get_visibility_scores(reference_position, drone_positions, target_pos)
        filtered_vis_mtrx = filter_visibility_matrix(visibility_matrix)
        distance_matrix = get_distances(reference_position, drone_positions, target_pos)

        update_distance_file(distance_matrix, experiment_folder, config.ALL_DISTANCES_FILE)

        if filtered_vis_mtrx is not None:
            # print_adj_matrix(visibility_matrix, 'visibility scores')
            print_adj_matrix(distance_matrix, 'distances')
        else:
            print('No target visible.')
