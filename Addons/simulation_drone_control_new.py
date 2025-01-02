import json
import math
import threading
import time
import socket
from geopy import Point
from geopy.distance import geodesic
import zmq

description = "control drones, move_to(latitude, longitude, absolute altitude above ground meters) example: move_to(41.902856, 12.496478, 10) use move_to for moving or flying drone to specific GPS coordinates, get_position() returns current position GPS coordinates where drone is now and altitude above ground meters and heading degrees 0..359 , set_drone_speed(drone speed meters per second) example: set_drone_speed(5) used to adjust or change or define the flying speed of the drone, rotate_gimbal(pitch degrees 20..-90) example: rotate_gimbal(-90), sleep(duration ms) example: sleep(250), Note: all instruction parameters are numbers, move and fly are treated as the same action and are synonyms."

parameters = {
    "type": "object",
    "properties": {
        "drones": {
            "type": "array",
            "items": {"type": "number"},
            "description": "The drone numbers, example: [3, 4]"
        },
        "is_initial_prompt": {
            "type": "boolean",
            "description": "True only for the first call, False for all subsequent calls"
        },
        "instructions": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "drone_id": {"type": "number"},
                    "instructions": {
                        "type": "array",
                        "items": {
                            "type": "object",
                            "properties": {
                                "instruction": {"type": "string"},
                                "parameters": {"type": "string"}
                            },
                            "required": ["instruction", "parameters"]
                        }
                    }
                },
                "required": ["drone_id", "instructions"]
            },
            "description": "List of instructions for each drone"
        }
    },
    "required": ["drones", "is_initial_prompt", "instructions"]
}

# Global default values
DEFAULT_DISTANCE = 5
DEFAULT_HEADING = 0
DEFAULT_ALTITUDE = 30
DEFAULT_ANGLE = -90
DEFAULT_SPEED = 1
DEFAULT_SLEEP_DUR_MS = 10

MAX_SPEED = 5
MIN_ALTITUDE = 10
MAX_ALTITUDE = 100

FOLDER_TIME = time.strftime("%Y-%m-%d-%H-%M")
SIMULATION = True

global initial_step

initial_step = True

if not SIMULATION:
    drone_variables = {}
else:
    simulated_drone_variables = {}
    
def send_command(command, host='localhost', port=5555): # default host and port
    """Send a command to the simulation."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.connect((host, port))
        command_str = json.dumps(command) + '\n'
        sock.sendall(command_str.encode('utf-8'))
        
def get_the_drone_positions(id):
    
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    pub_address = "tcp://localhost:5557"  # Publisher address
    socket.connect(pub_address)
    socket.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all topics

    print(f"Connected to simulation publisher at {pub_address}")
    message = socket.recv_string()
    
    try:
        data = json.loads(message)
    except json.JSONDecodeError as e:
        print(f"JSON decode error: {e}")

    drones = data.get("drones", [])

    drone_info = next((d for d in drones if d.get("drone_id") == id), None)
        
    telemetry = {'id': id, 'latitude': drone_info.get('latitude'), 'longitude': drone_info.get('longitude'), 'altitude': drone_info.get('altitude')}
    
    return telemetry
    

def get_the_drone_positions(id):
    
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    pub_address = "tcp://localhost:5557"  # Publisher address
    socket.connect(pub_address)
    socket.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all topics

    print(f"Connected to simulation publisher at {pub_address}")
    message = socket.recv_string()
    
    try:
        data = json.loads(message)
    except json.JSONDecodeError as e:
        print(f"JSON decode error: {e}")

    drones = data.get("drones", [])

    drone_info = next((d for d in drones if d.get("drone_id") == id), None)
        
    telemetry = {'id': id, 'latitude': drone_info.get('latitude'), 'longitude': drone_info.get('longitude'), 'altitude': drone_info.get('altitude')}
    
    return telemetry
    
def is_reached(id):
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    pub_address = "tcp://localhost:5557"  # Publisher address
    socket.connect(pub_address)
    socket.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all topics

    print(f"Connected to simulation publisher at {pub_address}")

    reached = False

    while not reached:
        message = socket.recv_string()
        try:
            data = json.loads(message)
        except json.JSONDecodeError as e:
            print(f"JSON decode error: {e}")
            continue

        drones = data.get("drones", [])

        drone_info = next((d for d in drones if d.get("drone_id") == id), None)

        # print(f"Drone {id} reached: {drone_info.get('reached')}")

        if drone_info.get('reached') == True:
            break


def get_drone_positions():

    _, n_positions = read_initial_positions_from_file()
    
    num_drones = n_positions
    
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    pub_address = "tcp://localhost:5557"  # Publisher address
    socket.connect(pub_address)
    socket.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all topics

    print(f"Connected to simulation publisher at {pub_address}")
    
    # Reference Data
    ''' 
    [
        {
            "id": 1,
            "latitude": 48.002,
            "longitude": 14.006,
            "altitude": 25
        },
        {
            "id": 2,
            "latitude": 48.005,
            "longitude": 14.005,
            "altitude": 30
        }
    ]
    
    '''
    drone_pos_data = []
    
    for id in range(1, num_drones+1):
        message = socket.recv_string()
        try:
            data = json.loads(message)
        except json.JSONDecodeError as e:
            print(f"JSON decode error: {e}")
            continue

        drones = data.get("drones", [])

        drone_info = next((d for d in drones if d.get("drone_id") == id), None)
        

        telemetry = {'id': id, 'latitude': drone_info.get('latitude'), 'longitude': drone_info.get('longitude'), 'altitude': drone_info.get('altitude')}
        drone_pos_data.append(telemetry)
  
    try:
        with open(r'C:\Users\cgadmin\Desktop\charlie-mnemonic-dev\charlie-mnemonic-dev\Data\charlie_shared_data\drone_pos.json', 'w') as f:  # to do: change path
            json.dump(drone_pos_data, f, indent=4)
            
    except Exception as e:
        print(f'An error occurred: {e}')
         
    return print('Drone positions written to file')       
       
def read_target_positions_from_file():
    try:
        with open(r'C:\Users\cgadmin\Desktop\charlie-mnemonic-dev\charlie-mnemonic-dev\Data\other_data\target_positions.json', 'r') as f: # to do: change path
            target_drone_pos = json.load(f)
            n_positions = len(target_drone_pos)
            return target_drone_pos, n_positions
    except FileNotFoundError:
        return [], 0
    except json.JSONDecodeError:
        return [], 0

def read_drone_positions_from_file():
    try:
        with open(r'C:\Users\cgadmin\Desktop\charlie-mnemonic-dev\charlie-mnemonic-dev\Data\charlie_shared_data\drone_pos.json', 'r') as f: # to do: change path
            drone_pos = json.load(f)
            n_positions = len(drone_pos)
            return drone_pos, n_positions
    except FileNotFoundError:
        return [], 0
    except json.JSONDecodeError:
        return [], 0

def read_initial_positions_from_file():
    # read initial positions
    print('Reading initial positions from file...')
    with open(r'C:\Users\cgadmin\Desktop\charlie-mnemonic-dev\charlie-mnemonic-dev\Data\charlie_shared_data/initial_drone_pos.json', 'r') as f: # to do: change path
        initial_drone_pos = json.load(f)
        n_positions = len(initial_drone_pos)
        print(f'Successfully read {n_positions} initial positions')

        return initial_drone_pos, n_positions

def simulation_drone_control_new(drones, instructions, is_initial_prompt, username=None):
    global simulated_drone_variables
    print(f"Running drone controller with drones: {drones} and instructions: {instructions} \n")

    # reset to keep initial positions up-to-date
    # Note: drone initialization is done only in the first call of this method per chat anyway
    initial_drone_pos = None
    n_positions = 0

    is_initial_prompt = bool(is_initial_prompt)  # in case this is specified as a string

    # if is_initial_prompt:
    #     print('Resetting all simulated drone variables...')
    #     simulated_drone_variables = {}

    print(f'Initialized drones: {simulated_drone_variables.keys()}')

    if SIMULATION:
        for drone in drones:

            if drone not in simulated_drone_variables:

                drone_id = int(drone)
                print(f'Drone with id {drone_id} not initialized')

                if initial_drone_pos is None:
                    initial_drone_pos, n_positions = read_initial_positions_from_file()

                # use predefined initial position or fallback position
                initial_pos = initial_drone_pos[drone_id - 1] if drone_id <= n_positions \
                    else {'latitude': 48.335982, 'longitude': 14.326521}

                if drone_id > n_positions:
                    print(f'Using fallback position for drone {drone_id} - there is no initial position for this drone')

                # Create and initialize a separate dictionary for each drone
                simulated_drone_variables[drone] = {
                    'latitude': initial_pos['latitude'],
                    'longitude': initial_pos['longitude'],
                    'altitude': 0,
                    'speed': 0,
                    'heading': 0,
                    'gimbal_roll': 0,
                    'gimbal_pitch': 0,
                    'gimbal_yaw': 0,
                }

    # Create a dictionary to represent the input data structure
    input_data = {
        "drones": drones,
        "instructions": instructions
    }

    # Print the JSON representation of the input data
    print("Input data to be executed:", json.dumps(input_data, indent=2))

    ret_tmp = run_drone(drones, instructions)

    # write positions to file

    print('Completed execution of all instructions')
    if SIMULATION:

        print('Collecting drone positions...')
        
        get_drone_positions()
    
    return ret_tmp
  
def run_drone(drones, instructions):
    errors = []

    if not isinstance(drones, list):
        drones = [int(drone) for drone in drones.split(',')]

    if not isinstance(instructions, list):
        instructions = [instructions]  # Ensure instructions is a list

    num_drones = len(drones)
    num_instructions = len(instructions)

    print(f"Number of drones: {num_drones}")
    print(f"Number of instructions: {num_instructions}")

    # Create a dictionary to store instructions for each drone
    drone_instructions = {drone: [] for drone in drones}

    # Sort instructions into the dictionary
    for instruction_set in instructions:
        drone_id = instruction_set['drone_id']
        if drone_id in drones:
            drone_instructions[drone_id] = instruction_set['instructions']
        else:
            errors.append(f"Warning: Instruction set for non-existent drone {drone_id}")

    def process_drone_instructions(drone, instructions):
        drone_responses = []
        for instruction in instructions:
            instruction_type = instruction.get("instruction")
            parameters = instruction.get("parameters", "")

            # print(f"Processing for Drone {drone} - Instruction: {instruction_type}, Parameters: {parameters}")

            # def move(drone_id, distance, direction, abs_altitude):

            #     if SIMULATION:

            #         lat = simulated_drone_variables[drone_id]['latitude']
            #         lon = simulated_drone_variables[drone_id]['longitude']
            #         alt = simulated_drone_variables[drone_id]['altitude']
            #         speed = simulated_drone_variables[drone_id]['speed']
            #         heading = simulated_drone_variables[drone_id]['heading']

            #         start = Point(lat, lon, abs_altitude)
            #         horizontal_distance = distance * math.cos(math.radians(0))
            #         d = geodesic(meters=horizontal_distance)
            #         destination = d.destination(start, direction)

            #         new_lat = destination.latitude
            #         new_lon = destination.longitude
            #         previous_altitude = alt

            #         if abs_altitude <= 0 and previous_altitude != 0:
            #             abs_altitude = previous_altitude

            #         if abs_altitude > MAX_ALTITUDE:
            #             abs_altitude = MAX_ALTITUDE

            #         new_alt = abs_altitude

            #         simulated_drone_variables[drone_id]['latitude'] = new_lat
            #         simulated_drone_variables[drone_id]['longitude'] = new_lon
            #         simulated_drone_variables[drone_id]['altitude'] = new_alt
            #         simulated_drone_variables[drone_id]['speed'] = speed
            #         simulated_drone_variables[drone_id]['heading'] = direction

            #         lat = simulated_drone_variables[drone_id]['latitude']
            #         lon = simulated_drone_variables[drone_id]['longitude']
            #         alt = simulated_drone_variables[drone_id]['altitude']
            #         speed = simulated_drone_variables[drone_id]['speed']
            #         heading = simulated_drone_variables[drone_id]['heading']

            #         print(
            #             f'Drone {drone_id} is at {lat}, {lon} GPS position with altitude {alt} and heading {heading}.')

            #         return f'Drone {drone_id} is at {lat}, {lon} GPS position with altitude {alt} and heading {heading}.'

            def sleep(drone_id, duration):
                
                print(f'Drone {drone_id} paused for {duration}ms.')

                return f'Drone {drone_id} paused for {duration}ms.'

            def move_to(drone_id, latitude, longitude, abs_altitude):

                if SIMULATION:

                    lat = simulated_drone_variables[drone_id]['latitude']
                    lon = simulated_drone_variables[drone_id]['longitude']
                    alt = simulated_drone_variables[drone_id]['altitude']
                    speed = simulated_drone_variables[drone_id]['speed']
                    heading = simulated_drone_variables[drone_id]['heading']

                    previous_altitude = alt

                    if abs_altitude <= 0 and previous_altitude != 0:
                        abs_altitude = previous_altitude

                    if abs_altitude > MAX_ALTITUDE:
                        abs_altitude = MAX_ALTITUDE

                    new_alt = abs_altitude

                    simulated_drone_variables[drone_id]['latitude'] = latitude
                    simulated_drone_variables[drone_id]['longitude'] = longitude
                    simulated_drone_variables[drone_id]['altitude'] = new_alt

                    lat = simulated_drone_variables[drone_id]['latitude']
                    lon = simulated_drone_variables[drone_id]['longitude']
                    alt = simulated_drone_variables[drone_id]['altitude']
                    speed = simulated_drone_variables[drone_id]['speed']
                    heading = simulated_drone_variables[drone_id]['heading']
                    
                    target_position = read_target_positions_from_file()
                    # {
                    #     "action": "assign_drone_to_target",
                    #     "drone_id": 1,
                    #     "target_id": 4
                    # },
                    for target in target_position[0]:
                        if target['latitude'] == lat and target['longitude'] == lon:
                            print(f'Drone{drone_id} is assigned to Target{target["id"]}')
                            command = {
                                "action": "assign_drone_to_target",
                                "drone_id": drone_id,
                                "target_id": target['id']
                            }
                            send_command(command)
  
                    waypoint = [lat, lon, 30]
                    command = {
                        "action": "assign_waypoint_to_drone",
                        "drone_id": drone_id,
                        "waypoint": waypoint
                    }
                    
                    send_command(command)
                    
                    # thread = threading.Thread(target=is_reached, args=(drone_id,)) # daemon is used to stop the thread when the main thread is stopped
                    # thread.start()
                    
                    is_reached(drone_id)
                    
                    drone_data = get_the_drone_positions(drone_id)
                    
                    print(f'Drone {drone_data["id"]} is at {drone_data["latitude"]}, {drone_data["longitude"]} GPS position with altitude {drone_data["altitude"]}.')
                    
                    return f'Drone {drone_data["id"]} is at {drone_data["latitude"]}, {drone_data["longitude"]} GPS position with altitude {drone_data["altitude"]}.'

            def set_drone_speed(drone_id, speed):

                if not SIMULATION:

                    drone_speed = drone_variables[drone_id]['drone_speed']

                    if speed <= 0:
                        new_speed = DEFAULT_SPEED
                    elif speed >= MAX_SPEED:
                        new_speed = MAX_SPEED
                    else:
                        new_speed = speed

                    drone_variables[drone_id]['drone_speed'] = new_speed

                    new_speed = drone_variables[drone_id]['drone_speed']

                    return f'Speed of Drone {drone_id} is set to {new_speed} m/s.'

                else:

                    if speed <= 0:
                        new_speed = DEFAULT_SPEED
                    elif speed >= MAX_SPEED:
                        new_speed = MAX_SPEED
                    else:
                        new_speed = speed

                    simulated_drone_variables[drone_id]['speed'] = new_speed

                    lat = simulated_drone_variables[drone_id]['latitude']
                    lon = simulated_drone_variables[drone_id]['longitude']
                    alt = simulated_drone_variables[drone_id]['altitude']
                    new_speed = simulated_drone_variables[drone_id]['speed']
                    heading = simulated_drone_variables[drone_id]['heading']
                    
                    command = {
                        "action": "set_speed",
                        "drone_id": drone_id,
                        "speed": new_speed
                    }
                    send_command(command)
                    
                    print(f'Speed of Drone {drone_id} is set to {new_speed} m/s.')

                    return f'Speed of Drone {drone_id} is set to {new_speed} m/s.'

            def rotate_gimbal_pitch_to(drone_id, gimbal_pitch):

                if SIMULATION:

                    if gimbal_pitch < -90:
                        gimbal_pitch = -90

                    if gimbal_pitch > 20:
                        gimbal_pitch = 20

                    simulated_drone_variables[drone_id]['gimbal_pitch'] = gimbal_pitch

                    new_gimbal_pitch = simulated_drone_variables[drone_id]['gimbal_pitch']

                    print(f'Pitch Angle of Camera Gimbal of Drone {drone_id} is {new_gimbal_pitch}.')

                    return f'Pitch Angle of Camera Gimbal of Drone {drone_id} is {new_gimbal_pitch}.'

            def get_position(drone_id):

                if SIMULATION:
                    lat = simulated_drone_variables[drone_id]['latitude']
                    lon = simulated_drone_variables[drone_id]['longitude']
                    alt = simulated_drone_variables[drone_id]['altitude']
                    speed = simulated_drone_variables[drone_id]['speed']
                    heading = simulated_drone_variables[drone_id]['heading']

                    print(
                        f'Drone {drone_id} is at {lat}, {lon} GPS position with altitude {alt} and heading {heading}.')

                    return f'Drone {drone_id} is at {lat}, {lon} GPS position with altitude {alt} and heading {heading}.'

            def rotate_drone(drone_id, heading):

                if SIMULATION:
                    simulated_drone_variables[drone_id]['heading'] = heading

                    new_heading = simulated_drone_variables[drone_id]['heading']

                    return f'Heading of Drone {drone_id} is {new_heading}.'

            def take_image(drone_id, camera):
                pass

            try:

                # if instruction_type == "move":
                #     params = parameters.split(',')

                #     distance = float(params[0]) if len(params) > 0 else DEFAULT_DISTANCE
                #     direction = int(float(params[1].strip())) if len(params) > 1 else DEFAULT_HEADING
                #     abs_altitude = int(float(params[2].strip())) if len(params) > 2 else DEFAULT_ALTITUDE
                #     response = move(drone, distance, direction, abs_altitude)

                # el
                if instruction_type == "move_to":
                    params = parameters.split(',')

                    latitude = float(params[0])
                    longitude = float(params[1])
                    abs_altitude = int(float(params[2].strip())) if len(params) > 2 else DEFAULT_ALTITUDE
                    response = move_to(drone, latitude, longitude, abs_altitude)

                elif instruction_type == "set_drone_speed":
                    params = parameters.split(',')
                    speed = float(params[0]) if len(params) > 0 else DEFAULT_SPEED
                    response = set_drone_speed(drone, speed)

                elif instruction_type == "rotate_gimbal":
                    params = parameters.split(',')
                    pitch = float(params[0]) if len(params) > 0 else DEFAULT_ANGLE
                    response = rotate_gimbal_pitch_to(drone, pitch)

                elif instruction_type == "sleep":
                    params = parameters.split(',')
                    sleep_duration = float(params[0]) if len(params) > 0 else DEFAULT_SLEEP_DUR_MS
                    response = sleep(drone, sleep_duration)

                elif instruction_type == "get_position":
                    response = get_position(drone)

                # elif instruction_type == "rotate_drone":
                #     params = parameters.split(',')
                #     heading = float(params[0]) if len(params) > 0 else DEFAULT_HEADING
                #     response =  rotate_drone(drone, heading)

                # elif instruction_type == "take_image":
                #     params = parameters.split(',')
                #     camera_mode = params[0].strip() 
                #     response =  take_image(drone, camera_mode) 

                else:
                    response = f"Error: Unknown instruction {instruction_type} for drone {drone}"

                drone_responses.append(response)

            except Exception as e:
                drone_responses.append(f"Error: {e} for drone {drone}")

        return drone_responses

    individual_results = {}
    threads = []

    def worker(drone, instructions_for_drone):
        result = process_drone_instructions(drone, instructions_for_drone)
        individual_results[drone] = result  # Store the result for this drone

    for drone in drones:
        instructions_for_drone = drone_instructions[drone]
        thread = threading.Thread(target=worker, args=(drone, instructions_for_drone))
        threads.append(thread)
        thread.start()

    for thread in threads:
        thread.join()

    combined_responses = []
    for drone, responses in individual_results.items():
        combined_responses.extend(responses)

    if errors:
        combined_responses += errors

    print(f"Combined Responses: {combined_responses}")

    return combined_responses

# if __name__ == "__main__":
#     # Define two drones (drone IDs: 1 and 2)
    
#     drones = [1,2,3,4, 5]

#     # Instructions for the drones
#     instructions = [
#         {
#             "drone_id": 1,
#             "instructions": [
#                 #{"instruction": "move", "parameters": "15, 90, 50"},
#                 {"instruction": "set_drone_speed", "parameters": "3"},
#                 {"instruction": "move_to", "parameters": "48.002, 14.006, 25"},
#                 #{"instruction": "get_position", "parameters": ""},
                
#                 #{"instruction": "rotate_drone", "parameters": "90"},
#                 #{"instruction": "rotate_gimbal", "parameters": "-45"},
#                 # {"instruction": "take_image", "parameters": "91"},
#             ]
#         },
                    
#         {
#             "drone_id": 2,
#             "instructions": [
#                 # {"instruction": "move", "parameters": "15, 90, 50"},
#                 {"instruction": "set_drone_speed", "parameters": "3"},
#                 {"instruction": "move_to", "parameters": "48.005, 14.005, 30"},
#                 # {"instruction": "get_position", "parameters": ""},
                
#                 # {"instruction": "rotate_drone", "parameters": "270"},
#                 # {"instruction": "rotate_gimbal", "parameters": "-45"},
#                 # {"instruction": "take_image", "parameters": "90"},
#             ]
#         },
        
#         {
#             "drone_id": 3,
#             "instructions": [
#                 # {"instruction": "move", "parameters": "15, 90, 50"},
#                 {"instruction": "set_drone_speed", "parameters": "3"},
#                 {"instruction": "move_to", "parameters": "48.005, 14.005, 30"},
#                 # {"instruction": "get_position", "parameters": ""},
                
#                 # {"instruction": "rotate_drone", "parameters": "270"},
#                 # {"instruction": "rotate_gimbal", "parameters": "-45"},
#                 # {"instruction": "take_image", "parameters": "90"},
#             ]
#         }, 
        
#         {
#             "drone_id": 4,
#             "instructions": [
#                 # {"instruction": "move", "parameters": "15, 90, 50"},
#                 {"instruction": "set_drone_speed", "parameters": "3"},
#                 {"instruction": "move_to", "parameters": "48.007, 14.002, 30"},
#                 # {"instruction": "get_position", "parameters": ""},
                
#                 # {"instruction": "rotate_drone", "parameters": "270"},
#                 # {"instruction": "rotate_gimbal", "parameters": "-45"},
#                 # {"instruction": "take_image", "parameters": "90"},
#             ]
#         }, 
        
#         {
#             "drone_id": 5,
#             "instructions": [
#                 # {"instruction": "move", "parameters": "15, 90, 50"},
#                 {"instruction": "set_drone_speed", "parameters": "3"},
#                 {"instruction": "move_to", "parameters": "48.006, 14.003, 30"},
#                 # {"instruction": "get_position", "parameters": ""},
                
#                 # {"instruction": "rotate_drone", "parameters": "270"},
#                 # {"instruction": "rotate_gimbal", "parameters": "-45"},
#                 # {"instruction": "take_image", "parameters": "90"},
#             ]
#         }
#     ]
        
        
    
#     is_initial_prompt = True
#     # Call the drone controller with drones and instructions
#     combined_responses = simulation_drone_control_new(drones, instructions, is_initial_prompt)

#     # Output the result
#     print("\nFinal Combined Responses:", combined_responses)