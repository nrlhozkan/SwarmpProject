## Description 

The Drone Target Simulation is a Python-based tool designed to simulate drone movements and target interactions within specified geographical boundaries. It offers both 2D and 3D visualizations, real-time updates, and the ability to record simulation data for later analysis as a video.

## Table of Contents
- [Installation](#Installation)
- [Setup](#Setup)
- [HowToRun](#HowToRun)


#  Installation
1. The script uses FFmpeg to create videos from simulation frames. Download and install from [FFmpeg]https://ffmpeg.org/download.html). 

2. Required python libraries.

```bash 
pip install matplotlib pyproj zmq numpy
```
# Setup

The simulation requires two JSON files containing the initial and current positions of drones and targets.They will be all created inside the Data folder by search and rescue file(search_and_rescue_prompt_script_new_myChanges.py). You can change file locations inside the scripts **to do**. 

```makefile
a. Initial drone positions
C:\Users\cgadmin\Desktop\charlie-mnemonic-dev\charlie-mnemonic-dev\Data\charlie_shared_data\initial_drone_pos.json

b. Initial Target Positions
C:\Users\cgadmin\Desktop\charlie-mnemonic-dev\charlie-mnemonic-dev\Data\other_data\initial_target_positions.json

c. Drone Positions
C:\Users\cgadmin\Desktop\charlie-mnemonic-dev\charlie-mnemonic-dev\Data\charlie_shared_data\drone_pos.json

d. Target Positions
C:\Users\cgadmin\Desktop\charlie-mnemonic-dev\charlie-mnemonic-dev\Data\other_data\initial_target_positions.json


```

1. Initial Drone Positions

```json


[
    {
        "id": 1,
        "latitude": 48.001000000000005,
        "longitude": 14.0005,
        "altitude": 30
    },
    {
        "id": 2,
        "latitude": 48.001000000000005,
        "longitude": 14.0015,
        "altitude": 30
    },
    {
        "id": 3,
        "latitude": 48.00216519317647,
        "longitude": 14.002166880122928,
        "altitude": 30
    },
    {
        "id": 4,
        "latitude": 48.001000000000005,
        "longitude": 14.0035,
        "altitude": 30
    },
    {
        "id": 5,
        "latitude": 48.000738829240824,
        "longitude": 14.003877607355538,
        "altitude": 30
    }
]

```

2. Target Positions

```json
[
    {
        "id": 1,
        "latitude": 48.0025626818807,
        "longitude": 14.006563740378258
    },
    {
        "id": 2,
        "latitude": 48.000738829240824,
        "longitude": 14.003877607355538
    },
    {
        "id": 3,
        "latitude": 48.00216519317647,
        "longitude": 14.002166880122926
    }
]
```
# HowToRun

1. Start Charlie
2. Run search and rescue script in separate terminal (**terminal 1**).  
3. Press **g**. This will save positions of drones and targets. 
4. Run Simulation Script in another terminal (**terminal2**)
5. After that press **g** again. 
6. After step 5, this will be normal process. 
7. When you decide to finish the test. Press **crtl + c** in **terminal 2**.
8. This will stop live simulation and show you other simulation. This has slider and steps of test. Every waypoint command means one step. 
9. If you close the simulation it will save the simulation as a video inside the simulations folder. And it will save all the data of the simulation as a JSON file. You can use this for later to show and see the simulation again. You can use post_processor.py 

# Specific Scripts

1. Simulation Script (simulator.py)

It is a live simulator. It reads the initial positions of drones and targets and visualize it in the beginning. When charlie send waypoint commands it visualize them live. This simulation has 3D and 2D options. You can select them as you want. You can send commands like (assign_waypoint_to_drone, set_speed, assign_target_to_drone). Find examples of them in test_1 and test_2 files. It has interval bug, sometimes it appears after pressing ctrl + c related to mathplotlib. I could not fixed yet. It also publish the telemetry data of drones and targets. You can find them in my drone control script (simulation_drone_control_new.py). If you need you can arrange them.

Notes: The speeds of drones are multiplied by 10 with to quickly reach the desired waypoints. If charlie set the speed to 3m/s, it will be 30 m/s in simulation. The speed of targets are 20 m/s. The targets moves randomly. When you send a waypoint to drones, charlie will wait for all drones until they all reach the desired or (commanded) waypoints. Then you will get your response. It will be same in real world application.

2. Post Processor

It is a post processor that simulate the same simulation after pressing crtl + c in the main simulation. When you run it, it will ask you to provide the file location of the recorded states (recorded_states.json). (*The file location will be where you run the simulation script*)

3. The others are mostly same codes. I just add some functionalities.




