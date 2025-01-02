## Description

The Drone Target Simulation is a Python-based tool designed to simulate drone movements and target interactions within specified geographical boundaries. It provides both 2D and 3D visualizations, real-time updates and the ability to record simulation data as video for later analysis.

## Table of Contents

- [Installation](#Installation)
- [Setup](#Setup)
- [HowToRun](#HowToRun)

# Installation

1. The script uses FFmpeg to create videos from simulation frames. Download and install from [FFmpeg]https://ffmpeg.org/download.html).
2. Required python libraries.

```bash
pip install matplotlib pyproj zmq numpy
```

# Setup

The simulation requires two JSON files containing the initial and current positions of the drones and targets, all created inside the data folder by search and rescue file (search_and_rescue_prompt_script_new_myChanges.py). You can change the file locations within the scripts (config and simulation_drone_control_new) **to do **.

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
2. Run the search and rescue script in a separate terminal (**terminal 1**).
3. Press **g**. This will save the positions of the drones and targets.
4. Run the simulation script in a separate terminal (**terminal2**).
5. After step 5 this will be a normal process.
6. If you decide to finish the test. Press **ctrl + c** on **terminal 2**.
7. This will stop the live simulation and show you another simulation. This has sliders and test steps. Each waypoint command is a step.
8. When you close the simulation, it will save the simulation as a video inside the simulations folder. And it will save all the data from the simulation as a JSON file. You can use this to show and view the simulation again later. You can use post_processor.py

# Specific Scripts

1. Simulation script (simulator.py)

This is a live simulator. It reads the initial positions of drones and targets and visualises them at the beginning. When Charlie sends waypoint commands, it visualises them live. This simulation has 3D and 2D options. You can select them as you like. You can send commands like (assign_waypoint_to_drone, set_speed, assign_target_to_drone). See test_1 and test_2 for examples. It has interval bug, sometimes it appears after pressing ctrl + c related to mathplotlib. I could not fix it yet. It also publishes the telemetry data of drones and targets. You can find it in my drone control script (simulation_drone_control_new.py). You can rearrange it if you like.

A few notes: The speeds of the drones are multiplied by 10 to reach the desired waypoints quickly. If Charlie set the speed to 3m/s, it will be 30m/s in the simulation. The speed of the targets is 20m/s. The targets move randomly. If you send a waypoint to drones, charlie will wait for all drones to reach the desired or (commanded) waypoint. Then you will get your answer. It will be the same in the real world.

2. Post Processor

This is a post processor that simulates the same simulation after pressing crtl + c in the main simulation. When you run it, it will ask you to specify the location of the recorded states file (recorded_states.json). (*The location of the file is where you run the simulation script*)

3. The rest is mostly the same code. I just add some functionality.
