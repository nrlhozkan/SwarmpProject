import socket
import json
import time

def send_command(command, host='localhost', port=5555):
    """Send a command to the simulation's command listener."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.connect((host, port))
            message = json.dumps(command) + '\n'
            sock.sendall(message.encode('utf-8'))
            response = sock.recv(4096).decode('utf-8')
            print("Response:", response)
    except ConnectionRefusedError:
        print("Failed to connect to the simulation server. Is it running?")
    except Exception as e:
        print(f"An error occurred: {e}")

# Define a single converging waypoint
collision_waypoint = [48.0025, 14.0010, 25.0]  # lat, lon, alt

commands = [
    {
        "action": "assign_waypoint_to_drone",
        "drone_id": 1,
        "waypoint": collision_waypoint
    },
    {
        "action": "assign_waypoint_to_drone",
        "drone_id": 2,
        "waypoint": collision_waypoint
    },
    {
        "action": "assign_waypoint_to_drone",
        "drone_id": 3,
        "waypoint": collision_waypoint
    },
    {
        "action": "assign_waypoint_to_drone",
        "drone_id": 4,
        "waypoint": collision_waypoint
    },
    {
        "action": "assign_waypoint_to_drone",
        "drone_id": 5,
        "waypoint": collision_waypoint
    }
]

# Send all commands
for command in commands:
    send_command(command)
    # Optional: add a slight delay if needed
    # time.sleep(0.5)
