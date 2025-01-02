import socket
import json
import time
import threading

def send_command(command, host='localhost', port=5555):
    """Send a command to the simulation's command listener."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.connect((host, port))
            message = json.dumps(command) + '\n'
            sock.sendall(message.encode('utf-8'))
            # Receive any immediate response
            response = sock.recv(4096).decode('utf-8')
            if response:
                print("Response:", response)
    except ConnectionRefusedError:
        print("Failed to connect to the simulation server. Is it running?")
    except Exception as e:
        print(f"An error occurred: {e}")

def listen_for_notifications(host='localhost', port=5556):
    """Listen for notifications from the simulation."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.bind((host, port))
            s.listen(5)
            print(f"Listening for notifications on {host}:{port}")
        except socket.error as e:
            print(f"Failed to bind for notifications: {e}")
            return
        while True:
            conn, addr = s.accept()
            print(f"Notification connection from {addr}")
            threading.Thread(target=handle_notification_connection, args=(conn,), daemon=True).start()

def handle_notification_connection(conn):
    with conn:
        while True:
            try:
                data = conn.recv(4096)
                if not data:
                    break
                messages = data.decode('utf-8').strip().split('\n')
                for message in messages:
                    if message:
                        try:
                            msg = json.loads(message)
                            print("Notification:", msg)
                        except json.JSONDecodeError:
                            print("Received invalid JSON notification.")
            except:
                break

def main():
    # Start the notification listener in a separate thread
    notif_thread = threading.Thread(target=listen_for_notifications, daemon=True)
    notif_thread.start()

    # Allow some time for the notification listener to start
    time.sleep(1)

    # Define waypoints for drones with unique command_ids
    commands = [
        {
            "action": "assign_waypoint_to_drone",
            "drone_id": 1,
            "waypoint": [48.0026, 14.0000, 25.0],  # (lat, lon, alt)
            "command_id": "cmd1"
        },
        {
            "action": "assign_waypoint_to_drone",
            "drone_id": 2,
            "waypoint": [48.0030, 14.0013, 30.0],  # (lat, lon, alt)
            "command_id": "cmd2"
        },
        {
            "action": "assign_waypoint_to_drone",
            "drone_id": 3,
            "waypoint": [48.0045, 14.0012, 35.0],  # (lat, lon, alt)
            "command_id": "cmd3"
        },
        {
            "action": "assign_waypoint_to_drone",
            "drone_id": 4,
            "waypoint": [48.0035, 14.0010, 40.0],  # (lat, lon, alt)
            "command_id": "cmd4"
        },
        {
            "action": "assign_waypoint_to_drone",
            "drone_id": 5,
            "waypoint": [48.0025, 14.0003, 45.0],  # (lat, lon, alt)
            "command_id": "cmd5"
        },
        # Assign drones to targets
        {
            "action": "assign_drone_to_target",
            "drone_id": 5,
            "target_id": 1
        },
        {
            "action": "assign_drone_to_target",
            "drone_id": 2,
            "target_id": 3
        }
    ]

    # Send all commands
    for command in commands:
        send_command(command)
        time.sleep(0.1)  # Small delay to ensure commands are processed

    # Keep the script running to receive notifications
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting external script.")

if __name__ == "__main__":
    main()
