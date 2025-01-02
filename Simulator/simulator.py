import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider, Button, RadioButtons
import threading
import queue
import time
import math
from typing import List, Tuple
import json
import socket
from pyproj import Transformer
import logging
import os
from datetime import datetime
import sys
import random
import zmq
import numpy as np
from utils import config

# Configure logging
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s [%(levelname)s] %(message)s',
    handlers=[
        logging.FileHandler("simulation.log"),
        logging.StreamHandler()
    ]
)

# Suppress Matplotlib debug messages
logging.getLogger('matplotlib').setLevel(logging.WARNING)
logging.getLogger('matplotlib.font_manager').setLevel(logging.WARNING)

# Define simulation boundaries
BORDER_LAT_RANGE = (48.0000, 48.0045)
BORDER_LON_RANGE = (14.0000, 14.00671)

def plot_drone_3d(ax, position, color='blue', name='D1'):
    """
    Plot a drone as a small circular marker and label in 3D.
    """
    lat, lon, alt = position
    drone_point = ax.scatter(lon, lat, alt, color=color, s=20, marker='o')
    label = ax.text(lon, lat, alt + 0.5, name, color='black', fontsize=10, ha='center', va='bottom')
    return [drone_point, label]

def plot_target_3d(ax, position, converter, color='red', name='T1'):
    """
    Plot a target as a small circular marker, label, and 50m radius circle in 3D.
    """
    lat, lon, alt = position
    target_point = ax.scatter(lon, lat, alt, color=color, s=20, marker='o')
    label = ax.text(lon, lat, alt + 0.5, name, color='black', fontsize=10, ha='center', va='bottom')

    radius = 100.0
    angles = np.linspace(0, 2*np.pi, 100) 
    x_center, y_center = converter.latlon_to_xy(lat, lon)
    circle_x = x_center + radius * np.cos(angles)
    circle_y = y_center + radius * np.sin(angles)
    circle_lats = []
    circle_lons = []
    for cx, cy in zip(circle_x, circle_y):
        clat, clon = converter.xy_to_latlon(cx, cy)
        circle_lats.append(clat)
        circle_lons.append(clon)
    circle_line, = ax.plot(circle_lons, circle_lats, [alt]*len(circle_lats),
                           color='green', linestyle='--')

    return [target_point, label, circle_line]

def plot_drone_2d(ax, lat, lon, color='blue', name='D1'):
    """
    Plot a drone as a small circular marker and label in 2D.
    """
    drone_point = ax.scatter(lon, lat, color=color, s=20, marker='o')
    label = ax.text(lon, lat + 0.00003, name, color='black', fontsize=10, ha='center', va='bottom')
    return [drone_point, label]

def plot_target_2d(ax, lat, lon, converter, color='red', name='T1'):
    """
    Plot a target as a small circular marker, label, and 50m radius circle in 2D.
    """
    target_point = ax.scatter(lon, lat, color=color, s=20, marker='o')
    label = ax.text(lon, lat + 0.00003, name, color='black', fontsize=10, ha='center', va='bottom')

    radius = 100.0
    angles = np.linspace(0, 2*np.pi, 100)
    x_center, y_center = converter.latlon_to_xy(lat, lon)
    circle_x = x_center + radius * np.cos(angles)
    circle_y = y_center + radius * np.sin(angles)
    circle_lats = []
    circle_lons = []
    for cx, cy in zip(circle_x, circle_y):
        clat, clon = converter.xy_to_latlon(cx, cy)
        circle_lats.append(clat)
        circle_lons.append(clon)
    circle_line, = ax.plot(circle_lons, circle_lats, color='green', linestyle='--')
    return [target_point, label, circle_line]

def draw_heading_arrow_2d(ax, lat, lon, heading, length=0.0005):
    """
    Draw a heading arrow in 2D using a quiver, with increased length and width for visibility.
    """
    math_angle = math.radians(90 - heading)
    dx = length * math.cos(math_angle)
    dy = length * math.sin(math_angle)
    arrow = ax.quiver(lon, lat, dx, dy, angles='xy', scale_units='xy', scale=1,
                      width=0.0020, headwidth=5, headlength=7, color='red')
    return arrow

class CoordinateConverter:
    """
    Handles coordinate conversions between lat/lon (WGS84) and a local UTM projection.
    """
    def __init__(self, ref_lat, ref_lon):
        utm_zone = int((ref_lon + 180) / 6) + 1
        hemisphere = 'north' if ref_lat >= 0 else 'south'
        epsg_code = 32600 + utm_zone if hemisphere == 'north' else 32700 + utm_zone
        self.transformer_to_utm = Transformer.from_crs("epsg:4326", f"epsg:{epsg_code}", always_xy=True)
        self.transformer_to_gps = Transformer.from_crs(f"epsg:{epsg_code}", "epsg:4326", always_xy=True)

    def latlon_to_xy(self, lat, lon):
        x, y = self.transformer_to_utm.transform(lon, lat)
        return x, y

    def xy_to_latlon(self, x, y):
        lon, lat = self.transformer_to_gps.transform(x, y)
        return lat, lon

class Drone:
    """
    Represents a drone with position, heading, and waypoints. 
    Can move towards assigned waypoints at a given speed.
    """
    def __init__(self, drone_id: int, initial_gps_position: Tuple[float, float, float], converter: CoordinateConverter):
        self.drone_id = drone_id
        self.gps_position = initial_gps_position
        self.x, self.y = converter.latlon_to_xy(initial_gps_position[0], initial_gps_position[1])
        self.z = initial_gps_position[2]
        self.position = (self.x, self.y, self.z)
        self.waypoint = None
        self.speed = 50.0
        self.is_moving = False
        self.waypoints = []
        self.tolerance = 0.1
        self.converter = converter
        self.last_position = self.position
        self.heading = 0.0
        self.reached = 0
        self.simulation = None

    def move_to(self, waypoint: Tuple[float, float, float]):
        """
        Set a waypoint for the drone and start moving towards it.
        """
        if not self.is_moving:
            self.waypoint = waypoint
            self.is_moving = True
            self.reached = 0
            logging.info(f"Drone {self.drone_id} starts moving to {self.waypoint}")
        else:
            logging.warning(f"Drone {self.drone_id} is already moving.")

    def set_speed(self, speed: float):
        """
        Set the drone's speed.
        """
        self.speed = speed
        logging.info(f"Drone {self.drone_id} speed set to {self.speed} m/s.")

    def update_position(self, time_step: float):
        """
        Update drone's position based on current speed and heading towards waypoint.
        """
        if self.waypoint is None:
            return
        dx = self.waypoint[0] - self.position[0]
        dy = self.waypoint[1] - self.position[1]
        dz = self.waypoint[2] - self.position[2]
        distance_to_waypoint = math.sqrt(dx**2 + dy**2 + dz**2)

        if distance_to_waypoint <= self.tolerance:
            # Waypoint reached
            self.position = self.waypoint
            self.x, self.y, self.z = self.position
            self.gps_position = self.converter.xy_to_latlon(self.x, self.y) + (self.z,)
            self.waypoint = None
            self.is_moving = False
            self.reached = 1
            self.update_heading()
            return

        ux = dx / distance_to_waypoint
        uy = dy / distance_to_waypoint
        uz = dz / distance_to_waypoint

        move_distance = self.speed * time_step
        move_distance = min(move_distance, distance_to_waypoint)

        self.last_position = self.position
        new_x = self.position[0] + ux * move_distance
        new_y = self.position[1] + uy * move_distance
        new_z = self.position[2] + uz * move_distance

        self.position = (new_x, new_y, new_z)
        self.x, self.y, self.z = self.position
        self.gps_position = self.converter.xy_to_latlon(self.x, self.y) + (self.z,)

        self.update_heading()

    def update_heading(self):
        """
        Update the drone's heading based on the last movement direction.
        """
        dx = self.x - self.last_position[0]
        dy = self.y - self.last_position[1]
        if abs(dx) > 1e-6 or abs(dy) > 1e-6:
            self.heading = math.atan2(dy, dx)

class Target:
    """
    Represents a moving target with a heading and speed.
    Can move autonomously and change heading randomly or at boundaries.
    """
    def __init__(self, target_id: int, initial_gps_position: Tuple[float, float, float], converter: CoordinateConverter):
        self.target_id = target_id
        self.gps_position = initial_gps_position
        self.x, self.y = converter.latlon_to_xy(initial_gps_position[0], initial_gps_position[1])
        self.z = initial_gps_position[2]
        self.position = (self.x, self.y, self.z)
        self.converter = converter
        self.speed = 10.0
        self.heading = random.uniform(0, 360)
        self.is_moving = False

    def set_position(self, new_gps_position: Tuple[float, float, float]):
        """
        Set the target's position directly.
        """
        self.gps_position = new_gps_position
        self.x, self.y = self.converter.latlon_to_xy(new_gps_position[0], new_gps_position[1])
        self.z = new_gps_position[2]
        self.position = (self.x, self.y, self.z)

    def randomize_heading(self):
        """
        Assign a random heading to the target.
        """
        self.heading = random.uniform(0, 360)

    def reverse_heading(self):
        """
        Reverse the target's heading by 180 degrees.
        """
        self.heading = (self.heading + 180) % 360

class Simulation:
    """
    Manages the simulation of drones and targets, processing commands, updating states,
    and recording data for later visualization.
    """
    def __init__(self, time_step: float = 0.05, pub_port=5557):
        self.drones: List[Drone] = []
        self.targets: List[Target] = []
        self.time_step = time_step
        self.running = True
        self.command_queue = queue.Queue()
        self.converter = None
        self.recorded_states = []
        self.assignments = {}
        self.final_positions_recorded = False
        self.waypoint_assigned_drones = []

        self.drones_reached = set()
        self.accept_new_commands = threading.Event()
        self.accept_new_commands.set()
        self.lock = threading.Lock()

        self.targets_should_move = False
        self.client_sockets = []
        # self.notif_socket = None

        # self.notif_host = notif_host
        # self.notif_port = notif_port
        self.pub_port = pub_port

        # self.setup_notification_socket()
        self.setup_publisher()

        self.command_listener_thread = threading.Thread(target=self.start_command_listener, daemon=True)
        self.command_listener_thread.start()

        self.simulation_time = 0.0
        self.last_heading_change_time = 0.0
        self.drone_commands = {}

    # def setup_notification_socket(self):
    #     """
    #     Set up a TCP socket to notify external scripts of events.
    #     """
    #     self.notif_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #     try:
    #         self.notif_socket.connect((self.notif_host, self.notif_port))
    #         logging.info(f"Connected to notification listener at {self.notif_host}:{self.notif_port}")
    #     except socket.error as e:
    #         logging.error(f"Failed to connect to notification listener: {e}")
    #         self.notif_socket = None

    def setup_publisher(self):
        """
        Set up a ZMQ publisher to broadcast drone/target positions.
        """
        self.context = zmq.Context()
        self.publisher = self.context.socket(zmq.PUB)
        try:
            self.publisher.bind(f"tcp://*:{self.pub_port}")
            logging.info(f"Publisher bound to port {self.pub_port}")
            time.sleep(1)
        except zmq.ZMQError as e:
            logging.error(f"Failed to bind publisher on port {self.pub_port}: {e}")
            self.running = False

    def publish_positions(self):
        """
        Publish current positions via ZeroMQ.
        """
        data = self.get_positions()
        message = json.dumps(data)
        try:
            self.publisher.send_string(message)
        except zmq.ZMQError as e:
            logging.error(f"ZeroMQ publish error: {e}")

    def simulation_update_loop(self):
        """
        Continuous loop updating simulation and publishing positions.
        """
        while self.running:
            self.update()
            self.publish_positions()
            time.sleep(self.time_step)

    def start_publishing(self):
        """
        Start the simulation update loop in a separate thread.
        """
        pub_thread = threading.Thread(target=self.simulation_update_loop, daemon=True)
        pub_thread.start()

    def send_feedback(self, message: dict):
        """
        Send feedback messages back to connected clients.
        """
        msg_str = json.dumps(message) + "\n"
        msg_bytes = msg_str.encode('utf-8')
        to_remove = []
        for client in self.client_sockets:
            try:
                client.sendall(msg_bytes)
            except Exception as e:
                logging.error(f"Error sending feedback to client: {e}")
                to_remove.append(client)
        for r in to_remove:
            self.client_sockets.remove(r)

    def notify_drone_reached(self, drone_id: int):
        """
        Notify that a drone reached its waypoint.
        If all assigned drones have reached, pause targets and allow new commands.
        """
        with self.lock:
            self.drones_reached.add(drone_id)
            logging.info(f"Drone {drone_id} has reached its waypoint.")
            
            if len(self.drones_reached) == len(self.waypoint_assigned_drones):
                self.targets_should_move = False
                for t in self.targets:
                    t.is_moving = False
                self.drones_reached.clear()
                self.waypoint_assigned_drones = []
                self.accept_new_commands.set()
                logging.info("All drones have reached their waypoints. Targets paused.")

    def record_current_state(self):
        """
        Record current simulation state for later replay or step visualization.
        """
        drone_positions = [drone.position for drone in self.drones]
        target_positions = [t.position for t in self.targets]
        drone_waypoints_gps = []
        drone_is_moving = [d.is_moving for d in self.drones]
        drone_reached = [d.reached for d in self.drones]
        for drone in self.drones:
            wps_gps = []
            for (wx, wy, wz) in drone.waypoints:
                lat, lon = self.converter.xy_to_latlon(wx, wy)
                wps_gps.append((lat, lon, wz))
            drone_waypoints_gps.append(wps_gps)

        self.recorded_states.append({
            "drones": drone_positions,
            "targets": target_positions,
            "waypoints": drone_waypoints_gps,
            "drone_is_moving": drone_is_moving,
            "drone_reached": drone_reached,
            "drone_headings": [math.degrees(d.heading) % 360 for d in self.drones],
            "target_headings": [t.heading for t in self.targets],
            "assignments": self.assignments.copy()
        })

    def add_drone(self, drone: Drone):
        """
        Add a drone to the simulation.
        """
        self.drones.append(drone)
        drone.simulation = self
        logging.info(f"Drone {drone.drone_id} added at {drone.position}")

    def add_target(self, target: Target):
        """
        Add a target to the simulation.
        """
        self.targets.append(target)
        logging.info(f"Target {target.target_id} added at {target.position}")

    def assign_waypoints(self, drone_waypoints: dict):
        """
        Assign multiple waypoints to multiple drones, waiting until they finish previous assignments.
        """
        self.accept_new_commands.wait()
        self.accept_new_commands.clear()

        for drone_id, waypoints in drone_waypoints.items():
            for wp in waypoints:
                self.assign_waypoint_to_drone(drone_id, tuple(wp), command_id=None)

    def assign_waypoint_to_drone(self, drone_id: int, waypoint: Tuple[float, float, float], command_id: str = None):
        """
        Assign a single waypoint to a specific drone.
        """
        self.waypoint_assigned_drones.append(drone_id)
        drone = next((d for d in self.drones if d.drone_id == drone_id), None)
        if drone:
            lat, lon, alt = waypoint
            x, y = self.converter.latlon_to_xy(lat, lon)
            z = alt
            drone.waypoints.append((x, y, z))
            if command_id:
                self.drone_commands[drone_id] = command_id
            if not drone.is_moving and len(drone.waypoints) == 1:
                next_wp = drone.waypoints.pop(0)
                drone.move_to(next_wp)
        else:
            logging.error(f"Drone {drone_id} not found.")

    def assign_drone_to_target(self, drone_id: int, target_id: int):
        """
        Assign or unassign a drone to a target.
        - If target_id is 0, it means the drone should have no assignment; 
        remove any existing assignment for that drone.
        - If target_id is non-zero, assign or reassign the drone to that target.
        Any previous assignment is overwritten.
        """
        logging.info(f"Current assignments: {self.assignments}")
        previous_target_id = self.assignments.get(drone_id)

        if target_id == 0:
            # Unassign the drone entirely
            if drone_id in self.assignments:
                self.assignments.pop(drone_id)
                logging.info(f"Drone {drone_id} unassigned from Target {previous_target_id}")
            else:
                logging.info(f"Drone {drone_id} was not assigned to any target.")
        else:
            # Assign the drone to the new target (overwrites old assignment)
            self.assignments[drone_id] = target_id
            if previous_target_id == target_id:
                logging.info(f"Drone {drone_id} is already assigned to Target {target_id}")
            elif previous_target_id and previous_target_id != target_id:
                logging.info(f"Drone {drone_id} reassigned from Target {previous_target_id} to Target {target_id}")
            else:
                logging.info(f"Drone {drone_id} assigned to Target {target_id}")

    def set_drone_speed(self, drone_id: int, speed: float, command_id: str = None):
        """
        Set the speed of a specific drone.
        """
        drone = next((d for d in self.drones if d.drone_id == drone_id), None)
        if drone:
            speed = speed * 10
            drone.set_speed(speed)
            if command_id:
                self.send_feedback({
                    "status": "speed_set",
                    "drone_id": drone_id,
                    "speed": speed,
                    "command_id": command_id
                })
        else:
            logging.error(f"Drone {drone_id} not found.")

    def set_target_position(self, target_id: int, position: Tuple[float, float, float]):
        """
        Set a target's position directly.
        """
        target = next((t for t in self.targets if t.target_id == target_id), None)
        if target:
            target.set_position(position)
        else:
            logging.error(f"Target {target_id} not found.")

    def process_commands(self, client_socket=None):
        """
        Process all commands in the queue.
        """
        while not self.command_queue.empty():
            cmd = self.command_queue.get()
            action = cmd.get("action")
            if action == "assign_waypoints":
                self.assign_waypoints(cmd.get("data", {}))
                self.targets_should_move = True
                for t in self.targets:
                    t.is_moving = True
            elif action == "assign_waypoint_to_drone":
                self.assign_waypoint_to_drone(cmd.get("drone_id"), tuple(cmd.get("waypoint")), command_id=None)
                self.targets_should_move = True
                for t in self.targets:
                    t.is_moving = True
            elif action == "assign_drone_to_target":
                self.assign_drone_to_target(cmd.get("drone_id"), cmd.get("target_id"))
            elif action == "set_speed":
                self.set_drone_speed(cmd.get("drone_id"), cmd.get("speed"), command_id=None)
            elif action == "set_target_position":
                self.set_target_position(cmd.get("target_id"), tuple(cmd.get("position")))
            elif action == "get_positions":
                positions = self.get_positions()
                if client_socket:
                    resp = json.dumps(positions).encode('utf-8')
                    client_socket.sendall(resp + b"\n")
            elif action == "stop":
                self.running = False
                logging.info("Simulation stopped.")
            else:
                logging.warning(f"Unknown action: {action}")

    def in_bounds(self, lat, lon):
        """
        Check if the given lat, lon is within simulation boundaries.
        """
        min_lat, max_lat = BORDER_LAT_RANGE
        min_lon, max_lon = BORDER_LON_RANGE
        return (min_lat < lat < max_lat) and (min_lon < lon < max_lon)

    def update_targets(self):
        """
        Update target positions, randomizing or reversing headings if needed.
        """
        if not self.targets_should_move:
            return

        current_time = self.simulation_time
        change_heading_due_time = (current_time - self.last_heading_change_time) >= random.uniform(5, 20)
        self.avoid_target_collisions()

        for t in self.targets:
            if t.is_moving:
                math_angle = math.radians(90 - t.heading)
                dx = t.speed * self.time_step * math.cos(math_angle)
                dy = t.speed * self.time_step * math.sin(math_angle)
                new_x = t.x + dx
                new_y = t.y + dy

                lat, lon = t.converter.xy_to_latlon(new_x, new_y)

                if not self.in_bounds(lat, lon):
                    t.reverse_heading()
                    math_angle = math.radians(90 - t.heading)
                    dx = t.speed * self.time_step * math.cos(math_angle)
                    dy = t.speed * self.time_step * math.sin(math_angle)
                    new_x = t.x + dx
                    new_y = t.y + dy

                    lat, lon = t.converter.xy_to_latlon(new_x, new_y)
                    if self.in_bounds(lat, lon):
                        t.x, t.y = new_x, new_y
                    else:
                        # Snap to boundary
                        min_lat, max_lat = BORDER_LAT_RANGE
                        min_lon, max_lon = BORDER_LON_RANGE

                        lat = max(min(lat, max_lat), min_lat)
                        lon = max(min(lon, max_lon), min_lon)
                        t.x, t.y = t.converter.latlon_to_xy(lat, lon)
                        t.position = (t.x, t.y, t.z)
                        t.randomize_heading()
                else:
                    t.x, t.y = new_x, new_y
                    t.position = (t.x, t.y, t.z)
                    t.gps_position = t.converter.xy_to_latlon(t.x, t.y) + (t.z,)

        if change_heading_due_time:
            for t in self.targets:
                t.randomize_heading()
            self.last_heading_change_time = current_time

    def avoid_target_collisions(self):
        """
        Avoid collisions by randomizing headings if targets get too close.
        """
        n = len(self.targets)
        for i in range(n):
            for j in range(i+1, n):
                t1 = self.targets[i]
                t2 = self.targets[j]
                dist = math.sqrt((t1.x - t2.x)**2 + (t1.y - t2.y)**2 + (t1.z - t2.z)**2)
                if dist < 1.0:
                    t1.randomize_heading()
                    t2.randomize_heading()

    def update_drones(self):
        """
        Update drone positions and check if they've reached their waypoints.
        """
        for drone in self.drones:
            if not drone.is_moving and drone.waypoints:
                next_wp = drone.waypoints.pop(0)
                drone.move_to(next_wp)

        for drone in self.drones:
            if drone.is_moving:
                drone.update_position(self.time_step)
                if drone.reached == 1:
                    self.notify_drone_reached(drone.drone_id)

    def all_drones_reached(self):
        """
        Check if all drones have reached their waypoints.
        """
        for d in self.drones:
            if d.is_moving or d.waypoints:
                return False
        return True

    def update(self):
        """
        Main update loop for the simulation: process commands, update targets/drones, record state.
        """
        self.process_commands()
        self.update_targets()
        self.update_drones()
        self.record_current_state()
        self.simulation_time += self.time_step

    def start_command_listener(self, host='localhost', port=5555):
        """
        Start a TCP server to listen for incoming commands.
        """
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            server_socket.bind((host, port))
            server_socket.listen(5)
            logging.info(f"Command listener started on {host}:{port}")
        except socket.error as e:
            logging.error(f"Failed to bind server on {host}:{port} due to {e}")
            return

        while self.running:
            try:
                client_socket, address = server_socket.accept()
                logging.info(f"Accepted connection from {address}")
                self.client_sockets.append(client_socket)
                threading.Thread(target=self.handle_client_connection, args=(client_socket,), daemon=True).start()
            except Exception as e:
                logging.error(f"Error accepting connections: {e}")
                break

        server_socket.close()
        logging.info("Command listener stopped.")

    def handle_client_connection(self, client_socket):
        """
        Handle a single client's connection, processing incoming JSON commands line by line.
        """
        with client_socket:
            data = b""
            while True:
                try:
                    packet = client_socket.recv(4096)
                    if not packet:
                        break
                    data += packet
                    try:
                        command_str = data.decode('utf-8')
                        commands = command_str.strip().split('\n')
                        for cmd in commands:
                            if cmd:
                                try:
                                    command = json.loads(cmd)
                                    self.command_queue.put(command)
                                    client_socket.sendall(b"Command received\n")
                                except json.JSONDecodeError:
                                    client_socket.sendall(b"Invalid JSON format\n")
                        data = b""
                    except Exception as e:
                        logging.error(f"Error processing commands: {e}")
                except Exception as e:
                    logging.error(f"Error handling client connection: {e}")
                    break
            if client_socket in self.client_sockets:
                self.client_sockets.remove(client_socket)

    def get_positions(self):
        """
        Get current positions of drones and targets as a dict.
        """
        drones_info = []
        for d in self.drones:
            lat, lon = d.converter.xy_to_latlon(d.x, d.y)
            drones_info.append({
                "drone_id": d.drone_id,
                "latitude": lat,
                "longitude": lon,
                "altitude": d.z,
                "heading": math.degrees(d.heading) % 360,
                "speed": d.speed,
                "reached": d.reached
            })
        targets_info = []
        for t in self.targets:
            lat, lon = t.converter.xy_to_latlon(t.x, t.y)
            targets_info.append({
                "target_id": t.target_id,
                "latitude": lat,
                "longitude": lon,
                "altitude": t.z,
                "heading": t.heading,
                "speed": t.speed
            })
        return {"drones": drones_info, "targets": targets_info}

    def cleanup(self):
        """
        Clean up ZeroMQ sockets and context when the simulation stops.
        """
        try:
            if self.publisher:
                self.publisher.close()
                logging.info("ZeroMQ publisher socket closed.")
            if self.context:
                self.context.term()
                logging.info("ZeroMQ context terminated.")
            # if self.notif_socket:
            #     self.notif_socket.close()
            #     logging.info("Notification socket closed.")
        except Exception as e:
            logging.error(f"Error during cleanup: {e}")

class Visualizer:
    """
    Visualizer for live simulation. Shows drones and targets in either 3D or 2D mode.
    Toggles between modes, updates their positions and headings live.
    """
    def __init__(self, simulation: Simulation, lat_range: Tuple[float, float], lon_range: Tuple[float, float]):
        self.simulation = simulation
        self.lat_range = lat_range
        self.lon_range = lon_range
        self.converter = simulation.converter

        # Create figure with both 3D and 2D axes
        self.fig = plt.figure(figsize=(16, 9), dpi=100, facecolor='white')
        self.ax_3d = self.fig.add_subplot(111, projection='3d')
        self.ax_3d.set_facecolor('white')
        self.ax_3d.grid(True)
        min_lon, max_lon = self.lon_range
        min_lat, max_lat = self.lat_range
        self.ax_3d.set_xlim(min_lon, max_lon)
        self.ax_3d.set_ylim(min_lat, max_lat)
        self.ax_3d.set_zlim(-1, 50)
        self.ax_3d.set_xlabel("Longitude (deg)")
        self.ax_3d.set_ylabel("Latitude (deg)")
        self.ax_3d.set_zlabel("Altitude (m)")
        self.ax_3d.set_title("Live Drone & Target Simulation (3D)")

        # 2D axes
        self.ax_2d = self.fig.add_axes([0.25, 0.1, 0.7, 0.7])  # [left, bottom, width, height]
        self.ax_2d.set_facecolor('white')
        self.ax_2d.grid(True)
        self.ax_2d.set_xlim(min_lon, max_lon)
        self.ax_2d.set_ylim(min_lat, max_lat)
        self.ax_2d.set_xlabel("Longitude (deg)")
        self.ax_2d.set_ylabel("Latitude (deg)")
        self.ax_2d.set_title("Live Drone & Target Simulation (2D)")
        self.ax_2d.set_visible(False)
        self.ax_2d.set_aspect('equal', adjustable='box')

        # Axis for the distance table
        self.ax_table = self.fig.add_axes([0.05, 0.1, 0.19, 0.4]) # [left, bottom, width, height]
        self.ax_table.axis('off')
        self.distance_table = None

        self.is_2d_mode = False
        self.drone_artists = []
        self.target_artists = []
        self.assignment_lines = []
        self.anim = None
        self.drone_info_text = []
        self.target_info_text = []
        self.fig.canvas.mpl_connect('close_event', self.on_close)

        # Add a button to toggle between 2D and 3D
        ax_button_2d3d = self.fig.add_axes([0.0, 0.0, 0.1, 0.05])  # [left, bottom, width, height]
        self.button_2d3d = Button(ax_button_2d3d, '2D/3D', color='white', hovercolor='0.975')
        self.button_2d3d.on_clicked(self.toggle_2d3d)

    def toggle_2d3d(self, event):
        """
        Toggle between 2D and 3D display modes.
        """
        self.is_2d_mode = not self.is_2d_mode
        if self.is_2d_mode:
            self.ax_3d.set_visible(False)
            self.ax_2d.set_visible(True)
        else:
            self.ax_2d.set_visible(False)
            self.ax_3d.set_visible(True)
        self.update_plot(None)
        self.fig.canvas.draw_idle()

    def clear_artists(self, ax):
        """
        Clear all artists (drones, targets, lines, text) from the current frame.
        """
        for artist in self.drone_artists + self.target_artists + self.assignment_lines + self.drone_info_text + self.target_info_text:
            try:
                artist.remove()
            except:
                pass
        self.drone_artists.clear()
        self.target_artists.clear()
        self.assignment_lines.clear()
        self.drone_info_text.clear()
        self.target_info_text.clear()

    def update_plot(self, frame):
        """
        Update the plot at each frame of the live animation.
        Includes updating a table of drone-target distances in meters.
        Altitude is excluded from distance calculations.
        """
        # If the figure is closed, stop the animation
        if not plt.fignum_exists(self.fig.number):
            if self.anim and self.anim.event_source:
                self.anim.event_source.stop()
            return

        # Decide which axes to use
        ax = self.ax_2d if self.is_2d_mode else self.ax_3d
        self.clear_artists(ax)

        # Clear and hide anything from a previous table
        self.ax_table.clear()
        self.ax_table.axis('off')

        # ---- Plot drones ----
        for i, drone in enumerate(self.simulation.drones):
            lat, lon = self.converter.xy_to_latlon(drone.x, drone.y)
            if self.is_2d_mode:
                drone_art = plot_drone_2d(ax, lat, lon, name=f'D{drone.drone_id}')
            else:
                alt = drone.z
                drone_art = plot_drone_3d(ax, (lat, lon, alt), name=f'D{drone.drone_id}')
            self.drone_artists.extend(drone_art)

        # ---- Plot targets ----
        for i, target in enumerate(self.simulation.targets):
            lat, lon = self.converter.xy_to_latlon(target.x, target.y)
            if self.is_2d_mode:
                target_art = plot_target_2d(ax, lat, lon, self.converter, name=f'T{target.target_id}')
            else:
                alt = target.z
                target_art = plot_target_3d(ax, (lat, lon, alt), self.converter, name=f'T{target.target_id}')
            self.target_artists.extend(target_art)

        # ---- Plot assignment lines ----
        for d_id, t_id in self.simulation.assignments.items():
            d = next((x for x in self.simulation.drones if x.drone_id == d_id), None)
            t = next((y for y in self.simulation.targets if y.target_id == t_id), None)
            if d and t:
                dlat, dlon = self.converter.xy_to_latlon(d.position[0], d.position[1])
                tlat, tlon = self.converter.xy_to_latlon(t.position[0], t.position[1])
                if self.is_2d_mode:
                    line, = ax.plot([dlon, tlon], [dlat, tlat], linestyle='--', linewidth=2, color='magenta')
                else:
                    dalt = d.position[2]
                    talt = t.position[2]
                    line, = ax.plot([dlon, tlon], [dlat, tlat], [dalt, talt],
                                    linestyle='--', linewidth=2, color='magenta')
                self.assignment_lines.append(line)

        # ---- Draw target headings ----
        for i, target in enumerate(self.simulation.targets):
            lat, lon = self.converter.xy_to_latlon(target.x, target.y)
            heading = target.heading
            if self.is_2d_mode:
                arrow = draw_heading_arrow_2d(ax, lat, lon, heading, length=0.00015)
                self.assignment_lines.append(arrow)
            else:
                math_angle = math.radians(90 - heading)
                dx = math.cos(math_angle)*0.00035
                dy = math.sin(math_angle)*0.00035
                alt = target.z
                arrow = ax.quiver(lon, lat, alt, dx, dy, 0,
                                  length=0.5, normalize=False, color='red',
                                  arrow_length_ratio=0.3)
                self.assignment_lines.append(arrow)

        # ---- Info text for drones ----
        p = 0.95
        for i, drone in enumerate(self.simulation.drones):
            lat, lon = self.converter.xy_to_latlon(drone.x, drone.y)
            alt = drone.z if not self.is_2d_mode else 0
            speed = drone.speed
            text = self.fig.text(
                0.02, p,
                f'D{drone.drone_id}: Lat={lat:.6f}, Lon={lon:.6f}, Alt={alt:.2f}m, Speed={speed:.2f} m/s',
                fontsize=10, ha='left', va='top'
            )
            self.drone_info_text.append(text)
            p -= 0.02

        # ---- Info text for targets ----
        p -= 0.05
        for i, target in enumerate(self.simulation.targets):
            lat, lon = self.converter.xy_to_latlon(target.x, target.y)
            alt = target.z if not self.is_2d_mode else 0
            heading = target.heading
            speed = target.speed
            text = self.fig.text(
                0.02, p - i*0.02,
                f'T{target.target_id}: Lat={lat:.6f}, Lon={lon:.6f}, Heading={heading:.2f}°',
                fontsize=10, ha='left', va='top'
            )
            self.target_info_text.append(text)

        # ---- Build and display distance table (in meters) ----
        # We compute only the horizontal (x,y) distance in UTM (ignore altitude).
        drone_ids = [f"D{d.drone_id}" for d in self.simulation.drones]
        target_ids = [f"T{t.target_id}" for t in self.simulation.targets]

        distances = []
        for d in self.simulation.drones:
            row = []
            for t in self.simulation.targets:
                dx = d.x - t.x
                dy = d.y - t.y
                # Skip altitude difference
                dist_m = math.sqrt(dx*dx + dy*dy)
                # if dist_m > 100: # similar to search and rescue script
                #     dist_m = -1
                row.append(f"{dist_m:.3f}")
            distances.append(row)

        # Create the table in ax_table
        self.distance_table = self.ax_table.table(
            cellText=distances,
            rowLabels=drone_ids,
            colLabels=target_ids,
            cellLoc='center',
            loc='upper left', 
            # if distances are less than and equal to 100, the cell color will be red
            cellColours=np.where(np.array(distances, dtype=float) < 100, 'yellow', 'white'),
        )
        # Adjust fonts and table scaling as needed
        self.distance_table.auto_set_font_size(False)
        self.distance_table.set_fontsize(8)
        self.distance_table.scale(1.2, 1.5)

    def start(self):
        """
        Start the live animation visualization.
        """
        self.anim = FuncAnimation(
            self.fig,
            self.update_plot,
            interval=int(self.simulation.time_step * 1000),
            blit=False,
            save_count=200
        )
        plt.show()

    def stop(self):
        """
        Stop the visualization gracefully.
        """
        if self.anim is not None and hasattr(self.anim, 'event_source') and self.anim.event_source is not None:
            try:
                self.anim.event_source.stop()
                logging.info("Animation event source stopped.")
            except Exception as e:
                logging.error(f"Error stopping animation event source: {e}")
            finally:
                self.anim = None
        if plt.fignum_exists(self.fig.number):
            plt.close(self.fig)
        logging.info("Visualization closed.")

    def on_close(self, event):
        """
        Handle figure close event by stopping the animation.
        """
        logging.info("Figure closed. Stopping visualization.")
        if self.anim and self.anim.event_source:
            try:
                self.anim.event_source.stop()
                self.anim = None
            except Exception as e:
                logging.error(f"Error stopping animation event source: {e}")
        self.stop()

def simulation_thread(simulation: Simulation):
    """
    Run the simulation updates in a separate thread.
    """
    while simulation.running:
        simulation.update()
        time.sleep(simulation.time_step)

def read_initial_positions_from_file():
    """
    Read initial drone positions from a JSON file.
    """
    logging.info("Reading initial drone positions...")
    try:
        with open(config.INITIAL_DRONE_POS_FILE, 'r') as f:
            data = json.load(f)
            logging.info(f"Read {len(data)} initial positions")
            return data, len(data)
    except FileNotFoundError:
        logging.error("File not found for initial positions.")
        return [], 0
    except json.JSONDecodeError:
        logging.error("Error decoding initial positions JSON.")
        return [], 0

def read_target_positions_from_file():
    """
    Read target positions from a JSON file.
    """
    logging.info("Reading target positions...")
    try:
        with open(config.TARGET_POS_FILE, 'r') as f:
            data = json.load(f)
            logging.info(f"Read {len(data)} target positions")
            return data, 0
    except FileNotFoundError:
        logging.error("Target positions file not found.")
        return [], 0
    except json.JSONDecodeError:
        logging.error("Error decoding target positions JSON.")
        return [], 0

def detect_steps(sim: Simulation):
    """
    Detect 'steps' in the simulation. A step is a movement sequence after which all drones are idle.
    """
    if not sim.recorded_states:
        return []
    def all_idle(frame):
        st = sim.recorded_states[frame]
        ways = st["waypoints"]
        moves = st["drone_is_moving"]
        return all(len(wp) == 0 for wp in ways) and all(not mv for mv in moves)

    steps = []
    num_frames = len(sim.recorded_states)
    previously_idle = all_idle(0)
    moved_since_idle = False
    start_of_step = None

    for f in range(num_frames):
        idle = all_idle(f)
        if not idle and previously_idle:
            start_of_step = f
            moved_since_idle = True
        if idle and moved_since_idle:
            steps.append((f"Step {len(steps)+1}", start_of_step, f))
            moved_since_idle = False
            start_of_step = None
        previously_idle = idle

    if moved_since_idle and start_of_step is not None:
        steps.append((f"Step {len(steps)+1}", start_of_step, num_frames-1))

    return steps

def create_step_view_auto(sim: Simulation, lat_range: Tuple[float, float], lon_range: Tuple[float, float]):
    """
    Create a step-based visualization with a slider, step selection, and a show paths button.
    Both 2D and 3D views are available, toggled by a button.
    """
    steps = detect_steps(sim)
    if not sim.recorded_states:
        logging.warning("No recorded states found. Nothing to show.")
        return

    if not steps:
        logging.warning("No steps detected (no movement). Nothing to show.")
        return

    all_states = sim.recorded_states
    movement_frames = []
    for (name, start, end) in steps:
        movement_frames.extend(range(start, end+1))
    movement_frames = sorted(movement_frames)

    all_steps_entry = ("All Steps", movement_frames)
    step_frame_data = [all_steps_entry] + [(name, list(range(start, end+1))) for (name, start, end) in steps]

    num_frames = len(all_states)
    drone_positions_per_frame = []
    target_positions_per_frame = []
    target_headings_per_frame = []

    for f in range(num_frames):
        state = all_states[f]
        dpf = []
        tpf = []
        thf = state["target_headings"]
        for (x, y, z) in state["drones"]:
            lat, lon = sim.converter.xy_to_latlon(x, y)
            dpf.append((lon, lat, z))
        for (x, y, z) in state["targets"]:
            lat, lon = sim.converter.xy_to_latlon(x, y)
            tpf.append((lon, lat, z))
        drone_positions_per_frame.append(dpf)
        target_positions_per_frame.append(tpf)
        target_headings_per_frame.append(thf)

    fig = plt.figure(figsize=(16, 9), dpi=100, facecolor='white')
    ax_3d = fig.add_subplot(111, projection='3d')
    ax_3d.set_facecolor('white')
    ax_3d.grid(True)
    min_lon, max_lon = lon_range
    min_lat, max_lat = lat_range
    ax_3d.set_xlim(min_lon, max_lon)
    ax_3d.set_ylim(min_lat, max_lat)
    ax_3d.set_zlim(-1, 50)
    ax_3d.set_xlabel("Longitude (deg)")
    ax_3d.set_ylabel("Latitude (deg)")
    ax_3d.set_zlabel("Altitude (m)")
    ax_3d.set_title("Step-based Drone Simulation Viewer (3D)")

    ax_2d = fig.add_axes([0.1, 0.1, 0.8, 0.8])
    ax_2d.set_facecolor('white')
    ax_2d.grid(True)
    ax_2d.set_xlim(min_lon, max_lon)
    ax_2d.set_ylim(min_lat, max_lat)
    ax_2d.set_xlabel("Longitude (deg)")
    ax_2d.set_ylabel("Latitude (deg)")
    ax_2d.set_title("Step-based Drone Simulation Viewer (2D)")
    ax_2d.set_visible(False)

    is_2d_mode = False
    drone_artists = []
    target_artists = []
    assignment_lines = []
    arrows = []

    step_names = [s[0] for s in step_frame_data]
    current_step_index = 0
    current_step_name, current_frames = step_frame_data[current_step_index]

    def duration_for_frames(frames):
        if not frames:
            return 0
        return (len(frames)-1) * sim.time_step

    current_duration = duration_for_frames(current_frames)

    ax_slider = plt.axes([0.25, 0.05, 0.50, 0.03], facecolor='lightgoldenrodyellow')
    slider = Slider(ax_slider, 'Time', 0, current_duration, valinit=0, valstep=sim.time_step, valfmt='%.4f s')

    # Lines for 3D paths
    full_path_lines_3d = []
    for _ in sim.drones:
        line_3d, = ax_3d.plot([], [], [], color='black', linewidth=2, alpha=0.5, label='_nolegend_')
        full_path_lines_3d.append(line_3d)

    # Lines for 2D paths
    full_path_lines_2d = []
    for _ in sim.drones:
        line_2d, = ax_2d.plot([], [], color='black', linewidth=2, alpha=0.5, label='_nolegend_')
        line_2d.set_visible(False)
        full_path_lines_2d.append(line_2d)

    collision_markers = []
    for _ in sim.drones:
        cm = ax_3d.scatter([], [], [], color='orange', marker='o', s=100, label='_nolegend_', alpha=0.8)
        collision_markers.append(cm)

    ax_radio = plt.axes([0.05, 0.6, 0.15, 0.25], facecolor='lightgoldenrodyellow')
    radio = RadioButtons(ax_radio, step_names, active=0)

    paths_shown = False
    ax_button = plt.axes([0.85, 0.9, 0.1, 0.05], facecolor='lightgoldenrodyellow')
    button = Button(ax_button, 'Show Paths', color='white', hovercolor='0.975')
    
    ax_button_2d3d = plt.axes([0.9, 0.0, 0.1, 0.05], facecolor='lightgoldenrodyellow') 
    button_2d3d = Button(ax_button_2d3d, '2D/3D', color='white', hovercolor='0.975')

    def clear_artists():
        for artist in drone_artists + target_artists + assignment_lines + arrows:
            try:
                artist.remove()
            except:
                pass
        drone_artists.clear()
        target_artists.clear()
        assignment_lines.clear()
        arrows.clear()

    def update_plot(time_val):
        if not current_frames:
            return
        local_idx = int(round(time_val / sim.time_step))
        if local_idx < 0:
            local_idx = 0
        if local_idx >= len(current_frames):
            local_idx = len(current_frames) - 1
        frame_idx = current_frames[local_idx]

        state = all_states[frame_idx]
        d_pos_gps = drone_positions_per_frame[frame_idx]
        t_pos_gps = target_positions_per_frame[frame_idx]
        t_headings = target_headings_per_frame[frame_idx]
        assignments = state.get("assignments", {})

        ax_current = ax_2d if is_2d_mode else ax_3d
        other_ax = ax_3d if is_2d_mode else ax_2d
        other_ax.set_visible(False)
        ax_current.set_visible(True)

        clear_artists()

        # Plot drones
        for i, drone_pos in enumerate(d_pos_gps):
            lat = drone_pos[1]
            lon = drone_pos[0]
            alt = drone_pos[2] if not is_2d_mode else 0
            if is_2d_mode:
                d_art = plot_drone_2d(ax_current, lat, lon, name=f'D{sim.drones[i].drone_id}')
            else:
                d_art = plot_drone_3d(ax_current, (lat, lon, alt), name=f'D{sim.drones[i].drone_id}')
            drone_artists.extend(d_art)

        # Plot targets
        for i, target_pos in enumerate(t_pos_gps):
            lat = target_pos[1]
            lon = target_pos[0]
            alt = target_pos[2] if not is_2d_mode else 0
            if is_2d_mode:
                t_art = plot_target_2d(ax_current, lat, lon, sim.converter, name=f'T{sim.targets[i].target_id}')
            else:
                t_art = plot_target_3d(ax_current, (lat, lon, alt), sim.converter, name=f'T{sim.targets[i].target_id}')
            target_artists.extend(t_art)

        # Plot assignments
        for d_id, t_id in assignments.items():
            d_idx = None
            t_idx = None
            for idx, dr in enumerate(sim.drones):
                if dr.drone_id == d_id:
                    d_idx = idx
                    break
            for idx, tg in enumerate(sim.targets):
                if tg.target_id == t_id:
                    t_idx = idx
                    break
            if d_idx is not None and t_idx is not None:
                dpos = d_pos_gps[d_idx]
                tpos = t_pos_gps[t_idx]
                dlat, dlon = dpos[1], dpos[0]
                tlat, tlon = tpos[1], tpos[0]
                dalt = dpos[2] if not is_2d_mode else 0
                talt = tpos[2] if not is_2d_mode else 0
                if is_2d_mode:
                    line, = ax_current.plot([dlon, tlon], [dlat, tlat], color='magenta', linestyle='--', linewidth=2)
                else:
                    line, = ax_current.plot([dlon, tlon], [dlat, tlat], [dalt, talt], color='magenta', linestyle='--', linewidth=2)
                assignment_lines.append(line)

        # Arrows for targets
        for i, (tpos, heading) in enumerate(zip(t_pos_gps, t_headings)):
            lat = tpos[1]
            lon = tpos[0]
            if is_2d_mode:
                arrow = draw_heading_arrow_2d(ax_current, lat, lon, heading, length=0.00015)
                arrows.append(arrow)
            else:
                math_angle = math.radians(90 - heading)
                dx = math.cos(math_angle)*0.00035
                dy = math.sin(math_angle)*0.00035
                alt = tpos[2]
                arrow = ax_current.quiver(lon, lat, alt, dx, dy, 0, length=0.5, normalize=False, color='red', arrow_length_ratio=0.3)
                arrows.append(arrow)

    def on_slider_change(val):
        update_plot(val)

    slider.on_changed(on_slider_change)

    def on_radio_click(label):
        nonlocal current_step_index, current_step_name, current_frames, current_duration
        current_step_index = step_names.index(label)
        current_step_name, current_frames = step_frame_data[current_step_index]
        current_duration = duration_for_frames(current_frames)
        if current_duration < 0:
            current_duration = 0
        slider.valmax = current_duration
        slider.ax.set_xlim(0, current_duration)
        slider.set_val(0)
        update_plot(0)
        fig.canvas.draw_idle()

    radio.on_clicked(on_radio_click)

    def on_button_clicked(event):
        nonlocal paths_shown
        paths_shown = not paths_shown
        selected_positions = []
        for f in current_frames:
            selected_positions.append(drone_positions_per_frame[f])
        lon_paths = [[] for _ in sim.drones]
        lat_paths = [[] for _ in sim.drones]
        alt_paths = [[] for _ in sim.drones]
        for sp in selected_positions:
            for i, (lon, lat, alt) in enumerate(sp):
                lon_paths[i].append(lon)
                lat_paths[i].append(lat)
                alt_paths[i].append(alt)

        if paths_shown:
            # Update both 2D and 3D paths
            for i, line_3d in enumerate(full_path_lines_3d):
                if lat_paths[i]:
                    line_3d.set_data(lon_paths[i], lat_paths[i])
                    line_3d.set_3d_properties(alt_paths[i])
                    line_3d.set_visible(not is_2d_mode)
                else:
                    line_3d.set_visible(False)

            for i, line_2d in enumerate(full_path_lines_2d):
                if lat_paths[i]:
                    line_2d.set_data(lon_paths[i], lat_paths[i])
                    line_2d.set_visible(is_2d_mode)
                else:
                    line_2d.set_visible(False)
            button.label.set_text('Hide Paths')
        else:
            for line_3d in full_path_lines_3d:
                line_3d.set_visible(False)
            for line_2d in full_path_lines_2d:
                line_2d.set_visible(False)
            button.label.set_text('Show Paths')

        fig.canvas.draw_idle()

    button.on_clicked(on_button_clicked)

    def toggle_2d3d_mode(event):
        nonlocal is_2d_mode
        is_2d_mode = not is_2d_mode
        if is_2d_mode:
            ax_3d.set_visible(False)
            ax_2d.set_visible(True)
            if paths_shown:
                for i, line_2d in enumerate(full_path_lines_2d):
                    line_2d.set_visible(True)
                for line_3d in full_path_lines_3d:
                    line_3d.set_visible(False)
        else:
            ax_2d.set_visible(False)
            ax_3d.set_visible(True)
            if paths_shown:
                for line_3d in full_path_lines_3d:
                    line_3d.set_visible(True)
                for line_2d in full_path_lines_2d:
                    line_2d.set_visible(False)
        update_plot(slider.val)
        fig.canvas.draw_idle()

    button_2d3d.on_clicked(toggle_2d3d_mode)

    update_plot(0)
    plt.show()

def create_video_from_movement_frames(sim: Simulation, output_filename: str, lat_range: Tuple[float, float], lon_range: Tuple[float, float], movement_frames: List[int]):
    """
    Create a video (MP4) from recorded movement frames with drones and targets.
    """
    if not movement_frames:
        logging.warning("No movement frames found, no video will be created.")
        return

    fig = plt.figure(figsize=(20, 15), dpi=100, facecolor='white')
    ax = fig.add_subplot(111, projection='3d')
    ax.set_facecolor('white')
    ax.grid(True)

    min_lon, max_lon = lon_range
    min_lat, max_lat = lat_range
    ax.set_xlim(min_lon, max_lon)
    ax.set_ylim(min_lat, max_lat)
    ax.set_zlim(-1, 50)
    ax.set_xlabel("Longitude (deg)")
    ax.set_ylabel("Latitude (deg)")
    ax.set_zlabel("Altitude (m)")
    ax.set_title("Recorded Mission Playback (No Wait Times)")
    ax.grid(True)

    drone_artists = []
    target_artists = []
    target_arrows = []
    assignment_lines = {}

    arrow_length = 0.0005

    all_states = sim.recorded_states

    def init():
        return []

    def update(i):
        frame = movement_frames[i]
        state = all_states[frame]
        d_pos = []
        t_pos = []
        t_head = state["target_headings"]
        assignments = state.get("assignments", {})

        for (x, y, z) in state["drones"]:
            lat, lon = sim.converter.xy_to_latlon(x, y)
            d_pos.append((lon, lat, z))
        for (x, y, z) in state["targets"]:
            lat, lon = sim.converter.xy_to_latlon(x, y)
            t_pos.append((lon, lat, z))

        for artist in drone_artists:
            try:
                artist.remove()
            except:
                pass
        drone_artists.clear()
        for artist in target_artists:
            try:
                artist.remove()
            except:
                pass
        target_artists.clear()
        for line in assignment_lines.values():
            try:
                line.remove()
            except:
                pass
        assignment_lines.clear()

        # Plot drones
        for idx, drone_pos in enumerate(d_pos):
            if idx < len(sim.drones):
                drone_art = plot_drone_3d(ax, (drone_pos[1], drone_pos[0], drone_pos[2]), name=f'D{sim.drones[idx].drone_id}')
                drone_artists.extend(drone_art)

        # Plot targets
        for idx, target_pos in enumerate(t_pos):
            if idx < len(sim.targets):
                target_art = plot_target_3d(ax, (target_pos[1], target_pos[0], target_pos[2]), sim.converter, name=f'T{sim.targets[idx].target_id}')
                target_artists.extend(target_art)

        # Plot assignments
        for d_id, t_id in assignments.items():
            d_idx = None
            t_idx = None
            for ddx, dr in enumerate(sim.drones):
                if dr.drone_id == d_id:
                    d_idx = ddx
                    break
            for ttx, tg in enumerate(sim.targets):
                if tg.target_id == t_id:
                    t_idx = ttx
                    break
            if d_idx is not None and t_idx is not None:
                dpos = d_pos[d_idx]
                tpos = t_pos[t_idx]
                line, = ax.plot(
                    [dpos[0], tpos[0]],
                    [dpos[1], tpos[1]],
                    [dpos[2], tpos[2]],
                    color='green',
                    linestyle='--',
                    linewidth=1,
                    label='_nolegend_'
                )
                assignment_lines[d_id] = line

        # Update target arrows
        for idx, (tpos, heading) in enumerate(zip(t_pos, t_head)):
            if idx < len(target_arrows):
                try:
                    target_arrows[idx].remove()
                except:
                    pass
            lon, lat, alt = tpos
            math_angle = math.radians(90 - heading)
            dx = math.cos(math_angle) * arrow_length
            dy = math.sin(math_angle) * arrow_length
            arrow = ax.quiver(
                lon, lat, alt,
                dx, dy, 0,
                length=0.5,
                normalize=False,
                color='red',
                arrow_length_ratio=0.3
            )
            if idx < len(target_arrows):
                target_arrows[idx] = arrow
            else:
                target_arrows.append(arrow)

        return []

    anim = FuncAnimation(fig, update, frames=len(movement_frames), init_func=init, blit=False)
    writer = FFMpegWriter(fps=30, bitrate=1800)
    try:
        anim.save(output_filename, writer=writer)
        logging.info(f"Video saved to {output_filename}")
    except Exception as e:
        logging.error(f"Failed to create video: {e}")
    finally:
        plt.close(fig)

def main():
    """
    Run the simulation in stand-alone mode: read initial positions, start simulation and visualization.
    """
    sim = Simulation(time_step=0.05)
    initial, n_d = read_initial_positions_from_file()

    if not initial:
        logging.error("No initial drone positions found. Exiting simulation.")
        return None, None

    ref_lat = BORDER_LAT_RANGE[0] 
    ref_lon = BORDER_LON_RANGE[0]
    
    sim.converter = CoordinateConverter(ref_lat, ref_lon)
    logging.info(f"Reference point set to latitude: {ref_lat}, longitude: {ref_lon}")

    for i, pos in enumerate(initial):
        gps = (pos["latitude"], pos["longitude"], pos.get("altitude", 0))
        d = Drone(i+1, gps, sim.converter)
        x, y = sim.converter.latlon_to_xy(gps[0], gps[1])
        d.position = (x, y, d.z)
        sim.add_drone(d)

    targets, nt = read_target_positions_from_file()
    for i, pos in enumerate(targets):
        gps = (pos["latitude"], pos["longitude"], pos.get("altitude", 0))
        t = Target(i+1, gps, sim.converter)
        x, y = sim.converter.latlon_to_xy(gps[0], gps[1])
        t.position = (x, y, t.z)
        t.is_moving = False
        sim.add_target(t)

    current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_folder = os.path.join("simulations", current_time)
    os.makedirs(out_folder, exist_ok=True)
    logging.info(f"Created output folder: {out_folder}")
    global output_folder
    output_folder = out_folder
    sim.start_publishing()
    return sim, out_folder

if __name__ == "__main__":
    sim, output_folder = main()
    
    logging.info("Starting simulation...")
    logging.info(f"Output folder: {output_folder}")
    with open("output_folder.txt", "w") as text_file:
        text_file.write(output_folder)
    
    if sim is None:
        sys.exit(1)

    lat_range = (47.9990, 48.0055)
    lon_range = (13.99990, 14.00681)

    sim_thread = threading.Thread(target=simulation_thread, args=(sim,), daemon=True)
    sim_thread.start()

    try:
        vis = Visualizer(sim, lat_range, lon_range)
        vis.start()
    except KeyboardInterrupt:
        logging.info("KeyboardInterrupt received, stopping simulation.")
        sim.running = False
        vis.stop()
    finally:
        if sim_thread.is_alive():
            sim_thread.join()

        sim.cleanup()

        data_file_path = os.path.join(output_folder, 'recorded_states.json')
        with open(data_file_path, 'w') as f:
            json.dump(sim.recorded_states, f, indent=4)
        logging.info(f"Recorded data saved to {data_file_path}")

        print(sim)
        
        steps = detect_steps(sim)
        movement_frames = []
        for (name, start, end) in steps:
            movement_frames.extend(range(start, end+1))
        movement_frames = sorted(movement_frames)

        create_step_view_auto(sim, lat_range, lon_range)

        video_file_path = os.path.join(output_folder, 'simulation.mp4')
        create_video_from_movement_frames(sim, video_file_path, lat_range, lon_range, movement_frames)
