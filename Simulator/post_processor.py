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

def detect_steps(recorded_states):
    """
    Detect 'steps' in the simulation. A step is a movement sequence after which all drones are idle.
    """
    if not recorded_states:
        return []
    def all_idle(frame):
        st = recorded_states[frame]
        ways = st["waypoints"]
        moves = st["drone_is_moving"]
        return all(len(wp) == 0 for wp in ways) and all(not mv for mv in moves)

    steps = []
    num_frames = len(recorded_states)
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

def read_initial_positions_from_file():
    """
    Read initial drone positions from a JSON file.
    """
    logging.info("Reading initial drone positions...")
    try:
        with open(r'C:\Users\cgadmin\Desktop\charlie-mnemonic-dev\charlie-mnemonic-dev\Data\charlie_shared_data\initial_drone_pos.json', 'r') as f:
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
        with open(r'C:\Users\cgadmin\Desktop\charlie-mnemonic-dev\charlie-mnemonic-dev\Data\other_data\target_positions.json', 'r') as f:
            data = json.load(f)
            logging.info(f"Read {len(data)} target positions")
            return data, 0
    except FileNotFoundError:
        logging.error("Target positions file not found.")
        return [], 0
    except json.JSONDecodeError:
        logging.error("Error decoding target positions JSON.")
        return [], 0

def create_step_view_auto(recorded_states, lat_range: Tuple[float, float], lon_range: Tuple[float, float]):
    """
    Create a step-based visualization with a slider, step selection, and a show paths button.
    Both 2D and 3D views are available, toggled by a button.
    """
    steps = detect_steps(recorded_states)
    
    initial, n_d = read_initial_positions_from_file()

    if not initial:
        logging.error("No initial drone positions found. Exiting simulation.")
        return None, None

    first_pos = initial[0]
    ref_lat = first_pos["latitude"]
    ref_lon = first_pos["longitude"]
    
    converter = CoordinateConverter(ref_lat, ref_lon)
    
    # time step for slider
    time_step = 0.05
    
    if not recorded_states:
        logging.warning("No recorded states found. Nothing to show.")
        return

    if not steps:
        logging.warning("No steps detected (no movement). Nothing to show.")
        return
    
    Drones = []
    Targets = []
    
    for i in enumerate(initial):
        Drones.append(i)

    print(Drones)
    
    targets, nt = read_target_positions_from_file()
    for i, pos in enumerate(targets):
        Targets.append(i)

    print(Targets)
    
    all_states = recorded_states
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
            lat, lon = converter.xy_to_latlon(x, y)
            dpf.append((lon, lat, z))
        for (x, y, z) in state["targets"]:
            lat, lon = converter.xy_to_latlon(x, y)
            tpf.append((lon, lat, z))
        drone_positions_per_frame.append(dpf)
        target_positions_per_frame.append(tpf)
        target_headings_per_frame.append(thf)

    # Create figure and axes
    fig = plt.figure(figsize=(16, 9), dpi=100, facecolor='white')

    # 3D Axes
    ax_3d = fig.add_subplot(111, projection='3d')
    ax_3d.set_facecolor('white')
    ax_3d.grid(True)
    ax_3d.set_xlim(lon_range)
    ax_3d.set_ylim(lat_range)
    ax_3d.set_zlim(-1, 50)
    ax_3d.set_xlabel("Longitude (deg)")
    ax_3d.set_ylabel("Latitude (deg)")
    ax_3d.set_zlabel("Altitude (m)")
    ax_3d.set_title("Step-based Drone Simulation Viewer (3D)")

    # 2D Axes with square aspect ratio and centered
    
    ax_2d = fig.add_axes([0.2, 0.2, 0.6, 0.6])  # [left, bottom, width, height]
    num_divisions = 10
    
    # Calculate tick positions for 10 divisions (11 ticks)
    lat_ticks = np.linspace(lat_range[0], lat_range[1], num_divisions + 1)
    lon_ticks = np.linspace(lon_range[0], lon_range[1], num_divisions + 1)
    
    # 2D Plot Ticks
    ax_2d.set_xticks(lon_ticks)
    ax_2d.set_yticks(lat_ticks)
    ax_2d.set_facecolor('white')
    ax_2d.grid(True)
    ax_2d.set_xlim(lon_range)
    ax_2d.set_ylim(lat_range)
    ax_2d.set_xlabel("Longitude (deg)")
    ax_2d.set_ylabel("Latitude (deg)")
    ax_2d.set_title("Step-based Drone Simulation Viewer (2D)")
    ax_2d.set_aspect('equal', adjustable='box') 
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
        return (len(frames)-1) * time_step

    current_duration = duration_for_frames(current_frames)

    ax_slider = plt.axes([0.25, 0.05, 0.50, 0.03], facecolor='lightgoldenrodyellow')
    slider = Slider(ax_slider, 'Time', 0, current_duration, valinit=0, valstep=time_step, valfmt='%.4f s')

    # Lines for 3D paths
    full_path_lines_3d = []
    for _ in Drones:
        line_3d, = ax_3d.plot([], [], [], color='black', linewidth=2, alpha=0.5, label='_nolegend_')
        full_path_lines_3d.append(line_3d)

    # Lines for 2D paths
    full_path_lines_2d = []
    for _ in Drones:
        line_2d, = ax_2d.plot([], [], color='black', linewidth=2, alpha=0.5, label='_nolegend_')
        line_2d.set_visible(False)
        full_path_lines_2d.append(line_2d)

    collision_markers = []
    for _ in Drones:
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
        local_idx = int(round(time_val / time_step))
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
            lon = drone_pos[0]
            lat = drone_pos[1]
            alt = drone_pos[2] if not is_2d_mode else 0
            if None in drone_pos:
                continue  # Skip incomplete data
            if is_2d_mode:
                d_art = plot_drone_2d(ax_current, lat, lon, name=f'D{i+1}')
            else:
                d_art = plot_drone_3d(ax_current, (lat, lon, alt), name=f'D{i+1}')
            drone_artists.extend(d_art)

        # Plot targets
        for i, target_pos in enumerate(t_pos_gps):
            lon = target_pos[0]
            lat = target_pos[1]
            alt = target_pos[2] if not is_2d_mode else 0
            if None in target_pos:
                continue  # Skip incomplete data
            if is_2d_mode:
                t_art = plot_target_2d(ax_current, lat, lon, converter, name=f'T{i+1}')
            else:
                t_art = plot_target_3d(ax_current, (lat, lon, alt), converter, name=f'T{i+1}')
            target_artists.extend(t_art)

        # Plot assignments
        for d_id, t_id in assignments.items():
            try:
                d_idx = int(d_id) - 1
                t_idx = int(t_id) - 1
            except ValueError:
                logging.warning(f"Invalid assignment IDs: Drone {d_id}, Target {t_id}")
                continue
            if d_idx < len(d_pos_gps) and t_idx < len(t_pos_gps):
                dpos = d_pos_gps[d_idx]
                tpos = t_pos_gps[t_idx]
                if None in dpos or None in tpos:
                    continue  # Skip if any position is incomplete
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
                arrow = draw_heading_arrow_2d(ax_current, lat, lon, heading, length=0.00035)
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
        lon_paths = [[] for _ in Drones]
        lat_paths = [[] for _ in Drones]
        alt_paths = [[] for _ in Drones]
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

def main():
    """
    Run the simulation in stand-alone mode: read initial positions, start simulation and visualization.
    """

    json_file_path = input("Enter the path to the JSON file: ")
    
    # json_file_path = r'C:\Users\cgadmin\Desktop\charlie\simulations\20241223_110718\recorded_states.json'
    
    
    # Load the saved data
    with open(json_file_path, 'r') as f:
        try:
            saved_data = json.load(f)
        except json.JSONDecodeError as jde:
            logging.error(f"Failed to parse JSON file: {jde}")
            sys.exit(1)
            
    # Initialize the simulation
    create_step_view_auto(saved_data, BORDER_LAT_RANGE, BORDER_LON_RANGE)

if __name__ == "__main__":
    main()