import numpy as np
import matplotlib.pyplot as plt
import serial
from enum import Enum
import struct
from scipy.spatial import KDTree

# ----------------------------------------------------------------------
# System Constants and Packet Format
# ----------------------------------------------------------------------
SERIAL_PORT = "/dev/tty.usbserial-0001"
MEASUREMENTS_PER_PLOT = 480
PLOT_MAX_RANGE = 4.0  # in meters
PLOT_AUTO_RANGE = False
PLOT_CONFIDENCE = True
PLOT_CONFIDENCE_COLOUR_MAP = "bwr_r"
PRINT_DEBUG = False

PACKET_LENGTH = 47
MEASUREMENT_LENGTH = 12 
MESSAGE_FORMAT = "<xBHH" + "HB" * MEASUREMENT_LENGTH + "HHB"

State = Enum("State", ["SYNC0", "SYNC1", "SYNC2", "LOCKED", "UPDATE_PLOT"])

# ----------------------------------------------------------------------
# Helper Functions for LIDAR Data Parsing and Coordinate Conversion
# ----------------------------------------------------------------------
def parse_lidar_data(data):
    # Unpack according to the message format
    length, speed, start_angle, *pos_data, stop_angle, timestamp, crc = \
        struct.unpack(MESSAGE_FORMAT, data)
    start_angle = float(start_angle) / 100.0
    stop_angle = float(stop_angle) / 100.0
    if stop_angle < start_angle:
        stop_angle += 360.0
    step_size = (stop_angle - start_angle) / (MEASUREMENT_LENGTH - 1)
    angles = [start_angle + step_size * i for i in range(MEASUREMENT_LENGTH)]
    distances = pos_data[0::2]  # in millimeters
    confidences = pos_data[1::2]
    if PRINT_DEBUG:
        print(length, speed, start_angle, *pos_data, stop_angle, timestamp, crc)
    return list(zip(angles, distances, confidences))

def get_xyc_data(measurements):
    """Convert a list of (angle, distance, confidence) tuples into x, y, confidence arrays.
       x, y are in meters in the LIDAR (local) coordinate system."""
    angles = np.array([m[0] for m in measurements])
    distances = np.array([m[1] for m in measurements])
    confidences = np.array([m[2] for m in measurements])
    x = np.sin(np.radians(angles)) * (distances / 1000.0)
    y = np.cos(np.radians(angles)) * (distances / 1000.0)
    return x, y, confidences

# ----------------------------------------------------------------------
# Helper Functions for ICP
# ----------------------------------------------------------------------
def transform_points(points, T):
    """Apply a 3x3 homogeneous transformation T to 2D points (N x 2)."""
    N = points.shape[0]
    homogeneous_points = np.hstack((points, np.ones((N, 1))))
    transformed = (T @ homogeneous_points.T).T
    return transformed[:, :2]

def icp(source, target, max_iterations=50, tolerance=1e-4):
    """
    Perform Iterative Closest Point matching to find the best-fit transform
    that maps 'source' points to 'target' points.
    
    Parameters:
      source: (N,2) array of points (the new scan in estimated global frame)
      target: (M,2) array of points (the previous scan in global frame)
      
    Returns:
      T: 3x3 homogeneous transformation matrix mapping source to target.
      mean_error: Final mean distance error.
      iterations: Number of iterations used.
    """
    src = source.copy()
    dst = target.copy()
    # Initial transformation (identity)
    T = np.eye(3)
    prev_error = float('inf')
    tree = KDTree(dst)

    for i in range(max_iterations):
        # Find the closest point in target for each point in source
        distances, indices = tree.query(src)
        # Compute centroids of source and corresponding target points
        centroid_src = np.mean(src, axis=0)
        centroid_dst = np.mean(dst[indices], axis=0)
        # Center the points
        src_centered = src - centroid_src
        dst_centered = dst[indices] - centroid_dst
        # Compute the cross-covariance matrix
        H = src_centered.T @ dst_centered
        # Singular Value Decomposition
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        # Ensure a proper rotation (determinant = +1)
        if np.linalg.det(R) < 0:
            Vt[1, :] *= -1
            R = Vt.T @ U.T
        t = centroid_dst - R @ centroid_src
        # Build the incremental transformation matrix
        T_inc = np.eye(3)
        T_inc[:2, :2] = R
        T_inc[:2, 2] = t
        # Update total transformation
        T = T_inc @ T
        # Transform source points
        src = transform_points(source, T)
        mean_error = np.mean(distances)
        if abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error
    return T, mean_error, i

# ----------------------------------------------------------------------
# Main Processing Functions
# ----------------------------------------------------------------------
def process_scan(measurements, robot_pose, prev_scan_global, global_map):
    """
    Process the current scan measurements.
      - Convert scan to (x,y) in LIDAR frame.
      - If a previous scan exists, run ICP to update the robot pose.
      - Update the global map.
      
    Returns:
      updated_robot_pose: the new robot pose (2D, as np.array)
      current_scan_global: current scan transformed to the global frame.
      updated_prev_scan_global: the current scan becomes the previous scan for next iteration.
      updated_global_map: accumulated global map (list of (x,y) points).
    """
    # Convert the raw measurements (angles, distances, confidence) into x, y coordinates.
    x, y, _ = get_xyc_data(measurements)
    current_scan_local = np.column_stack((x, y))
    
    # Build the initial global estimate using the current robot pose.
    T_initial = np.eye(3)
    T_initial[0, 2] = robot_pose[0]
    T_initial[1, 2] = robot_pose[1]
    current_scan_est = transform_points(current_scan_local, T_initial)
    
    # If we have a previous scan, use ICP to compute the correction.
    if prev_scan_global is not None:
        T_icp, error, iterations = icp(current_scan_est, prev_scan_global)
        # Update the transformation: combine the initial guess and the ICP correction.
        T_updated = T_icp @ T_initial
        # Extract the updated robot pose (translation part).
        updated_robot_pose = T_updated[:2, 2]
        # Transform the current scan using the updated transformation.
        current_scan_global = transform_points(current_scan_local, T_updated)
    else:
        # For the very first scan, there is no correction.
        updated_robot_pose = robot_pose
        current_scan_global = current_scan_est
    
    # Accumulate the points into the global map.
    updated_global_map = np.vstack((global_map, current_scan_global)) if global_map.size else current_scan_global
    return updated_robot_pose, current_scan_global, current_scan_global, updated_global_map

def update_plot(global_map, current_scan_global, robot_pose, figure):
    """Update the plot with the global map, current scan, and robot position."""
    plt.figure(figure.number)
    plt.clf()
    # Plot the global map (small grey dots)
    plt.plot(global_map[:, 0], global_map[:, 1], 'k.', markersize=1, label="Global Map")
    # Plot the current scan (colored by a default blue)
    plt.plot(current_scan_global[:, 0], current_scan_global[:, 1], 'b.', label="Current Scan")
    # Plot the robot's current position
    plt.plot(robot_pose[0], robot_pose[1], 'ro', label="Robot")
    if PLOT_AUTO_RANGE:
        max_extent = np.max(np.abs(global_map)) * 1.2
        plt.xlim(-max_extent, max_extent)
        plt.ylim(-max_extent, max_extent)
    else:
        plt.xlim(-PLOT_MAX_RANGE, PLOT_MAX_RANGE)
        plt.ylim(-PLOT_MAX_RANGE, PLOT_MAX_RANGE)
    plt.legend(loc="upper right")
    plt.pause(0.0001)

# ----------------------------------------------------------------------
# Main Loop
# ----------------------------------------------------------------------
def main():
    # Open serial port connection
    lidar_serial = serial.Serial(SERIAL_PORT, 230400, timeout=0.5)
    state = State.SYNC0
    measurements = []
    data = b''

    # Initialize variables for scan matching
    robot_pose = np.array([0.0, 0.0])
    prev_scan_global = None  # Will hold the previous scan (in global coordinates)
    global_map = np.array([]).reshape(0, 2)  # Accumulated map points

    # Set up plot
    plt.ion()
    fig = plt.figure(figsize=(10, 10))
    plt.xlim(-PLOT_MAX_RANGE, PLOT_MAX_RANGE)
    plt.ylim(-PLOT_MAX_RANGE, PLOT_MAX_RANGE)
    plt.title("ICP-based SLAM Map")
    
    running = True
    def on_plot_close(event):
        nonlocal running
        running = False
    fig.canvas.mpl_connect('close_event', on_plot_close)
    
    while running:
        # --- Packet Synchronization and Data Gathering ---
        if state == State.SYNC0:
            data = b''
            measurements = []
            if lidar_serial.read() == b'\x54':
                data = b'\x54'
                state = State.SYNC1
        elif state == State.SYNC1:
            if lidar_serial.read() == b'\x2C':
                data += b'\x2C'
                state = State.SYNC2
            else:
                state = State.SYNC0
        elif state == State.SYNC2:
            data += lidar_serial.read(PACKET_LENGTH - 2)
            if len(data) != PACKET_LENGTH:
                state = State.SYNC0
                continue
            measurements += parse_lidar_data(data)
            state = State.LOCKED
        elif state == State.LOCKED:
            data = lidar_serial.read(PACKET_LENGTH)
            if len(data) != PACKET_LENGTH or data[0] != 0x54:
                print("WARNING: Serial sync lost")
                state = State.SYNC0
                continue
            measurements += parse_lidar_data(data)
            if len(measurements) > MEASUREMENTS_PER_PLOT:
                state = State.UPDATE_PLOT
        elif state == State.UPDATE_PLOT:
            # --- ICP-based Scan Matching ---
            robot_pose, current_scan_global, prev_scan_global, global_map = \
                process_scan(measurements, robot_pose, prev_scan_global, global_map)
            
            # Update the plot with the current global map and pose.
            update_plot(global_map, current_scan_global, robot_pose, fig)
            
            # Prepare for the next update
            measurements = []
            state = State.LOCKED

if __name__ == "__main__":
    main()
