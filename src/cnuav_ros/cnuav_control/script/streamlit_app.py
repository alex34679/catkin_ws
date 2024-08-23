import streamlit as st
import matplotlib.pyplot as plt
import json
import os
import time
import numpy as np
from scipy.spatial.transform import Rotation as R

def load_data():
    if os.path.exists('/tmp/trajectory_data.json'):
        with open('/tmp/trajectory_data.json', 'r') as f:
            data = json.load(f)
        return data
    else:
        return None


import numpy as np

def calculate_rms_error(actual_positions, reference_positions):
    # 计算位置误差的RMS并转换为毫米
    errors = []
    for actual, reference in zip(actual_positions, reference_positions):
        error = np.sqrt(
            (actual['x'] - reference['x']) ** 2 +
            (actual['y'] - reference['y']) ** 2 +
            (actual['z'] - reference['z']) ** 2
        )
        errors.append(error)
    
    # 计算误差的均方根 (RMS)
    rms_error = np.sqrt(np.mean(np.square(errors)))
    
    # 转换为毫米（假设输入位置的单位为米）
    rms_error_mm = rms_error * 1000
    
    return rms_error_mm


def plot_data(data):
    st.subheader("Position, Velocity, Acceleration, and Orientation Data")
    
    # Plot position data
    plot_vel_component_data(data['actual_times'], data['actual_velocities'], data['reference_velocities'], "Velocity")
    plot_component_data(data['actual_times'], data['actual_positions'], data['reference_positions'], "Position")
    plot_position_comparison(data['actual_times'], data['actual_positions'], data['reference_positions'], "Position")
    
    # Calculate and display the quadratic sum of position errors
    if 'actual_positions' in data and 'reference_positions' in data:
        error_sum = calculate_rms_error(data['actual_positions'], data['reference_positions'])
        st.write(f"Position MSE(mean-square error) : {error_sum:.2f} mm")
    
    # Plot velocity data
    if 'actual_times' in data and 'actual_velocities' in data and 'reference_velocities' in data:
        plot_component_data(data['actual_times'], data['actual_velocities'], data['reference_velocities'], "Velocity")
    
    # Plot quaternion data as Euler angles
    if 'actual_times' in data and 'actual_orientations' in data and 'reference_orientations' in data:
        plot_euler_angles(data['actual_times'], data['actual_orientations'], data['reference_orientations'], "Orientation")
    
    # Plot trajectory paths
    if 'actual_positions' in data and 'reference_positions' in data:
        plot_trajectory_paths(data['actual_positions'], data['reference_positions'])

def quaternion_to_euler(quaternions):
    # Convert a list of quaternions to Euler angles (roll, pitch, yaw)
    euler_angles = []
    for q in quaternions:
        r = R.from_quat([q['x'], q['y'], q['z'], q['w']])
        euler = r.as_euler('xyz', degrees=True)
        euler_angles.append({'roll': euler[0], 'pitch': euler[1], 'yaw': euler[2]})
    return euler_angles

def plot_euler_angles(times, actual_orientations, reference_orientations, label):
    st.subheader(f"Orientation (Euler Angles) over Time")

    # Convert quaternions to Euler angles
    actual_euler = quaternion_to_euler(actual_orientations)
    reference_euler = quaternion_to_euler(reference_orientations)

    fig, axs = plt.subplots(3, 1, figsize=(12, 18))
    fig.suptitle(f'{label} over Time')

    # Initialize min and max values
    min_roll = min_pitch = min_yaw = float('inf')
    max_roll = max_pitch = max_yaw = float('-inf')

    # Determine the min and max values for each angle type
    for euler_angles in actual_euler + reference_euler:
        min_roll = min(min_roll, euler_angles['roll'])
        max_roll = max(max_roll, euler_angles['roll'])
        min_pitch = min(min_pitch, euler_angles['pitch'])
        max_pitch = max(max_pitch, euler_angles['pitch'])
        min_yaw = min(min_yaw, euler_angles['yaw'])
        max_yaw = max(max_yaw, euler_angles['yaw'])

    # Plot roll, pitch, yaw components
    for i, (data, title) in enumerate(zip([actual_euler, reference_euler], ['Actual', 'Reference'])):
        axs[0].plot(times, [d['roll'] for d in data], label=f'{title} Roll', linestyle='-', marker='o')
        axs[1].plot(times, [d['pitch'] for d in data], label=f'{title} Pitch', linestyle='-', marker='x')
        axs[2].plot(times, [d['yaw'] for d in data], label=f'{title} Yaw', linestyle='-', marker='^')

    axs[0].set_xlabel('Time (seconds)')
    axs[0].set_ylabel(f'{label} Roll')
    axs[0].legend()
    axs[0].grid(True)
    axs[0].set_ylim([min_roll, max_roll])

    axs[1].set_xlabel('Time (seconds)')
    axs[1].set_ylabel(f'{label} Pitch')
    axs[1].legend()
    axs[1].grid(True)
    axs[1].set_ylim([min_pitch, max_pitch])

    axs[2].set_xlabel('Time (seconds)')
    axs[2].set_ylabel(f'{label} Yaw')
    axs[2].legend()
    axs[2].grid(True)
    axs[2].set_ylim([min_yaw, max_yaw])

    st.pyplot(fig)

def plot_component_data(times, actual_data, reference_data, label):
    st.subheader(f"{label} over Time")

    fig, axs = plt.subplots(2, 1, figsize=(12, 15))
    fig.suptitle(f'{label} over Time')

    for i, (data, title) in enumerate(zip([actual_data, reference_data], ['Actual', 'Reference'])):
        axs[i].plot(times, [d['x'] for d in data], label=f'{title} X', linestyle='-', marker='o')
        axs[i].plot(times, [d['y'] for d in data], label=f'{title} Y', linestyle='-', marker='x')
        axs[i].plot(times, [d['z'] for d in data], label=f'{title} Z', linestyle='-', marker='^')
        axs[i].set_xlabel('Time (seconds)')
        axs[i].set_ylabel(f'{label} Components')
        axs[i].legend()
        axs[i].grid(True)

    st.pyplot(fig)
    
def plot_vel_component_data(times, actual_data, reference_data, label):
    st.subheader(f"{label} over Time")

    fig, axs = plt.subplots(2, 1, figsize=(12, 15))
    fig.suptitle(f'{label} over Time')

    for i, (data, title) in enumerate(zip([actual_data, reference_data], ['Actual', 'Reference'])):
        # 计算速度模 (即 x^2 + y^2 + z^2 的平方根)
        speeds = [np.sqrt(d['x']**2 + d['y']**2 + d['z']**2) for d in data]
        
        # 绘制速度模
        axs[i].plot(times, speeds, label=f'{title} Speed', linestyle='-', marker='o')
        
        axs[i].set_xlabel('Time (seconds)')
        axs[i].set_ylabel(f'{label} Speed (m/s)')
        axs[i].legend()
        axs[i].grid(True)

    st.pyplot(fig)
    
def plot_position_comparison(times, actual_positions, reference_positions, label):
    
    st.subheader(f"{label} Comparison over Time")

    fig, axs = plt.subplots(3, 1, figsize=(12, 15))
    fig.suptitle(f'{label} over Time')


    # Plot X component

    axs[0].plot(times, [pos['x'] for pos in actual_positions], label='Actual X', linestyle='-', marker='o')
    axs[0].plot(times, [pos['x'] for pos in reference_positions], label='Reference X', linestyle='-', marker='x')
    axs[0].set_xlabel('Time (seconds)')
    axs[0].set_ylabel('X Position (m)')
    axs[0].legend()
    axs[0].grid(True)

    # Plot Y component
    axs[1].plot(times, [pos['y'] for pos in actual_positions], label='Actual Y', linestyle='-', marker='o')
    axs[1].plot(times, [pos['y'] for pos in reference_positions], label='Reference Y', linestyle='-', marker='x')
    axs[1].set_xlabel('Time (seconds)')
    axs[1].set_ylabel('Y Position (m)')
    axs[1].legend()
    axs[1].grid(True)

    # Plot Z component
    axs[2].plot(times, [pos['z'] for pos in actual_positions], label='Actual Z', linestyle='-', marker='o')
    axs[2].plot(times, [pos['z'] for pos in reference_positions], label='Reference Z', linestyle='-', marker='x')
    axs[2].set_xlabel('Time (seconds)')
    axs[2].set_ylabel('Z Position (mm)')
    axs[2].legend()
    axs[2].grid(True)

    st.pyplot(fig)



def plot_trajectory_paths(actual_positions, reference_positions):
    st.subheader("Trajectory Paths Comparison")

    fig, ax = plt.subplots(figsize=(12, 8))
    fig.suptitle('Trajectory Paths Comparison')

    actual_x = [p['x'] for p in actual_positions]
    actual_y = [p['y'] for p in actual_positions]
    reference_x = [p['x'] for p in reference_positions]
    reference_y = [p['y'] for p in reference_positions]

    ax.plot(actual_x, actual_y, label='Actual Path', linestyle='-', marker='o')
    ax.plot(reference_x, reference_y, label='Reference Path', linestyle='--', marker='x')
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.legend()
    ax.grid(True)

    st.pyplot(fig)

def main():
    st.title("Trajectory Tracking and Error Visualization")
    
    while True:
        data = load_data()
        
        if data:
            plot_data(data)
            st.success("Data loaded and plotted successfully!")
            break
        else:
            st.write("Waiting for data...")
            time.sleep(5)  # Check every 5 seconds

if __name__ == '__main__':
    main()
