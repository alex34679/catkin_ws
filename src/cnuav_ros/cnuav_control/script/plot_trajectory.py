#!/usr/bin/env python3
import rospy
import signal
import json
from geometry_msgs.msg import Vector3, Quaternion
from quadrotor_msgs.msg import AutopilotFeedback
from std_msgs.msg import Float64
import signal
import threading
callback_event = threading.Event()

received_data = False

# Initialize global lists to store data
actual_positions = []
reference_positions = []
actual_velocities = []
reference_velocities = []
actual_orientations = []
reference_orientations = []
actual_times = []

def vector3_to_dict(v):
    return {'x': v.x, 'y': v.y, 'z': v.z}

def quaternion_to_dict(q):
    return {'x': q.x, 'y': q.y, 'z': q.z, 'w': q.w}

def float64_to_python_float(f):
    return f.data

def save_data_to_file():
    # Find the minimum length among all the arrays
    min_length = min(len(actual_positions), len(reference_positions),
                     len(actual_orientations), len(reference_orientations),
                     len(actual_velocities), len(reference_velocities),
                     len(actual_times))

    if min_length > 0:
        # Truncate all arrays to the minimum length
        actual_positions_truncated = actual_positions[:min_length]
        reference_positions_truncated = reference_positions[:min_length]
        actual_orientations_truncated = actual_orientations[:min_length]
        reference_orientations_truncated = reference_orientations[:min_length]
        actual_velocities_truncated = actual_velocities[:min_length]
        reference_velocities_truncated = reference_velocities[:min_length]
        actual_times_truncated = actual_times[:min_length]

        # Save the truncated data to the file
        with open('/tmp/trajectory_data.json', 'w') as f:
            json.dump({
                'actual_positions': actual_positions_truncated,
                'reference_positions': reference_positions_truncated,
                'actual_orientations': actual_orientations_truncated,
                'reference_orientations': reference_orientations_truncated,
                'actual_velocities': actual_velocities_truncated,
                'reference_velocities': reference_velocities_truncated,
                'actual_times': actual_times_truncated
            }, f)
        rospy.loginfo("Data saved to /tmp/trajectory_data.json")
    else:
        rospy.loginfo("No data received; nothing to save.")


def callback(msg):
    global received_data
    callback_event.clear()  # Reset the event at the start of the callback
    
    
    global actual_positions, reference_positions, actual_orientations, reference_orientations
    global actual_times, received_data

    # Collect data
    actual_position = msg.state_estimate.pose.pose.position
    reference_position = msg.reference_state.pose.position
    
    actual_velocitie = msg.state_estimate.twist.twist.linear
    reference_velocitie = msg.reference_state.velocity.linear
    
    actual_orientation = msg.state_estimate.pose.pose.orientation
    reference_orientation = msg.reference_state.pose.orientation
    
    actual_time = msg.reference_state.time_from_start.to_sec()

    actual_positions.append(vector3_to_dict(actual_position))
    reference_positions.append(vector3_to_dict(reference_position))
    actual_velocities.append(vector3_to_dict(actual_velocitie))
    reference_velocities.append(vector3_to_dict(reference_velocitie))
    actual_orientations.append(quaternion_to_dict(actual_orientation))
    reference_orientations.append(quaternion_to_dict(reference_orientation))
    actual_times.append(actual_time)

    received_data = True
    callback_event.set()  # Set the event to signal that the callback is done

def handle_shutdown(signum, frame):
    rospy.loginfo("Shutdown signal received. Waiting for callback to complete...")
    callback_event.wait()  # Wait for the callback to finish
    save_data_to_file()  # Save the data only if received_data is True
    rospy.signal_shutdown("Shutdown complete.")


def listener():
    rospy.init_node('trajectory_plotter', anonymous=True)

    # Register the signal handler for Ctrl+C (SIGINT)
    signal.signal(signal.SIGINT, handle_shutdown)

    rospy.Subscriber('/hummingbird/autopilot/feedback', AutopilotFeedback, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
