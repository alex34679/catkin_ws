#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Time, Header
import json
import numpy as np
import matplotlib.pyplot as plt
from cnuav_control.msg import TrajectoryTracking
from std_msgs.msg import Float64

received_data = False

# Initialize global lists to store data
actual_positions = []
reference_positions = []
actual_velocities = []
reference_velocities = []
actual_accelerations = []
reference_accelerations = []
actual_orientations = []
reference_orientations = []
actual_times = []

def vector3_to_dict(v):
    return {'x': v.x, 'y': v.y, 'z': v.z}

def quaternion_to_dict(q):
    return {'x': q.x, 'y': q.y, 'z': q.z, 'w': q.w}

def float64_to_python_float(f):
    return f.data

def callback(msg):
    global actual_positions, reference_positions, actual_velocities, reference_velocities
    global actual_accelerations, reference_accelerations, actual_orientations, reference_orientations
    global actual_times, received_data

    # Collect data
    actual_positions.extend([vector3_to_dict(p) for p in msg.actual_position])
    reference_positions.extend([vector3_to_dict(p) for p in msg.reference_position])
    actual_velocities.extend([vector3_to_dict(v) for v in msg.actual_velocity])
    reference_velocities.extend([vector3_to_dict(v) for v in msg.reference_velocity])
    actual_accelerations.extend([vector3_to_dict(a) for a in msg.actual_acceleration])
    reference_accelerations.extend([vector3_to_dict(a) for a in msg.reference_acceleration])
    actual_orientations.extend([quaternion_to_dict(o) for o in msg.actual_orientation])
    reference_orientations.extend([quaternion_to_dict(o) for o in msg.reference_orientation])
    # Directly use float64 array for times
    actual_times.extend([float64_to_python_float(t) for t in msg.actual_time]) 


    received_data = True
    # Save collected data to a file
    with open('/tmp/trajectory_data.json', 'w') as f:
        json.dump({
            'actual_positions': actual_positions,
            'reference_positions': reference_positions,
            'actual_velocities': actual_velocities,
            'reference_velocities': reference_velocities,
            'actual_accelerations': actual_accelerations,
            'reference_accelerations': reference_accelerations,
            'actual_orientations': actual_orientations,
            'reference_orientations': reference_orientations,
            'actual_times': actual_times
        }, f)

    rospy.signal_shutdown('Data received and saved. Node shutting down.')
    
def listener():
    rospy.init_node('trajectory_plotter', anonymous=True)
    rospy.Subscriber('/hummingbird/trajectory_tracking', TrajectoryTracking, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
    
    
    