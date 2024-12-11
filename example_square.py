#!/usr/bin/env nix-shell
#!nix-shell -i python3 -p python39Packages.pyserial 

import struct
import serial
import time
import math
import build_a_packet as bp

l1 = 0.215
l2 = 0.215

def get_ik(x, y, L1=l1, L2=l2):
    """
    Calculate inverse kinematics for 2-link planar robot arm
    
    Args:
        x: Target x position of end effector (m)
        y: Target y position of end effector (m)
        L1: Length of first link (m) 
        L2: Length of second link (m)
        
    Returns:
        theta1, theta2: Joint angles in radians
    """
    # Check if point is reachable
    r = math.sqrt(x*x + y*y)
    if r > (L1 + L2) or r < abs(L1 - L2):
        raise ValueError("Target position not reachable")

    # Calculate theta2 using cosine law
    cos_theta2 = (x*x + y*y - L1*L1 - L2*L2)/(2*L1*L2)
    if cos_theta2 > 1 or cos_theta2 < -1:
        raise ValueError("Target position not reachable")
    
    # There are two possible solutions for theta2 (elbow-up vs elbow-down)
    # Using elbow-down configuration
    theta2 = math.acos(cos_theta2)  # Negative for elbow-down configuration
    
    # Calculate theta1 using geometric approach
    beta = math.atan2(y, x)
    psi = math.atan2(L2*math.sin(theta2), L1 + L2*math.cos(theta2))
    theta1 = beta - psi  # Subtract psi for elbow-down config

    return theta1, theta2

def get_inverse_jacobian(theta1, theta2, L1=l1, L2=l2):
    """
    Calculate the inverse Jacobian matrix for a 2-link planar robot arm
    
    Args:
        theta1: Angle of first joint (rad)
        theta2: Angle of second joint (rad) 
        L1: Length of first link (m)
        L2: Length of second link (m)
        
    Returns:
        2x2 inverse Jacobian matrix
    """
    # Calculate forward kinematics terms
    c1 = math.cos(theta1)
    c2 = math.cos(theta2)
    c12 = math.cos(theta1 + theta2)
    s1 = math.sin(theta1)
    s2 = math.sin(theta2)
    s12 = math.sin(theta1 + theta2)
    
    # Calculate Jacobian elements
    J11 = -L1*s1 - L2*s12
    J12 = -L2*s12
    J21 = L1*c1 + L2*c12
    J22 = L2*c12
    
    # Calculate determinant
    det = J11*J22 - J12*J21
    
    if abs(det) < 1e-6:
        raise ValueError("Jacobian is singular")
        
    # Calculate inverse
    return [[J22/det, -J12/det],
            [-J21/det, J11/det]]

def get_joint_velocities(x_dot, y_dot, theta1, theta2, L1=l1, L2=l2):
    """
    Calculate joint velocities given end effector velocities using the inverse Jacobian
    
    Args:
        x_dot: Target x velocity of end effector (m/s)
        y_dot: Target y velocity of end effector (m/s)
        theta1: Current angle of first joint (rad)
        theta2: Current angle of second joint (rad)
        L1: Length of first link (m)
        L2: Length of second link (m)
        
    Returns:
        dtheta1, dtheta2: Joint velocities in rad/s
    """
    # Get inverse Jacobian
    J_inv = get_inverse_jacobian(theta1, theta2, L1, L2)
    
    # Calculate joint velocities using J^-1 * v
    dtheta1 = J_inv[0][0]*x_dot + J_inv[0][1]*y_dot
    dtheta2 = J_inv[1][0]*x_dot + J_inv[1][1]*y_dot
    
    return dtheta1, dtheta2

diff1 = 0.0
diff2 = 0.0
if __name__ == "__main__":
    ser = bp.configure_serial("/dev/ttyUSB0")
    # Define a simple square G-code path with interpolation
    '''gcode_path = [ cool logo
        (0.12, 0.30),  # Bottom left point
        (0.23, 0.30),  # Bottom right point 
        (0.15, 0.35),  # Middle left point
        (0.20, 0.35),  # Middle right point
        (0.17, 0.28),  # Center point
        (0.12, 0.30)   # Back to start
    ]'''
    '''
    # pentagram
    gcode_path = []
    center_x = 0.17500
    center_y = 0.25
    radius = 0.055
    for i in range(5):
        angle_deg = -90 + i * 144
        angle_rad = angle_deg * math.pi / 180.0
        x = center_x + radius * math.cos(angle_rad)
        y = center_y + radius * math.sin(angle_rad)
        gcode_path.append((x, y))
    # Close the pentagram by appending the first point
    gcode_path.append(gcode_path[0])
    '''
        # pentagram
    gcode_path = []
    center_x = 0.17500
    center_y = 0.25
    radius = 0.075  # Increased radius to make points stick out further
    for i in range(5):
        angle_deg = -90 + i * 144
        angle_rad = angle_deg * math.pi / 180.0
        x = center_x + radius * math.cos(angle_rad)
        y = center_y + radius * math.sin(angle_rad)
        gcode_path.append((x, y))
    # Close the pentagram by appending the first point
    gcode_path.append(gcode_path[0])


    

    ''' heart
    gcode_path = []
    center_x = 0.17500
    center_y = 0.25
    scale = 0.055

    num_points = 10
    for i in range(num_points + 1):
        t = math.pi * 2 * i / num_points
        x_raw = 16 * math.sin(t) ** 3
        y_raw = 13 * math.cos(t) - 5 * math.cos(2 * t) - 2 * math.cos(3 * t) - math.cos(4 * t)
        x = center_x + scale * (x_raw / 16)  # Normalize x_raw to range [-1, 1]
        y = center_y + scale * (y_raw / 17)  # Normalize y_raw to approximately [-1, 1]
        gcode_path.append((x, y))

    # Close the heart shape by appending the first point
    gcode_path.append(gcode_path[0])
    '''
    
    ''' square
    gcode_path = []
    center_x = 0.17500
    center_y = 0.25
    scale = 0.05  # Reduced scale for better control

    # Define a simple square path
    square_points = [
        (-1, -1),  # Bottom left
        (-1, 1),   # Top left  
        (1, 1),    # Top right
        (1, -1),   # Bottom right
        (-1, -1)   # Back to start
    ]

    # Add the square points to the path
    for point in square_points:
        x, y = point
        gcode_path.append((center_x + scale * x, center_y + scale * y))
        
        # Add extra points at corners for smoother motion
        gcode_path.append((center_x + scale * x, center_y + scale * y))
    '''
    # Function to interpolate between two points
    def interpolate_points(start, end, steps):
        x1, y1 = start
        x2, y2 = end
        return [(x1 + (x2 - x1) * t / steps, y1 + (y2 - y1) * t / steps) for t in range(steps + 1)]
    
    # Loop through the G-code path with interpolation
    while True:
        for i in range(len(gcode_path) - 1):
            start_point = gcode_path[i]
            end_point = gcode_path[i + 1]
            interpolated_points = interpolate_points(start_point, end_point, 25)  # Reduced interpolation steps

            for target_x, target_y in interpolated_points:
                # Calculate inverse kinematics for each interpolated point
                q1, q2 = get_ik(target_x, target_y)
                # Calculate joint velocities (assuming zero velocity for simplicity)
                dq1, dq2 = get_joint_velocities(0, 0, q1, q2)

                # Send commands to motors with reduced delay
                bp.send_packet(ser, bp.build_a_packet(id=1, q=q1, dq=-dq1, Kp=24, Kd=0.8, tau=0.0))
                bp.read_and_update_motor_data(ser)
                time.sleep(0.004)  # Reduced delay

                bp.send_packet(ser, bp.build_a_packet(id=2, q=q2, dq=dq2, Kp=24, Kd=0.5, tau=0.0))
                bp.read_and_update_motor_data(ser)
                time.sleep(0.004)  # Reduced delay

                # Update diff values based on motor feedback
                if bp.motor_data['mot1_angle'] is not None and bp.motor_data['mot2_angle'] is not None:
                    diff1 = q1 - bp.motor_data['mot1_angle']
                    diff2 = q2 - bp.motor_data['mot2_angle']
                else:
                    print("Waiting for motor data...")