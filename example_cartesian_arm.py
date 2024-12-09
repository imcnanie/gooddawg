#!/usr/bin/env nix-shell
#!nix-shell -i python3 -p python39Packages.pyserial 

import struct
import serial
import time
import math
import build_a_packet as bp

# Note: These transforms are a work in progress!
def get_inverse_jacobian(theta1, theta2, L1=0.22, L2=0.22):
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
    # Calculate Jacobian elements
    J11 = -L1*math.sin(theta1) - L2*math.sin(theta1 + theta2)
    J12 = -L2*math.sin(theta1 + theta2)
    J21 = L1*math.cos(theta1) + L2*math.cos(theta1 + theta2)
    J22 = L2*math.cos(theta1 + theta2)
    
    # Calculate determinant
    det = J11*J22 - J12*J21
    
    # Calculate inverse
    return [[J22/det, -J12/det],
            [-J21/det, J11/det]]

def get_ik(x, y, L1=0.22, L2=0.22):
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
    
    theta2 = math.acos(cos_theta2)
    
    # Calculate theta1 using geometric approach
    beta = math.atan2(y, x)
    psi = math.acos((x*x + y*y + L1*L1 - L2*L2)/(2*L1*math.sqrt(x*x + y*y)))
    theta1 = beta - psi

    return -theta1, -theta2

def get_joint_velocities(x_dot, y_dot, theta1, theta2, L1=0.215, L2=0.215):
    """
    Calculate joint velocities given end effector velocities using the Jacobian
    
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
    # Calculate Jacobian inverse
    J_inv = get_inverse_jacobian(theta1, theta2, L1, L2)
    
    # Calculate joint velocities using J^-1 * v
    dtheta1 = J_inv[0][0]*x_dot + J_inv[0][1]*y_dot
    dtheta2 = J_inv[1][0]*x_dot + J_inv[1][1]*y_dot
    
    return dtheta1, dtheta2

diff1 = 0.0
diff2 = 0.0
if __name__ == "__main__":
    ser = bp.configure_serial("/dev/ttyUSB0")
    while True:
        q = math.sin(time.time())*0.09 + 0.2  # Sine wave oscillating between -150 and +150
        dq = math.cos(time.time())*0.09

        # these are good gains to balance vel pos, and feed forward tau of 0.3 seems to comp grav
        # bp.send_packet(ser, bp.build_a_packet(id=1, q=q, dq=0.02, Kp=3, Kd=0.5, tau=0.0))

        #bp.send_packet(ser, bp.build_a_packet(id=0, q=0.0, dq=0, Kp=3, Kd=0.5, tau=0.0)) # to do velocity mode we make p 0, tau is how torqy, KD is vel
        #time.sleep(0.005)  # Small delay to control update rate

        q1, q2 = get_ik(q, 0)
        #print(q1,q2)

        dq1, dq2 = get_joint_velocities(dq, 0, q1, q2, L1=0.215, L2=0.215)
        #print(dq1, dq2)


        bp.send_packet(ser, bp.build_a_packet(id=1, q=q1, dq=-dq1, Kp=4, Kd=0.1, tau=0.0)) # to do velocity mode we make p 0, tau is how torqy, KD is vel
       

        bp.read_and_update_motor_data(ser)
        time.sleep(0.01)

        bp.send_packet(ser, bp.build_a_packet(id=2, q=-q2*2, dq=dq2, Kp=4, Kd=0.1, tau=0.0)) # to do velocity mode we make p 0, tau is how torqy, KD is vel

        bp.read_and_update_motor_data(ser)
        time.sleep(0.01)
        if bp.motor_data['mot1_angle'] is not None and bp.motor_data['mot2_angle'] is not None:
            diff1 = bp.motor_data['mot1_angle']
            diff2 = bp.motor_data['mot2_angle']
            print(f"diff1: {diff1:.3f}, diff2: {diff2:.3f}")
        else:
            print("Waiting for motor data...")
