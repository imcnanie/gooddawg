#!/usr/bin/env nix-shell
#!nix-shell -i python3 -p python39Packages.pyserial 

import struct
import serial
import time
import math
import build_a_packet as bp

MOTOR_ID = 2
if __name__ == "__main__":
    ser = bp.configure_serial("/dev/ttyUSB0")
    #calibration_angles = [0, math.pi/2, math.pi]  # link 1
    presets= [[0, math.pi/2, math.pi], 
              [50*math.pi/180, math.pi/2, 150*math.pi/180]]  # link 2
    calibration_angles = presets[MOTOR_ID-1]
    measured_angles = []
    
    for target_angle in calibration_angles:
        input(f"Please move the arm to {math.degrees(target_angle)} degrees and press Enter...")
        
        # Take multiple readings and average them for stability
        readings = []
        for _ in range(10):
            # Send zero packet to enable reading motor angle
            bp.send_packet(ser, bp.build_a_packet(id=MOTOR_ID, q=0, dq=0, Kp=0, Kd=0.0, tau=0.00))
            bp.read_and_update_motor_data(ser)
            if bp.motor_data['mot'+str(MOTOR_ID)+'_angle'] is not None:
                readings.append(bp.motor_data['mot'+str(MOTOR_ID)+'_angle'])
            time.sleep(0.1)
        
        if readings:
            avg_reading = sum(readings) / len(readings)
            measured_angles.append(avg_reading)
            print(f"Measured angle: {avg_reading:.3f} radians")
        else:
            print("Error: Could not read motor angle")
            exit(1)
    
    # Calculate offset using linear regression
    if len(measured_angles) == 3:
        slope = (measured_angles[2] - measured_angles[0]) / (calibration_angles[2] - calibration_angles[0])
        offset = measured_angles[0] - slope * calibration_angles[0]
        
        print("\nCalibration Results:")
        print(f"Slope: {slope:.6f}")
        print(f"Offset: {offset:.6f}")
        print(f"\nCorrection equation:")
        print(f"true_angle = (measured_angle - {offset:.6f}) / {slope:.6f}")
    else:
        print("Error: Missing measurements")