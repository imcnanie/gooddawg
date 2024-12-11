#!/usr/bin/env nix-shell
#!nix-shell -i python3 -p python39Packages.pyserial 

import struct
import serial
import time
import math
import build_a_packet as bp


if __name__ == "__main__":
    ser = bp.configure_serial("/dev/ttyUSB0")
    while True:
        q = math.sin(time.time())*0.09 + 0.2  # Sine wave oscillating between -150 and +150
        dq = math.cos(time.time())*0.09


        bp.send_packet(ser, bp.build_a_packet(id=1, q=q, dq=dq, Kp=4, Kd=0.3, tau=0.0)) # to do velocity mode we make p 0, dq is vel, tau is feedforward
        bp.read_and_update_motor_data(ser)
        time.sleep(0.01)

        bp.send_packet(ser, bp.build_a_packet(id=2, q=q, dq=dq, Kp=4, Kd=0.3, tau=0.0)) # to do velocity mode we make p 0, dq is vel, tau is feedforward
        bp.read_and_update_motor_data(ser)
        time.sleep(0.01)

        if bp.motor_data['mot1_angle'] is not None and bp.motor_data['mot2_angle'] is not None:
            diff1 = bp.motor_data['mot1_angle']
            diff2 = bp.motor_data['mot2_angle']
            print(f"diff1: {diff1:.3f}, diff2: {diff2:.3f}")
        else:
            print("Waiting for motor data...")
