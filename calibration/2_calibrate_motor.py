#!/usr/bin/env nix-shell
#!nix-shell -i python3 -p python39Packages.pyserial 

import struct
import serial
import time
import math
import sys
sys.path.append("..")
import build_a_packet as bp

MOTOR_ID = 0

if __name__ == "__main__":
    ser = bp.configure_serial("/dev/ttyUSB0")
    cnt = -50 # -50 for mot 1
    LUT = {}
    while True:


        cnt += 0.01
        q1 = int(cnt)/100

        print(q1)


        # you need to up these pids from 2->24 so it actaully goes to an accurate angle, this can be dangerous
        bp.send_packet(ser, bp.build_a_packet(id=MOTOR_ID, q=q1, dq=0, Kp=3, Kd=0.0, tau=0.00)) # to do velocity mode we make p 0, tau is how torqy, KD is vel
       
        bp.read_and_update_motor_data(ser)
        time.sleep(0.01)

        if bp.motor_data['mot'+str(MOTOR_ID)+'_angle'] is not None:
            a1 = bp.motor_data['mot'+str(MOTOR_ID)+'_angle']
            #print(f"q1: {q1:.3f}, a1: {a1:.3f}")
            LUT[q1] = a1
            print(LUT)
        else:
            print("Waiting for motor data...")