#!/usr/bin/env nix-shell
#!nix-shell -i python3 -p python39Packages.pyserial 

import struct
import serial
import time
import math


POLY = 0x04C11DB7

def crc32_core(data_words):
    crc = 0xFFFFFFFF
    for word in data_words:
        crc ^= word
        for _ in range(32):
            if crc & 0x80000000:
                crc = (crc << 1) ^ POLY
            else:
                crc <<= 1
            crc &= 0xFFFFFFFF
    return crc

def get_go1_crc(hex_string):
    data_bytes = bytes.fromhex(hex_string)
    data_words = struct.unpack('<7I', data_bytes[:28])
    crc = crc32_core(data_words)
    crc_bytes = struct.pack('<I', crc)
    return crc_bytes.hex()

def torque_to_hex(key):
    if key >= 0:
        speed = int(key) // 256
        sign_byte = '00'
    else:
        speed = (int(key) + 65281) // 256
        sign_byte = 'ff'
    speed_byte = '{:02x}'.format(speed & 0xFF)  # Ensure valid hex by masking
    hex_value = speed_byte + sign_byte
    return hex_value

def vel_to_hex(key):
    # Convert to 16-bit value
    value = int(key*65000) & 0xFFFF
    # Get low and high bytes for little endian
    low_byte = value & 0xFF
    high_byte = (value >> 8) & 0xFF
    # Return as hex string with bytes swapped (little endian)
    return f"{low_byte:02x}{high_byte:02x}"

"""def pos_to_hex(number):
    # Determine if number is negative
    is_negative = number < 0
    sign = "ffff" if is_negative else "0000"
    
    # Convert absolute value to hex, keeping only last 4 digits
    hex_value = "{:04x}".format(abs(number) & 0xFFFF)
    
    # Return combined string
    return f"{hex_value}{sign}"
"""
def pos_to_hex(number):
    # Convert to 16-bit value

    value = int((number)*65000+65536) & 0xFFFF
    # Determine sign based on number
    sign = "ffff" if number < 0 else "0000"
    # Get low and high bytes
    low_byte = value & 0xFF 
    high_byte = (value >> 8) & 0xFF
    #print(f"{low_byte:02x}{high_byte:02x}{sign}")
    ##sign = "ff7f" "torque mode" make this ff7f
    return f"{low_byte:02x}{high_byte:02x}{sign}" #0000000 - 00ff0000 45 to -138 degrees

    # 0000 0000, ffff ffff is 0
    #
def p_to_hex(P):
    #scaled = int(P * 2) in freedog, this is scaled by 2, I don't think it's neces
    
    # Convert to 2-digit hex#    
    return '{:02x}'.format(P)

def d_to_hex(D):
    # Scale the D value to match known point: D=1.0 -> f909
    scaled = int(D * 2553)  # 2553 (decimal) = 09f9 (hex) (little endian)
    
    # Convert to 16-bit little-endian
    low_byte = (scaled & 0xFF)
    high_byte = (scaled >> 8) & 0xFF
    
    # Format as 4-digit hex with bytes swapped (little-endian)
    return f"{low_byte:02x}{high_byte:02x}"
    
# Kp must be an int
# dq is += 0.5
#torque = T_ff (tau) + Kp*(q - p) + kd*(dq-w)
# p is current rotor angle, w is current angular velocity
def build_a_packet(id, q, dq, Kp, Kd, tau):
    header = "feee"
    motor_id = '{:02x}'.format(id) #01 02 03
    reserved = "ba0aff000000000000" # might contain some mode information
    torque = torque_to_hex(int(tau*100000)) # xxxx
    vel = vel_to_hex(dq) # xxxx
    if id == 2:
        position = pos_to_hex(0.379897*q + -0.120322) # position-> xxxx xxxx <- sign (0000 or ffff)
    else:
        position = pos_to_hex(-0.235337*q + 0.459373-0.03) # position-> xxxx xxxx <- sign (0000 or ffff)
    kp = p_to_hex(int(Kp)) # xx
    reserved2 = "00" # this might be more kp precision! seems to make motor go crazy
    kd = d_to_hex(Kd) # xx
    reserved3 = "020000000000"
    packet = header + motor_id + reserved + torque + vel + position + kp + reserved2 + kd + reserved3
    crc = get_go1_crc(packet)
    #print(header + " " + motor_id + " " + reserved + " " + torque + " " + vel + " " + position + " " + kp + " " + reserved2 + " " + kd + " " + reserved3)
    return packet + crc

# serial
def configure_serial(port):
    ser = serial.Serial(
        port=port,                   
        baudrate=5000000,            
        bytesize=serial.EIGHTBITS,   
        stopbits=serial.STOPBITS_ONE, 
        parity=serial.PARITY_NONE,   
        timeout=0,                   
        rtscts=False,                
        dsrdtr=False                 
    )

    ser.reset_input_buffer()
    ser.reset_output_buffer()

    return ser


def send_packet(ser, packet):
    ser.write(bytes.fromhex(packet))



def interpret_signed_angle(hex_string):
    """
    Interpret the given hex string as a single signed 32-bit integer in little-endian format.

    Parameters:
    hex_string (str): A hex string of 8 characters representing a 32-bit value in little-endian.

    Returns:
    int: The interpreted signed integer.
    """
    # Ensure the hex string has exactly 8 characters (4 bytes)
    if len(hex_string) != 8:
        return "Error: Invalid input length"

    # Interpret the entire 8-character hex string as a little-endian signed 32-bit integer
    # Convert hex to an integer with two's complement interpretation
    signed_value = float(int.from_bytes(bytes.fromhex(hex_string), "little", signed=True))

    return -signed_value/20000 #TODO it's probably not div/2 measure angle accurately! This should be radians


def interpret_signed_angle_2byte(hex_string):
    signed_value = float(int.from_bytes(bytes.fromhex(hex_string), "little", signed=True))
    return -signed_value #TODO it's probably not div/2 measure angle accurately! This should be radians

motor_data = {"mot0_angle": None,
              "mot1_angle": None,
              "mot2_angle": None,
              "mot0_velocity": 0.0,
              "mot1_velocity": 0.0,
              "mot2_velocity": 0.0,}

def read_and_update_motor_data(ser):
    response = ser.read(int(156/2)).hex()

    try:
        if response.startswith("feee00010a00"):
            motor_data["mot0_angle"] = float(interpret_signed_angle(response[60:68]))
            motor_data["mot0_velocity"] = interpret_signed_angle_2byte(response[28:32])/10000.0 
        if response.startswith("feee01010a00"):
            angle = float(interpret_signed_angle(response[60:68]))
            motor_data["mot1_angle"] = (angle - -1.539519) / 0.793123
            motor_data["mot1_velocity"] = interpret_signed_angle_2byte(response[28:32])/10000.0 
        if response.startswith("feee02010a00"):
            angle = float(interpret_signed_angle(response[60:68]))
            motor_data["mot2_angle"] = (angle - 0.378445) / -1.219688 
            motor_data["mot2_velocity"] = interpret_signed_angle_2byte(response[28:32])/10000.0 

    except (ValueError, TypeError) as e:
        print("Data corruption detected or invalid angle data:", e)

if __name__ == "__main__":
    ser = configure_serial("/dev/ttyUSB0")
    while True:
        q = 3.14*math.sin(time.time()*3)  # Sine wave oscillating between -150 and +150
        # Kp must be an int
        # dq is += 0.5
        #torque = T_ff (tau) + Kp*(q - p) + kd*(dq-w)
        # p is current rotor angle, w is current angular velocity
        send_packet(ser, build_a_packet(id=2, q=q, dq=0.0, Kp=1, Kd=0.05, tau=0.0)) # to do velocity mode we make p 0, tau is how torqy, KD is vel
        time.sleep(0.001)  # Small delay to control update rate

        read_and_update_motor_data(ser)
        print(motor_data)

    # q is pos target, dq is vel target, tau is torque command
    #print(build_a_packet(id=2, q=-1121.1, dq = 0.0, Kp = 3.1, Kd = 1.879, tau = 0.0)cd)
