# gooddawg
Move your Unitree Go1 dog legs- without the dog  

![Gif of the robot leg moving in a straight line](ik_leg.gif)

# Wiring
- I use a [U2D2](https://www.robotis.us/u2d2/), the adapter for controlling dynamixels. Go1's motors operate at 5 000 000 bps, so we need a good RS485 adapter to talk to it.

# Normal Operation
- unplug the legs, they are now free.
- Manually fully extend leg (so the encoder 0 point makes sense for my example scripts)
- hook up the RS485 adapter and power (23-25v, low voltage may cause a brownout)
- run sudo ./example_cartesian_arm.py, the leg should move in a straight-ish line. 

# Code example
```python
import build_a_packet as bp

ser = bp.configure_serial("/dev/ttyUSB0") # connect to your U2D2

# you may send angles with q, velocities with dq, or feedforward torque with tau
# set Kp to 0 for velocity mode
# set Kp, Kd to 0 and send torques with tau
bp.send_packet(ser, bp.build_a_packet(id=1, q=q, dq=dq, Kp=4, Kd=0.3, tau=0.0))
bp.read_and_update_motor_data(ser) # read back some feedback data

time.sleep(0.01) # don't send too fast or you'll saturate the bus
print(bp.motor_data) # print angles and angular velocities
```

# special thanks/previous work
- [AATB for decoding the CRC on Go1 and Go2](https://github.com/aatb-ch/unitree_crc)
- [benrg for finding the initial CRC polynomial](https://crypto.stackexchange.com/questions/113287/do-i-have-any-hope-of-decoding-this-crc/113310#113310)
- [would not have been possible without the amazing freedog sdk](https://github.com/Bin4ry/free-dog-sdk)
- [devemin's awesome X](x.com/devemin/)
- [Bin4ry](https://github.com/Bin4ry)
- [d0tslash](https://x.com/d0tslash)
- [my messy notes on reverse engineering this](https://github.com/imcnanie/gooddog) 