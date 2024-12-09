# gooddawg
This library is just like the [freedog sdk](https://github.com/Bin4ry/free-dog-sdk), but 
![Gif of the robot leg moving in a straight line](ik_leg.gif)

# Wiring
- I use a [U2D2](https://www.robotis.us/u2d2/), the adapter for controlling dynamixels. Go1's motors operate at 5 000 000 bps, so we need a good RS485 adapter to talk to it.

# Normal Operation
- unplug the legs, they are now free.
- Manually fully extend leg (so the encoder 0 point makes sense for my example scripts)
- hook up the RS485 adapter and power (23-25v, low voltage may cause a brownout)
- run sudo ./example_cartesian_arm.py, the leg should move in a straight-ish line. 

# special thanks/previous work
- [AATB for decoding the CRC on Go1 and Go2](https://github.com/aatb-ch/unitree_crc)
- [benrg for finding the initial CRC polynomial](https://crypto.stackexchange.com/questions/113287/do-i-have-any-hope-of-decoding-this-crc/113310#113310)
- [would not have been possible without the amazing freedog sdk](https://github.com/Bin4ry/free-dog-sdk)
- [devemin's awesome X](x.com/devemin/)
- [Bin4ry](https://github.com/Bin4ry)
- [d0tslash](https://x.com/d0tslash)
- [my messy notes on reverse engineering this](https://github.com/imcnanie/gooddog) 