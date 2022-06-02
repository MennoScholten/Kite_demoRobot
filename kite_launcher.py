#! /bin/bash source /home/ubuntu/Desktop/KITE_demo_system/devel/setup.bash
#! /usr/bin/env python3.8

import os

os.system("sudo xrandr --newmode \"1024x600_60.00\"  49.00  1024 1072 1168 1312  600 603 613 624 -hsync +vsync")
os.system("sudo xrandr --addmode HDMI-1 \"1024x600_60.00\"")
os.system("roslaunch kite_gui kite_gui.launch")

