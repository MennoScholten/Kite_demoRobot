# ROSNODE 1 HMI_controller 
# KITE ROBOTICS EXHIBITION STAND: DEMONSTATION SYSTEM
# Created on: 25-04-2022
# Version 1.0
# 
# Program designed by: Menno Scholten

#!/usr/bin/env python3.8
from math import sqrt
import rospy
import time
import os

from kivymd.uix.boxlayout import MDBoxLayout
from kivy.properties import ObjectProperty
from kivymd.uix.button import MDFlatButton
from kivymd.uix.dialog import MDDialog
from kivy.core.window import Window
from kivy.config import Config
from kivy.clock import Clock

from kivymd.app import MDApp
from kivy.lang import Builder
from std_msgs.msg import UInt16
from geometry_msgs.msg import Twist

joymsg = Twist()
motormsg = Twist()
navimsg = Twist()

linRes = 0.184

# class for creating the navigation drawer (left side menu)
class ContentNavigationDrawer(MDBoxLayout):
    screen_manager = ObjectProperty()
    nav_drawer = ObjectProperty()
    
# class which handels the gui
class GuiApp(MDApp):
    dialogClose = None      # Dialog variable for closing the gui
    dialogStart = None      # Dialog variable for starting the drivers

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.theme_cls.theme_style = "Dark" # sets theme of GUI to DARK MODE

        self.screen = Builder.load_file('/home/ubuntu/Desktop/KITE_demo_system/src/kite_demo_system/kite_gui/scripts/ros_gui.kv') # kivymd file location which is used to create GUI

    def build(self):
        Window.size = (1024, 600)       # Sets window size to Raspbery Pi Touchscreen size
        Window.borderless = True        # Removes border of application
        
        return self.screen

    def on_start(self):
        self.fps_monitor_start()        # Shows a FPS (frames/second) monitor at the top

    def joyCallback(self, data):                                     # ROS Subscriber callback handler. Handles joystick callbacks
        joymsg.linear.x = data.linear.x
        joymsg.linear.z = data.linear.z
        self.screen.ids.joyPosX.text = '%s' % int(joymsg.linear.x)
        self.screen.ids.joyPosZ.text = '%s' % int(joymsg.linear.z)

    def motorCallback(self, data):                                  # ROS Subscriber callback handler. Handles motor information callbacks.
        motormsg.linear.x = data.linear.x
        motormsg.linear.z = data.linear.z

        motormsg.angular.x = data.angular.x
        motormsg.angular.y = data.angular.y
        motormsg.angular.z = data.angular.z

        self.screen.ids.motorPos_one.text = '%s' % int(motormsg.linear.x)
        self.screen.ids.motorPos_two.text = '%s' % int(motormsg.linear.z)
        self.screen.ids.motorSpeed_one.text = '%s' % int(motormsg.angular.x)
        self.screen.ids.motorSpeed_two.text = '%s' % int(motormsg.angular.z)

    
    def positionCallback(self,data):                            # ROS Subscriber callback handler. Handles the current position callbacks.
        self.screen.ids.curX.text = '%s' % int(data.linear.x)
        self.screen.ids.curZ.text = '%s' % int(data.linear.z)
        self.screen.ids.curX_main.text = '%s' % int(data.linear.x)
        self.screen.ids.curZ_main.text = '%s' % int(data.linear.z)

    def navigationCallback(self, data):                         # ROS Subscriber callback handler. Handles the desired position callbacks.
        navimsg.linear.x = data.linear.x
        navimsg.linear.z = data.linear.z

        self.screen.ids.targX.text = '%s' % int(navimsg.linear.x)
        self.screen.ids.targZ.text = '%s' % int(navimsg.linear.z)
        self.screen.ids.targX_main.text = '%s' % int(navimsg.linear.x)
        self.screen.ids.targZ_main.text = '%s' % int(navimsg.linear.z)

    def startCallback(self):       # Dialog handler for creating a dialog when pressing the 'Start' button 
        if not self.dialogStart:
            self.dialogStart = MDDialog(
                text="Do you want to start the software and drivers?",
                buttons=[
                    MDFlatButton(
                        text="YES",
                        theme_text_color = "Custom",
                        text_color=self.theme_cls.primary_color, on_release = self.startProgram
                    ),
                    MDFlatButton(
                        text="NO",
                        theme_text_color = "Custom",
                        text_color=self.theme_cls.primary_color, on_release = self.close_startDialog
                    ),
                ],
            )
        self.dialogStart.open()

    def stopCallback(self):     # Dialog handler for creating a dialog when pressing the 'Stop' button
        if not self.dialogClose:
            self.dialogClose = MDDialog(
                text="Do you want to close the program?",
                buttons=[
                    MDFlatButton(
                        text="YES",
                        theme_text_color = "Custom",
                        text_color=self.theme_cls.primary_color, on_release = self.stopProgram
                    ),
                    MDFlatButton(
                        text="NO",
                        theme_text_color = "Custom",
                        text_color=self.theme_cls.primary_color, on_release = self.close_closeDialog
                    ),
                ],
            )
        self.dialogClose.open()

    def startProgram(self, obj):    # Handles the starting of the drivers.
        # os.system("rosrun rosserial_python serial_node.py /dev/ttyACM0")
        time.sleep(1)
        self.close_startDialog

    def stopProgram(self, obj):     # Handles the exiting of ROSNODES and the ROSCORE and closes the GUI application
        msg = 0
        pub.publish(msg)
        self.dialogClose.dismiss()
        os.system("rosnode kill -a")        # Kills all running ROSNODES
        time.sleep(1)
        os.system("killall - roscore")      # Kills running ROSCORE
        os.system("killall - rosmaster")    # Kills running ROSMASTER
        time.sleep(1)
        self.stop()

    def close_startDialog(self, obj):   # Exits the dialog for starting the drivers 
        self.dialogStart.dismiss()

    def close_closeDialog(self, obj):   # Exits the dialog for closing the GUI Application
        self.dialogClose.dismiss()
        
    def motor1(self, *args):        # Publishes: Mode = 1 to system_Mode (Joystick motor 1 controll)
        msg = 1
        pub.publish(msg)

    def motor2(self, *args):        # Publishes: Mode = 2 to system_Mode (Joystick motor 2 controll)
        msg = 2
        pub.publish(msg)

    def homePos(self, *args):       # Publishes: Mode = 3 to system_Mode (Set current position as Homeposition [0,0] (x,z))
        msg = 3
        pub.publish(msg)

    def gohomePos(self, *args):     # Publishes: Mode = 7 to system_Mode (Go to homeposition [0,0](x,z))
        msg = 7
        pub.publish(msg)

    def autoPos(self, *args):       # Publishes: Mode = 4 to system_Mode (Set system in automatic demonstration mode)
        msg = 4
        pub.publish(msg)

    def manualPos(self, *args):     # Publishes: Mode = 5 to system_Mode (Set system in manual demonstration mode)
        msg = 5
        pub.publish(msg)

    def resetMotor(self, *args):    # Publishes: Mode = 6 to system_Mode (Reset any failures in motors)
        msg = 6
        pub.publish(msg)


if __name__ == '__main__':

    gui = GuiApp()  # Gui variable      
    pub = rospy.Publisher('system_Mode', UInt16, queue_size = 1)        # Setup of ROS Publisher: system_Mode

    rospy.init_node('kite_gui', anonymous=True)     # Initializes current node

    rospy.Subscriber("joystick_teleop", Twist, gui.joyCallback, queue_size = 1)     # Subscribes to ROSNODE : joystick_teleop
    rospy.Subscriber("motor_Position", Twist, gui.motorCallback, queue_size = 4)    # Subscribes to ROSNODE : motor_Position
    rospy.Subscriber("navigation_position", Twist, gui.navigationCallback)          # Subscribes to ROSNODE : navigation_position
    rospy.Subscriber("cur_position", Twist, gui.positionCallback)                   # Subscribes to ROSNODE : cur_position

    gui.run()       # Starts GUI application

    rospy.sleep(0.0001) # loops with a maximum of 10000 Hz
