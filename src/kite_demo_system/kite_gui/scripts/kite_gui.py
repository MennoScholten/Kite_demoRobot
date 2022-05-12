# ROSNODE 1 HMI_controller 
# KITE ROBOTICS EXHIBITION STAND: DEMONSTATION SYSTEM
# Created on: 25-04-2022
# Version 1.0
# 
# Program designed by: Menno Scholten

#!/usr/bin/env python3.8
from distutils.command.config import config
from distutils.log import error
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
    dialogError = None      # Dialog variable for starting the drivers
    error = False           # Error flag
    errorVal = 0            # Value of error
    dialogOpen = False      # Dialog open flag
    singleShot = False      # Single excecution flag of function

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.theme_cls.theme_style = "Dark" # sets theme of GUI to DARK MODE

        self.screen = Builder.load_file('/home/ubuntu/Desktop/KITE_demo_system/src/kite_demo_system/kite_gui/scripts/ros_gui.kv') # kivymd file location which is used to create GUI

    def build(self):
        # Window.size = (1024, 900)       # Sets window size to Raspbery Pi Touchscreen size
        # Window.borderless = True        # Removes border of application
        
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

        self.screen.ids.motorPos_one.text = '%s mm' % int(motormsg.linear.x)
        self.screen.ids.motorPos_two.text = '%s mm' % int(motormsg.linear.z)
        self.screen.ids.motorSpeed_one.text = '%s RPM' % int(5.5 * motormsg.angular.x / 63)  # Calc speed in RPM
        self.screen.ids.motorSpeed_two.text = '%s RPM' % int(5.5 * motormsg.angular.z / 63)     # Calc speed in RPM

    
    def positionCallback(self,data):                            # ROS Subscriber callback handler. Handles the current position callbacks.
        self.screen.ids.curX.text = '%s mm' % int(data.linear.x)
        self.screen.ids.curZ.text = '%s mm' % int(data.linear.z)
        self.screen.ids.curX_main.text = '%s mm' % int(data.linear.x)
        self.screen.ids.curZ_main.text = '%s mm' % int(data.linear.z)
        if(int(data.angular.y) < 0 and self.singleShot == False):
            self.screen.ids.toolbar.md_bg_color = (1, 0, 0, 1)
            self.error = True
            if(int(data.angular.y) == -5555):
                self.errorVal = 1
            elif(int(data.angular.y) == -7777):
                self.errorVal = 2
            elif(int(data.angular.y) == -9999):
                self.errorVal = 3
            else:
                self.errorVal = 4

            self.singleShot == True

    def navigationCallback(self, data):                         # ROS Subscriber callback handler. Handles the desired position callbacks.
        navimsg.linear.x = data.linear.x
        navimsg.linear.z = data.linear.z

        self.screen.ids.targX.text = '%s mm' % int(navimsg.linear.x)
        self.screen.ids.targZ.text = '%s mm' % int(navimsg.linear.z)
        self.screen.ids.targX_main.text = '%s mm' % int(navimsg.linear.x)
        self.screen.ids.targZ_main.text = '%s mm' % int(navimsg.linear.z)

    def errorPress(self):
        if(self.error == True):
            if not self.dialogClose:
                if(self.errorVal == 1):
                    self.dialogError = MDDialog(
                    title="Error in TOP RIGHT motordriver!",
                    text="Do you want to reset motordrivers?",
                    buttons=[
                        MDFlatButton(
                            text="YES",
                            theme_text_color = "Custom",
                            text_color=self.theme_cls.primary_color, on_release = self.resetMotor
                        ),
                        MDFlatButton(
                            text="NO",
                            theme_text_color = "Custom",
                            text_color=self.theme_cls.primary_color, on_release = self.close_errorDialog
                        ),
                    ],
                    )

                elif(self.errorVal == 2):
                    self.dialogError = MDDialog(
                    title="Error in TOP LEFT motordriver!",
                    text="Do you want to reset motordrivers?",
                    buttons=[
                        MDFlatButton(
                            text="YES",
                            theme_text_color = "Custom",
                            text_color=self.theme_cls.primary_color, on_release = self.resetMotor
                        ),
                        MDFlatButton(
                            text="NO",
                            theme_text_color = "Custom",
                            text_color=self.theme_cls.primary_color, on_release = self.close_errorDialog
                        ),
                    ],
                    )

                elif(self.errorVal == 3):
                    self.dialogError = MDDialog(
                    title="Error in BOTH motordrivers!",
                    text="Do you want to reset motordrivers?",
                    buttons=[
                        MDFlatButton(
                            text="YES",
                            theme_text_color = "Custom",
                            text_color=self.theme_cls.primary_color, on_release = self.resetMotor
                        ),
                        MDFlatButton(
                            text="NO",
                            theme_text_color = "Custom",
                            text_color=self.theme_cls.primary_color, on_release = self.close_errorDialog
                        ),
                    ],
                    )

                elif(self.errorVal == 4):
                    self.dialogError = MDDialog(
                    title="Unknown error in motordrivers!",
                    text="Do you want to reset motordrivers?",
                    buttons=[
                        MDFlatButton(
                            text="YES",
                            theme_text_color = "Custom",
                            text_color=self.theme_cls.primary_color, on_release = self.resetMotor
                        ),
                        MDFlatButton(
                            text="NO",
                            theme_text_color = "Custom",
                            text_color=self.theme_cls.primary_color, on_release = self.close_errorDialog
                        ),
                    ],
                    )
            self.dialogError.open() 
            self.dialogOpen = True 


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

    def close_errorDialog(self, obj):   # Exits the dialog for starting the drivers 
        self.dialogError.dismiss()

    def close_closeDialog(self, obj):   # Exits the dialog for closing the GUI Application
        self.dialogClose.dismiss()
        
    def motor1(self, *args):        # Publishes: Mode = 1 to system_Mode (Joystick motor 1 controll)
        msg = 1
        pub.publish(msg)

    def motor2(self, *args):        # Publishes: Mode = 2 to system_Mode (Joystick motor 2 controll)
        msg = 2
        pub.publish(msg)

    def homePos(self, *args):       # Publishes: Mode = 3 to system_Mode (Set current position as Homeposition [0,0] (x,z))
        self.screen.ids.toolbar.md_bg_color = (0.00392157, 0.30196078, 0.59607843, 1)
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
        self.error == False
        self.errorVal = 0
        if(self.dialogOpen == True):
            self.dialogError.dismiss()
            self.dialogOpen = False
        msg = 6
        pub.publish(msg)
        time.sleep(2)
        self.singleShot == False
        self.screen.ids.toolbar.md_bg_color = (1, 0.73, 0, 1)


if __name__ == '__main__':

    Config.set('graphics', 'fullscreen', 'auto')
    Config.set('graphics', 'window_state', 'maximized')
    Config.write()

    gui = GuiApp()  # Gui variable      
    pub = rospy.Publisher('system_Mode', UInt16, queue_size = 1)        # Setup of ROS Publisher: system_Mode

    rospy.init_node('kite_gui', anonymous=True)     # Initializes current node

    rospy.Subscriber("joystick_teleop", Twist, gui.joyCallback, queue_size = 1)     # Subscribes to ROSNODE : joystick_teleop
    rospy.Subscriber("motor_Position", Twist, gui.motorCallback, queue_size = 4)    # Subscribes to ROSNODE : motor_Position
    rospy.Subscriber("navigation_position", Twist, gui.navigationCallback)          # Subscribes to ROSNODE : navigation_position
    rospy.Subscriber("cur_position", Twist, gui.positionCallback)                   # Subscribes to ROSNODE : cur_position

    gui.run()       # Starts GUI application

    rospy.sleep(0.0001) # loops with a maximum of 10000 Hz
