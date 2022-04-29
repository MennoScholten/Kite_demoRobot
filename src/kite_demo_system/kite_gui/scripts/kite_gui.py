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

class ContentNavigationDrawer(MDBoxLayout):
    screen_manager = ObjectProperty()
    nav_drawer = ObjectProperty()
    

class GuiApp(MDApp):
    L_1 = float()
    L_2 = float()
    dialog = None
    dialogMotor1 = None
    dialogMotor2 = None
    dialogMotors = None
    dialogClose = None
    dialogStart = None
    singleDialog = True

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.theme_cls.theme_style = "Dark"

        self.screen = Builder.load_file('/home/ubuntu/Desktop/KITE_demo_system/src/kite_demo_system/kite_gui/scripts/ros_gui.kv')

    def build(self):
        Window.size = (1024, 600)
        Window.borderless = True
        
        return self.screen

    def on_start(self):
        self.fps_monitor_start()

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

    
    def positionCallback(self,data):
        self.screen.ids.curX.text = '%s' % int(data.linear.x)
        self.screen.ids.curZ.text = '%s' % int(data.linear.z)
        self.screen.ids.curX_main.text = '%s' % int(data.linear.x)
        self.screen.ids.curZ_main.text = '%s' % int(data.linear.z)

    def navigationCallback(self, data):
        navimsg.linear.x = data.linear.x
        navimsg.linear.z = data.linear.z

        self.screen.ids.targX.text = '%s' % int(navimsg.linear.x)
        self.screen.ids.targZ.text = '%s' % int(navimsg.linear.z)
        self.screen.ids.targX_main.text = '%s' % int(navimsg.linear.x)
        self.screen.ids.targZ_main.text = '%s' % int(navimsg.linear.z)

    def startCallback(self):
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

    def stopCallback(self):
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

    def startProgram(self, obj):
        # os.system("rosrun rosserial_python serial_node.py /dev/ttyACM0")
        time.sleep(1)
        self.close_startDialog

    def stopProgram(self, obj):
        msg = 0
        pub.publish(msg)
        self.dialogClose.dismiss()
        os.system("rosnode kill -a")
        time.sleep(1)
        os.system("killall - roscore")
        os.system("killall - rosmaster")
        time.sleep(1)
        self.stop()

    def close_startDialog(self, obj):
        self.dialogStart.dismiss()

    def close_closeDialog(self, obj):
        self.dialogClose.dismiss()
        
    def motor1(self, *args):
        msg = 1
        pub.publish(msg)

    def motor2(self, *args):
        msg = 2
        pub.publish(msg)

    def homePos(self, *args):
        msg = 3
        pub.publish(msg)

    def gohomePos(self, *args):
        msg = 7
        pub.publish(msg)

    def autoPos(self, *args):
        msg = 4
        pub.publish(msg)

    def manualPos(self, *args):
        msg = 5
        pub.publish(msg)

    def resetMotor(self, *args):
        msg = 6
        pub.publish(msg)


if __name__ == '__main__':

    gui = GuiApp()
    pub = rospy.Publisher('system_Mode', UInt16, queue_size = 1)

    rospy.init_node('kite_gui', anonymous=True)

    rospy.Subscriber("joystick_teleop", Twist, gui.joyCallback, queue_size = 1)
    rospy.Subscriber("motor_Position", Twist, gui.motorCallback, queue_size = 4)
    rospy.Subscriber("navigation_position", Twist, gui.navigationCallback)
    rospy.Subscriber("cur_position", Twist, gui.positionCallback)

    gui.run()

    rospy.sleep(0.0001)
