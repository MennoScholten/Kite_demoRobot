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
        #Clock.schedule_interval(self.curPos, 1)

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

        l1 = data.linear.x * 0.15 + 2155.2262062252
        l2 = data.linear.z * 0.15 + 2593.2260495977

        motormsg.angular.x = data.angular.x
        motormsg.angular.y = data.angular.y
        motormsg.angular.z = data.angular.z

        self.screen.ids.motorPos_one.text = '%s' % int(motormsg.linear.x)
        self.screen.ids.motorPos_two.text = '%s' % int(motormsg.linear.z)
        self.screen.ids.motorSpeed_one.text = '%s' % int(motormsg.angular.x)
        self.screen.ids.motorSpeed_two.text = '%s' % int(motormsg.angular.z)
        # if(motormsg.angular.y == (-5555.0) and singleDialog == True):
        #     print('error-1')
        #     self.show_motor2_dialog
        #     print('error-2')
        #     singleDialog = False

        # if(motormsg.angular.y == (-5555.0)):
        #     print('error-1')
        #     self.show_motor2_dialog
        #     print('error-2')
        #     singleDialog = False

        if(motormsg.angular.y == (-5555.0) and self.singleDialog == True):
            print('error 1')
            if not self.dialog:
                print('error 2')
                self.dialog = MDDialog(
                    text="Maxonmotor Upper Right & Upper Left corner: failure!",
                    buttons=[
                        MDFlatButton(
                            text="OK",
                            theme_text_color = "Custom",
                            text_color=self.theme_cls.primary_color,
                        ),
                        MDFlatButton(
                            text="Ot",
                            theme_text_color = "Custom",
                            text_color=self.theme_cls.primary_color,
                        ),
                    ],
                )
                print('error 3')
            print('error 4')
            self.dialog.open()
            print('error 5')

        # elif(motormsg.angular.y == -7777.0 and singleDialog == True):
        #     self.show_motor1_dialog()
        #     singleDialog = False

        # elif(motormsg.angular.y == -9999.0 and singleDialog == True):
        #     self.show_motor_dialog()
        #     singleDialog = False

        self.curPos(l1, l2)

    def navigationCallback(self, data):
        navimsg.linear.x = data.linear.x
        navimsg.linear.z = data.linear.z

        self.screen.ids.targX.text = '%s' % int(navimsg.linear.x)
        self.screen.ids.targZ.text = '%s' % int(navimsg.linear.z)

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
        os.system("rosrun rosserial_python serial_node.py /dev/ttyACM0")
        time.sleep(1)
        os.system("rosrun kite_postion position_node")
        time.sleep(1)
        os.system("rosrun kite_joystick_teleop kite_joystick_teleop.py")
        time.sleep(1)
        os.system("rosrun kite_navigation kite_navigation")
        time.sleep(10)
        self.close_startDialog

    def stopProgram(self, obj):
        msg = 0
        pub.publish(msg)
        self.dialogClose.dismiss()
        time.sleep(8)
        self.stop()

    def close_startDialog(self, obj):
        self.dialogStart.dismiss()

    def close_closeDialog(self, obj):
        self.dialogClose.dismiss()

    # def show_motor1_dialog(self):
    #     if not self.dialogMotor1:
    #         self.dialogMotor1 = MDDialog(
    #             text="Maxonmotor Upper Left corner: failure!",
    #             buttons=[
    #                 MDFlatButton(
    #                     text="OK",
    #                     theme_text_color = "Custom",
    #                     text_color=self.theme_cls.primary_color,
    #                 ),
    #             ],
    #         )
    #     self.dialogMotor1.open()

    # def show_motor2_dialog(self):
    #     if not self.dialogMotor2:
    #         self.dialogMotor2 = MDDialog(
    #             text="Maxonmotor Upper Right corner: failure!",
    #             buttons=[
    #                 MDFlatButton(
    #                     text="OK",
    #                     theme_text_color = "Custom",
    #                     text_color=self.theme_cls.primary_color,
    #                 ),
    #             ],
    #         )
    #     self.dialogMotor2.open()

    # def show_alert_dialog(self):
    #     if not self.dialog:
    #         self.dialog = MDDialog(
    #             text="Maxonmotor Upper Right & Upper Left corner: failure!",
    #             buttons=[
    #                 MDFlatButton(
    #                     text="OK",
    #                     theme_text_color = "Custom",
    #                     text_color=self.theme_cls.primary_color,
    #                 ),
    #                 MDFlatButton(
    #                     text="Ot",
    #                     theme_text_color = "Custom",
    #                     text_color=self.theme_cls.primary_color,
    #                 ),
    #             ],
    #         )
    #     self.dialog.open()

    # def show_motor_dialog(self):
    #     if not self.dialogMotors:
    #         self.dialogMotors = MDDialog(
    #             text="Maxonmotor Upper Right & Upper Left corner: failure!",
    #             buttons=[
    #                 MDFlatButton(
    #                     text="OK",
    #                     theme_text_color = "Custom",
    #                     text_color=self.theme_cls.primary_color,
    #                 ),
    #             ],
    #         )
    #     self.dialogMotors.open()

    def curPos(self, len1, len2):
        delX = int()
        delZ = int()
        L_1sqre = len1 * len1
        L_2sqre = len2 * len2

        # inverse kinematics. See chapter 2.4.5 of document: 220316_TDD_Afstudeerstage_MennoScholten_463048_V1
        delX = ((L_1sqre - L_2sqre + 2560000) / (3200)) - 150
        z_sqrtIn = 1-(L_1sqre - L_2sqre + 2560000) / (3200 * len1)
        delZ = 2100 - len1 * sqrt(z_sqrtIn)
        self.screen.ids.curX.text = '%s' % int(delX)
        self.screen.ids.curZ.text = '%s' % int(delZ)
        
    def motor1(self, *args):
        msg = 1
        pub.publish(msg)

    def motor2(self, *args):
        msg = 2
        pub.publish(msg)

    def homePos(self, *args):
        msg = 3
        pub.publish(msg)
        singleDialog = False


    def autoPos(self, *args):
        msg = 4
        pub.publish(msg)

    def manualPos(self, *args):
        msg = 5
        pub.publish(msg)

    def resetMotor(self, *args):
        msg = 6
        pub.publish(msg)
        singleDialog = True


if __name__ == '__main__':

    gui = GuiApp()
    pub = rospy.Publisher('system_Mode', UInt16, queue_size = 1)

    rospy.init_node('kite_gui', anonymous=True)

    rospy.Subscriber("joystick_teleop", Twist, gui.joyCallback, queue_size = 1)
    rospy.Subscriber("motor_Position", Twist, gui.motorCallback, queue_size = 4)
    rospy.Subscriber("navigation_position", Twist, gui.navigationCallback)

    gui.run()

    rospy.sleep(0.0001)
