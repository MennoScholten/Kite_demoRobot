#!/usr/bin/env python3.8

import rospy
from kivymd.app import MDApp
from kivy.lang import Builder
from std_msgs.msg import UInt16



class TutorialApp(MDApp):

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.screen = Builder.load_file('/home/ubuntu/Desktop/KITE_demo_system/src/kite_demo_system/kite_gui/scripts/ros_gui.kv')

    def build(self):
        return self.screen


    def motor1(self, *args):
        print("Button pressed")
        
        msg = 1
        pub.publish(msg)

    def motor2(self, *args):
        print("Button pressed")
        msg = 2
        pub.publish(msg)

    



if __name__ == '__main__':

    pub = rospy.Publisher('system_Mode', UInt16, queue_size = 1)

    rospy.init_node('kite_gui', anonymous=True)

    rospy.Subscriber("joystick_teleop", Twist, gui.joyCallback)
    rospy.Subscriber("motor_Position", Twist, gui.motorCallback)

    TutorialApp().run()

    rospy.sleep(0.1)