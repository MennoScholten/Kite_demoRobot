import rospy

from smbus2 import SMBus
import time

from geometry_msgs.msg import Twist

# RPi Channel 1
channel = 1
# ADS1115 address and registers
address = 0x48
reg_config = 0x01
reg_conversion = 0x00

bus = SMBus(channel)

# Config value:
# - Single conversion
# - A0 input
# - 4.096V reference
config1 = [0xD2, 0xB3]

# Config value:
# - Single conversion
# - A1 input
# - 4.096V reference
config2 = [0xC2, 0xB3]

deadzoneX = 0.1 #Joystick deadzone value (X-AXIS) 
deadzoneZ = 0.1 #Joystick deadzone value (Z-AXIS) 
X_rev = 1   # Change to -1 to reverse axis
Z_rev = -1   # CHange to -1 to reverse axis

msg = Twist()

def publisher():
    pub = rospy.Publisher('joystick_teleop', Twist, queue_size= 10) # sets a ROS publisher with name Joystick_teleop
    rospy.init_node('joystick_node', anonymous = True)              # sets a ROS node with name Joystick_node
    rate = rospy.Rate(10)                                           # sets the ROS spinrate to 10 Hz

    while not rospy.is_shutdown():
        ##################################################################
        ## X_AXIS
        ##################################################################

        # Start conversion A0 channel (X-AXIS)
        bus.write_i2c_block_data(address, reg_config, config1)          # I2C communication between ADS & raspberry pi
        # Wait for conversion
        time.sleep(0.01)
        # Read 16-bit result
        result = bus.read_i2c_block_data(address, reg_conversion, 2)    # I2C communication between ADS & raspberry pi
        # Convert from 2-complement
        value = ((result[0] & 0xFF) << 8) | (result[1] & 0xFF)
        if value & 0x8000 != 0:
            value -= 1 << 16
        # Convert value to voltage
        v = value * 4.096 / 32768
        
        if(v <= 2.5 - deadzoneX or v >= 2.5 + deadzoneX):               # sets deadzone on X-Axis
            if(v <= 2.5):
                x = X_rev * (v * (-50) + 125)
                if(x >= X_rev * 98):
                    x = X_rev * 100
                msg.linear.x = x

            if(v >= 2.5):
                x = X_rev * (v * (-55) + 125)
                if(x <= X_rev *  -98):
                    x = X_rev * -100
                msg.linear.x = x
        else :
            x = 0  
            msg.linear.x = x  

        time.sleep(0.01)
        ##################################################################
        ## Z_AXIS
        ##################################################################
        # Start conversion A1 channel (Y-AXIS)
        bus.write_i2c_block_data(address, reg_config, config2)          # I2C communication between ADS & raspberry pi
        # Wait for conversion
        time.sleep(0.01)
        # Read 16-bit result
        result = bus.read_i2c_block_data(address, reg_conversion, 2)    # I2C communication between ADS & raspberry pi
        # Convert from 2-complement
        value = ((result[0] & 0xFF) << 8) | (result[1] & 0xFF)
        if value & 0x8000 != 0:
            value -= 1 << 16
        # Convert value to voltage
        v = value * 4.096 / 32768

        if(v <= 2.5 - deadzoneZ or v >= 2.5 + deadzoneZ):               # sets deadzone on Z-Axis
            if(v <= 2.5):
                z = Z_rev * (v * (-50) + 125)
                if(z <= Z_rev * 98):
                    z = Z_rev * 100
                msg.linear.z = z

            if(v >= 2.5):
                z = Z_rev * (v * (-55) + 125)
                if(z >= Z_rev * -98):
                    z = Z_rev * -100
                msg.linear.z = z

        else :
            z = 0  
            msg.linear.z = z 

        pub.publish(msg)        # Publishers the ROS geometry_Twist message to the ROS Network
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()             # Keeps looping while ros active
    except rospy.ROSInterruptException:
        pass




  