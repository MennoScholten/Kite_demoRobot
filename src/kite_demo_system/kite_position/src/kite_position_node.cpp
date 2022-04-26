#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>

#define wsX 1100                        // Workspace distance X-axis (mm)
//#define wsZ 2000                        // Workspace distance Z-axis (mm)
//#define wsX 1300
#define wsZ 1900
#define xOffset 150                     // Offset from workspace to cablepoint X-axis (mm)
#define zOffset 150                     // Offset from workspace to cablepoint Z-axis (mm)
#define xMotordist wsX + 2 * xOffset    // Distance between cablepoints X-axis (mm)
#define zMotordist wsZ + 2 * zOffset    // Distacne between cablepoints Z-axis (mm)

#define linSpeed 0.5                    // Maximum linear speed of system (M/s) (movement on one axis)
//#define mmRev 130                       // mm of cable movement per revolution of cable reel 
#define mmRev 100                       // mm of cable movement per revolution of cable reel 
#define stepdegree 0.05                 // Angle of rotation per step
#define pulsRev (360 / stepdegree)        // Amount of step pulses needed per revolution
//#define mmPuls (mmRev / pulsRev)          // mm of cable movement per step
#define mmPuls 0.184
//#define mmPuls 2

//#define maxSpeed 2000                  // Maximum speed of the motors in pulses/second
#define maxSpeed 500                  // Maximum speed of the motors in pulses/second
//#define maxSpeed 150

std_msgs::Bool getNewPos;
unsigned int modePointer = 0;

geometry_msgs::Twist waypointmsg;
geometry_msgs::Twist joystickmsg;
geometry_msgs::Twist motormsg;

void subWaypointCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    waypointmsg.linear.x = msg->linear.x;
    waypointmsg.linear.z = msg->linear.z;   
}

void subJoystickCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    joystickmsg.linear.x = msg->linear.x;
    joystickmsg.linear.z = msg->linear.z;

}

void subModeCallback(const std_msgs::UInt16::ConstPtr& msg) {
    modePointer = msg->data;   
}

void subMotorPositionCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    motormsg.linear.x = msg->linear.x;
    motormsg.linear.z = msg->linear.z;

}

double calcLength(int x, int z);

void calcSpeed(double len1, float &speed1, double len2, float &speed2);

void calcSpeedJoy(int joyx, int joyz, float& speed1, float& speed2);


int calcStepAmount(double value);



int main(int argc, char** argv)
{   
    bool singleShot = false;
    int joyX = 0;
    int joyZ = 0;
    int posX = 0;
    int posZ = 0;
    int curX = 0;
    int curZ = 0;
    double steps1 = 0.0;
    double steps2 = 0.0;
    float speed1 = 0.0;
    float speed2 = 0.0;
    int i = 0;

    const double lengthCable1 = calcLength(xOffset, wsZ + zOffset);         // Length calbe 1 (UP LEFT) at rest [0, 0]
    const double lengthCable2 = calcLength(wsX + xOffset, wsZ + zOffset);   // Length cable 2 (UP RIGHT) at rest [0, 0]
    // Init ROS node
    ros::init(argc, argv, "kite_position_node"); 
    ros::NodeHandle nh;
    ros::Rate r(30);        // 10 Hz spinrate
  
    ros::Publisher motorPub = nh.advertise<geometry_msgs::Twist>("motor_control", 100);
    ros::Publisher waypointPub = nh.advertise<std_msgs::Bool>("nextWaypoint", 100);

    ros::Subscriber subWaypoint = nh.subscribe("navigation_position", 1000, subWaypointCallback);
    ros::Subscriber subJoystick = nh.subscribe("joystick_teleop", 1000, subJoystickCallback);
    ros::Subscriber subMode     = nh.subscribe("system_Mode", 1000, subModeCallback);
    ros::Subscriber subMotorpos = nh.subscribe("motor_Position", 1000, subMotorPositionCallback);

    // Create Twist message
    geometry_msgs::Twist msgs;

    
    while(ros::ok()) {
        
        if(modePointer == 0) {
            // Mode 0 Start pointer no Function

        }

        else if(modePointer == 1 && int(motormsg.angular.y) == 0) {
            // Mode 1 Single motorcontrol via joystick (motor 1)
            joyX = int(joystickmsg.linear.z);

            if(joyX > 0) {
                    
                curX = 200000;
            }
            else if(joyX < 0) {
                curX = -200000;
            }
            else if(joyX == 0) {
                curX = motormsg.linear.z;
            }
            
            int steps = calcStepAmount(curX);
            unsigned int speed = 0;     

            if(joyX > 0) {
                speed = 20 * joyX;
            }

            else if(joyX < 0) {
                speed = -(20 * joyX);
            }
            else {
                speed = 0;
            }

            msgs.linear.x = steps;
            msgs.linear.z = motormsg.linear.z;
            msgs.angular.x = speed;
            msgs.angular.z = 0;
            msgs.angular.y = 0;
        }
        
        else if (modePointer == 2 && int(motormsg.angular.y) == 0) {
            // Mode 2 Single motorcontrol via joystick (motor 2)
            joyZ = int(joystickmsg.linear.z);

            if(joyZ > 0) {
                    
                curX = 200000;
            }
            else if(joyZ < 0) {
                curX = -200000;
            }
            else if(joyZ == 0) {
                curX = motormsg.linear.z;
            }
            
            int steps = calcStepAmount(curX);
            unsigned int speed = 0;     

            if(joyZ > 0) {
                speed = 20 * joyZ;
            }

            else if(joyZ < 0) {
                speed = -(20 * joyZ);
            }
            else {
                speed = 0;
            }

            msgs.linear.x = motormsg.linear.x;
            msgs.linear.z = steps;
            msgs.angular.x = 0;
            msgs.angular.z = speed;
            msgs.angular.y = 0;
        }

        else if (modePointer == 3 && int(motormsg.angular.y) == 0) {
            // Mode 3 set current position as "HOME" position
            msgs.linear.x = motormsg.linear.x;
            msgs.linear.z = motormsg.linear.z;
            msgs.angular.x = 0;
            msgs.angular.z = 0;
            msgs.angular.y = -250;

        }

        else if (modePointer == 4 && int(motormsg.angular.y) == 0) {
            // Mode 4 Normal control via waypoints

            int stepsM1_old = int(motormsg.linear.x);
            int stepsM2_old = int(motormsg.linear.z);

            posX = waypointmsg.linear.x;
            posZ = waypointmsg.linear.z;

            double lengthCable1New = calcLength(xOffset + posX, (wsZ + zOffset) - posZ);
            double lengthCable2New = calcLength((wsX + xOffset) - posX, (wsZ + zOffset) - posZ);

            steps1 = calcStepAmount(lengthCable1New - lengthCable1);
            steps2 = calcStepAmount(lengthCable2New - lengthCable2);

            calcSpeed((steps1 - stepsM1_old), speed1, (steps2 - stepsM2_old), speed2);
            msgs.linear.x = steps1;
            msgs.linear.z = steps2;
            msgs.angular.x = speed1;
            msgs.angular.z = speed2;
            msgs.angular.y = 0;

            if(stepsM1_old == steps1 && stepsM2_old == steps2 && singleShot != true) {
                getNewPos.data = true;
                singleShot = true;
                i = 0;
                
            }
            else if((stepsM1_old != steps1 && stepsM2_old != steps2) || singleShot == true){
                getNewPos.data = false;
                
                if(i >= 2 && singleShot == true){
                    singleShot = false;
                    /*
                    calcSpeed((steps1 - stepsM1_old), speed1, (steps2 - stepsM2_old), speed2);
                    msgs.linear.x = steps1;
                    msgs.linear.z = steps2;
                    msgs.angular.x = speed1;
                    msgs.angular.z = speed2;
                    msgs.angular.y = 0;
                    */
                }        
                i += 1;
            }
        }

        else if (modePointer == 5 && int(motormsg.angular.y) == 0) {
            // Mode 5 Normal control via joystick.

            joyX = int(joystickmsg.linear.x);
            joyZ = int(joystickmsg.linear.z);
            int stepsM1_old = int(motormsg.linear.x);
            int stepsM2_old = int(motormsg.linear.z);

            double lenCableM1 = stepsM1_old * mmPuls + lengthCable1;
            double lenCableM2 = stepsM2_old * mmPuls + lengthCable2;

            if(joyX > 0) {
                    
                posX = wsX;
            }
            else if(joyX < 0) {
                posX = 0;
            }
            else if(joyX == 0) {
                posX = motormsg.linear.x;
            }

            if(joyZ > 0) {
                    
                curX = 200000;
            }
            else if(joyZ < 0) {
                curX = -200000;
            }
            else if(joyZ == 0) {
                curX = motormsg.linear.z;
            }

            /*
            if((posX + joyX) >= wsX) {
                    
                posX = wsX;
            }
            else if((posX + joyX) <= 0) {
                posX = 0;
            }
            else {
                posX += joyX / 10;
            }

            if((posZ + joyZ) >= wsZ) {
                    
                posZ = wsZ;
            }
            else if((posZ + joyZ) <= 0) {
                posZ = 0;
            }
            else {
                posZ += joyZ / 10;
            }
            */
            /*
            else if(joyZ == 0) {
                //posZ = zMotordist - lenCableM1 * sqrt( 1 - (pow(lenCableM2, 2.0) - pow(lenCableM1, 2.0) - pow(xMotordist, 2.0)) / (xMotordist * 2 * lenCableM1)) - zOffset;  
            }*/

            /*
            speed1 = 200 * abs(joyX);
            speed2 = 200 * abs(joyZ);
            */

            double lengthCable1New = calcLength(xOffset + posX, (wsZ + zOffset) - posZ);
            double lengthCable2New = calcLength((wsX + xOffset) - posX, (wsZ + zOffset) - posZ);

            steps1 = calcStepAmount(lengthCable1New - lengthCable1);
            steps2 = calcStepAmount(lengthCable2New - lengthCable2);

            
            calcSpeedJoy(joyX, joyZ, speed1, speed2);

            msgs.linear.x = steps1;
            msgs.angular.x = speed1;
            msgs.linear.z = steps2;
            msgs.angular.z = speed2;
            msgs.angular.y = 0;

        }
        else if (modePointer == 6 && int(motormsg.angular.y) == 0) {
            // Mode 6 ERROR handler.

            msgs.linear.x = motormsg.linear.x;
            msgs.angular.x = 0.0;
            msgs.linear.z = motormsg.linear.z;
            msgs.angular.z = 0.0;
            msgs.angular.y = -666.0;

        }
        else if (int(motormsg.angular.y) != 0) {
            msgs.linear.x = motormsg.linear.x;
            msgs.linear.z = motormsg.linear.z;

            msgs.angular.x = 0;
            msgs.angular.z = 0;
            modePointer = 0;
        }

        // Publish it and resolve any remaining callbacks
        motorPub.publish(msgs);
        waypointPub.publish(getNewPos);
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}

double calcLength(int x, int z) {
    return(sqrt(pow(x, 2.0) + pow(z, 2.0)));
}

void calcSpeed(double len1, float& speed1, double len2, float& speed2) {
    float time = 0.0;

    if(abs(len1) < abs(len2)) {
        time = abs(len2 / maxSpeed);
        speed1 = abs(len1 / time);
        speed2 = maxSpeed;
    }
    else if(abs(len1) > abs(len2)){
        time = abs(len1 / maxSpeed);
        speed1 = maxSpeed;
        speed2 = abs(len2 / time);
    }
    else {
        speed1 = maxSpeed;
        speed2 = maxSpeed;
    }
}

void calcSpeedJoy(int joyx, int joyz, float& speed1, float& speed2) {
    float time = 0.0;
    int speedVar = 200;
    /*
    if(abs(joyx) > abs(joyz)) {
        posX = posX + joyx / 10;
        speedVar = (abs(joyx) * 3) + 200;
        time = abs(posX / speedVar);
        speed2 = speedVar;
    }
    
    else if(abs(joyz) > abs(joyx)) {
        
        speedVar = (abs(joyz) * 10) + 200;
        time = abs(len1 / speedVar);
        speed1 = speedVar;
        speed2 = int(abs(len2 / time));
        
    }
    else {
        speedVar = 500;
        speed1 = speedVar;
        speed2 = speedVar;
    }
    */
}

int calcStepAmount(double value) {
    return(int(value / mmPuls));
}