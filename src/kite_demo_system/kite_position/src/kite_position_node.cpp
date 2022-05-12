// ROSNODE 5 Position controller
// KITE ROBOTICS EXHIBITION STAND: DEMONSTRATION SYSTEM
// Created on: 29-03-2022
// Version 1.1

// Program designed by: Menno Scholten

#include <ros/ros.h>
#include <ros/console.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>

#define wsX 1107                        // Workspace distance X-axis (mm)
#define wsZ 1900                        // Workspace distance Z-axis (mm)
#define xOffset 150                     // Offset from workspace to cablepoint X-axis (mm)
#define zOffset 100                     // Offset from workspace to cablepoint Z-axis (mm)
#define xMotordist wsX + 2 * xOffset    // Distance between cablepoints X-axis (mm)
#define zMotordist wsZ + 2 * zOffset    // Distacne between cablepoints Z-axis (mm)

//#define mmPuls 0.182
#define mmPuls 0.17583

#define maxSpeed 500                  // Maximum speed of the motors in pulses/second


unsigned int modePointer = 0;       // Global variable for system modes
geometry_msgs::Twist waypointmsg;       // Global variable for receiving new waypoint messages
geometry_msgs::Twist joystickmsg;       // Global variable for receiving joystick messages
geometry_msgs::Twist motormsg;          // Global variable for receiving motor position 
 

void subWaypointCallback(const geometry_msgs::Twist::ConstPtr& msg) {   // callback handler for getting waypoint messages 
    waypointmsg.linear.x = msg->linear.x;
    waypointmsg.linear.z = msg->linear.z;  
    waypointmsg.linear.y = msg->linear.y; 
}

void subJoystickCallback(const geometry_msgs::Twist::ConstPtr& msg) {   // callback handler for getting joystick messages
    joystickmsg.linear.x = msg->linear.x;
    joystickmsg.linear.z = msg->linear.z;

}

void subModeCallback(const std_msgs::UInt16::ConstPtr& msg) {           // callback handler for getting system mode messages
    modePointer = msg->data;            
}

void subMotorPositionCallback(const geometry_msgs::Twist::ConstPtr& msg) {  // callback handler for getting motor position messages
    motormsg.linear.x = msg->linear.x;
    motormsg.linear.z = msg->linear.z;
    motormsg.angular.y = msg->angular.y;

}

double calcLength(int x, int z);            // Calculate pythagoras theorom length of hypotenuse (a^2 + b^ 2 = C^2)

void calcSpeed(double len1, float &speed1, double len2, float &speed2, int Maxspeed);   // Calculate speed of both moters via linear interpolation

void calcSpeedJoy(int joyx, int joyz, float& speed1, float& speed2);            // Calculate speed of both motores via linear interpolation using joystick input as speed

int calcStepAmount(double value);   // Calculate amount of servomotor steps from distance


int main(int argc, char** argv)
{   
    bool singleShot = false;        // Variable used to restrict the amount of requests
    int joyX = 0;                   // Variable for joystick X-axis input
    int joyZ = 0;                   // Variable for joystick Z-axis input
    int posX = 0;                   // variable for desired position X-axis
    int posZ = 0;                   // Variable for desired position Z-axis
    int curX = 0;                   // Variable for current position X-axis
    int curZ = 0;                   // Variable for current position Z-axis
    double steps1 = 0.0;            // Variable for amount of steps motor 1
    double steps2 = 0.0;            // Variable for amount of steps motor 2
    float speed1 = 0.0;             // Variable for speed (steps/sec) of motor 1
    float speed2 = 0.0;             // Variable for speed (steps/sec) of motor 2
    int speed = 0;                  // Variable for setting maximum speed
    int i = 0;                      // Integer counter for restricting amount of requests

    double xMotordist_sqre = xMotordist * xMotordist;   // Variable used for kinematics

    const double lengthCable1 = calcLength(xOffset, 2000);         // Length calbe 1 (motor system UP LEFT) at rest [0, 0]
    const double lengthCable2 = calcLength(wsX + xOffset, 2000);   // Length cable 2 (motor system UP RIGHT) at rest [0, 0]
    
    ros::init(argc, argv, "kite_position_node"); // Initializing the ROS node
    ros::NodeHandle nh;     // Setup the ROSNODE handler
    ros::Rate r(30);        // 30 Hz spinrate
  
    // Setup different ROS subsribers and publishers 
    ros::Publisher motorPub = nh.advertise<geometry_msgs::Twist>("motor_control", 100);
    ros::Publisher waypointPub = nh.advertise<std_msgs::Bool>("nextWaypoint", 100);
    ros::Publisher curposPub = nh.advertise<geometry_msgs::Twist>("cur_position", 100);

    ros::Subscriber subWaypoint = nh.subscribe("navigation_position", 1000, subWaypointCallback);
    ros::Subscriber subJoystick = nh.subscribe("joystick_teleop", 1000, subJoystickCallback);
    ros::Subscriber subMode     = nh.subscribe("system_Mode", 1000, subModeCallback);
    ros::Subscriber subMotorpos = nh.subscribe("motor_Position", 1000, subMotorPositionCallback);

    geometry_msgs::Twist msgs;              // Variable for publishing desired motor position
    geometry_msgs::Twist curPositionPub;    // Variable for publishing current motor position messages  
    std_msgs::Bool getNewPos;               // Variable for publishing new navigation position messages

    while(ros::ok()) {
        
        if(modePointer == 0) {
            // Mode 0 Start pointer NO Function

        }

        else if(modePointer == 1 && int(motormsg.angular.y) == 0) {
            // Mode 1 Single motorcontrol via joystick (motor 1)
            joyX = int(joystickmsg.linear.z); // get current joystick position

            if(joyX > 0) {                  // sets maximum positive position (mm)
                    
                curX = 200000;
            }
            else if(joyX < 0) {             // sets maximum negative position (mm)
                curX = -200000;
            }
            else if(joyX == 0) {
                curX = motormsg.linear.z;   // sets current position in steps
            }
            
            int steps = calcStepAmount(curX);   // calculates desired steps
            speed = 0;                          // sets maximum speed variable to 0 

            if(joyX > 0) {
                speed = 20 * joyX;          // sets maximum speed acording to amount of joystick movement
            }

            else if(joyX < 0) {
                speed = -(20 * joyX);       // sets maximum speed acording to amount of joystick movement
            }
            else {
                speed = 0;                  // sets maximum speed acording to amount of joystick movement
            }

            // Fill motorcontrol message with calculated variables
            msgs.linear.x = steps;
            msgs.linear.z = motormsg.linear.z;
            msgs.angular.x = speed;
            msgs.angular.z = 0;
            msgs.angular.y = 0;
            curPositionPub.angular.y = 0;
        }
        
        else if (modePointer == 2 && int(motormsg.angular.y) == 0) {
            // Mode 2 Single motorcontrol via joystick (motor 2)
            joyZ = int(joystickmsg.linear.z);   // get current joystick position

            if(joyZ > 0) {              // sets maximum positive position (mm)
                    
                curX = 200000;
            }
            else if(joyZ < 0) {         // sets maximum negative position (mm)
                curX = -200000;
            }
            else if(joyZ == 0) {        // sets current position in steps
                curX = motormsg.linear.z;
            }
            
            int steps = calcStepAmount(curX);   // calculates desired steps
            speed = 0;                          // sets maximum speed variable to 0          

            if(joyZ > 0) {                  
                speed = 20 * joyZ;          // sets maximum speed acording to amount of joystick movement
            }

            else if(joyZ < 0) {
                speed = -(20 * joyZ);       // sets maximum speed acording to amount of joystick movement
            }
            else {
                speed = 0;                  // sets maximum speed acording to amount of joystick movement
            }

            // Fill motorcontrol message with calculated variables
            msgs.linear.x = motormsg.linear.x;
            msgs.linear.z = steps;
            msgs.angular.x = 0;
            msgs.angular.z = speed;
            msgs.angular.y = 0;
            curPositionPub.angular.y = 0;
        }

        else if (modePointer == 3 && int(motormsg.angular.y) == 0) {
            // Mode 3 set current position as "HOME" position

            // Set all variables to 0 (no movement at position [0,0] (x,z))
            joyX = 0;
            joyZ = 0;
            posX = 0;
            posZ = 0;
            curX = 0;
            curZ = 0;
            msgs.linear.x = 0;
            msgs.linear.z = 0;
            msgs.angular.x = 0;
            msgs.angular.z = 0;
            msgs.angular.y = -250; // Home variable picked-up by arduino 
            curPositionPub.angular.y = 0;

        }

        else if (modePointer == 4 && int(motormsg.angular.y) == 0) {
            // Mode 4 Normal control via waypoints

            int stepsM1_old = int(motormsg.linear.x);   // Get last known steps of motor 1 (LEFT UP)
            int stepsM2_old = int(motormsg.linear.z);   // get last known steps of motor 2 (RIGHT UP)

            double l1 = stepsM1_old * mmPuls + 2005.6171220131579;  // Calculate length of cable 1 (LEFT UP) via last known steps
            double l2 = stepsM2_old * mmPuls + 2362.2127338578123;  // Calculate length of cable 1 (RIGHT UP) via last known steps

            double l1_sqre = l1 * l1;   // Variable containing squared length of cable 1 
            double l2_sqre = l2 * l2;   // Variable containing squared length of cable 2


            // FORWARD KINEMATICS (calculate position via cable length). 
            // For more information: chapter 2.4.5 -> 220316_TDD_Afstudeerstage_MennoScholten_463048_V1.docx 
            double delX = ((l1_sqre - l2_sqre + 1979649) / (2814)) - xOffset;

            double z_cosIn = ((l1_sqre - l2_sqre + 1979649) / (l1 * 2814));
            double delZ = 2000 - l1*sin(acos(z_cosIn));

            
            posX = waypointmsg.linear.x;        // Received waypoint position X-axis
            posZ = waypointmsg.linear.z;        // Received waypoint position Z-axis
            speed = waypointmsg.linear.y;       // Received speed at waypoint position
            curPositionPub.linear.x = delX;     // Forward kinematics publish X-axis
            curPositionPub.linear.z = delZ;     // Forward kinematics publish Z-axis
            
            double lengthCable1New = calcLength((xOffset + posX), (wsZ + zOffset) - posZ);              // Calculate length of cable 1 with desired position
            double lengthCable2New = calcLength(((wsX + xOffset) - posX), ((wsZ + zOffset) - posZ));    // Calculate length of cable 2 with desired position

            steps1 = calcStepAmount(lengthCable1New - lengthCable1);    // Calculate steps for cable 1
            steps2 = calcStepAmount(lengthCable2New - lengthCable2);    // Calculate steps for cable 2

            double absSteps1 = steps1 - stepsM1_old;   
            double absSteps2 = steps2 - stepsM2_old;

            calcSpeed(absSteps1, speed1, absSteps2, speed2, speed);     // Calculate speed of motors via linear interpolation

            //calcSpeed(steps1, speed1, steps2, speed2, speed);   

            // Fill motorcontrol message with calculated variables
            msgs.linear.x = steps1;
            msgs.linear.z = steps2;
            msgs.angular.x = speed1;
            msgs.angular.z = speed2;
            msgs.angular.y = 0;
            curPositionPub.angular.y = 0;


            // Check if near desired position: if yes then ask for new position
            if((abs(absSteps1) <= 100 && abs(absSteps2) <= 100) && singleShot != true) {
                getNewPos.data = true;
                waypointPub.publish(getNewPos);
                i = 0;
                singleShot = true; // limits amount of requests
            }
            // sets 'ask for new position' variable to 'NO'. repeating this two times.
            else if((stepsM1_old != steps1 || stepsM2_old != steps2) && singleShot == true){
                i += 1;
                if(i <= 3) {
                    getNewPos.data = false;
                    waypointPub.publish(getNewPos);
                }  

                // set limited request value back to false
                else if(i >= 4 && singleShot == true){
                    singleShot = false;
                    i = 0;
                }       
                
            }
            
        }

        else if (modePointer == 5 && int(motormsg.angular.y) == 0) {
            // Mode 5 Normal control via joystick.

            joyX = int(joystickmsg.linear.x);           // get joystick X-axis value 
            joyZ = int(joystickmsg.linear.z);           // get joystick Z-axis value
            int stepsM1_old = int(motormsg.linear.x);   // get previous known steps motor 1 (LEFT UP)
            int stepsM2_old = int(motormsg.linear.z);   // get previous known steps motor 2 (RIGHT UP)

            double l1 = stepsM1_old * mmPuls + 2005.6171220131579;  // Calculate length of cable 1 (LEFT UP) via last known steps
            double l2 = stepsM2_old * mmPuls + 2362.2127338578123;  // Calculate length of cable 1 (RIGHT UP) via last known steps

            double l1_sqre = l1 * l1;   // Variable containing squared length of cable 1 
            double l2_sqre = l2 * l2;   // Variable containing squared length of cable 2


            // FORWARD KINEMATICS (calculate position via cable length). 
            // For more information: chapter 2.4.5 -> 220316_TDD_Afstudeerstage_MennoScholten_463048_V1.docx 
            double delX = ((l1_sqre - l2_sqre + 1979649) / (2814)) - xOffset;

            double z_cosIn = ((l1_sqre - l2_sqre + 1979649) / (l1 * 2814));
            double delZ = 2000 - l1*sin(acos(z_cosIn));
  
            curPositionPub.linear.x = delX;     // Forward kinematics publish X-axis
            curPositionPub.linear.z = delZ;     // Forward kinematics publish Z-axis


            // Maximum allowed position variables
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


            // Calculate length of new cable positions
            double lengthCable1New = calcLength(xOffset + posX, (wsZ + zOffset) - posZ);
            double lengthCable2New = calcLength((wsX + xOffset) - posX, (wsZ + zOffset) - posZ);

            // Calculate amount of steps with new cable positions
            steps1 = calcStepAmount(lengthCable1New - lengthCable1);
            steps2 = calcStepAmount(lengthCable2New - lengthCable2);

            // calculateds speed with joystick inputs.
            calcSpeed(steps1 -stepsM1_old, speed1, steps2 -stepsM2_old, speed2, maxSpeed);

            // Fill motorcontrol message with calculated variables
            msgs.linear.x = steps1;
            msgs.angular.x = speed1;
            msgs.linear.z = steps2;
            msgs.angular.z = speed2;
            msgs.angular.y = 0;
            curPositionPub.angular.y = 0;

        }
        else if (modePointer == 6) {
            // Mode 6 ERROR handler.

            // Fill motorcontrol message with specific variables
            msgs.linear.x = motormsg.linear.x;
            msgs.angular.x = 0.0;
            msgs.linear.z = motormsg.linear.z;
            msgs.angular.z = 0.0;
            msgs.angular.y = -666.0;        // Negative 666 (motor reset known in program Arduino)
            curPositionPub.angular.y = 0;

        }
        else if (modePointer == 7 && int(motormsg.angular.y) == 0) {
            // Mode 7: Go To Home position handler.

            posX = 0;
            posZ = 0;
            speed = 500;

            joyX = int(joystickmsg.linear.x);           // get joystick X-axis value 
            joyZ = int(joystickmsg.linear.z);           // get joystick Z-axis value
            int stepsM1_old = int(motormsg.linear.x);   // get previous known steps motor 1 (LEFT UP)
            int stepsM2_old = int(motormsg.linear.z);   // get previous known steps motor 2 (RIGHT UP)

            double l1 = stepsM1_old * mmPuls + 2005.6171220131579;  // Calculate length of cable 1 (LEFT UP) via last known steps
            double l2 = stepsM2_old * mmPuls + 2362.2127338578123;  // Calculate length of cable 1 (RIGHT UP) via last known steps

            double l1_sqre = l1 * l1;   // Variable containing squared length of cable 1 
            double l2_sqre = l2 * l2;   // Variable containing squared length of cable 2


            // FORWARD KINEMATICS (calculate position via cable length). 
            // For more information: chapter 2.4.5 -> 220316_TDD_Afstudeerstage_MennoScholten_463048_V1.docx 
            double delX = ((l1_sqre - l2_sqre + 1979649) / (2814)) - xOffset;

            double z_cosIn = ((l1_sqre - l2_sqre + 1979649) / (l1 * 2814));
            double delZ = 2000 - l1*sin(acos(z_cosIn));
            
            curPositionPub.linear.x = delX;     // Forward kinematics publish X-axis
            curPositionPub.linear.z = delZ;     // Forward kinematics publish Z-axis

            
            double lengthCable1New = calcLength((xOffset + posX), (wsZ + zOffset) - posZ);
            double lengthCable2New = calcLength(((wsX + xOffset) - posX), ((wsZ + zOffset) - posZ));

            steps1 = calcStepAmount(lengthCable1New - lengthCable1);
            steps2 = calcStepAmount(lengthCable2New - lengthCable2);

            calcSpeed(steps1, speed1, steps2, speed2, speed);
            msgs.linear.x = 0;
            msgs.linear.z = 0;
            msgs.angular.x = speed1;
            msgs.angular.z = speed2;
            msgs.angular.y = 0;
            curPositionPub.angular.y = 0;

        }
        else if (int(motormsg.angular.y) != 0) {
            // Motor in fault status 

            // sets all variables to known values and no movement.
            msgs.linear.x = motormsg.linear.x;
            msgs.linear.z = motormsg.linear.z;
            if(int(motormsg.angular.y) == -5555) {
                curPositionPub.angular.y = -5555.0;
            }
            else if(int(motormsg.angular.y) == -7777){
                curPositionPub.angular.y = -7777.0;
            }
            else if(int(motormsg.angular.y) == -9999) {
                curPositionPub.angular.y = -9999.0;
            }
            else{
                curPositionPub.angular.y = 0;
            }

            joyX = 0;
            joyZ = 0;
            posX = 0;
            posZ = 0;
            curX = 0;
            curZ = 0;

            msgs.angular.x = 0;
            msgs.angular.z = 0;
            modePointer = 0;
        }

        // Publish messages and resolve any remaining callbacks
        motorPub.publish(msgs);
        curposPub.publish(curPositionPub);
        ros::spinOnce();
        r.sleep(); // limits the program to 30 Hz
    }
    return 0;
}

double calcLength(int x, int z) {
    return(sqrt(pow(x, 2.0) + pow(z, 2.0))); // pythagoras theorem
}

void calcSpeed(double len1, float& speed1, double len2, float& speed2, int Maxspeed) {
    float time = 0.0;

    // Linear interpolation
    if(abs(len1) < abs(len2)) {
        time = abs(len2 / Maxspeed);
        speed1 = abs(len1 / time);
        speed2 = Maxspeed;
    }
    else if(abs(len1) > abs(len2)){
        time = abs(len1 / Maxspeed);
        speed1 = Maxspeed;
        speed2 = abs(len2 / time);
    }
    else {
        speed1 = Maxspeed;
        speed2 = Maxspeed;
    }
}

void calcSpeedJoy(int joyx, int joyz, float& speed1, float& speed2) {
    float time = 0.0;
    int speedVar = 200;

}

int calcStepAmount(double value) {
    return(int(value / mmPuls));
}