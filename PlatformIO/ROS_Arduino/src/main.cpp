#define USE_USBCON
#include "ros.h"
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

#include <Arduino.h>
#include <math.h>
#include <AccelStepper.h>

#define enaPinMaxonRB 4            // Output pin Enable     Maxonmotor TOP RIGHT
#define dirPinMaxonRB 5            // Output pin Direction  Maxonmotor TOP RIGHT
#define stepPinMaxonRB 7           // Output pin Step       Maxonmotor TOP RIGHT
#define faultPinMaxonRB 6          // Input pin fault detection Maxonmotor TOP RIGHT

#define enaPinMaxonLB 8           // Output pin Enable     Maxonmotor TOP LEFT
#define dirPinMaxonLB 9           // Output pin Direction  Maxonmotor TOP LEFT
#define stepPinMaxonLB 11          // Output pin Step       Maxonmotor TOP LEFT
#define faultPinMaxonLB 10         // Input pin fault detection Maxonmotor TOP LEFT

#define dirInvMaxonRB true         // Inverting of direction-pin Maxonmotor TOP RIGHT
#define stepInvMaxonRB false        // Inverting of step-pin      Maxonmotor TOP RIGHT
#define enaInvMaxonRB false         // Inverting of enable-pin    Maxonmotor TOP RIGHT

#define dirInvMaxonLB false         // Inverting of direction-pin Maxonmotor TOP LEFT
#define stepInvMaxonLB false        // Inverting of step-pin      Maxonmotor TOP LEFT
#define enaInvMaxonLB false         // Inverting of enable-pin    Maxonmotor TOP LEFT

#define MaxAcceleration 3000000     // Maximum rotational acceleration of Maxonmotors (steps per second^2)
#define MaxSpeed 20000              // Maximum rotational speed of Maxonmotors        (steps per second)

ros::NodeHandle nh;                 // Initializes ROS Nodehandler
geometry_msgs::Twist pos_msg;
long int stepsRB = 0;
long int stepsLB = 0;
long int speedRB = 0;
long int speedLB = 0;
int cmdValue = 0;

int speedRB_ = 0;
int speedLB_ = 0;

long int curStepsRB = 0;       // Initializes first steps TOP RIGHT Maxonmotor on position [0, 0] (LEFT BOTTOM)
long int curStepsLB = 0;       // Initializes first steps TOP LEFT Maxonmotor on position [0, 0] (LEFT BOTTOM)
double nextSpeedRB = 0.0;      // Initializes first speed value of TOP RIGHT Maxonmotor
double nextSpeedLB = 0.0;      // Initializes first speed value of TOP LEFT Maxonmotor

void subMotorcontrol( const geometry_msgs::Twist& cmd_msg){
  stepsRB = cmd_msg.linear.z;                // Gets steps for TOP RIGHT Maxonmotor
  stepsLB = cmd_msg.linear.x;                // Gets steps for TOP LEFT Maxonmotor
  speedRB = int(cmd_msg.angular.z);               // Gets speed for TOP RIGHT Maxonmotor
  speedLB = int(cmd_msg.angular.x);               // Gets speed for TOP LEFT Maxonmotor
  cmdValue = cmd_msg.angular.y;
}

ros::Subscriber<geometry_msgs::Twist> subtest("motor_control", &subMotorcontrol);      // Initializes a ROS subscriber
ros::Publisher Position("motor_Position", &pos_msg);                  // Initializes a ROS Publisher

AccelStepper MaxonRB(AccelStepper::DRIVER, stepPinMaxonRB, dirPinMaxonRB);    // Maxonmotor TOP RIGHT pulleysystem
AccelStepper MaxonLB(AccelStepper::DRIVER, stepPinMaxonLB, dirPinMaxonLB);    // Maxonmotor TOP LEFT pulleysystem

void setup() {
  pinMode(faultPinMaxonRB, INPUT);
  pinMode(faultPinMaxonLB, INPUT);

  MaxonRB.setEnablePin(enaPinMaxonRB);
  MaxonLB.setEnablePin(enaPinMaxonLB);
  MaxonRB.setPinsInverted(dirInvMaxonRB, stepInvMaxonRB, enaInvMaxonRB);      // Correct pin direction TOP RIGHT Maxonmotor
  MaxonLB.setPinsInverted(dirInvMaxonLB, stepInvMaxonLB, enaInvMaxonLB);      // Correct pin direction TOP LEFT Maxonmotor
  MaxonRB.setAcceleration(MaxAcceleration);                                   // Sets maximum acceleration of TOP RIGHT Maxonmotor
  MaxonLB.setAcceleration(MaxAcceleration);                                   // Sets maximum acceleration of TOP LEFT Maxonmotor
  MaxonRB.enableOutputs();                                                    // Sets enable outputs HIGH of TOP RIGHT Maxonmotor
  MaxonRB.enableOutputs();                                                    // Sets enable outputs HIGH of TOP LEFT Maxonmotor

  MaxonRB.setMaxSpeed(MaxSpeed);  // Sets initial maximum speed of TOP RIGHT Maxonmotor
  MaxonLB.setMaxSpeed(MaxSpeed);  // Sets initial maximum speed of TOP LEFT Maxonmotor

  nh.initNode();                  // Initalizes the Nodehandler
  nh.subscribe(subtest);          // Subscribes to a publisher node
  nh.advertise(Position);         // Publishes a publisher node
}


void loop() { 
  int i = 0;
  pos_msg.linear.z = curStepsRB;             // Fill standard message with data
  pos_msg.linear.x = curStepsLB;             // Fill standard message with data
  pos_msg.angular.z = 0;             // Fill standard message with data
  pos_msg.angular.x = 0;
  Position.publish( &pos_msg );             // Publisher of ROS node message
  nh.spinOnce();                            // Excecute ROS once 
  bool homeCompleted = false;
  bool singleShot = false;

  while(homeCompleted != true) {
    if(cmdValue == -250) {
      MaxonRB.setCurrentPosition(stepsRB);
      MaxonLB.setCurrentPosition(stepsLB);
      pos_msg.linear.z = 0;             // Fill standard message with data
      pos_msg.linear.x = 0;
      Position.publish( &pos_msg );             // Publisher of ROS node message
      nh.spinOnce();
      homeCompleted = true;
      singleShot = false;
      break;
    }
    else if(cmdValue == -666 && singleShot == false){
      MaxonRB.setCurrentPosition(MaxonRB.currentPosition());
      MaxonLB.setCurrentPosition(MaxonLB.currentPosition());

      MaxonRB.disableOutputs();
      MaxonLB.disableOutputs();
      delay(100);
      MaxonRB.enableOutputs();                                                    // Sets enable outputs HIGH of TOP RIGHT Maxonmotor
      MaxonLB.enableOutputs(); 
      pos_msg.angular.y = 0;                                                   // Sets enable outputs HIGH of TOP LEFT Maxonmotor
      Position.publish( &pos_msg );             // Publisher of ROS node message
      singleShot = true;
    }
    else{
      MaxonRB.moveTo(stepsRB);
      MaxonLB.moveTo(stepsLB);
      MaxonRB.setSpeed(speedRB);
      MaxonLB.setSpeed(speedLB); 

      MaxonRB.runSpeedToPosition();
      MaxonLB.runSpeedToPosition();
    }
    nh.spinOnce(); 
  }
  

  while(homeCompleted == true) {
    
    while(curStepsRB != stepsRB || curStepsLB != stepsLB) {
      MaxonRB.moveTo(stepsRB);
      MaxonLB.moveTo(stepsLB);
      MaxonRB.setSpeed(speedRB);
      MaxonLB.setSpeed(speedLB);     

      if(digitalRead(faultPinMaxonRB) == true && digitalRead(faultPinMaxonLB) == true) {
        MaxonRB.runSpeedToPosition();
        MaxonLB.runSpeedToPosition();
        //MaxonRB.runSpeed();
        //MaxonLB.runSpeed();

        curStepsRB = MaxonRB.currentPosition();
        curStepsLB = MaxonLB.currentPosition();

        i += 1;  
        
        if(i >= 5000) {
          pos_msg.linear.z = MaxonRB.currentPosition();             // Fill standard message with data
          pos_msg.linear.x = MaxonLB.currentPosition();             // Fill standard message with data
          pos_msg.angular.z = speedRB;             // Fill standard message with data
          pos_msg.angular.x = speedLB;
          pos_msg.angular.y = 0;
          Position.publish( &pos_msg );             // Publisher of ROS node message
          nh.spinOnce();
          i = 0;
        }
        else{
        }
        
        
      }
      else if(cmdValue == -666 && singleShot == false){
        MaxonRB.setCurrentPosition(MaxonRB.currentPosition());
        MaxonLB.setCurrentPosition(MaxonLB.currentPosition());

        MaxonRB.disableOutputs();
        MaxonLB.disableOutputs();
        delay(100);
        MaxonRB.enableOutputs();                                                    // Sets enable outputs HIGH of TOP RIGHT Maxonmotor
        MaxonLB.enableOutputs();                                                    // Sets enable outputs HIGH of TOP LEFT Maxonmotor
        homeCompleted = false;
        singleShot = true;
        pos_msg.angular.y = 0;
        break;
      }
      else {
        if(digitalRead(faultPinMaxonRB) != true && digitalRead(faultPinMaxonLB) == true) {
          pos_msg.angular.y = -5555.0;
          homeCompleted = false;
          break;
        }
        else if(digitalRead(faultPinMaxonRB) == true && digitalRead(faultPinMaxonLB) != true) {
          pos_msg.angular.y = -7777.0;
          homeCompleted = false;
          break;
        }
        else {
          pos_msg.angular.y = -9999.0;
          homeCompleted = false;
          break;
        }
        Position.publish( &pos_msg );             // Publisher of ROS node message
        nh.spinOnce();
      }    
    }
    
    pos_msg.linear.z = MaxonRB.currentPosition();             // Fill standard message with data
    pos_msg.linear.x = MaxonLB.currentPosition();             // Fill standard message with data
    pos_msg.angular.z = 0;             // Fill standard message with data
    pos_msg.angular.x = 0;
    Position.publish( &pos_msg );             // Publisher of ROS node message
    delay(1);
    nh.spinOnce();
  }
}
