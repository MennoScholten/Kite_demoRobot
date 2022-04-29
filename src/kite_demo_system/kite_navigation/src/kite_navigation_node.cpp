#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

bool messageGot = false;


int posX[9] = {125, 145, 395, 415, 553, 692, 712, 962, 982};                                                // X-axis coordinate positions

int posZ[16]   = {0, 125, 380, 760, 1140, 1520, 1775, 1900, 1900, 1775, 1520, 1140, 760, 380, 125, 0};      // Z-axis coordinate positions

int maxSpeed[16]   = {375, 450, 500, 500, 500, 450, 375, 300, 300, 375, 450, 500, 500, 500, 450, 375};      // Maximum speed at specifiek Z-axis locations

int pX = 0;         // Position counter X-axis
int pZ = 0;         // Position counter Z-axis
int pS = 0;         // Position counter maximum speed

void subCallback(const std_msgs::Bool::ConstPtr& msg) {
    messageGot = msg->data;   // subcibed topic: message collected
}


int main(int argc, char** argv)
{
    // Init ROS node
    ros::init(argc, argv, "kite_navigation_node");      // initialize ROS
    ros::NodeHandle nh;                                 // Create a nodehandler

   
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("navigation_position", 100);        // Advertise (publisher) a rostopic: navigation_position
    ros::Subscriber sub = nh.subscribe("nextWaypoint", 1000, subCallback);                      // Subscribe to a rostopic: nextWaypoint
    
    geometry_msgs::Twist msgs;  // Create Twist message        

    
    while(ros::ok()){
        
        if(messageGot == true) {

            msgs.linear.x = posX[pX];               // fill message with position data X-axis
            msgs.linear.z = posZ[pZ];               // fill message with position data Z-axis
            msgs.linear.y = maxSpeed[pS];           // fill message with speed data 

            
            pub.publish(msgs);      // Publish message a

            pZ += 1;    // increment position counter Z-axis
            pS += 1;    // increment speed counter

            if(pZ == 8) {     
                pX += 1;        // increment position counter X-axis at correct time
                if(pX >= 9){
                    pX = 0;     // resets position counter X-axis
                }
            }
            
            else if(pZ >= 16) {
                pZ = 0;         // resets position counter Z-axis
                pS = 0;         // resets speed counter
                pX += 1;        // increment position counter X-axis at correct time
                if(pX >= 9){
                    pX = 0;     // resets position counter X-axis
                }
            }

            messageGot = false; // sets message variable back to false

        }
        else {
        }

        ros::spinOnce();        // resolve any remaining callbacks
    }
    return 0;
}