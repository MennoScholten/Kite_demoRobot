#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

bool messageGot = false;


int posX[18] = {125, 125, 145, 145, 395, 395, 415, 415, 665, 665, 775, 775, 885, 885, 905, 905, 975, 975};
//int posX[17] = {0, 0, 50, 50, 100, 100, 150, 150, 200, 200, 250, 250, 300, 300, 315, 315, 315};

int posZ[18]   = {0, 1700, 1700, 0, 0, 1700, 1700, 0, 0, 1700, 1700, 0, 0, 1700, 1700, 0, 0, 1700};
//int posZ[17]   = {0, 900, 900, 0, 0, 900, 900, 0, 0, 900, 900, 0, 0, 900, 900, 0, 900};

int pX = 0;
int pZ = 0;

void subCallback(const std_msgs::Bool::ConstPtr& msg) {
    messageGot = msg->data;   
}


int main(int argc, char** argv)
{
    // Init ROS node
    ros::init(argc, argv, "kite_navigation_node");
    ros::NodeHandle nh;

   
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("navigation_position", 100);
    ros::Subscriber sub = nh.subscribe("nextWaypoint", 1000, subCallback);
    // Create Twist message
    geometry_msgs::Twist msgs;

    
    while(ros::ok()){
        
        if(messageGot == true) {

            msgs.linear.x = posX[pX];
            msgs.linear.z = posZ[pZ];
            // Publish it and resolve any remaining callbacks
            pub.publish(msgs);
            //ros::spinOnce(); 

            pX += 1;
            pZ += 1;

            if(pX >= 18) {
                pX = 0;
            }
            if(pZ >= 18) {
                pZ = 0;
            }

            messageGot = false;

        }
        else {
        }

        ros::spinOnce();
    }
    return 0;
}