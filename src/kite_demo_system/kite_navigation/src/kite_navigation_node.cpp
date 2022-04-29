#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

bool messageGot = false;


int posX[9] = {125, 145, 395, 415, 553, 692, 712, 962, 982};
//int posX[17] = {0, 0, 50, 50, 100, 100, 150, 150, 200, 200, 250, 250, 300, 300, 315, 315, 315};

int posZ[16]   = {0, 125, 340, 680, 1020, 1360, 1575, 1700, 1700, 1575, 1360, 1020, 680, 340, 125, 0};
//int posZ[17]   = {0, 900, 900, 0, 0, 900, 900, 0, 0, 900, 900, 0, 0, 900, 900, 0, 900};

int maxSpeed[16]   = {375, 450, 500, 500, 500, 450, 375, 300, 300, 375, 450, 500, 500, 500, 450, 375};

int pX = 0;
int pZ = 0;
int pS = 0;

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
            msgs.linear.y = maxSpeed[pS];
            // Publish it and resolve any remaining callbacks
            pub.publish(msgs);
            //ros::spinOnce(); 

            //pX += 1;
            pZ += 1;
            pS += 1;

            if(pZ == 8) {
                pX += 1;
                if(pX >= 9){
                    pX = 0;
                }
            }
            
            else if(pZ >= 16) {
                pZ = 0;
                pS = 0;
                pX += 1;
                if(pX >= 9){
                    pX = 0;
                }
            }

            messageGot = false;

        }
        else {
        }

        ros::spinOnce();
    }
    return 0;
}