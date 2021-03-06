// this file is the heading following code of the sailboat

#include "ros/ros.h"
#include <math.h>
#include "tf/tf.h"
#include <algorithm>
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64.h"


using namespace std;

double theta, psi_w; // heading of the boat and the angle of the wind
double kr=1.0;
double ks=1.0;
double theta_bar; // the heading the boat should follow
double urmax=M_PI/4;
tf::Quaternion q_sail, q_wind;

double sign(double x)
{
    if (x>0)
        return 1;
    else
        return -1;
}

void stateCallback(const geometry_msgs::Quaternion::ConstPtr& msg) {
    q_sail[0]=msg->x;
    q_sail[1]=msg->y;
    q_sail[2]=msg->z;
    q_sail[3]=msg->w;
}

void windCallback(const geometry_msgs::Quaternion::ConstPtr& msg) {
    q_wind[0]=msg->x;
    q_wind[1]=msg->y;
    q_wind[2]=msg->z;
    q_wind[3]=msg->w;
}


int main(int argc, char **argv)
{   
	ros::init(argc, argv, "node_heading");
    ros::NodeHandle n;
    ros::Subscriber state_sail= n.subscribe("heading_boat", 1000, stateCallback);
    ros::Subscriber wind= n.subscribe("wind_angle", 1000, windCallback);
    ros::Publisher com_servo = n.advertise<geometry_msgs::Vector3>("actuators", 1000);

    n.param<double>("thetabar", theta_bar, 0);
    ros::Rate loop_rate(50);
    double t0 = ros::Time::now().toSec();
    while(ros::ok()){
    	geometry_msgs::Vector3 msg;
        double roll,pitch;
        tf::Matrix3x3(q_sail).getRPY(roll, pitch, theta);
        tf::Matrix3x3(q_wind).getRPY(roll, pitch, psi_w);
        double  tf = ros::Time::now().toSec();

        if (tf-t0<0.5)
        {
            ROS_INFO("Do anything");
        }
        //publication of the command
    	else{

            double ur;//=kr*atan(tan(0.5*(theta-theta_bar)));
            if (cos(theta-theta_bar)>=0)
            {
                ur=urmax*sin(theta-theta_bar);
            }
            else
            {
                ur=urmax*sign(sin(theta-theta_bar));
            }
            ROS_INFO("ur=%f", ur);
        	double us=ks*(M_PI/4)*(cos(psi_w-theta_bar)+1);
        	msg.x=ur;
        	msg.y=us;
            msg.z=0;
        	com_servo.publish(msg);
        }
    	ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
