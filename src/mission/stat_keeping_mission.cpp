//Code for the challenge 2

#include "ros/ros.h"
#include "eigen3/Eigen/Dense"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"


Eigen::Vector2d SK1, SK2;
double r, t, ts;

void timeCallback(const std_msgs::Float64::ConstPtr& msg)
{
	t=msg->data;
}

int main(int argc, char **argv)
{   
	ros::init(argc, argv, "node_mission2");
    ros::NodeHandle n;
    ros::Subscriber pos_sail= n.subscribe("time_stay", 1000, timeCallback);
    ros::Publisher radius_pub = n.advertise<std_msgs::Float64>( "radius",1000 );
    ros::Publisher q_pub = n.advertise<std_msgs::Bool>( "q",1000 );
    ros::Publisher SK_pub = n.advertise<geometry_msgs::Point>( "sk_point",1000 );

    n.param<double>("SKx1", SK1[0], 0);
    n.param<double>("SKy1", SK1[1], 0);
    n.param<double>("SKx2", SK2[0], 0);
    n.param<double>("SKy2", SK2[1], 0);
    n.param<double>("radius_r", r, 0);
    n.param<double>("time_s", ts, 0);
    ros::Rate loop_rate(100);
    double t0 = ros::Time::now().toSec();
    while(ros::ok()){

    	geometry_msgs::Point msg;
    	std_msgs::Float64 rad;
        std_msgs::Bool  msg_q;
    	double  tf = ros::Time::now().toSec();

		if (tf-t0<0.1)
        {
        	ROS_INFO("Do anything");
        }
        
        else{

            ROS_INFO("r=%f",r);
        	if (t<ts)
        	{

        		msg.x=SK1[0];
        		msg.y=SK1[1];
        		msg.z=0;
                msg_q.data=false;
        	}

        	else{
                ROS_INFO("change the station keeping point");
                msg_q.data=true;
        		msg.x=SK2[0];
        		msg.y=SK2[1];
        		msg.z=0;
        	}
        	
        	rad.data=r;
        	SK_pub.publish(msg);
            q_pub.publish(msg_q);
        	radius_pub.publish(rad);

    		}
            ros::spinOnce();
            loop_rate.sleep();
		}
	return 0;
}