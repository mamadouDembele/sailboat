// this file is the line following code of the sailboat

#include "ros/ros.h"
#include "eigen3/Eigen/Dense"
#include <vector>
#include <math.h>
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "tf/tf.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"


using namespace std;

Eigen::Vector2d m; // Position of the boat
int i=0;

void poseCallback(const geometry_msgs::Point::ConstPtr& msg) {
    m[0]=msg->x;
    m[1]=msg->y;
}


int main(int argc, char **argv)
{   
	ros::init(argc, argv, "node_draw_waypoint");
    ros::NodeHandle n;
    ros::Subscriber pos_sail= n.subscribe("boat_pose", 1000, poseCallback);
    ros::Publisher pose_boat_pub = n.advertise<visualization_msgs::Marker>( "visualization_pos_boat",0 );

    ros::Rate loop_rate(100);
    double t0 = ros::Time::now().toSec();
    while(ros::ok()){
    	visualization_msgs::Marker marker_B;

    	double  tf = ros::Time::now().toSec();

		if (tf-t0<1)
        {
        	//ROS_INFO("mx=%f", m[0]);
	        //ROS_INFO("my=%f", m[1]);
        	ROS_INFO("Do anything");
        }
        
        else{

            ROS_INFO("PRINT");
            marker_B.header.frame_id = "map";
            marker_B.header.stamp = ros::Time::now();
            marker_B.ns = "pos_boat";
            marker_B.id = i;
            marker_B.action = visualization_msgs::Marker::ADD;
            marker_B.type = visualization_msgs::Marker::SPHERE;
            marker_B.pose.position.x =m[0];
            marker_B.pose.position.y = m[1];
            marker_B.pose.position.z=0;
            marker_B.pose.orientation.x=0;
            marker_B.pose.orientation.y=0;
            marker_B.pose.orientation.z=0;
            marker_B.pose.orientation.w=1;
            marker_B.scale.x = 0.5;
            marker_B.scale.y = 0.5;
            marker_B.scale.z = 0.5;
            marker_B.color.a = 1.0; 
            marker_B.color.r = 1.0f;
            marker_B.color.g = 1.0f;
            marker_B.color.b = 1.0f;
            i++;
            tf=0;
            t0=ros::Time::now().toSec();
		    pose_boat_pub.publish(marker_B);
	    }
    	ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}