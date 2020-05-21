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


double xa,ya,xb,yb,xc,yc,xd,yd;

int main(int argc, char **argv)
{   
	ros::init(argc, argv, "node_hideSeek");
    ros::NodeHandle n;
    ros::Publisher line_pub = n.advertise<visualization_msgs::Marker>( "visualization_line",0 );
    ros::Publisher line2_pub = n.advertise<visualization_msgs::Marker>( "visualization_line2",0 );
    n.param<double>("posxa", xa, 0);
    n.param<double>("posya", ya, 0);
    n.param<double>("posxb", xb, 0);
    n.param<double>("posyb", yb, 0);
    n.param<double>("posxc", xc, 0);
    n.param<double>("posyc", yc, 0);
    n.param<double>("posxd", xd, 0);
    n.param<double>("posyd", yd, 0);
    ros::Rate loop_rate(10);
    while(ros::ok()){
    	visualization_msgs::Marker marker_line, marker_line2;

        //line        
        marker_line.header.frame_id = "map";
        marker_line.header.stamp = ros::Time::now();
        marker_line.ns = "line1";
        marker_line.id = 0;
        marker_line.action = visualization_msgs::Marker::ADD;
        marker_line.type = visualization_msgs::Marker::LINE_STRIP;
        marker_line.scale.x = 0.5;
        geometry_msgs::Point cA;
        geometry_msgs::Point cB;
        cA.x=xa;
        cA.y=ya;
        cA.z=0;
        cB.x=xb;
        cB.y=yb;
        cB.z=0;
        marker_line.points.push_back(cA);
        marker_line.points.push_back(cB);
        marker_line.pose.orientation.x=0;
        marker_line.pose.orientation.y=0;
        marker_line.pose.orientation.z=0;
        marker_line.pose.orientation.w=1;
        marker_line.color.a=0.5;
        marker_line.color.r = 0.0f;
        marker_line.color.g = 0.0f;
        marker_line.color.b = 1.0f;

        //line        
        marker_line2.header.frame_id = "map";
        marker_line2.header.stamp = ros::Time::now();
        marker_line2.ns = "line2";
        marker_line2.id = 0;
        marker_line2.action = visualization_msgs::Marker::ADD;
        marker_line2.type = visualization_msgs::Marker::LINE_STRIP;
        marker_line2.scale.x = 0.5;
        geometry_msgs::Point cC;
        geometry_msgs::Point cD;
        cC.x=xc;
        cC.y=yc;
        cC.z=0;
        cD.x=xd;
        cD.y=yd;
        cD.z=0;
        marker_line2.points.push_back(cC);
        marker_line2.points.push_back(cD);
        marker_line2.pose.orientation.x=0;
        marker_line2.pose.orientation.y=0;
        marker_line2.pose.orientation.z=0;
        marker_line2.pose.orientation.w=1;
        marker_line2.color.a=0.5;
        marker_line2.color.r = 0.0f;
        marker_line2.color.g = 0.0f;
        marker_line2.color.b = 1.0f;

        line_pub.publish(marker_line);
        line2_pub.publish(marker_line2);
    	ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}