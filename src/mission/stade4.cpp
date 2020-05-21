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


double x,y,xa,ya,xb,yb,xc,yc,xd,yd;
double r=5;


int main(int argc, char **argv)
{   
	ros::init(argc, argv, "node_stade4");
    ros::NodeHandle n;
    ros::Publisher circle_pub = n.advertise<visualization_msgs::Marker>( "visualization_circle",0 );
    ros::Publisher cube_pub = n.advertise<visualization_msgs::Marker>( "visualization_cube",0 );
    ros::Publisher arrow_pub = n.advertise<visualization_msgs::Marker>( "visualization_arrow",0 );
    ros::Publisher line_pub = n.advertise<visualization_msgs::Marker>( "visualization_line",0 );
    ros::Publisher line2_pub = n.advertise<visualization_msgs::Marker>( "visualization_line2",0 );
    n.param<double>("posx", x, 0);
    n.param<double>("posy", y, 0);
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
    	visualization_msgs::Marker points, marker_A, marker_c, marker_line, marker_line2;

    	//point A
        string ns = ros::this_node::getNamespace();
    	marker_A.header.frame_id = "map";
        marker_A.header.stamp = ros::Time::now();
        marker_A.ns = ns;
        marker_A.id = 0;
        marker_A.action = visualization_msgs::Marker::ADD; 
        marker_A.type = visualization_msgs::Marker::ARROW;
        marker_A.pose.position.x = x;
        marker_A.pose.position.y = y;
        marker_A.pose.position.z=1;
        tf::Quaternion q;
        double phi=-M_PI/2;
        q.setRPY(0, phi, 0);
        tf::quaternionTFToMsg(q, marker_A.pose.orientation);
        marker_A.scale.x = 0.5;
    	marker_A.scale.y = 0.5;
    	marker_A.scale.z = 0.5;
    	marker_A.color.a = 1.0;
        marker_A.color.r = 1.0f;
        marker_A.color.g = 1.0f;
        marker_A.color.b = 1.0f;

        marker_c.header.frame_id = "map";
        marker_c.header.stamp = ros::Time::now();
        marker_c.ns = ns;
        marker_c.id = 0;
        marker_c.action = visualization_msgs::Marker::ADD; 
        marker_c.type = visualization_msgs::Marker::CUBE;
        marker_c.pose.position.x = x;
        marker_c.pose.position.y = y;
        marker_c.pose.position.z=0.5;
        marker_c.pose.orientation.x=0;
        marker_c.pose.orientation.y=0;
        marker_c.pose.orientation.z=0;
        marker_c.pose.orientation.w=1;
        marker_c.scale.x = 0.5;
        marker_c.scale.y = 0.5;
        marker_c.scale.z = 1.0;
        marker_c.color.a = 1.0;
        marker_c.color.r = 1.0f;
        marker_c.color.g = 1.0f;
        marker_c.color.b = 1.0f;

        //Circle
        points.header.frame_id ="map";
        points.header.stamp = ros::Time::now();
        points.id = 0;
        points.ns=ns;
        points.action = visualization_msgs::Marker::ADD; 
        points.type = visualization_msgs::Marker::LINE_STRIP;
        points.pose.orientation.w=1;
        points.scale.x = 0.2;
    	//points.scale.y = 0.2;
    	points.color.a = 1.0;
        points.color.r = 1.0f;
        points.color.g = 0.0f;
        points.color.b = 0.0f;
        for (double i = 0.0; i < 1.1; i+=0.01)
        {
        	double abs_x = x + 5 * cos(2*i*M_PI);
	        double ord_y = y + 5 * sin(2*i*M_PI);
	        geometry_msgs::Point p;
	        p.x = abs_x;
	        p.y = ord_y;
	        p.z=0;
	        points.points.push_back(p);

	    }

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
	    circle_pub.publish(points);
	    arrow_pub.publish(marker_A);
        cube_pub.publish(marker_c);
    	ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}