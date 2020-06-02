#include "ros/ros.h"
#include "eigen3/Eigen/Dense"
#include <math.h>
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "tf/tf.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Float64.h"

//---------------------------------------------//

Eigen::Vector2d SK, a, b, c, d, c1, c2;
double r_c, r;

double norme(Eigen::Vector2d x)
{
    return sqrt(pow(x[0],2)+pow(x[1],2));
}

double angle(Eigen::Vector2d x)
{
    return atan2(x[1],x[0]);
}

void skCallback(const geometry_msgs::Point::ConstPtr& msg) {
    SK[0]=msg->x;
    SK[1]=msg->y;
}

void aCallback(const geometry_msgs::Point::ConstPtr& msg) {
    a[0]=msg->x;
    a[1]=msg->y;
}

void bCallback(const geometry_msgs::Point::ConstPtr& msg) {
    b[0]=msg->x;
    b[1]=msg->y;
}
void cCallback(const geometry_msgs::Point::ConstPtr& msg) {
    c[0]=msg->x;
    c[1]=msg->y;
}

void dCallback(const geometry_msgs::Point::ConstPtr& msg) {
    d[0]=msg->x;
    d[1]=msg->y;
}

void c1Callback(const geometry_msgs::Point::ConstPtr& msg) {
    c1[0]=msg->x;
    c1[1]=msg->y;
}

void c2Callback(const geometry_msgs::Point::ConstPtr& msg) {
    c2[0]=msg->x;
    c2[1]=msg->y;
}

void rCallback(const std_msgs::Float64::ConstPtr& msg) {
    r=msg->data;
}

void radiusCallback(const std_msgs::Float64::ConstPtr& msg) {
    r_c=msg->data;
}

int main(int argc, char **argv)
{   
	ros::init(argc, argv, "node_figure_sk");
    ros::NodeHandle n;
    ros::Subscriber pos_sk= n.subscribe("sk", 1000, skCallback);
    ros::Subscriber pos_a= n.subscribe("a", 1000, aCallback);
    ros::Subscriber pos_b= n.subscribe("b", 1000, bCallback);
    ros::Subscriber pos_c= n.subscribe("c", 1000, cCallback);
    ros::Subscriber pos_d= n.subscribe("d", 1000, dCallback);
    ros::Subscriber pos_c1= n.subscribe("c1", 1000, c1Callback);
    ros::Subscriber pos_c2= n.subscribe("c2", 1000, c2Callback);
    ros::Subscriber sub_r= n.subscribe("radius_r", 1000, rCallback);
    ros::Subscriber pos_rc= n.subscribe("radius_rc", 1000, radiusCallback);
    ros::Publisher line1_pub = n.advertise<visualization_msgs::Marker>( "visualization_line1",0 );
    ros::Publisher line2_pub = n.advertise<visualization_msgs::Marker>( "visualization_line2",0 );
    ros::Publisher circle_pub = n.advertise<visualization_msgs::Marker>( "visualization_circle",0 );
    ros::Publisher circle1_pub = n.advertise<visualization_msgs::Marker>( "visualization_circle1",0 );
    ros::Publisher circle2_pub = n.advertise<visualization_msgs::Marker>( "visualization_circle2",0 );


    ros::Rate loop_rate(100);
    double t0 = ros::Time::now().toSec();
    while(ros::ok()){
    	visualization_msgs::Marker marker, marker2, points, points1, points2;
	    	
    	//visualisation
    	double  tf = ros::Time::now().toSec();

		if (tf-t0<0.3)
        {
        	//ROS_INFO("Do anything");
        }
        
        else{

	    	marker.header.frame_id = "map";
	        marker.header.stamp = ros::Time::now();
	        marker.ns = "line1";
	        marker.id = 0;
	        marker.action = visualization_msgs::Marker::ADD;
	        marker.type = visualization_msgs::Marker::ARROW;
	        marker.scale.x = norme(d-b);
	        marker.scale.y = 0.3;
	        marker.scale.z = 0.1;
	        marker.color.a = 1.0;
	        marker.pose.position.x = b[0];
	        marker.pose.position.y = b[1];
	        marker.pose.position.z=0;
	       	tf::Quaternion q;
	       	double phi=angle(d-b);
			q.setRPY(0, 0, phi);
			tf::quaternionTFToMsg(q, marker.pose.orientation);
	        marker.color.r = 0.0f;
	        marker.color.g = 1.0f;
	        marker.color.b = 1.0f;

	        marker2.header.frame_id = "map";
	        marker2.header.stamp = ros::Time::now();
	        marker2.ns = "line2";
	        marker2.id = 0;
	        marker2.action = visualization_msgs::Marker::ADD;
	        marker2.type = visualization_msgs::Marker::ARROW;
	        marker2.scale.x = norme(a-c);
	        marker2.scale.y = 0.3;
	        marker2.scale.z = 0.1;
	        marker2.color.a = 1.0;
	        marker2.pose.position.x = c[0];
	        marker2.pose.position.y = c[1];
	        tf::Quaternion q2;
	       	phi=angle(a-c);
			q2.setRPY(0, 0, phi);
			tf::quaternionTFToMsg(q2, marker2.pose.orientation);
	        marker2.color.r = 0.0f;
	        marker2.color.g = 1.0f;
	        marker2.color.b = 1.0f;


	    	points.header.frame_id ="map";
	        points.header.stamp = ros::Time::now();
	        points.id = 0;
	        points.ns="circle";
	        points.action = visualization_msgs::Marker::ADD; 
	        points.type = visualization_msgs::Marker::LINE_STRIP;
	        points.pose.orientation.w=1;
	        points.scale.x = 0.1;
	    	//points.scale.y = 0.2;
	    	points.color.a = 1.0;
	        points.color.r = 1.0f;
	        points.color.g = 0.0f;
	        points.color.b = 0.0f;
	        for (double i = 0.0; i < 1.1; i+=0.01)
	        {
	        	double abs_x = SK[0] + r * cos(2*i*M_PI);
		        double ord_y = SK[1] + r * sin(2*i*M_PI);
		        geometry_msgs::Point p1;
		        p1.x = abs_x;
		        p1.y = ord_y;
		        p1.z=0;
		        points.points.push_back(p1);

		    }


		    points1.header.frame_id ="map";
	        points1.header.stamp = ros::Time::now();
	        points1.id = 0;
	        points1.ns="circle1";
	        points1.action = visualization_msgs::Marker::ADD; 
	        points1.type = visualization_msgs::Marker::LINE_STRIP;
	        points1.pose.orientation.w=1;
	        points1.scale.x = 0.1;
	    	//points.scale.y = 0.2;
	    	points1.color.a = 1.0;
	        points1.color.r = 1.0f;
	        points1.color.g = 1.0f;
	        points1.color.b = 0.0f;
	        for (double i = 0.0; i < 1.1; i+=0.01)
	        {
	        	double abs_x = c1[0] + r_c * cos(2*i*M_PI);
		        double ord_y = c1[1] + r_c * sin(2*i*M_PI);
		        geometry_msgs::Point p2;
		        p2.x = abs_x;
		        p2.y = ord_y;
		        p2.z=0;
		        points1.points.push_back(p2);

		    }


		    points2.header.frame_id ="map";
	        points2.header.stamp = ros::Time::now();
	        points2.id = 0;
	        points2.ns="circle2";
	        points2.action = visualization_msgs::Marker::ADD; 
	        points2.type = visualization_msgs::Marker::LINE_STRIP;
	        points2.pose.orientation.w=1;
	        points2.scale.x = 0.1;
	    	//points.scale.y = 0.2;
	    	points2.color.a = 1.0;
	        points2.color.r = 1.0f;
	        points2.color.g = 1.0f;
	        points2.color.b = 0.0f;
	        for (double i = 0.0; i < 1.1; i+=0.01)
	        {
	        	double abs_x = c2[0] + r_c * cos(2*i*M_PI);
		        double ord_y = c2[1] + r_c * sin(2*i*M_PI);
		        geometry_msgs::Point p3;
		        p3.x = abs_x;
		        p3.y = ord_y;
		        p3.z=0;
		        points2.points.push_back(p3);

		    }

			line1_pub.publish(marker);
			line2_pub.publish(marker2);
			circle_pub.publish(points);
			circle1_pub.publish(points1);
			circle2_pub.publish(points2);
		}
    	ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}