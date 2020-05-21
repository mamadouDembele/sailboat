#include "ros/ros.h"
#include "eigen3/Eigen/Dense"
#include <math.h>
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "tf/tf.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Float64.h"

//---------------------------------------------//

Eigen::Vector2d SK, a, b, c, d, c1;
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


void rCallback(const std_msgs::Float64::ConstPtr& msg) {
    r=msg->data;
}

void radiusCallback(const std_msgs::Float64::ConstPtr& msg) {
    r_c=msg->data;
}

int main(int argc, char **argv)
{   
	ros::init(argc, argv, "node_figure_avoid");
    ros::NodeHandle n;
    ros::Subscriber pos_sk= n.subscribe("sk", 1000, skCallback);
    ros::Subscriber pos_a= n.subscribe("a", 1000, aCallback);
    ros::Subscriber pos_b= n.subscribe("b", 1000, bCallback);
    ros::Subscriber pos_c= n.subscribe("c", 1000, cCallback);
    ros::Subscriber pos_d= n.subscribe("d", 1000, dCallback);
    ros::Subscriber pos_c1= n.subscribe("c1", 1000, c1Callback);
    ros::Subscriber sub_r= n.subscribe("radius", 1000, rCallback);
    ros::Subscriber pos_rc= n.subscribe("radius_rc", 1000, radiusCallback);
    ros::Publisher cube_pub = n.advertise<visualization_msgs::Marker>( "visualization_cube",0 );
    ros::Publisher arrow_pub = n.advertise<visualization_msgs::Marker>( "visualization_arrow",0 );
    ros::Publisher line1_pub = n.advertise<visualization_msgs::Marker>( "visualization_line1",0 );
    ros::Publisher line2_pub = n.advertise<visualization_msgs::Marker>( "visualization_line2",0 );
    ros::Publisher line3_pub = n.advertise<visualization_msgs::Marker>( "visualization_line3",0 );
    ros::Publisher circle_pub = n.advertise<visualization_msgs::Marker>( "visualization_circle",0 );
    ros::Publisher line4_pub = n.advertise<visualization_msgs::Marker>( "visualization_line4",0 );

    ros::Rate loop_rate(100);
    double t0 = ros::Time::now().toSec();
    while(ros::ok()){
    	visualization_msgs::Marker marker_A, marker_c, marker, marker2, marker3, points, points1, marker4;
    	//visualisation
    	double  tf = ros::Time::now().toSec();

		if (tf-t0<0.3)
        {
        	//ROS_INFO("Do anything");
        }
        
        else{
        	ROS_INFO("radius r=%f", r);

        	marker_A.header.frame_id = "map";
            marker_A.header.stamp = ros::Time::now();
            marker_A.ns = "arrow";
            marker_A.id = 0;
            marker_A.action = visualization_msgs::Marker::ADD; 
            marker_A.type = visualization_msgs::Marker::ARROW;
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
            marker_A.pose.position.x = SK[0];
            marker_A.pose.position.y = SK[1];

            marker_c.header.frame_id = "map";
            marker_c.header.stamp = ros::Time::now();
            marker_c.ns = "cube";
            marker_c.id = 0;
            marker_c.action = visualization_msgs::Marker::ADD; 
            marker_c.type = visualization_msgs::Marker::CUBE;
            marker_c.pose.position.x = SK[0];
            marker_c.pose.position.y = SK[1];
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

	    	marker.header.frame_id = "map";
	        marker.header.stamp = ros::Time::now();
	        marker.ns = "line1";
	        marker.id = 0;
	        marker.action = visualization_msgs::Marker::ADD;
	        marker.type = visualization_msgs::Marker::ARROW;
	        marker.scale.x = norme(d-a);
	        marker.scale.y = 1.0;
	        marker.scale.z = 0.1;
	        marker.color.a = 1.0;
	        marker.pose.position.x = d[0];
	        marker.pose.position.y = d[1];
	        marker.pose.position.z=0;
	       	phi=angle(a-d);
			q.setRPY(0, 0, phi);
			tf::quaternionTFToMsg(q, marker.pose.orientation);
	        marker.color.r = 0.0f;
	        marker.color.g = 1.0f;
	        marker.color.b = 0.0f;

	        marker2.header.frame_id = "map";
	        marker2.header.stamp = ros::Time::now();
	        marker2.ns = "line2";
	        marker2.id = 0;
	        marker2.action = visualization_msgs::Marker::ADD;
	        marker2.type = visualization_msgs::Marker::ARROW;
	        marker2.scale.x = norme(b-c);
	        marker2.scale.y = 1.0;
	        marker2.scale.z = 0.1;
	        marker2.color.a = 1.0;
	        marker2.pose.position.x = b[0];
	        marker2.pose.position.y = b[1];
	        tf::Quaternion q2;
	       	phi=angle(c-b);
			q2.setRPY(0, 0, phi);
			tf::quaternionTFToMsg(q2, marker2.pose.orientation);
	        marker2.color.r = 0.0f;
	        marker2.color.g = 1.0f;
	        marker2.color.b = 1.0f;

	        marker3.header.frame_id = "map";
	        marker3.header.stamp = ros::Time::now();
	        marker3.ns = "line3";
	        marker3.id = 0;
	        marker3.action = visualization_msgs::Marker::ADD;
	        marker3.type = visualization_msgs::Marker::ARROW;
	        marker3.scale.x = norme(c-d);
	        marker3.scale.y = 1.0;
	        marker3.scale.z = 0.1;
	        marker3.color.a = 1.0;
	        marker3.pose.position.x = c[0];
	        marker3.pose.position.y = c[1];
	       	phi=angle(d-c);
			q2.setRPY(0, 0, phi);
			tf::quaternionTFToMsg(q2, marker3.pose.orientation);
	        marker3.color.r = 0.0f;
	        marker3.color.g = 1.0f;
	        marker3.color.b = 1.0f;

	    	points.header.frame_id ="map";
	        points.header.stamp = ros::Time::now();
	        points.id = 0;
	        points.ns="circle";
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
	        	double abs_x = SK[0] + r * cos(2*i*M_PI);
		        double ord_y = SK[1] + r * sin(2*i*M_PI);
		        geometry_msgs::Point p1;
		        p1.x = abs_x;
		        p1.y = ord_y;
		        p1.z=0;
		        points.points.push_back(p1);

		    }


		    marker4.header.frame_id = "map";
	        marker4.header.stamp = ros::Time::now();
	        marker4.ns = "line4";
	        marker4.id = 0;
	        marker4.action = visualization_msgs::Marker::ADD;
	        marker4.type = visualization_msgs::Marker::ARROW;
	        marker4.scale.x = norme(b-a);
	        marker4.scale.y = 1.0;
	        marker4.scale.z = 0.1;
	        marker4.color.a = 1.0;
	        marker4.pose.position.x = a[0];
	        marker4.pose.position.y = a[1];
	       	phi=angle(b-a);
			q2.setRPY(0, 0, phi);
			tf::quaternionTFToMsg(q2, marker4.pose.orientation);
	        marker4.color.r = 0.0f;
	        marker4.color.g = 1.0f;
	        marker4.color.b = 1.0f;

		    arrow_pub.publish(marker_A);
            cube_pub.publish(marker_c);
			line1_pub.publish(marker);
			line2_pub.publish(marker2);
			line3_pub.publish(marker3);
			circle_pub.publish(points);
			line4_pub.publish(marker4);
		}
    	ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}