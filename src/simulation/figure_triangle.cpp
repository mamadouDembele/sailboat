#include "ros/ros.h"
#include "eigen3/Eigen/Dense"
#include <math.h>
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "tf/tf.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Float64.h"

//---------------------------------------------//

Eigen::Vector2d SK, a, b, c;
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


void rcCallback(const std_msgs::Float64::ConstPtr& msg) {
    r=msg->data;
}

void rccCallback(const std_msgs::Float64::ConstPtr& msg) {
    r_c=msg->data;
}

int main(int argc, char **argv)
{   
	ros::init(argc, argv, "node_figure_triangle");
    ros::NodeHandle n;
    ros::Subscriber pos_sk= n.subscribe("sk", 1000, skCallback);
    ros::Subscriber pos_a= n.subscribe("a_triangle", 1000, aCallback);
    ros::Subscriber pos_b= n.subscribe("b_triangle", 1000, bCallback);
    ros::Subscriber pos_c= n.subscribe("c_triangle", 1000, cCallback);
    ros::Subscriber sub_r= n.subscribe("radius_circle", 1000, rcCallback);
    ros::Subscriber sub_rc= n.subscribe("radius_circle_cuic", 1000, rccCallback);
    ros::Publisher cube_pub = n.advertise<visualization_msgs::Marker>( "visualization_cube",0 );
    ros::Publisher arrow_pub = n.advertise<visualization_msgs::Marker>( "visualization_arrow",0 );
    ros::Publisher circle_pub = n.advertise<visualization_msgs::Marker>( "visualization_circle",0 );
    ros::Publisher circle1_pub = n.advertise<visualization_msgs::Marker>( "visualization_circle_circonscrit",0 );
	ros::Publisher trian_pub = n.advertise<visualization_msgs::Marker>( "visualization_triangle",0 );    

    ros::Rate loop_rate(100);
    double t0 = ros::Time::now().toSec();
    while(ros::ok()){
    	visualization_msgs::Marker marker_A, marker_c, points, marker, points1;
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
	        points1.ns="circle_circons";
	        points1.action = visualization_msgs::Marker::ADD; 
	        points1.type = visualization_msgs::Marker::LINE_STRIP;
	        points1.pose.orientation.w=1;
	        points1.scale.x = 0.1;
	    	//points.scale.y = 0.2;
	    	points1.color.a = 1.0;
	        points1.color.r = 0.0f;
	        points1.color.g = 1.0f;
	        points1.color.b = 0.0f;
	        for (double i = 0.0; i < 1.1; i+=0.01)
	        {
	        	double abs_x = SK[0] + r_c * cos(2*i*M_PI);
		        double ord_y = SK[1] + r_c * sin(2*i*M_PI);
		        geometry_msgs::Point p1;
		        p1.x = abs_x;
		        p1.y = ord_y;
		        p1.z=0;
		        points1.points.push_back(p1);

		    }


		    //visualisation
	    	for (int i=0; i<6; i++)
	    	{

		    	marker.header.frame_id = "map";
		        marker.header.stamp = ros::Time::now();
		        marker.ns = "triangle";
		        marker.id = i;
		        marker.action = visualization_msgs::Marker::ADD;
		        marker.scale.x = 0.3;
		        marker.scale.y = 0.3;
		        marker.scale.z = 0.3;
		        marker.color.a = 1.0;
		        marker.pose.position.z=0;
		        marker.pose.orientation.x=0;
		       	marker.pose.orientation.y=0;
		       	marker.pose.orientation.z=0;
		       	marker.pose.orientation.w=1;
		       	marker.color.r = 0.0f;
			    marker.color.g = 0.0f;
			    marker.color.b = 1.0f;
			    marker.type = visualization_msgs::Marker::SPHERE;
		       	tf::Quaternion q;
		        if (i==0)
		        {
			        marker.pose.position.x = a[0];
			        marker.pose.position.y = a[1];
		    	}

		    	if (i==1)
		        {
			        marker.pose.position.x = b[0];
			        marker.pose.position.y = b[1];
		    	}

		    	if (i==2)
		        {
			        marker.pose.position.x = c[0];
			        marker.pose.position.y = c[1];
		    	}

		    	if (i==3)
		        {
			        marker.type = visualization_msgs::Marker::ARROW;
			        marker.scale.x = norme(b-a);
			        marker.scale.z = 0.1;
			        marker.pose.position.x = a[0];
			        marker.pose.position.y = a[1];
	        		q.setRPY(0, 0, angle(b-a));
	        		tf::quaternionTFToMsg(q, marker.pose.orientation);
			        marker.color.r = 1.0f;
			        marker.color.g = 1.0f;
			        marker.color.b = 0.0f;
		    	}

		    	if (i==4)
		        {
			        marker.type = visualization_msgs::Marker::ARROW;
			        marker.scale.x = norme(c-b);
			        marker.scale.z = 0.1;
			        marker.pose.position.x = b[0];
			        marker.pose.position.y = b[1];
	        		q.setRPY(0, 0, angle(c-b));
	        		tf::quaternionTFToMsg(q, marker.pose.orientation);
			        marker.color.r = 1.0f;
			        marker.color.g = 1.0f;
			        marker.color.b = 0.0f;
		    	}

		    	if (i==5)
		        {
			        marker.type = visualization_msgs::Marker::ARROW;
			        marker.scale.x = norme(c-a);
			        marker.scale.z = 0.1;
			        marker.pose.position.x = c[0];
			        marker.pose.position.y = c[1];
	        		q.setRPY(0, 0, angle(a-c));
	        		tf::quaternionTFToMsg(q, marker.pose.orientation);
			        marker.color.r = 1.0f;
			        marker.color.g = 1.0f;
			        marker.color.b = 0.0f;
		    	}
		        trian_pub.publish( marker );
		    }

		    arrow_pub.publish(marker_A);
            cube_pub.publish(marker_c);
			circle_pub.publish(points);
			circle1_pub.publish(points1);


		}
    	ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}