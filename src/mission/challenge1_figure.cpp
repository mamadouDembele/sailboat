#include "ros/ros.h"
#include "eigen3/Eigen/Dense"
#include <math.h>
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "tf/tf.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Float64.h"


using namespace std;

Eigen::Vector2d b1,b2,b3, fl, sl;
Eigen::Vector3d c;
double r;

double norme(Eigen::Vector2d x)
{
    return sqrt(pow(x[0],2)+pow(x[1],2));
}


void rCallback(const std_msgs::Float64::ConstPtr& msg) {
    r=msg->data;
}

void slCallback(const geometry_msgs::Point::ConstPtr& msg) {
    sl[0]=msg->x;
    sl[1]=msg->y;
}

void b1Callback(const geometry_msgs::Point::ConstPtr& msg) {
    b1[0]=msg->x;
    b1[1]=msg->y;
}

void b2Callback(const geometry_msgs::Point::ConstPtr& msg) {
    b2[0]=msg->x;
    b2[1]=msg->y;
}

void b3Callback(const geometry_msgs::Point::ConstPtr& msg) {
    b3[0]=msg->x;
    b3[1]=msg->y;
}

void flCallback(const geometry_msgs::Point::ConstPtr& msg) {
    fl[0]=msg->x;
    fl[1]=msg->y;
}

void colorCallback(const geometry_msgs::Point::ConstPtr& msg) {
    c[0]=msg->x;
    c[1]=msg->y;
    c[2]=msg->z;
}

int main(int argc, char **argv)
{   
	ros::init(argc, argv, "node_fr_figure");
    ros::NodeHandle n;
    ros::Publisher circle1_pub = n.advertise<visualization_msgs::Marker>( "visualization_circle1",0 );
    ros::Publisher circle2_pub = n.advertise<visualization_msgs::Marker>( "visualization_circle2",0 );
    ros::Publisher circle3_pub = n.advertise<visualization_msgs::Marker>( "visualization_circle3",0 );
    ros::Publisher st_line_pub = n.advertise<visualization_msgs::Marker>( "visualization_start_line",0 );
    ros::Publisher finish_pub = n.advertise<visualization_msgs::Marker>( "visualization_finish_line",0 );
    ros::Publisher line0_pub = n.advertise<visualization_msgs::Marker>( "visualization_line0",0 );
    ros::Publisher line1_pub = n.advertise<visualization_msgs::Marker>( "visualization_line1",0 );
    ros::Publisher line2_pub = n.advertise<visualization_msgs::Marker>( "visualization_line2",0 );
    ros::Publisher line3_pub = n.advertise<visualization_msgs::Marker>( "visualization_line3",0 );
    ros::Publisher cube_pub = n.advertise<visualization_msgs::Marker>( "visualization_cube",0 );
    ros::Publisher arrow_pub = n.advertise<visualization_msgs::Marker>( "visualization_arrow",0 );
    ros::Subscriber sub_r= n.subscribe("rad_r", 1000, rCallback);
    ros::Subscriber sub_sl= n.subscribe("start_line", 1000, slCallback);
    ros::Subscriber sub_b1= n.subscribe("buoy1", 1000, b1Callback);
    ros::Subscriber sub_b2= n.subscribe("buoy2", 1000, b2Callback);
    ros::Subscriber sub_b3= n.subscribe("buoy3", 1000, b3Callback);
    ros::Subscriber sub_c= n.subscribe("color", 1000, colorCallback);
    ros::Subscriber sub_fl= n.subscribe("finish_line", 1000, flCallback);

    ros::Rate loop_rate(100);
    double t0 = ros::Time::now().toSec();
    while(ros::ok()){
    	visualization_msgs::Marker points1, points, pointsf, points2, points3, marker_A, marker_c, line0, line1, line2, line3;



        double  tf = ros::Time::now().toSec();

        if (tf-t0<0.3)
        {
            ROS_INFO("Do anything");
        }
        
        else{

            ROS_INFO("radius=%f",r);
            ROS_INFO("fl[0]=%f",fl[0]);
            ROS_INFO("fl[1]=%f",fl[1]);
            for (int i=0; i<3; i++)
            {

                marker_A.header.frame_id = "map";
                marker_A.header.stamp = ros::Time::now();
                marker_A.ns = "arrow";
                marker_A.id = i;
                marker_A.action = visualization_msgs::Marker::ADD; 
                marker_A.type = visualization_msgs::Marker::ARROW;
                marker_A.pose.position.z=1;
                tf::Quaternion q;
                double phi=-M_PI/2;
                q.setRPY(0, phi, 0);
                tf::quaternionTFToMsg(q, marker_A.pose.orientation);
                marker_A.scale.x = 0.3;
                marker_A.scale.y = 0.3;
                marker_A.scale.z = 0.3;
                marker_A.color.a = 1.0;
                marker_A.color.r = 1.0f;
                marker_A.color.g = 1.0f;
                marker_A.color.b = 1.0f;

                marker_c.header.frame_id = "map";
                marker_c.header.stamp = ros::Time::now();
                marker_c.ns = "cube";
                marker_c.id = i;
                marker_c.action = visualization_msgs::Marker::ADD; 
                marker_c.type = visualization_msgs::Marker::CUBE;
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

                if (i==0)
                {
                    marker_A.pose.position.x = b1[0];
                    marker_A.pose.position.y = b1[1];
                    marker_c.pose.position.x = b1[0];
                    marker_c.pose.position.y = b1[1];
                }
                
                if (i==1)
                {
                    marker_A.pose.position.x = b2[0];
                    marker_A.pose.position.y = b2[1];
                    marker_c.pose.position.x = b2[0];
                    marker_c.pose.position.y = b2[1];
                }

                if (i==2)
                {
                    marker_A.pose.position.x = b3[0];
                    marker_A.pose.position.y = b3[1];
                    marker_c.pose.position.x = b3[0];
                    marker_c.pose.position.y = b3[1];
                }

                arrow_pub.publish(marker_A);
                cube_pub.publish(marker_c);
                
                
            }
            
            //start line
            points.header.frame_id ="map";
            points.header.stamp = ros::Time::now();
            points.id = 0;
            points.ns="start_line";
            points.action = visualization_msgs::Marker::ADD; 
            points.type = visualization_msgs::Marker::LINE_STRIP;
            points.pose.orientation.w=1;
            points.scale.x = 0.3;
            //points.scale.y = 0.2;
            points.color.a = 1.0;
            points.color.r = 1.0f;
            points.color.g = 1.0f;
            points.color.b = 0.0f;
            geometry_msgs::Point p1, p2;
            p1.x = sl[0]+50;
            p1.y = sl[1];
            p1.z=0;
            p2.x = sl[0]-60;
            p2.y = sl[1];
            p2.z=0;
            points.points.push_back(p1);
            points.points.push_back(p2);

            //line0
            line0.header.frame_id ="map";
            line0.header.stamp = ros::Time::now();
            line0.id = 0;
            line0.ns="line0";
            line0.action = visualization_msgs::Marker::ADD; 
            line0.type = visualization_msgs::Marker::LINE_STRIP;
            line0.pose.orientation.w=1;
            line0.scale.x = 0.5;
            //points.scale.y = 0.2;
            line0.color.a = 1.0;
            line0.color.r = 0.0f;
            line0.color.g = 0.0f;
            line0.color.b = 1.0f;
            p1.x = sl[0];
            p1.y = sl[1];
            p1.z=0;
            p2.x = b1[0];
            p2.y = b1[1];
            p2.z=0;
            line0.points.push_back(p1);
            line0.points.push_back(p2);

            //line1
            line1.header.frame_id ="map";
            line1.header.stamp = ros::Time::now();
            line1.id = 0;
            line1.ns="line1";
            line1.action = visualization_msgs::Marker::ADD; 
            line1.type = visualization_msgs::Marker::LINE_STRIP;
            line1.pose.orientation.w=1;
            line1.scale.x = 0.3;
            //points.scale.y = 0.2;
            line1.color.a = 1.0;
            line1.color.r = 0.0f;
            line1.color.g = 0.0f;
            line1.color.b = 1.0f;
            p1.x = b1[0];
            p1.y = b1[1];
            p1.z=0;
            p2.x = b2[0];
            p2.y = b2[1];
            p2.z=0;
            line1.points.push_back(p1);
            line1.points.push_back(p2);

            //line2
            line2.header.frame_id ="map";
            line2.header.stamp = ros::Time::now();
            line2.id = 0;
            line2.ns="line2";
            line2.action = visualization_msgs::Marker::ADD; 
            line2.type = visualization_msgs::Marker::LINE_STRIP;
            line2.pose.orientation.w=1;
            line2.scale.x = 0.3;
            line2.color.a = 1.0;
            line2.color.r = 0.0f;
            line2.color.g = 0.0f;
            line2.color.b = 1.0f;
            p1.x = b2[0];
            p1.y = b2[1];
            p1.z=0;
            p2.x = b3[0];
            p2.y = b3[1];
            p2.z=0;
            line2.points.push_back(p1);
            line2.points.push_back(p2);

            //line3
            line3.header.frame_id ="map";
            line3.header.stamp = ros::Time::now();
            line3.id = 0;
            line3.ns="line3";
            line3.action = visualization_msgs::Marker::ADD; 
            line3.type = visualization_msgs::Marker::LINE_STRIP;
            line3.pose.orientation.w=1;
            line3.scale.x = 0.3;
            line3.color.a = 1.0;
            line3.color.r = 0.0f;
            line3.color.g = 0.0f;
            line3.color.b = 1.0f;
            p1.x = b3[0];
            p1.y = b3[1];
            p1.z=0;
            p2.x = fl[0];
            p2.y = fl[1];
            p2.z=0;
            line3.points.push_back(p1);
            line3.points.push_back(p2);

            //finish line
            pointsf.header.frame_id ="map";
            pointsf.header.stamp = ros::Time::now();
            pointsf.id = 0;
            pointsf.ns="finish_line";
            pointsf.action = visualization_msgs::Marker::ADD; 
            pointsf.type = visualization_msgs::Marker::LINE_STRIP;
            pointsf.pose.orientation.w=1;
            pointsf.scale.x = 0.3;
            pointsf.color.a = 1.0;
            pointsf.color.r = 1.0f;
            pointsf.color.g = 1.0f;
            pointsf.color.b = 1.0f;
            geometry_msgs::Point pf1, pf2;
            pf1.x = fl[0];
            pf1.y = fl[1]+50;
            pf1.z=0;
            pf2.x = fl[0];
            pf2.y = fl[1]-50;
            pf2.z=0;
            pointsf.points.push_back(pf1);
            pointsf.points.push_back(pf2);

            //Circle1
            points1.header.frame_id ="map";
            points1.header.stamp = ros::Time::now();
            points1.id = 0;
            points1.ns="Circle1";
            points1.action = visualization_msgs::Marker::ADD; 
            points1.type = visualization_msgs::Marker::LINE_STRIP;
            points1.pose.orientation.w=1;
            points1.scale.x = 0.1;
        	//points.scale.y = 0.2;
        	points1.color.a = 1.0;
            points1.color.r = 1.0f;
            points1.color.g = 0.0f;
            points1.color.b = 0.0f;
            if (c[0]==1)
            {
                points1.color.r = 0.0f;
                points1.color.g = 1.0f;
                points1.color.b = 0.0f;
            }
            for (double i = 0.0; i < 1.1; i+=0.01)
            {
            	double abs_x = b1[0] + r * cos(2*i*M_PI);
    	        double ord_y = b1[1] + r * sin(2*i*M_PI);
    	        geometry_msgs::Point p;
    	        p.x = abs_x;
    	        p.y = ord_y;
    	        p.z=0;
    	        points1.points.push_back(p);

    	    }

            //Circle2
            points2.header.frame_id ="map";
            points2.header.stamp = ros::Time::now();
            points2.id = 0;
            points2.ns="Circle2";
            points2.action = visualization_msgs::Marker::ADD; 
            points2.type = visualization_msgs::Marker::LINE_STRIP;
            points2.pose.orientation.w=1;
            points2.scale.x = 0.1;
            //points.scale.y = 0.2;
            points2.color.a = 1.0;
            points2.color.r = 1.0f;
            points2.color.g = 0.0f;
            points2.color.b = 0.0f;
            if (c[1]==1)
            {
                points2.color.r = 0.0f;
                points2.color.g = 1.0f;
                points2.color.b = 0.0f;
            }
            for (double i = 0.0; i < 1.1; i+=0.01)
            {
                double abs_x = b2[0] + r * cos(2*i*M_PI);
                double ord_y = b2[1] + r * sin(2*i*M_PI);
                geometry_msgs::Point p;
                p.x = abs_x;
                p.y = ord_y;
                p.z=0;
                points2.points.push_back(p);

            }

            //Circle3
            points3.header.frame_id ="map";
            points3.header.stamp = ros::Time::now();
            points3.id = 0;
            points3.ns="Circle3";
            points3.action = visualization_msgs::Marker::ADD; 
            points3.type = visualization_msgs::Marker::LINE_STRIP;
            points3.pose.orientation.w=1;
            points3.scale.x = 0.1;
            //points.scale.y = 0.2;
            points3.color.a = 1.0;
            points3.color.r = 1.0f;
            points3.color.g = 0.0f;
            points3.color.b = 0.0f;
            if (c[2]==1)
            {
                points3.color.r = 0.0f;
                points3.color.g = 1.0f;
                points3.color.b = 0.0f;
            }
            for (double i = 0.0; i < 1.1; i+=0.01)
            {
                double abs_x = b3[0] + r * cos(2*i*M_PI);
                double ord_y = b3[1] + r * sin(2*i*M_PI);
                geometry_msgs::Point p;
                p.x = abs_x;
                p.y = ord_y;
                p.z=0;
                points3.points.push_back(p);

            }

    	    circle1_pub.publish(points1);
            circle2_pub.publish(points2);
            circle3_pub.publish(points3);
            st_line_pub.publish(points);
            finish_pub.publish(pointsf);
            line0_pub.publish(line0);
            line1_pub.publish(line1);
            line3_pub.publish(line3);
            line2_pub.publish(line2);
    	    
        }
        ros::spinOnce();
        loop_rate.sleep();
        
    }
    return 0;
}