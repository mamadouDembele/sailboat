#include "ros/ros.h"
#include "eigen3/Eigen/Dense"
#include <math.h>
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "tf/tf.h"

Eigen::Vector2d c1, c2, c3, c4;
double couleur1, couleur2, couleur3, couleur4;
int i1=0, i2=0, i3=0, i4=0;

void line(visualization_msgs::Marker& marker, int i, Eigen::Vector2d a, Eigen::Vector2d b){
	marker.header.frame_id ="map";
    marker.header.stamp = ros::Time::now();
    marker.id = i;
    marker.ns="grille";
    marker.action = visualization_msgs::Marker::ADD; 
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.pose.orientation.w=1;
    marker.scale.x = 0.3;
    marker.color.a = 1.0;
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    geometry_msgs::Point p1, p2;
    p1.x = a[0];
    p1.y = a[1];
    p1.z=0;
    p2.x = b[0];
    p2.y = b[1];
    p2.z=0;
    marker.points.push_back(p1);
    marker.points.push_back(p2);
}

void grille_sqr(visualization_msgs::Marker& marker, int i, Eigen::Vector2d a, Eigen::Vector2d b, Eigen::Vector2d c, Eigen::Vector2d d, ros::Publisher grille_pub){
	line(marker, i, a, b);
	marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    grille_pub.publish(marker);
    marker.points.clear();
	line(marker, i+1, b, c);
	marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    grille_pub.publish(marker);
    marker.points.clear();
	line(marker, i+2, c, d);
	marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    grille_pub.publish(marker);
    marker.points.clear();
	line(marker, i+3, a, d);
	marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    grille_pub.publish(marker);
    marker.points.clear();
}

void circle(visualization_msgs::Marker& marker, int i, Eigen::Vector2d c, double couleur)
{
	marker.header.frame_id ="map";
    marker.header.stamp = ros::Time::now();
    marker.id = i;
    marker.ns="circle_area_scanning";
    marker.action = visualization_msgs::Marker::ADD; 
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.pose.orientation.w=1;
    marker.scale.x = 0.1;
	marker.color.a = 1.0;
	marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
	if (couleur==1.0){
		marker.color.r = 1.0f;
    	marker.color.g = 0.0f;
    	marker.color.b = 0.0f;
	}

    if (couleur==2.0){
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
    }

    if (couleur==3.0){
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
    }

    if (couleur==4.0){
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 1.0f;
    }

    for (double t = 0.0; t < 1.1; t+=0.01)
    {
    	double abs_x = c[0] + 2. * cos(2*t*M_PI);
        double ord_y = c[1] + 2. * sin(2*t*M_PI);
        geometry_msgs::Point p1;
        p1.x = abs_x;
        p1.y = ord_y;
        p1.z=0;
        marker.points.push_back(p1);

    }
}

void cp1Callback(const geometry_msgs::Point::ConstPtr& msg) {
    c1[0]=msg->x;
    c1[1]=msg->y;
    couleur1=msg->z;
}

void cp2Callback(const geometry_msgs::Point::ConstPtr& msg) {
    c2[0]=msg->x;
    c2[1]=msg->y;
    couleur2=msg->z;
}

void cp3Callback(const geometry_msgs::Point::ConstPtr& msg) {
    c3[0]=msg->x;
    c3[1]=msg->y;
    couleur3=msg->z;
}

void cp4Callback(const geometry_msgs::Point::ConstPtr& msg) {
    c4[0]=msg->x;
    c4[1]=msg->y;
    couleur4=msg->z;
}

int main(int argc, char **argv)
{   
    
    ros::init(argc, argv, "node_area_scanning_figure");
    ros::NodeHandle n;
    ros::Publisher grille_pub = n.advertise<visualization_msgs::Marker>( "visualization_area_scanning",0 );
    ros::Publisher validPoint1 = n.advertise<visualization_msgs::Marker>( "visualization_validPoint1",0 );
    ros::Publisher validPoint2 = n.advertise<visualization_msgs::Marker>( "visualization_validPoint2",0 );
    ros::Publisher validPoint3 = n.advertise<visualization_msgs::Marker>( "visualization_validPoint3",0 );
    ros::Publisher validPoint4 = n.advertise<visualization_msgs::Marker>( "visualization_validPoint4",0 );
    ros::Subscriber sub1= n.subscribe("center_point1", 1000, cp1Callback);
    ros::Subscriber sub2= n.subscribe("center_point2", 1000, cp2Callback);
    ros::Subscriber sub3= n.subscribe("center_point3", 1000, cp3Callback);
    ros::Subscriber sub4= n.subscribe("center_point4", 1000, cp4Callback);

    ros::Rate loop_rate(100);
    double t0 = ros::Time::now().toSec();
    while(ros::ok()){
    	visualization_msgs::Marker marker, marker1, marker2, marker3, marker4;
    	double value=50, carre=5;

    	for (int i=0; i< 21; i++)
    	{
    		Eigen::Vector2d a={-value+i*carre, value};
    		Eigen::Vector2d b={-value+i*carre, -value};
    		Eigen::Vector2d c={-value, -value+i*carre};
    		Eigen::Vector2d d={value, -value+i*carre};
    		line(marker, i, a, b);
    		grille_pub.publish(marker);
    		marker.points.clear();
    		line(marker, i+21, c, d);
    		grille_pub.publish(marker);
    		marker.points.clear();
    	}

    	Eigen::Vector2d a={-value, value+4*carre};
		Eigen::Vector2d b={-value, value+3*carre};
		Eigen::Vector2d c={-value+carre, value+3*carre};
		Eigen::Vector2d d={-value+carre, value+ 4*carre};
		grille_sqr(marker, 43, a, b, c, d, grille_pub);

		a={value-carre, value+4*carre};
		b={value-carre, value+3*carre};
		c={value, value+3*carre};
		d={value, value+ 4*carre};
		grille_sqr(marker, 50, a, b, c, d, grille_pub);

		a={-value, -value-3*carre};
		b={-value, -value-4*carre};
		c={-value+carre, -value-4*carre};
		d={-value+carre, -value- 3*carre};
		grille_sqr(marker, 55, a, b, c, d, grille_pub);

		a={value-carre, -value-3*carre};
		b={value-carre, -value-4*carre};
		c={value, -value-4*carre};
		d={value, -value- 3*carre};
		grille_sqr(marker, 60, a, b, c, d, grille_pub);

		double  tf = ros::Time::now().toSec();
        if (tf-t0<0.6)
        {
            //ROS_INFO("mx=%f", m[0]);
            //ROS_INFO("my=%f", m[1]);
            //ROS_INFO("Do anything");
        }

        else{

            circle(marker1, i1, c1, couleur1);
            circle(marker2, i2, c2, couleur2);
            circle(marker3, i3, c3, couleur3);
            circle(marker4, i4, c4, couleur4);
            i1++;
            i2++;
            i3++;
            i4++;
            validPoint1.publish(marker1);
            validPoint2.publish(marker2);
            validPoint3.publish(marker3);
            validPoint4.publish(marker4);
            marker1.points.clear();
            marker2.points.clear();
            marker3.points.clear();
            marker4.points.clear();
            tf=0;
            t0=ros::Time::now().toSec();
        }
		
    	ros::spinOnce();
        loop_rate.sleep();
        
    }
    return 0;
}