#include "ros/ros.h"
#include "eigen3/Eigen/Dense"
#include <math.h>
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "tf/tf.h"


void line(visualization_msgs::Marker& marker, int i, Eigen::Vector2d a, Eigen::Vector2d b){
	marker.header.frame_id ="map";
    marker.header.stamp = ros::Time::now();
    marker.id = i;
    marker.ns="grille";
    marker.action = visualization_msgs::Marker::ADD; 
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.pose.orientation.w=1;
    marker.scale.x = 0.5;
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

void circle(visualization_msgs::Marker& marker, int i, Eigen::Vector2d c, int couleur)
{
	marker.header.frame_id ="map";
    marker.header.stamp = ros::Time::now();
    marker.id = i;
    marker.ns="circle_area_scanning";
    marker.action = visualization_msgs::Marker::ADD; 
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.pose.orientation.w=1;
    marker.scale.x = 0.2;
	marker.color.a = 1.0;
	marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
	if (couleur==1){
		marker.color.r = 1.0f;
    	marker.color.g = 1.0f;
    	marker.color.b = 0.0f;
	}
    for (double t = 0.0; t < 1.1; t+=0.01)
    {
    	double abs_x = c[0] + 5 * cos(2*t*M_PI);
        double ord_y = c[1] + 5 * sin(2*t*M_PI);
        geometry_msgs::Point p1;
        p1.x = abs_x;
        p1.y = ord_y;
        p1.z=0;
        marker.points.push_back(p1);

    }
}

int main(int argc, char **argv)
{   
    
    ros::init(argc, argv, "node_area_scanning_figure");
    ros::NodeHandle n;
    ros::Publisher grille_pub = n.advertise<visualization_msgs::Marker>( "visualization_area_scanning",0 );
    ros::Rate loop_rate(10);
    while(ros::ok()){
    	visualization_msgs::Marker marker;
    	double value=3*50, carre=3*5;
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

		for (int j=0; j<20; j++){
			for (int i=0; i<20; i++)
    		{
	    		Eigen::Vector2d c={-value+i*carre+carre/2, value-j*carre-carre/2};
	    		circle(marker, i+20*j, c, 0);
	    		grille_pub.publish(marker);
	    		marker.points.clear();
    		}

		}
		
    	ros::spinOnce();
        loop_rate.sleep();
        
    }
    return 0;
}