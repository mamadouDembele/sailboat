// this file is the line following code of the sailboat

#include "ros/ros.h"
#include "eigen3/Eigen/Dense"
#include <vector>
#include <math.h>
#include <algorithm>
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "tf/tf.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"


using namespace std;

Eigen::Vector2d m; // Position of the boat
Eigen::Vector2d a, b; // the ligne we want to follow
double r, zeta=M_PI/4, urmax=M_PI/4;// rayon de couloir, l'angle de près, l'angle maximale du gouvernail
double theta, psi_w; // cap de la voile, l'angle du vent
double q=1; // valeur d'hystérésis and the speed of the wind
double ur,us;
double ks=1.0; // constante k pour regler l'angle de la sail
double Gamma=M_PI/4; // Constance pour rendre la ligne plus attractive
tf::Quaternion q_sail, q_wind;


double sign(double x)
{
    if (x>0)
        return 1;
    else
        return -1;
}


double norme(Eigen::Vector2d x)
{
    return sqrt(pow(x[0],2)+pow(x[1],2));
}

double angle(Eigen::Vector2d x)
{
    return atan2(x[1],x[0]);
}

void controler_line(Eigen::Vector2d m, double theta, double psi_w, Eigen::Vector2d a, Eigen::Vector2d b, double& ur, double& us, double& q)
{
	Eigen::Vector2d diff=b-a;
	double nor=norme(diff);
	double e=((b-a)[0]*(m-a)[1]-(b-a)[1]*(m-a)[0])/nor;
	double phi=angle(b-a);
	double theta_bar;
	theta_bar=phi-2*Gamma*atan(e/r)/M_PI;
	if (fabs(e)>r/2)
	{
		q=sign(e);
	}

	if ((cos(psi_w-theta_bar)+cos(zeta))<0 || (fabs(e)<r && ((cos(psi_w-phi)+cos(zeta))<0)))
	{
		theta_bar=M_PI+psi_w - q*zeta;
	}
	
	if (cos(theta-theta_bar)>=0)
	{
		ur=urmax*sin(theta-theta_bar);
	}
	else
	{
		ur=urmax*sign(sin(theta-theta_bar));
	}


	us=ks*(M_PI/4)*(cos(psi_w-theta_bar)+1);

}


void poseCallback(const geometry_msgs::Point::ConstPtr& msg) {
    m[0]=msg->x;
    m[1]=msg->y;
}

void windCallback(const geometry_msgs::Quaternion::ConstPtr& msg) {
    q_wind[0]=msg->x;
    q_wind[1]=msg->y;
    q_wind[2]=msg->z;
    q_wind[3]=msg->w;
}

void headCallback(const geometry_msgs::Quaternion::ConstPtr& msg) {
    q_sail[0]=msg->x;
    q_sail[1]=msg->y;
    q_sail[2]=msg->z;
    q_sail[3]=msg->w;
}


int main(int argc, char **argv)
{   
	ros::init(argc, argv, "node_line");
    ros::NodeHandle n;
    ros::Subscriber pos_sail= n.subscribe("boat_pose", 1000, poseCallback);
    ros::Subscriber head_sail= n.subscribe("heading_boat", 1000, headCallback);
    ros::Subscriber wind= n.subscribe("wind_angle", 1000, windCallback);
    ros::Publisher com_servo = n.advertise<geometry_msgs::Vector3>("actuators", 1000);
    ros::Publisher pointA_pub = n.advertise<visualization_msgs::Marker>( "visualization_pointA",0 );
    ros::Publisher pointB_pub = n.advertise<visualization_msgs::Marker>( "visualization_pointB",0 );
    ros::Publisher line_pub = n.advertise<visualization_msgs::Marker>( "visualization_line",0 );
    ros::Publisher lineLeft_pub = n.advertise<visualization_msgs::Marker>( "visualization_lineLeft",0 );
    ros::Publisher lineRight_pub = n.advertise<visualization_msgs::Marker>( "visualization_lineRight",0 );
    ros::Publisher poswind_pub = n.advertise<geometry_msgs::Point>("pos_wind", 1000);

    n.param<double>("a_x", a[0], 0);
    n.param<double>("b_x", b[0], 0);
    n.param<double>("a_y", a[1], 0);
    n.param<double>("b_y", b[1], 0);
    n.param<double>("radius", r, 0);
    ros::Rate loop_rate(50);
    double t0 = ros::Time::now().toSec();
    while(ros::ok()){
    	geometry_msgs::Vector3 msg;
    	geometry_msgs::Point wind_point;
    	visualization_msgs::Marker marker_line, marker_A, marker_B, marker_lineLeft, marker_lineRight;


    	double roll,pitch;
        tf::Matrix3x3(q_sail).getRPY(roll, pitch, theta);
        tf::Matrix3x3(q_wind).getRPY(roll, pitch, psi_w);

    	//regulation and publication

    	double  tf = ros::Time::now().toSec();

		if (tf-t0<0.3)
        {
        	ROS_INFO("Do anything");
        }
        
        else{
        	//ROS_INFO("mx=%f", m[0]);
	        //ROS_INFO("my=%f", m[1]);
	    	controler_line(m, theta, psi_w, a, b, ur, us,q);
	    	ROS_INFO("angle_sail us=%f", us);
	    	msg.x=ur;
	    	msg.y=us;
	    	msg.z=0;
	    	com_servo.publish(msg);

	    	//visualisation

	    	//point A
	    	marker_A.header.frame_id = "map";
	        marker_A.header.stamp = ros::Time::now();
	        marker_A.ns = "Point_A";
	        marker_A.id = 0;
	        marker_A.action = visualization_msgs::Marker::ADD; 
	        marker_A.type = visualization_msgs::Marker::SPHERE;
	        marker_A.pose.position.x = a[0];
	        marker_A.pose.position.y = a[1];
	        marker_A.pose.position.z=0;
	        marker_A.pose.orientation.x=0;
	       	marker_A.pose.orientation.y=0;
	       	marker_A.pose.orientation.z=0;
	       	marker_A.pose.orientation.w=1;
	        marker_A.scale.x = 0.5;
	    	marker_A.scale.y = 0.5;
	    	marker_A.scale.z = 0.5;
	    	marker_A.color.a = 1.0;
	        marker_A.color.r = 1.0f;
	        marker_A.color.g = 1.0f;
	        marker_A.color.b = 1.0f;


	        //point B
	    	marker_B.header.frame_id = "map";
	        marker_B.header.stamp = ros::Time::now();
	        marker_B.ns = "point_B";
	        marker_B.id = 0;
	        marker_B.action = visualization_msgs::Marker::ADD;
	        marker_B.type = visualization_msgs::Marker::SPHERE;
	        marker_B.pose.position.x =b[0];
	        marker_B.pose.position.y = b[1];
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
	        marker_B.color.g = 0.0f;
	        marker_B.color.b = 0.0f;


	        //Line
			marker_line.header.frame_id = "map";
	        marker_line.header.stamp = ros::Time::now();
	        marker_line.ns = "line";
	        marker_line.id = 0;
	        marker_line.action = visualization_msgs::Marker::ADD;
	        marker_line.type = visualization_msgs::Marker::LINE_STRIP;
	        marker_line.scale.x = 0.1;
	        geometry_msgs::Point cA;
	        geometry_msgs::Point cB;

	        //
	        if (b[0]==a[0])
	        {
	        	cA.x=a[0];
		        cA.y=-160;
		        cA.z=0;
		        cB.x=a[0];
		        cB.y=160;
		        cB.z=0;

	        }
	        else{
	        	double coeff_a=(b[1]-a[1])/(b[0]-a[0]);
		        double coeff_b=a[1]-coeff_a*a[0];
		        cA.x=160;
		        cA.y=coeff_a*cA.x+coeff_b;
		        cA.z=0;
		        cB.x=-160;
		        cB.y=coeff_a*cB.x+coeff_b;
		        cB.z=0;
	        }
	        
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


	        //Line left
			marker_lineLeft.header.frame_id = "map";
	        marker_lineLeft.header.stamp = ros::Time::now();
	        marker_lineLeft.ns = "line_left";
	        marker_lineLeft.id = 0;
	        marker_lineLeft.action = visualization_msgs::Marker::ADD;
	        marker_lineLeft.type = visualization_msgs::Marker::LINE_STRIP;
	        marker_lineLeft.scale.x = 0.1;
	        
	        if (a[0]==b[0])
	        {
		        Eigen::Vector2d u={-1,0};
		        Eigen::Vector2d b2=a+r*u;
		        cA.x=b2[0];
		        cA.y=150;
		        cA.z=0;
		        cB.x=b2[0];
		        cB.y=-150;
		        cB.z=0;
	        }

	        else{
	        	Eigen::Vector2d diff=b-a;
	        	double nor=norme(diff);
		        Eigen::Vector2d u={-((b-a)/nor)[1],((b-a)/nor)[0]};
		        Eigen::Vector2d b2=a+r*u;
		        double coeff_a=(b[1]-a[1])/(b[0]-a[0]);
	        	double coeff_b1=b2[1]-coeff_a*b2[0];
		        cA.x=150;
		        cA.y=coeff_a*cA.x+coeff_b1;
		        cA.z=0;
		        cB.x=-150;
		        cB.y=coeff_a*cB.x+coeff_b1;
		        cB.z=0;
	        }
	        
	        marker_lineLeft.points.push_back(cA);
	        marker_lineLeft.points.push_back(cB);
	        marker_lineLeft.pose.orientation.x=0;
	       	marker_lineLeft.pose.orientation.y=0;
	       	marker_lineLeft.pose.orientation.z=0;
	       	marker_lineLeft.pose.orientation.w=1;
	       	marker_lineLeft.color.a=0.5;
	        marker_lineLeft.color.r = 1.0f;
	        marker_lineLeft.color.g = 0.0f;
	        marker_lineLeft.color.b = 0.0f;
		    
		    //Line right
			marker_lineRight.header.frame_id = "map";
	        marker_lineRight.header.stamp = ros::Time::now();
	        marker_lineRight.ns = "line_right";
	        marker_lineRight.id = 0;
	        marker_lineRight.action = visualization_msgs::Marker::ADD;
	        marker_lineRight.type = visualization_msgs::Marker::LINE_STRIP;
	        marker_lineRight.scale.x = 0.1;
	        
	        if (a[0]==b[0])
	        {
		        Eigen::Vector2d u={1,0};
		        Eigen::Vector2d b2=a+r*u;
		        cA.x=b2[0];
		        cA.y=150;
		        cA.z=0;
		        cB.x=b2[0];
		        cB.y=-150;
		        cB.z=0;
	        }

	        else{
	        	Eigen::Vector2d diff=b-a;
	        	double nor=norme(diff);
		        Eigen::Vector2d u={((b-a)/nor)[1],-((b-a)/nor)[0]};
		        Eigen::Vector2d b2=a+r*u;
		        double coeff_a=(b[1]-a[1])/(b[0]-a[0]);
	        	double coeff_b1=b2[1]-coeff_a*b2[0];
		        cA.x=150;
		        cA.y=coeff_a*cA.x+coeff_b1;
		        cA.z=0;
		        cB.x=-150;
		        cB.y=coeff_a*cB.x+coeff_b1;
		        cB.z=0;
	        }
	        
	        marker_lineRight.points.push_back(cA);
	        marker_lineRight.points.push_back(cB);
	        marker_lineRight.pose.orientation.x=0;
	       	marker_lineRight.pose.orientation.y=0;
	       	marker_lineRight.pose.orientation.z=0;
	       	marker_lineRight.pose.orientation.w=1;
	       	marker_lineRight.color.a=0.5;
	        marker_lineRight.color.r = 1.0f;
	        marker_lineRight.color.g = 0.0f;
	        marker_lineRight.color.b = 0.0f;
	        wind_point.x=b[0]+5.0;
	        wind_point.y=b[1]+5.0;
	        wind_point.z=0;

	        // Publication
	        poswind_pub.publish(wind_point);
		    pointA_pub.publish(marker_A);
		    pointB_pub.publish(marker_B);
		    line_pub.publish(marker_line);
		    lineLeft_pub.publish(marker_lineLeft);
	        lineRight_pub.publish(marker_lineRight);
	    }
    	ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}