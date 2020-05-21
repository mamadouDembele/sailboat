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
double q=0;
bool initStateFinie=true;
Eigen::Vector2d m;
double r, zeta=M_PI/4, urmax=M_PI/4;
double theta, psi_w;
double ks=1.0;
double us, ur;
Eigen::Vector2d SK;
tf::Quaternion q_sail, q_wind;
//---------------------------------------------//

double sign(double x)
{
    if (x<0)
        return -1;
    else
        return 1;
}


double angle(Eigen::Vector2d x)
{
    return atan2(x[1],x[0]);
}


void f0(double p1, double p2, Eigen::Vector2d& v)
{
	v[0]=-(pow(p1,3)+pow(p2,2)*p1-p1+p2);
	v[1]=-(pow(p2,3)+pow(p1,2)*p2-p1-p2);
}

void f(Eigen::Vector2d m, Eigen::Matrix2d D, Eigen::Vector2d c, Eigen::Vector2d& v)
{
	Eigen::Matrix2d D_=D.inverse();
	Eigen::Vector2d z=D_*(m-c);
	Eigen::Vector2d w;
	f0(z[0], z[1], w);
	v=D*w;
	//v+=20*(m-c)/pow(norme(m-c),3);
}

void controller_circle(Eigen::Vector2d m, double theta, double psi_w, Eigen::Matrix2d D, Eigen::Vector2d c, double& ur, double& us)
{
	Eigen::Vector2d v;
	f(m, D, c, v);
	double theta_bar=angle(v);
	if (cos(theta-theta_bar)>=0)
	{
		ur=urmax*sin(theta-theta_bar);
	}
	else
	{
		ur=urmax*sign(sin(theta-theta_bar));
	}
	//ur=kr*atan(tan(0.5*(theta-theta_bar)));
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
	ros::init(argc, argv, "node_keeping2");
    ros::NodeHandle n;
    ros::Subscriber pos_sail= n.subscribe("boat_pose", 1000, poseCallback);
    ros::Subscriber head_sail= n.subscribe("heading_boat", 1000, headCallback);
    ros::Subscriber wind= n.subscribe("wind_angle", 1000, windCallback);
    ros::Publisher pointsk_pub = n.advertise<visualization_msgs::Marker>( "visualization_sk",0 );
    ros::Publisher circle1_pub = n.advertise<visualization_msgs::Marker>( "visualization_circle1",0 );
    ros::Publisher circle2_pub = n.advertise<visualization_msgs::Marker>( "visualization_circle2",0 );
    ros::Publisher com_servo = n.advertise<geometry_msgs::Vector3>("actuators", 1000);

    ros::Rate loop_rate(100);
    double t0 = ros::Time::now().toSec();
    n.param<double>("SKx", SK[0], 0);
    n.param<double>("SKy", SK[1], 0);
    n.param<double>("radius", r, 0);
    while(ros::ok()){
    	geometry_msgs::Vector3 msg;
	    visualization_msgs::Marker marker_sk, circle1, circle2;

    	double roll,pitch;
        tf::Matrix3x3(q_sail).getRPY(roll, pitch, theta);
        tf::Matrix3x3(q_wind).getRPY(roll, pitch, psi_w);

        
        double r_c=r/2;
        Eigen::Matrix2d R;
		R << r_c, 0, 
			0,r_c;

		Eigen::Matrix2d Conter1, Conter2;
		Conter1 << 1,0,
			0,-1;

		Eigen::Matrix2d D1=R*Conter1;

        double  tf = ros::Time::now().toSec();

		if (tf-t0<0.3)
        {
        	ROS_INFO("Do anything");
        }
        
        else{
	    	controller_circle(m, theta, psi_w, D1, SK, ur, us);

	    	//ROS_INFO(" offside ur=%f", ur);
	    	msg.x=ur;
	    	msg.y=us;
	    	msg.z=0;
	    	com_servo.publish(msg);

	    	//visualisation
	    	//Stattion keeping point
	    	marker_sk.header.frame_id = "map";
	        marker_sk.header.stamp = ros::Time::now();
	        marker_sk.ns = "Point_A";
	        marker_sk.id = 0;
	        marker_sk.action = visualization_msgs::Marker::ADD; 
	        marker_sk.type = visualization_msgs::Marker::CUBE;
	        marker_sk.pose.position.x = SK[0];
	        marker_sk.pose.position.y = SK[1];
	        marker_sk.pose.position.z=0;
	        marker_sk.pose.orientation.x=0;
	       	marker_sk.pose.orientation.y=0;
	       	marker_sk.pose.orientation.z=0;
	       	marker_sk.pose.orientation.w=1;
	        marker_sk.scale.x = 1;
	    	marker_sk.scale.y = 1;
	    	marker_sk.scale.z = 1;
	    	marker_sk.color.a = 1.0;
	        marker_sk.color.r = 0.0f;
	        marker_sk.color.g = 1.0f;
	        marker_sk.color.b = 0.0f;

	        //Circle1
	        circle1.header.frame_id ="map";
	        circle1.header.stamp = ros::Time::now();
	        circle1.id = 0;
	        circle1.ns="Circle1";
	        circle1.action = visualization_msgs::Marker::ADD; 
	        circle1.type = visualization_msgs::Marker::LINE_STRIP;
	        circle1.pose.orientation.w=1;
	        circle1.scale.x = 0.2;
	    	//points.scale.y = 0.2;
	    	circle1.color.a = 1.0;
	        circle1.color.r = 1.0f;
	        circle1.color.g = 0.0f;
	        circle1.color.b = 0.0f;
	        for (double i = 0.0; i < 1.1; i+=0.01)
	        {
	        	double x = SK[0] + r * cos(2*i*M_PI);
		        double y = SK[1] + r * sin(2*i*M_PI);
		        geometry_msgs::Point p;
		        p.x = x;
		        p.y = y;
		        p.z=0;
		        circle1.points.push_back(p);

		    }

		    //Circle 2
	        circle2.header.frame_id ="map";
	        circle2.header.stamp = ros::Time::now();
	        circle2.id = 0;
	        circle2.ns="Circle2";
	        circle2.action = visualization_msgs::Marker::ADD; 
	        circle2.type = visualization_msgs::Marker::LINE_STRIP;
	        circle2.pose.orientation.w=1;
	        circle2.scale.x = 0.2;
	    	circle2.color.a = 1.0;
	        circle2.color.r = 0.0f;
	        circle2.color.g = 1.0f;
	        circle2.color.b = 0.0f;
	        for (double i = 0.0; i < 1.1; i+=0.01)
	        {
	        	double x = SK[0] + r_c * cos(2*i*M_PI);
		        double y = SK[1] + r_c * sin(2*i*M_PI);
		        geometry_msgs::Point p;
		        p.x = x;
		        p.y = y;
		        p.z=0;
		        circle2.points.push_back(p);

		    }

		    pointsk_pub.publish(marker_sk);
		    circle1_pub.publish(circle1);
		    circle2_pub.publish(circle2);

	    }
	    	ros::spinOnce();
        	loop_rate.sleep();
    }
    return 0;
}