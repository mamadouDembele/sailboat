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

Eigen::Vector2d m={0,0}; // Position of the boat
Eigen::Vector2d a={14,9},b={-23,-18}; // the ligne we want to follow
double r=10, zeta=M_PI/4, urmax=M_PI/4;// rayon de couloir, l'angle de près, l'angle maximale du gouvernail
double theta=0, psi_w=0; // cap de la voile, l'angle du vent
double q=1; // valeur d'hystérésis 
double ur,us;
double ks=1.0; // constante k pour regler l'angle de la sail
double c=1.5*2/M_PI; // Constance pour rendre la ligne plus attractive
tf::Quaternion q_sail={0,0,0,1}, q_wind={0,0,0,1};

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
	double nor=norme(b-a);
	double e=((b-a)[0]*(m-a)[1]-(b-a)[1]*(m-a)[0])/nor;
	double phi=angle(b-a);
	double theta_bar;
	theta_bar=phi-c*atan(e/r);
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
    ros::Publisher line_pub = n.advertise<visualization_msgs::Marker>( "visualization_line",0 );
    ros::Publisher com_servo = n.advertise<geometry_msgs::Vector3>("actuators", 1000);
    ros::Rate loop_rate(50);
    while(ros::ok()){
    	geometry_msgs::Vector3 msg;
    	visualization_msgs::Marker marker;

    	double roll,pitch;
        tf::Matrix3x3(q_sail).getRPY(roll, pitch, theta);
        tf::Matrix3x3(q_wind).getRPY(roll, pitch, psi_w);

    	//regulation and publication
    	controler_line(m, theta, psi_w, a, b, ur, us,q);
    	msg.x=ur;
    	msg.y=us;
    	msg.z=0;
    	com_servo.publish(msg);

    	//visualisation
    	for (int i=0; i<3; i++)
    	{

	    	marker.header.frame_id = "map";
	        marker.header.stamp = ros::Time::now();
	        marker.ns = "line";
	        marker.id = i;
	        marker.action = visualization_msgs::Marker::ADD;
	        marker.scale.x = 1;
	        marker.scale.y = 1;
	        marker.scale.z = 1;
	        marker.color.a = 1.0;
	        marker.pose.position.z=0;
	        marker.pose.orientation.x=0;
	       	marker.pose.orientation.y=0;
	       	marker.pose.orientation.z=0;
	       	marker.pose.orientation.w=1;
	        if (i==0)
	        {
		        marker.type = visualization_msgs::Marker::SPHERE;
		        marker.pose.position.x = a[0];
		        marker.pose.position.y = a[1];
		        marker.color.r = 1.0f;
		        marker.color.g = 1.0f;
		        marker.color.b = 1.0f;
	    	}

	    	if (i==1)
	        {
		        marker.type = visualization_msgs::Marker::SPHERE;
		        marker.pose.position.x = b[0];
		        marker.pose.position.y = b[1];
		        marker.color.r = 0.0f;
		        marker.color.g = 1.0f;
		        marker.color.b = 0.0f;
	    	}

	    	if (i==2)
	        {
		        marker.type = visualization_msgs::Marker::ARROW;
		        marker.scale.x = 6*norme(b-a);
		        marker.scale.z = 0.1;
		        marker.pose.position.x = a[0]-30*(b-a)[0]/norme(b-a);
		        marker.pose.position.y = a[1]-30*(b-a)[1]/norme(b-a);
		       	tf::Quaternion q;
        		q.setRPY(0, 0, angle(b-a));
        		tf::quaternionTFToMsg(q, marker.pose.orientation);
		        marker.color.r = 0.0f;
		        marker.color.g = 0.0f;
		        marker.color.b = 1.0f;
	    	}
	        line_pub.publish( marker );
	    }

    	ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}