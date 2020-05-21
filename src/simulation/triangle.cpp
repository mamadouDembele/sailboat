// this file is the line following code of the sailboat

#include "ros/ros.h"
#include "eigen3/Eigen/Dense"
#include <vector>
#include <math.h>
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "tf/tf.h"
#include <algorithm>
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"


using namespace std;

Eigen::Vector2d m; // Position of the boat
Eigen::Vector2d a={-40,30},b={40,25}, c={-38,-25}; // the ligne we want to follow
double r=10, zeta=M_PI/4, urmax=M_PI/4;// rayon de couloir, l'angle de près, l'angle maximale du gouvernail
double theta=0, psi_w=0; // cap de la voile, l'angle du vent
double q=0; 
double q_hys=1;// valeur d'hystérésis
double ur,us;
double ks=1.0; // constante k pour regler l'angle de la sail
double c_ligne=1.5*2/M_PI; // Constance pour rendre la ligne plus attractive
tf::Quaternion q_sail={0,0,0,1}, q_wind={0,0,0,1};
bool initStateFinie=true;


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

void controler_line(Eigen::Vector2d m, double theta, double psi_w, Eigen::Vector2d a, Eigen::Vector2d b, double& ur, double& us, double& q_hys)
{
	Eigen::Vector2d diff=b-a;
	double nor=norme(diff);
	double e=((b-a)[0]*(m-a)[1]-(b-a)[1]*(m-a)[0])/nor;
	double phi=angle(b-a);
	double theta_bar;
	theta_bar=phi-c_ligne*atan(e/r);
	if (fabs(e)>r/2)
	{
		q_hys=sign(e);
	}

	if ((cos(psi_w-theta_bar)+cos(zeta))<0 || (fabs(e)<r && ((cos(psi_w-phi)+cos(zeta))<0)))
	{
		theta_bar=M_PI+psi_w - q_hys*zeta;
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
	ros::init(argc, argv, "node_triangle");
    ros::NodeHandle n;
    ros::Subscriber pos_sail= n.subscribe("boat_pose", 1000, poseCallback);
    ros::Subscriber head_sail= n.subscribe("heading_boat", 1000, headCallback);
    ros::Subscriber wind= n.subscribe("wind_angle", 1000, windCallback);
    ros::Publisher trian_pub = n.advertise<visualization_msgs::Marker>( "visualization_triangle",0 );
    ros::Publisher com_servo = n.advertise<geometry_msgs::Vector3>("actuators", 1000);
    ros::Rate loop_rate(50);
    double t0 = ros::Time::now().toSec();
    while(ros::ok()){
    	geometry_msgs::Vector3 msg;
    	visualization_msgs::Marker marker;

    	double roll,pitch;
        tf::Matrix3x3(q_sail).getRPY(roll, pitch, theta);
        tf::Matrix3x3(q_wind).getRPY(roll, pitch, psi_w);

        //Initialisation of the state finite machine
        double  tf = ros::Time::now().toSec();

        if (tf-t0<0.3)
        {
        	ROS_INFO("Do anything");
        }
        //ros::Duration(1).sleep();
        else{
	        ROS_INFO("mx=%f", m[0]);
	        ROS_INFO("my=%f", m[1]);
	        if (initStateFinie==true)
			{
				Eigen::Vector2d diff1=m-a;
				double d1=norme(diff1);
				ROS_INFO("d1=%f", d1);
				Eigen::Vector2d diff2=m-b;
				double d2=norme(diff2);
				ROS_INFO("d2=%f", d2);
				Eigen::Vector2d diff3=m-c;
				double d3=norme(diff3);
				ROS_INFO("d3=%f", d3);
				if (d1==min(min(d1,d2),d3))
				{
					q=0;
					ROS_INFO("State 0");
				}
				
				if (d2==min(min(d1,d2),d3))
				{
					q=1;
					ROS_INFO("State 1");
				}

				if (d3==min((d1,d2),d3))
				{
					q=2;
					ROS_INFO("State 2");
				}
				initStateFinie=false;
			}

	        // State finite machine
	        if ((q==0 && (b-a)[0]*(b-m)[0]+(b-a)[1]*(b-m)[1]<0) || (q==0 && norme(b-m)<norme(b-a)/5))
	    	{
	    		ROS_INFO("State 1");
	    		q=1;
	    	}

	    	if ((q==1 && (c-b)[0]*(c-m)[0]+(c-b)[1]*(c-m)[1]<0) || (q==1 && norme(c-m)<norme(c-b)/5))
	    	{
	    		ROS_INFO("State 2");
	    		q=2;
	    	}

	    	if ((q==2 && (a-c)[0]*(a-m)[0]+(a-c)[1]*(a-m)[1]<0) || (q==2 && norme(a-m)<norme(a-c)/5))
	    	{
	    		ROS_INFO("State 0");
	    		q=0;
	    	}


	        if (q==0)
	        	controler_line(m, theta, psi_w, a, b, ur, us,q_hys);
	        if (q==1)
	        	controler_line(m, theta, psi_w, b, c, ur, us,q_hys);
	        if (q==2)
	        	controler_line(m, theta, psi_w, c, a, ur, us,q_hys);


	    	//regulation and publication
	    	
	    	msg.x=ur;
	    	msg.y=us;
	    	msg.z=0;
	    	com_servo.publish(msg);

	    	//visualisation
	    	for (int i=0; i<6; i++)
	    	{

		    	marker.header.frame_id = "map";
		        marker.header.stamp = ros::Time::now();
		        marker.ns = "triangle";
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
		       	tf::Quaternion q;
		        if (i==0)
		        {
			        marker.type = visualization_msgs::Marker::SPHERE;
			        marker.pose.position.x = a[0];
			        marker.pose.position.y = a[1];
			        marker.color.r = 1.0f;
			        marker.color.g = 0.0f;
			        marker.color.b = 0.0f;
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
			        marker.type = visualization_msgs::Marker::SPHERE;
			        marker.pose.position.x = c[0];
			        marker.pose.position.y = c[1];
			        marker.color.r = 0.0f;
			        marker.color.g = 0.0f;
			        marker.color.b = 1.0f;
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
			        marker.color.r = 0.0f;
			        marker.color.g = 1.0f;
			        marker.color.b = 1.0f;
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
			        marker.color.b = 1.0f;
		    	}
		        trian_pub.publish( marker );
		    }
		}
	    ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}