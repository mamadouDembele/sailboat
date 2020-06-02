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
#include "std_msgs/Float64.h"


using namespace std;

Eigen::Vector2d m; // Position of the boat
Eigen::Vector2d SK={0,0}; // the ligne we want to follow
double zeta=M_PI/4, urmax=M_PI/4, radius;// rayon de couloir, l'angle de près, l'angle maximale du gouvernail
double theta=0, psi_w=0; // cap de la voile, l'angle du vent
double q=0; 
double q_hys=1;// valeur d'hystérésis
double ur,us;
double ks=1.0; // constante k pour regler l'angle de la sail
double Gamma=M_PI/4; // Constance pour rendre la ligne plus attractive
tf::Quaternion q_sail, q_wind;
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
	double r=5;
	double nor=norme(b-a);
	double e=((b-a)[0]*(m-a)[1]-(b-a)[1]*(m-a)[0])/nor;
	double phi=angle(b-a);
	double theta_bar;
	theta_bar=phi-2*Gamma*atan(e/r)/M_PI;
	/*if (fabs(e)>r/2)
	{
		q_hys=sign(e);
	}

	if ((cos(psi_w-theta_bar)+cos(zeta))<0 || (fabs(e)<r && ((cos(psi_w-phi)+cos(zeta))<0)))
	{
		theta_bar=M_PI+psi_w - q_hys*zeta;
	}
	*/
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
    ros::Publisher com_servo = n.advertise<geometry_msgs::Vector3>("actuators", 1000);
    ros::Publisher pointa = n.advertise<geometry_msgs::Point>("a_triangle", 1000);
    ros::Publisher pointb = n.advertise<geometry_msgs::Point>("b_triangle", 1000);
    ros::Publisher pointc = n.advertise<geometry_msgs::Point>("c_triangle", 1000);
    ros::Publisher radius_r_pub = n.advertise<std_msgs::Float64>("radius_circle", 1000);
    ros::Publisher radius_rc_pub = n.advertise<std_msgs::Float64>("radius_circle_cuic", 1000);
    ros::Publisher pointsk = n.advertise<geometry_msgs::Point>("sk", 1000);

    n.param<double>("radius_r", radius, 0);
    ros::Rate loop_rate(100);
    double t0 = ros::Time::now().toSec();
    while(ros::ok()){
    	geometry_msgs::Vector3 msg;
    	geometry_msgs::Point cA, cB, cC, cSK;
    	std_msgs::Float64 rad_rc, rad_r;
    	visualization_msgs::Marker marker;

    	double roll,pitch;
        tf::Matrix3x3(q_sail).getRPY(roll, pitch, theta);
        tf::Matrix3x3(q_wind).getRPY(roll, pitch, psi_w);

        //Initialisation of the state finite machine
        double  tf = ros::Time::now().toSec();

        if (tf-t0<0.5)
        {
        	ROS_INFO("Do anything");
        }
        //ros::Duration(1).sleep();
        else{

        	Eigen::Matrix2d Rot;
			Rot << cos(psi_w), -sin(psi_w), 
				sin(psi_w),cos(psi_w);

			double R=radius/2; // this parameter can change

			Eigen::Vector2d a_r1={R, 0}, b_r1={-R/2, R*sqrt(3)/2}, c_r1={-R/2, -R*sqrt(3)/2};

			// In the global frame
			Eigen::Vector2d a=Rot*a_r1+SK, c=Rot*c_r1+SK,b=Rot*b_r1+SK;

	        if (initStateFinie==true)
			{
				double d1=norme(m-a);
				ROS_INFO("d1=%f", d1);
				double d2=norme(m-b);
				ROS_INFO("d2=%f", d2);
				double d3=norme(m-c);
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
	        if ((q==0 && (b-a)[0]*(b-m)[0]+(b-a)[1]*(b-m)[1]<0) || (q==0 && norme(m-b)<norme(a-b)/20))
	    	{
	    		ROS_INFO("State 1");
	    		q=1;
	    	}

	    	if ((q==1 && (c-b)[0]*(c-m)[0]+(c-b)[1]*(c-m)[1]<0) || (q==1 && norme(m-c)<norme(c-b)/20))
	    	{
	    		ROS_INFO("State 2");
	    		q=2;
	    	}

	    	if ((q==2 && (a-c)[0]*(a-m)[0]+(a-c)[1]*(a-m)[1]<0) || (q==2 && norme(m-a)<norme(c-a)/20))
	    	{
	    		ROS_INFO("State 0");
	    		q=0;
	    	}


	        if (q==0)
	        	controler_line(m, theta, psi_w, a, b, ur, us, q_hys);
	        if (q==1)
	        	controler_line(m, theta, psi_w, b, c, ur, us, q_hys);
	        if (q==2)
	        	controler_line(m, theta, psi_w, c, a, ur, us, q_hys);

	    	
	    	msg.x=ur;
	    	msg.y=us;
	    	msg.z=0;
	    	com_servo.publish(msg);

	    	cA.x=a[0];
	    	cA.y=a[1];
	    	cA.z=0;
	    	cSK.x=SK[0];
	    	cSK.y=SK[1];
	    	cSK.z=0;
	    	cB.x=b[0];
	    	cB.y=b[1];
	    	cB.z=0;
	    	cC.x=c[0];
	    	cC.y=c[1];
	    	cC.z=0;
	    	rad_rc.data=R;
	    	rad_r.data=radius;

	    	pointa.publish(cA);
	    	pointb.publish(cB);
	    	pointc.publish(cC);
	    	radius_rc_pub.publish(rad_rc);
	    	radius_r_pub.publish(rad_r);
	    	pointsk.publish(cSK);

		}
	    ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}