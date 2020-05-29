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
#include "std_msgs/Bool.h"

using namespace std;
bool initStateFinie=true;
Eigen::Vector2d m, SK;
double r, zeta=M_PI/4, urmax=M_PI/4, q=0, q_h=1;
double theta, psi_w;
double ks=1.0;
double us, ur;
double Gamma=M_PI/4; // Constance pour rendre la ligne plus attractive
tf::Quaternion q_sail, q_wind;
int mode;


//-----Fonction-----//

double sign(double x)
{
    if (x<0)
        return -1;
    else
        return 1;
}

double norme(Eigen::Vector2d x)
{
    return sqrt(pow(x[0],2)+pow(x[1],2));
}

double angle(Eigen::Vector2d x)
{
    return atan2(x[1],x[0]);
}


void publication_point(geometry_msgs::Point& msg, Eigen::Vector2d a)
{
    msg.x=a[0];
    msg.y=a[1];
    msg.z=0;
}

void publication_float(std_msgs::Float64& msg, double a)
{
    msg.data=a;
}

//----------controller-------------------//

void controler_line(Eigen::Vector2d m, double theta, double psi_w, Eigen::Vector2d a, Eigen::Vector2d b, double& ur, double& us)
{
	double nor=norme(b-a);
	double e=((b-a)[0]*(m-a)[1]-(b-a)[1]*(m-a)[0])/nor;
	double phi=angle(b-a);
	double theta_bar;
	theta_bar=phi-2*Gamma*atan(e/2)/M_PI;
	
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



//-------fonction Callback------------//

void poseCallback(const geometry_msgs::Point::ConstPtr& msg) {
    m[0]=msg->x;
    m[1]=msg->y;
}

void skCallback(const geometry_msgs::Point::ConstPtr& msg) {
    SK[0]=msg->x;
    SK[1]=msg->y;
}

void rCallback(const std_msgs::Float64::ConstPtr& msg) {
    r=msg->data;
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


//------------fonction main-------------------//

int main(int argc, char **argv)
{   
	ros::init(argc, argv, "node_avoidance");
    ros::NodeHandle n;
    ros::Subscriber pos_sail= n.subscribe("boat_pose", 1000, poseCallback);
    ros::Subscriber head_sail= n.subscribe("heading_boat", 1000, headCallback);
    ros::Subscriber wind= n.subscribe("wind_angle", 1000, windCallback);
    ros::Publisher com_servo = n.advertise<geometry_msgs::Vector3>("actuators", 1000);
    ros::Publisher pointa = n.advertise<geometry_msgs::Point>("a_avoid", 1000);
    ros::Publisher pointb = n.advertise<geometry_msgs::Point>("b_avoid", 1000);
    ros::Publisher pointc = n.advertise<geometry_msgs::Point>("c_avoid", 1000);
    ros::Publisher pointd = n.advertise<geometry_msgs::Point>("d_avoid", 1000);
    ros::Publisher radius_pub = n.advertise<std_msgs::Float64>("radius_r", 1000);
    ros::Publisher pointsk = n.advertise<geometry_msgs::Point>("sk", 1000);

	n.param<double>("SKx", SK[0], 0);
	n.param<double>("SKy", SK[1], 0);
	n.param<double>("radius_r", r, 0);

    ros::Rate loop_rate(300);
    double t0 = ros::Time::now().toSec();
    while(ros::ok()){
    	geometry_msgs::Vector3 msg;
    	geometry_msgs::Point cA, cB, cC, cD, cSK;
    	std_msgs::Float64 radius;
 
    	double roll,pitch;
        tf::Matrix3x3(q_sail).getRPY(roll, pitch, theta);
        tf::Matrix3x3(q_wind).getRPY(roll, pitch, psi_w);

    	Eigen::Matrix2d R;
		R << cos(psi_w), -sin(psi_w), 
			sin(psi_w),cos(psi_w);

		double l=r/3, L=r/3; // this parameter can change

		Eigen::Vector2d a_r1={-l, -L/2},b_r1={l, -L/3-l*tan(M_PI/3)};
		Eigen::Vector2d c_r1={l, L/2},d_r1={-l, L/3+l*tan(M_PI/3)};

		// In the global frame
		Eigen::Vector2d a=R*a_r1+SK,d=R*d_r1+SK;
		Eigen::Vector2d c=R*c_r1+SK,b=R*b_r1+SK;

		//-------------------------------------------//

		// Initialisation de la machine d'Etat
		double  tf = ros::Time::now().toSec();

		if (tf-t0<0.3)
        {
        	ROS_INFO("Do anything");
        }
        
        else{

			if (initStateFinie==true)
			{
				double d0=norme(m-a);
				double d1=norme(m-b);
				double d2=norme(m-c);
				double d3=norme(m-d);
				if (d0==min(min(d0,d1),min(d2,d3)))
				{
					q=1;
					ROS_INFO("State 1");
				}

				if (d1==min(min(d0,d1),min(d2,d3)))
				{
					q=2;
					ROS_INFO("State 2");
				}

				if (d2==min(min(d0,d1),min(d2,d3)))
				{
					q=3;
					ROS_INFO("State 3");
				}

				else
				{
					q=0;
					ROS_INFO("State 0");
				}
				initStateFinie=false;
			}

			// Machine d'état
	    	if ((q==0 && (a-d)[0]*(a-m)[0]+(a-d)[1]*(a-m)[1]<0) || (q==0 && norme(a-m)<norme(a-d)/3))
	    	{
	    		ROS_INFO("State 1");
	    		q=1;
	    	}

	    	if ((q==1 && (b-a)[0]*(b-m)[0]+(b-a)[1]*(b-m)[1]<0) || (q==1 && norme(b-m)<norme(b-a)/3))
	    	{
	    		ROS_INFO("State 2");
	    		q=2;
	    	}
	    	if ((q==2 && (c-b)[0]*(c-m)[0]+(c-b)[1]*(c-m)[1]<0) || (q==2 && (norme(c-m)<norme(c-b)/3)) )
	    	{
	    		ROS_INFO("State 3");
	    		q=3;
	    	}

	    	if ((q==3 && (d-c)[0]*(d-m)[0]+(d-c)[1]*(d-m)[1]<0) || (q==3 && (norme(d-m)<norme(d-c)/3)))
	    	{
	    		ROS_INFO("State 0");
	    		q=0;
	    	}

	    	if (q==0)
	    		controler_line(m, theta, psi_w, d, a, ur, us);
	    	if (q==1)
	    		controler_line(m, theta, psi_w, a, b, ur, us);
	    	if (q==2)
	    		controler_line(m, theta, psi_w, b, c, ur, us);
	    	if (q==3)
	    		controler_line(m, theta, psi_w, c, d, ur, us);

	    	
			//-------publication of the command-------//
	    	msg.x=ur;
	    	msg.y=us;
	    	msg.z=0;
	    	com_servo.publish(msg);

	    	//--------publication for the visualisation---------//
	    	publication_point(cA, a);
            publication_point(cSK, SK);
            publication_point(cB, b);
            publication_point(cC, c);
            publication_point(cD, d);
            publication_float(radius, r);

	    	pointa.publish(cA);
	    	pointb.publish(cB);
	    	pointc.publish(cC);
	    	pointd.publish(cD);
	    	radius_pub.publish(radius);
	    	pointsk.publish(cSK);
	    }
    	ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}