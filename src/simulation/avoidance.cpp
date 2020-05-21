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
bool initStateFinie=true, cond=true, cond2=true, new_sk;
Eigen::Vector2d m, SK;
double r, zeta=M_PI/4, urmax=M_PI/4, q=0, q_h=1;
double theta, psi_w;
double kc=4*2/M_PI, ks=1.0;
double us, ur;
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


//----------controller-------------------//

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
	v+=3*(m-c)/pow(norme(m-c),3);
}


void controler_line(Eigen::Vector2d m, double theta, double psi_w, Eigen::Vector2d a, Eigen::Vector2d b, double& ur, double& us, double& q_h, double ks)
{
	double nor=norme(b-a);
	double e=((b-a)[0]*(m-a)[1]-(b-a)[1]*(m-a)[0])/nor;
	double phi=angle(b-a);
	double theta_bar;
	theta_bar=phi-kc*atan(e/r);
	if (fabs(e)>r/2)
	{
		q_h=sign(e);
	}

	/*if ((cos(psi_w-theta_bar)+cos(zeta))<0 )
	{
		ROS_INFO("je suis dans la no go zone");
		theta_bar=M_PI+psi_w - q_h*zeta;
	}*/
	
	if (cos(theta-theta_bar)>=0)
	{
		ur=urmax*sin(theta-theta_bar);
	}
	else
	{
		ur=urmax*sign(sin(theta-theta_bar));
	}
	//us=ks*(M_PI/4)*(cos(psi_w-theta_bar)+1);
	us=ks*M_PI/4;

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
	us=ks*(M_PI/4)*(cos(psi_w-theta_bar)+1);
	//us=(norme(A)-vist);
	//ROS_INFO("Tout va bien ur=%f, us=%f",ur,us);

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

/*void new_skCallback(const std_msgs::Bool::ConstPtr& msg) {
    new_sk=msg->data;
}*/

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
    ros::Publisher pointc1 = n.advertise<geometry_msgs::Point>("c1", 1000);
    ros::Publisher pointa = n.advertise<geometry_msgs::Point>("a", 1000);
    ros::Publisher pointb = n.advertise<geometry_msgs::Point>("b", 1000);
    ros::Publisher pointc = n.advertise<geometry_msgs::Point>("c", 1000);
    ros::Publisher pointd = n.advertise<geometry_msgs::Point>("d", 1000);
    ros::Publisher radius_rc = n.advertise<std_msgs::Float64>("radius_rc", 1000);
    ros::Publisher radius_pub = n.advertise<std_msgs::Float64>("radius", 1000);
    ros::Publisher pointsk = n.advertise<geometry_msgs::Point>("sk", 1000);
    ros::Publisher time_pub = n.advertise<std_msgs::Float64>("time_stay", 1000);
    ros::Subscriber sub_sk= n.subscribe("sk_point", 1000, skCallback);
    //ros::Subscriber sub_q= n.subscribe("q", 1000, new_skCallback);


	//-----------select the mode------------//
    n.param<int>("mode_sk",mode,1);
    if (mode==0)
    {
    	n.param<double>("SKx", SK[0], 0);
    	n.param<double>("SKy", SK[1], 0);
    	n.param<double>("radius_r", r, 0);
    	ROS_INFO("mode 0"); // this mode is use to test the station keeping's algorithms
    }

    else{
    	
    	ROS_INFO("mode 1"); // this mode is use for the challenge 2
    }

    ros::Rate loop_rate(100);
    double t0 = ros::Time::now().toSec();
    while(ros::ok()){
    	geometry_msgs::Vector3 msg;
    	geometry_msgs::Point cA, cB, cC, cD, cC1, cC2, cSK, msg_c;
    	std_msgs::Float64 radius, rad_r, Ts;
    	visualization_msgs::Marker marker, points, points1, points2;

    	double roll,pitch;
        tf::Matrix3x3(q_sail).getRPY(roll, pitch, theta);
        tf::Matrix3x3(q_wind).getRPY(roll, pitch, psi_w);

    	Eigen::Matrix2d R;
		R << cos(psi_w), -sin(psi_w), 
			sin(psi_w),cos(psi_w);

		double l=r/3, L=r/2; // this parameter can change

		Eigen::Vector2d a_r1={-l, -L/2},b_r1={l, -L/2-l*tan(M_PI/3)};
		Eigen::Vector2d c_r1={l, L/2},d_r1={-l, L/2+l*tan(M_PI/3)};
		Eigen::Vector2d c1_r1={0,-L/3};

		// In the global frame
		Eigen::Vector2d a=R*a_r1+SK,d=R*d_r1+SK;
		Eigen::Vector2d c=R*c_r1+SK,b=R*b_r1+SK;
		Eigen::Vector2d c1=R*c1_r1+SK;

		double r_c=l/2;
		Eigen::Matrix2d Ra;
		Ra << r_c, 0, 
			0,r_c;

		Eigen::Matrix2d Conter;
		Conter << 1,0,
			0,1;

		Eigen::Matrix2d D=Ra*Conter;

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
				double d1=norme(m-b);
				double d2=norme(m-c);
				double d3=norme(m-d);
				if (d1==min(min(d1,d2),d3))
				{
					q=2;
					ROS_INFO("State 2");
				}

				if (d2==min(min(d1,d2),d3))
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
	    		controler_line(m, theta, psi_w, d, a, ur, us, q_h, 1.0);
	    	if (q==1)
	    		controler_line(m, theta, psi_w, a, b, ur, us, q_h, 1.0);
	    	if (q==2)
	    		controler_line(m, theta, psi_w, b, c, ur, us, q_h, 1.0);
	    	if (q==3)
	    		controler_line(m, theta, psi_w, c, d, ur, us, q_h, 1.0);

	    	//-------------------------------//
	    	/*if (mode !=0)
	    	{
	    		double ts, tsf, t_s;
	    		if (norme(m-SK)<r && cond==true)
		    	{	
		    		ROS_INFO("we enter in the circle");
		    		ts=ros::Time::now().toSec();
		    		//ROS_INFO("ts=%f",ts);
		    		cond=false;
		    	}

		    	if (norme(m-SK)<r){
		    		tsf=ros::Time::now().toSec();
		    	}

		    	if (cond==false){
		    		t_s=tsf-ts;
		    		Ts.data=t_s;
		    		time_pub.publish(Ts);
		    	}

		    	//-------we change the station keeping point--------//
		    	if (new_sk==true && cond2==true)
		    	{
		    		initStateFinie=true;
		    		cond2=false;
		    	}
	    	}*/
			
			//-------publication of the command-------//
	    	msg.x=ur;
	    	msg.y=us;
	    	msg.z=0;
	    	com_servo.publish(msg);

	    	//--------publication for the visualisation---------//
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
	    	cD.x=d[0];
	    	cD.y=d[1];
	    	cD.z=0;
	    	cC1.x=c1[0];
	    	cC1.y=c1[1];
	    	cC1.z=0;
	    	radius.data=r_c;
	    	rad_r.data=r;

	    	pointa.publish(cA);
	    	pointb.publish(cB);
	    	pointc.publish(cC);
	    	pointc1.publish(cC1);
	    	pointd.publish(cD);
	    	radius_rc.publish(radius);
	    	radius_pub.publish(rad_r);
	    	pointsk.publish(cSK);
	    }
    	ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}