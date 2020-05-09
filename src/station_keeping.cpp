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
double q;
bool initStateFinie=true;
Eigen::Vector2d m={0,0};
double r=10, zeta=M_PI/4, urmax=M_PI/4;
double theta=0, psi_w=0;
double c=1.5*2/M_PI;
double ks=1.0;
double vist;
double us, ur;
double q_h=1;
tf::Quaternion q_sail={0,0,0,1}, q_wind={0,0,0,1};
//---------------------------------------------//

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
}


void controler_line(Eigen::Vector2d m, double theta, double psi_w, Eigen::Vector2d a, Eigen::Vector2d b, double& ur, double& us, double& q_h)
{
	double nor=norme(b-a);
	double e=((b-a)[0]*(m-a)[1]-(b-a)[1]*(m-a)[0])/nor;
	double phi=angle(b-a);
	double theta_bar;
	theta_bar=phi-c*atan(e/r);
	if (fabs(e)>r/2)
	{
		q_h=sign(e);
	}

	if ((cos(psi_w-theta_bar)+cos(zeta))<0 || (fabs(e)<r && ((cos(psi_w-phi)+cos(zeta))<0)))
	{
		theta_bar=M_PI+psi_w - q_h*zeta;
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

void controller_circle(Eigen::Vector2d m, double theta, double psi_w, Eigen::Matrix2d D, Eigen::Vector2d c, double& ur, double& us)
{
	Eigen::Vector2d v;
	f(m, D, c, v);
	double theta_bar=angle(v);
	ur=2*atan(tan(0.5*(theta-theta_bar)));
	us=ks*(M_PI/4)*(cos(psi_w-theta_bar)+1);
	//us=(norme(A)-vist);
	//ROS_INFO("Tout va bien ur=%f, us=%f",ur,us);

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
	ros::init(argc, argv, "node_keeping");
    ros::NodeHandle n;
    ros::Subscriber pos_sail= n.subscribe("boat_pose", 1000, poseCallback);
    ros::Subscriber head_sail= n.subscribe("heading_boat", 1000, headCallback);
    ros::Subscriber wind= n.subscribe("wind_angle", 1000, windCallback);
    ros::Publisher keeping_pub = n.advertise<visualization_msgs::Marker>( "visualization_Skeeping",0 );
    ros::Publisher com_servo = n.advertise<geometry_msgs::Vector3>("actuators", 1000);

    ros::Rate loop_rate(50);
    while(ros::ok()){
    	geometry_msgs::Vector3 msg;
    	visualization_msgs::Marker marker;

    	double roll,pitch;
        tf::Matrix3x3(q_sail).getRPY(roll, pitch, theta);
        tf::Matrix3x3(q_wind).getRPY(roll, pitch, psi_w);
    	double alpha=M_PI/6;
    	Eigen::Vector2d SK={18,20};// Station keeping point

    	Eigen::Matrix2d R;
		R << sin(psi_w), cos(psi_w), 
			-cos(psi_w),sin(psi_w);

		double l=30;

		Eigen::Vector2d a_r1={-l,-l*tan(alpha)},d_r1={l,-l*tan(alpha)};
		Eigen::Vector2d c_r1={l,l*tan(alpha)},b_r1={-l,l*tan(alpha)};
		Eigen::Vector2d c1_r1={l,0},c2_r1={-l,0};

		// In the global frame
		Eigen::Vector2d a=R*a_r1+SK,d=R*d_r1+SK;
		Eigen::Vector2d c=R*c_r1+SK,b=R*b_r1+SK;
		Eigen::Vector2d c1=R*c1_r1+SK,c2=R*c2_r1+SK;

		double r_c=l*tan(alpha);
		Eigen::Matrix2d Ra;
		Ra << r_c, 0, 
			0,r_c;

		Eigen::Matrix2d Conter1, Conter2;
		Conter1 << 1,0,
			0,1;
		Conter2 << 1,0,
			0,-1;

		Eigen::Matrix2d D1=Ra*Conter1;
		Eigen::Matrix2d D2=Ra*Conter2;

		//-------------------------------------------//

		// Initialisation de la machine d'Etat

		if (initStateFinie==true)
		{
			double d1=norme(m-c);
			double d2=norme(m-b);
			if (d1<d2)
			{
				q=2;
				ROS_INFO("State 2");
			}
			else
			{
				q=0;
				ROS_INFO("State 0");
			}
			initStateFinie=false;
		}

		// Machine d'état
    	if (q==0 && (d-b)[0]*(d-m)[0]+(d-b)[1]*(d-m)[1]<0)
    	{
    		ROS_INFO("State 1");
    		q=1;
    	}

    	// (c1-d)[0]*(c1-m)[0]+(c1-d)[1]*(c1-m)[1]<0 use this condition on the boat
    	if (q==1 && sqrt(pow((c-m)[0],2)+pow((c-m)[1],2))<r_c/2)
    	{
    		ROS_INFO("State 2");
    		q=2;
    	}
    	if (q==2 && (a-c)[0]*(a-m)[0]+(a-c)[1]*(a-m)[1]<0)
    	{
    		ROS_INFO("State 3");
    		q=3;
    	}

    	//(c2-a)[0]*(b-m)[0]+(b-c2)[1]*(b-m)[1]<0 use this condition on the boat
    	if (q==3 && sqrt(pow((b-m)[0],2)+pow((b-m)[1],2))<r_c/2)
    	{
    		ROS_INFO("State 0");
    		q=0;
    	}

    	if (q==0)
    		controler_line(m, theta, psi_w, b, d, ur, us, q_h);
    	if (q==1)
    		controller_circle(m, theta, psi_w, D1, c1, ur, us);
    	if (q==2)
    		controler_line(m, theta, psi_w, c, a, ur, us, q_h);
    	if (q==3)
    		controller_circle(m, theta, psi_w, D2, c2, ur, us);

    	msg.x=ur;
    	msg.y=us;
    	msg.z=0;
    	com_servo.publish(msg);
    	
    	//visualisation
    	for (int i=0; i<8; i++)
    	{

	    	marker.header.frame_id = "map";
	        marker.header.stamp = ros::Time::now();
	        marker.ns = "Skeeping";
	        marker.id = i;
	        marker.action = visualization_msgs::Marker::ADD;
	        marker.pose.orientation.x=0;
		    marker.pose.orientation.y=0;
		    marker.pose.orientation.z=0;
		    marker.pose.orientation.w=1;
		    marker.scale.x = 1;
	        marker.scale.y = 1;
	        marker.scale.z = 1;
	        marker.color.a = 1.0;
	        marker.pose.position.z=0;
	        marker.type = visualization_msgs::Marker::SPHERE;
	        marker.color.r = 1.0f;
		    marker.color.g = 1.0f;
		    marker.color.b = 1.0f;
	        if (i==0)
	        {
		        marker.pose.position.x = a[0];
		        marker.pose.position.y = a[1];
		        marker.color.r = 1.0f;
		    	marker.color.g = 0.0f;
		    	marker.color.b = 0.0f;
	    	}

	    	if (i==1)
	        {
		        marker.pose.position.x = b[0];
		        marker.pose.position.y = b[1];
		        marker.color.r = 1.0f;
		        marker.color.g = 0.5f;
		        marker.color.b = 0.0f;
	    	}

	    	if (i==2)
	        {
		        marker.pose.position.x = c[0];
		        marker.pose.position.y = c[1];
		        marker.color.r = 1.0f;
		        marker.color.g = 1.0f;
		        marker.color.b = 0.0f;
	    	}

	    	if (i==3)
	        {
		        marker.pose.position.x = d[0];
		        marker.pose.position.y = d[1];
		        marker.color.r = 0.0f;
		        marker.color.g = 1.0f;
		        marker.color.b = 0.0f;
	    	}

	    	if (i==4)
	        {
		        marker.pose.position.x = c1[0];
		        marker.pose.position.y = c1[1];
		        marker.color.r = 1.0f;
		        marker.color.g = 0.0f;
		        marker.color.b = 1.0f;
		        
	    	}

	    	if (i==5)
	        {
		        marker.pose.position.x = c2[0];
		        marker.pose.position.y = c2[1];
		        marker.color.r = 0.0f;
		        marker.color.g = 1.0f;
		        marker.color.b = 1.0f;
	    	}

	    	if (i==6)
	        {
		        marker.type = visualization_msgs::Marker::ARROW;
		        marker.scale.x = norme(d-b);
		        marker.scale.y = 1.0;
		        marker.scale.z = 0.1;
		        marker.color.a = 1.0;
		        marker.pose.position.x = b[0];
		        marker.pose.position.y = b[1];
		       	tf::Quaternion q;
		       	double phi=angle(d-b);
        		q.setRPY(0, 0, phi);
        		tf::quaternionTFToMsg(q, marker.pose.orientation);
		        marker.color.r = 0.0f;
		        marker.color.g = 1.0f;
		        marker.color.b = 0.0f;
	    	}

	    	if (i==7)
	        {
		        marker.type = visualization_msgs::Marker::ARROW;
		        marker.scale.x = norme(a-c);
		        marker.scale.y = 1.0;
		        marker.scale.z = 0.1;
		        marker.color.a = 1.0;
		        marker.pose.position.x = c[0];
		        marker.pose.position.y = c[1];
		       	tf::Quaternion q;
		       	double phi=angle(a-c);
        		q.setRPY(0, 0, phi);
        		tf::quaternionTFToMsg(q, marker.pose.orientation);
		        marker.color.r = 0.0f;
		        marker.color.g = 1.0f;
		        marker.color.b = 1.0f;
	    	}

	        keeping_pub.publish( marker );
	    }

    	ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}