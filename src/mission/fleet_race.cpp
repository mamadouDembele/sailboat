
#include "ros/ros.h"
#include "eigen3/Eigen/Dense"
#include <vector>
#include <math.h>
#include <algorithm>
#include "std_msgs/Float64.h"
#include "geometry_msgs/Point.h"
#include "tf/tf.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"


using namespace std;

Eigen::Vector2d m; // Position of the boat
Eigen::Vector2d b1, b2, b3, sl, fl;
Eigen::Vector3d couleur={0,0,0};
Eigen::Vector2d a,b,v;
double r=10, r_valid, zeta=M_PI/4, urmax=M_PI/4;// rayon de couloir, l'angle de près, l'angle maximale du gouvernail
double theta, psi_w; // cap de la voile, l'angle du vent
double q_h=1, q=0; // valeur d'hystérésis 
double ur,us;
double ks=1.0; // constante k pour regler l'angle de la sail
double c=1.5*2/M_PI; // Constance pour rendre la ligne plus attractive
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

void controler_line(Eigen::Vector2d m, double theta, double psi_w, Eigen::Vector2d a, Eigen::Vector2d b, double& ur, double& us, double& q_h)
{
	Eigen::Vector2d diff=b-a;
	double nor=norme(diff);
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

void trajectory(Eigen::Vector2d sl, Eigen::Vector2d b1, Eigen::Vector2d b2, Eigen::Vector2d b3, Eigen::Vector2d& a, Eigen::Vector2d& b, Eigen::Vector2d& v)
{
    double d1=norme(b1-sl), d2=norme(b2-sl), d3=norme(b3-sl);
    double d21, d22;
    if (d1==min(min(d1,d2),d3)){
        a=b1;
        d21=norme(b2-a);
        d22=norme(b3-a);
        if (d21==min(d21, d22)){
            b=b2;
            v=b3;
        }
        else{
            b=b3;
            v=b2;
        }
    }
    if (d2==min(min(d1,d2),d3)){
        a=b2;
        d21=norme(b1-a);
        d22=norme(b3-a);
        if (d21==min(d21, d22)){
            b=b1;
            v=b3;
        }
        else{
            b=b3;
            v=b1;
        }
    }
    if (d3==min(min(d1,d2),d3)){
        a=b3;
        d21=norme(b1-a);
        d22=norme(b2-a);
        if (d21==min(d21, d22)){
            b=b1;
            v=b2;
        }
        else{
            b=b2;
            v=b1;
        }
    }

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
	ros::init(argc, argv, "node_fleet_race");
    ros::NodeHandle n;
    ros::Subscriber pos_sail= n.subscribe("boat_pose", 1000, poseCallback);
    ros::Subscriber head_sail= n.subscribe("heading_boat", 1000, headCallback);
    ros::Subscriber wind= n.subscribe("wind_angle", 1000, windCallback);
    ros::Publisher com_servo = n.advertise<geometry_msgs::Vector3>("actuators", 1000);
    ros::Publisher start_pub = n.advertise<geometry_msgs::Point>("start_line", 1000);
    ros::Publisher finish_pub = n.advertise<geometry_msgs::Point>("finish_line", 1000);
    ros::Publisher buoy1_pub = n.advertise<geometry_msgs::Point>("buoy1", 1000);
    ros::Publisher buoy2_pub = n.advertise<geometry_msgs::Point>("buoy2", 1000);
    ros::Publisher buoy3_pub = n.advertise<geometry_msgs::Point>("buoy3", 1000);
    ros::Publisher color_pub = n.advertise<geometry_msgs::Point>("color", 1000);
    ros::Publisher r_pub = n.advertise<std_msgs::Float64>("rad_r", 1000);

    n.param<double>("b1x", b1[0],0);
    n.param<double>("b1y", b1[1],0);
    n.param<double>("b2x", b2[0],0);
    n.param<double>("b2y", b2[1],0);
    n.param<double>("b3x", b3[0],0);
    n.param<double>("b3y", b3[1],0);
    n.param<double>("slx", sl[0],0);
    n.param<double>("sly", sl[1],0);
    n.param<double>("flx", fl[0],0);
    n.param<double>("fly", fl[1],0);
    n.param<double>("radius_r", r_valid,0);
    ros::Rate loop_rate(300);
    double t0 = ros::Time::now().toSec();
    while(ros::ok()){
        std_msgs::Float64 msg_r;
    	geometry_msgs::Vector3 msg;
        geometry_msgs::Point msg_sl, msg_fl, msg_b1, msg_b2, msg_b3, msg_c;

    	double roll,pitch;
        tf::Matrix3x3(q_sail).getRPY(roll, pitch, theta);
        tf::Matrix3x3(q_wind).getRPY(roll, pitch, psi_w);
        trajectory(sl, b1, b2, b3, a, b, v);

    	//regulation and publication

        //ROS_INFO("b1[0]=%f",b1[0]);
        //ROS_INFO("b1[0]=%f",b1[1]);

    	double  tf = ros::Time::now().toSec();

		if (tf-t0<0.3)
        {
        	//ROS_INFO("mx=%f", m[0]);
	        //ROS_INFO("my=%f", m[1]);
        	ROS_INFO("Do anything");
        }
        
        else{
            ROS_INFO("Value of q=%f", q);

            if (q==0 && norme(m-a)<r_valid)
            {
                q=1;
                couleur[0]=1;
                ROS_INFO("state 1");
            }

            if (q==1 && norme(m-b)<r_valid)
            {
                q=2;
                couleur[1]=1;
                ROS_INFO("state 2");
            }

            if (q==2 && norme(m-v)<r_valid)
            {
                q=3;
                couleur[2]=1;
                ROS_INFO("state 3");
            }

            if (q==3 && norme(m-fl)<r_valid)
            {
                q=-1;
                ROS_INFO("we are finish the challenge");
            }

            if (q==0){
                controler_line(m, theta, psi_w, sl, a, ur, us, q_h);
                ROS_INFO("Follow line 0");
            }
            if (q==1){
                controler_line(m, theta, psi_w, a, b, ur, us, q_h);
                ROS_INFO("Follow line 1");
            }
            if (q==2){
                controler_line(m, theta, psi_w, b, v, ur, us, q_h);
                ROS_INFO("Follow line 2");
            }
            if (q==3){
                controler_line(m, theta, psi_w, v, fl, ur, us, q_h);
                ROS_INFO("Follow line 3");
            }
            if (q==-1)
            {
                ur=0;
                us=0;
            }

	    	msg.x=ur;
	    	msg.y=us;
	    	msg.z=0;
            //ROS_INFO("ur=%f, us=%f", ur, us);
	    	com_servo.publish(msg);
        }

        msg_sl.x=sl[0];
        msg_sl.y=sl[1];
        msg_fl.x=fl[0];
        msg_fl.y=fl[1];
        msg_b1.x=a[0];
        msg_b1.y=a[1];
        msg_b2.x=b[0];
        msg_b2.y=b[1];
        msg_b3.x=v[0];
        msg_b3.y=v[1];
        msg_c.x=couleur[0];
        msg_c.y=couleur[1];
        msg_c.z=couleur[2];
        msg_r.data=r_valid;
        start_pub.publish(msg_sl);
        finish_pub.publish(msg_fl);
        buoy1_pub.publish(msg_b1);
        buoy2_pub.publish(msg_b2);
        buoy3_pub.publish(msg_b3);
        color_pub.publish(msg_c);
        r_pub.publish(msg_r);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}