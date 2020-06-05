//Code for the challenge 2

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
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

using namespace std;
bool initStateFinie=true, cond=true;
Eigen::Vector2d m;
double r, zeta=M_PI/4, urmax=M_PI/4, q;
double theta, psi_w;
double Gamma=M_PI/4, ks=1.0;
double us, ur;
tf::Quaternion q_sail, q_wind;
int mode;
Eigen::Vector2d SK1, SK2;
double ts;
int part=0;

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
	ros::init(argc, argv, "node_mission2");
    ros::NodeHandle n;
    ros::Subscriber pos_sail= n.subscribe("boat_pose", 1000, poseCallback);
    ros::Subscriber head_sail= n.subscribe("heading_boat", 1000, headCallback);
    ros::Subscriber wind= n.subscribe("wind_angle", 1000, windCallback);
    ros::Publisher com_servo = n.advertise<geometry_msgs::Vector3>("actuators", 1000);
    ros::Publisher pointc1 = n.advertise<geometry_msgs::Point>("c1", 1000);
    ros::Publisher pointc2 = n.advertise<geometry_msgs::Point>("c2", 1000);
    ros::Publisher pointa = n.advertise<geometry_msgs::Point>("a", 1000);
    ros::Publisher pointb = n.advertise<geometry_msgs::Point>("b", 1000);
    ros::Publisher pointc = n.advertise<geometry_msgs::Point>("c", 1000);
    ros::Publisher pointa_triangle = n.advertise<geometry_msgs::Point>("a_triangle", 1000);
    ros::Publisher pointb_triangle = n.advertise<geometry_msgs::Point>("b_triangle", 1000);
    ros::Publisher pointc_triangle = n.advertise<geometry_msgs::Point>("c_triangle", 1000);
    ros::Publisher pointd = n.advertise<geometry_msgs::Point>("d", 1000);
    ros::Publisher radius_rc = n.advertise<std_msgs::Float64>("radius_rc", 1000);
    ros::Publisher radius_pub = n.advertise<std_msgs::Float64>("radius_r", 1000);
    ros::Publisher radius_r_pub = n.advertise<std_msgs::Float64>("radius_circle", 1000);
    ros::Publisher radius_rc_pub = n.advertise<std_msgs::Float64>("radius_circle_cuic", 1000);
    ros::Publisher pointsk = n.advertise<geometry_msgs::Point>("sk", 1000);

    n.param<double>("SKx1", SK1[0], 0);
    n.param<double>("SKy1", SK1[1], 0);
    n.param<double>("SKx2", SK2[0], 0);
    n.param<double>("SKy2", SK2[1], 0);
    n.param<double>("radius_r", r, 0);
    n.param<double>("time_s", ts, 0);
    ros::Rate loop_rate(300);
    double t0 = ros::Time::now().toSec();
    while(ros::ok()){
        geometry_msgs::Vector3 msg;
        double t, tsf, t_s;

        double roll,pitch;
        tf::Matrix3x3(q_sail).getRPY(roll, pitch, theta);
        tf::Matrix3x3(q_wind).getRPY(roll, pitch, psi_w);
        
        //-------------------------------------------//

    	double  tf = ros::Time::now().toSec();

		if (tf-t0<0.1)
        {
        	ROS_INFO("Do anything");
        }
        
        else{

        	if (part==0)
        	{
                geometry_msgs::Point cA, cB, cC, cD, cC1, cC2, cSK, msg_c;
                std_msgs::Float64 radius, rad_r;

                double alpha=M_PI/6;

                Eigen::Matrix2d R;
                R << sin(psi_w), cos(psi_w), 
                    -cos(psi_w),sin(psi_w);

                double l=r/4; // this parameter can change

                Eigen::Vector2d a_r1={-l,-l*tan(alpha)},d_r1={l,-l*tan(alpha)};
                Eigen::Vector2d c_r1={l,l*tan(alpha)},b_r1={-l,l*tan(alpha)};
                Eigen::Vector2d c1_r1={l,0},c2_r1={-l,0};

                // In the global frame
                Eigen::Vector2d a=R*a_r1+SK1,d=R*d_r1+SK1;
                Eigen::Vector2d c=R*c_r1+SK1,b=R*b_r1+SK1;
                Eigen::Vector2d c1=R*c1_r1+SK1,c2=R*c2_r1+SK1;

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
                if ((q==0 && (d-b)[0]*(d-m)[0]+(d-b)[1]*(d-m)[1]<0) || (q==0 && norme(d-m)<norme(d-b)/15))
                {
                    ROS_INFO("State 1");
                    q=1;
                }

                if ((q==1 && sqrt(pow((c-m)[0],2)+pow((c-m)[1],2))<r_c) || (q==1 && fabs(((a-c)[0]*(m-c)[1]-(a-c)[1]*(m-c)[0])/norme(a-c))<r_c/2))
                {
                    ROS_INFO("State 2");
                    q=2;
                }
                if ((q==2 && (a-c)[0]*(a-m)[0]+(a-c)[1]*(a-m)[1]<0) || (q==2 && (norme(a-m)<norme(a-c)/15)) )
                {
                    ROS_INFO("State 3");
                    q=3;
                }

                if ((q==3 && sqrt(pow((b-m)[0],2)+pow((b-m)[1],2))<r_c) || (q==3 && fabs(((d-b)[0]*(m-b)[1]-(d-b)[1]*(m-b)[0])/norme(d-b))<r_c/2))
                {
                    ROS_INFO("State 0");
                    q=0;
                }

                if (q==0)
                    controler_line(m, theta, psi_w, b, d, ur, us);
                if (q==1)
                    controller_circle(m, theta, psi_w, D1, c1, ur, us);
                if (q==2)
                    controler_line(m, theta, psi_w, c, a, ur, us);
                if (q==3)
                    controller_circle(m, theta, psi_w, D2, c2, ur, us);

                msg.x=ur;
                msg.y=us;
                msg.z=0;
                com_servo.publish(msg);

                //--------publication for the visualisation---------//
                publication_point(cA, a);
                publication_point(cSK, SK1);
                publication_point(cB, b);
                publication_point(cC, c);
                publication_point(cD, d);
                publication_point(cC1, c1);
                publication_point(cC2, c2);
                publication_float(radius, r_c);
                publication_float(rad_r, r);

                pointa.publish(cA);
                pointb.publish(cB);
                pointc.publish(cC);
                pointc1.publish(cC1);
                pointc2.publish(cC2);
                pointd.publish(cD);
                radius_rc.publish(radius);
                radius_pub.publish(rad_r);
                pointsk.publish(cSK);

                if (norme(m-SK1)<r && cond==true)
                {   
                    ROS_INFO("we enter in the circle");
                    t=ros::Time::now().toSec();
                    cond=false;
                }

                if (norme(m-SK1)<r){
                    t_s=ros::Time::now().toSec();
                }

                if (cond==false){
                    tsf=t_s-t;
                }

                if (tsf>ts){
                    part=1;
                    initStateFinie=true;
                }
            }

        	else{

                geometry_msgs::Point cA, cB, cC, cSK;
                std_msgs::Float64 rad_rc, rad_r;
                visualization_msgs::Marker marker;

                Eigen::Matrix2d Rot;
                Rot << cos(psi_w), -sin(psi_w), 
                    sin(psi_w),cos(psi_w);

                double R=r/2; // this parameter can change

                Eigen::Vector2d a_r1={R, 0}, b_r1={-R/2, R*sqrt(3)/2}, c_r1={-R/2, -R*sqrt(3)/2};

                // In the global frame
                Eigen::Vector2d a=Rot*a_r1+SK2, c=Rot*c_r1+SK2, b=Rot*b_r1+SK2;

                if (initStateFinie==true)
                {
                    double d1=norme(m-a);
                    double d2=norme(m-b);
                    double d3=norme(m-c);
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
                if ((q==0 && (b-a)[0]*(b-m)[0]+(b-a)[1]*(b-m)[1]<0) || (norme(b-m)<norme(a-b)/4))
                {
                    ROS_INFO("State 1");
                    q=1;
                }

                if ((q==1 && (c-b)[0]*(c-m)[0]+(c-b)[1]*(c-m)[1]<0) || (norme(c-m)<norme(c-b)/4))
                {
                    ROS_INFO("State 2");
                    q=2;
                }

                if ((q==2 && (a-c)[0]*(a-m)[0]+(a-c)[1]*(a-m)[1]<0) || (norme(a-m)<norme(a-c)/4))
                {
                    ROS_INFO("State 0");
                    q=0;
                }


                if (q==0)
                    controler_line(m, theta, psi_w, a, b, ur, us);
                if (q==1)
                    controler_line(m, theta, psi_w, b, c, ur, us);
                if (q==2)
                    controler_line(m, theta, psi_w, c, a, ur, us);

                
                msg.x=ur;
                msg.y=us;
                msg.z=0;
                com_servo.publish(msg);

                publication_point(cA, a);
                publication_point(cSK, SK2);
                publication_point(cB, b);
                publication_point(cC, c);
                publication_float(rad_rc, R);
                publication_float(rad_r, r);

                pointa_triangle.publish(cA);
                pointb_triangle.publish(cB);
                pointc_triangle.publish(cC);
                radius_rc_pub.publish(rad_rc);
                radius_r_pub.publish(rad_r);
                pointsk.publish(cSK);
        	}
        	

    		}
            ros::spinOnce();
            loop_rate.sleep();
		}
	return 0;
}