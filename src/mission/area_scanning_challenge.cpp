// this file is the heading following code of the sailboat

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

Eigen::Vector2d m1, m2, m3, m4; // Position of the boat
double r, zeta=M_PI/4, urmax=M_PI/4;// rayon de couloir, l'angle de près, l'angle maximale du gouvernail
double theta1, theta2, theta3, theta4, psi_w; // cap de la voile, l'angle du vent
double q1=1, q2=1, q3=1, q4=1; // valeur d'hystérésis and the speed of the wind
double ur1,us1, ur2,us2, ur3,us3, ur4,us4;
double ks=1.0; // constante k pour regler l'angle de la sail
double Gamma=M_PI/4; // Constance pour rendre la ligne plus attractive
tf::Quaternion q_sail1, q_sail2, q_sail3, q_sail4, q_wind;

Eigen::Vector2d actualposition1={-47.5, 67.5}, actualposition2={47.5, 67.5}, actualposition3={-47.5, -67.5}, actualposition4={47.5,-67.5};
Eigen::Vector2d next_point1, next_point2, next_point3, next_point4;
vector<Eigen::Vector2d> v, point_valide;

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


Eigen::Vector2d near_point(Eigen::Vector2d actualposition, vector<Eigen::Vector2d> v)
{
    double distMinimal=500000000000;
    Eigen::Vector2d next_point;
    for (int i=0; i<v.size(); i++)
    {
        if (distMinimal>norme(actualposition-v[i])){
            distMinimal=norme(actualposition-v[i]);
            next_point=v[i];
        }
    }
    return next_point;
} 

void publication_command(geometry_msgs::Vector3& msg, double ur, double us)
{
    msg.x=ur;
    msg.y=us;
    msg.z=0;
}

void pose1Callback(const geometry_msgs::Point::ConstPtr& msg) {
    m1[0]=msg->x;
    m1[1]=msg->y;
}

void pose2Callback(const geometry_msgs::Point::ConstPtr& msg) {
    m2[0]=msg->x;
    m2[1]=msg->y;
}

void pose3Callback(const geometry_msgs::Point::ConstPtr& msg) {
    m3[0]=msg->x;
    m3[1]=msg->y;
}

void pose4Callback(const geometry_msgs::Point::ConstPtr& msg) {
    m4[0]=msg->x;
    m4[1]=msg->y;
}


void state1Callback(const geometry_msgs::Quaternion::ConstPtr& msg) {
    q_sail1[0]=msg->x;
    q_sail1[1]=msg->y;
    q_sail1[2]=msg->z;
    q_sail1[3]=msg->w;
}

void state2Callback(const geometry_msgs::Quaternion::ConstPtr& msg) {
    q_sail2[0]=msg->x;
    q_sail2[1]=msg->y;
    q_sail2[2]=msg->z;
    q_sail2[3]=msg->w;
}

void state3Callback(const geometry_msgs::Quaternion::ConstPtr& msg) {
    q_sail3[0]=msg->x;
    q_sail3[1]=msg->y;
    q_sail3[2]=msg->z;
    q_sail3[3]=msg->w;
}

void state4Callback(const geometry_msgs::Quaternion::ConstPtr& msg) {
    q_sail4[0]=msg->x;
    q_sail4[1]=msg->y;
    q_sail4[2]=msg->z;
    q_sail4[3]=msg->w;
}

void windCallback(const geometry_msgs::Quaternion::ConstPtr& msg) {
    q_wind[0]=msg->x;
    q_wind[1]=msg->y;
    q_wind[2]=msg->z;
    q_wind[3]=msg->w;
}



int main(int argc, char **argv)
{   
	ros::init(argc, argv, "node_area_scanning");
    ros::NodeHandle n;
    ros::Subscriber pos_sail1= n.subscribe("boat1_pose", 1000, pose1Callback);
    ros::Subscriber pos_sail2= n.subscribe("boat2_pose", 1000, pose2Callback);
    ros::Subscriber pos_sail3= n.subscribe("boat3_pose", 1000, pose3Callback);
    ros::Subscriber pos_sail4= n.subscribe("boat4_pose", 1000, pose4Callback);
    ros::Subscriber state_sail1= n.subscribe("heading_boat1", 1000, state1Callback);
    ros::Subscriber state_sail2= n.subscribe("heading_boat2", 1000, state2Callback);
    ros::Subscriber state_sail3= n.subscribe("heading_boat3", 1000, state3Callback);
    ros::Subscriber state_sail4= n.subscribe("heading_boat4", 1000, state4Callback);
    ros::Subscriber wind= n.subscribe("wind_angle", 1000, windCallback);
    ros::Publisher com_servo1 = n.advertise<geometry_msgs::Vector3>("actuators1", 1000);
    ros::Publisher com_servo2 = n.advertise<geometry_msgs::Vector3>("actuators2", 1000);
    ros::Publisher com_servo3 = n.advertise<geometry_msgs::Vector3>("actuators3", 1000);
    ros::Publisher com_servo4 = n.advertise<geometry_msgs::Vector3>("actuators4", 1000);

    ros::Publisher pointA_pub = n.advertise<visualization_msgs::Marker>( "visualization_pointA",0 );
    ros::Publisher pointB_pub = n.advertise<visualization_msgs::Marker>( "visualization_pointB",0 );
    ros::Publisher line_pub = n.advertise<visualization_msgs::Marker>( "visualization_line",0 );
    ros::Publisher lineLeft_pub = n.advertise<visualization_msgs::Marker>( "visualization_lineLeft",0 );
    ros::Publisher lineRight_pub = n.advertise<visualization_msgs::Marker>( "visualization_lineRight",0 );


    for (int j=0; j<20; j++){
        for (int i=0; i<20; i++)
        {
            Eigen::Vector2d c={-50+i*5+5.0/2.0, 50-j*5-5.0/2.0};
            v.push_back(c);
            ROS_INFO("cx=%f, cy=%f", c[0], c[1]);
        }
    }
    ros::Rate loop_rate(300);
    double t0 = ros::Time::now().toSec();
    while(ros::ok()){
    	geometry_msgs::Vector3 msg1, msg2, msg3, msg4;
        visualization_msgs::Marker marker_line, marker_A, marker_B, marker_lineLeft, marker_lineRight;
        double roll,pitch;
        tf::Matrix3x3(q_sail1).getRPY(roll, pitch, theta1);
        tf::Matrix3x3(q_sail2).getRPY(roll, pitch, theta2);
        tf::Matrix3x3(q_sail3).getRPY(roll, pitch, theta3);
        tf::Matrix3x3(q_sail4).getRPY(roll, pitch, theta4);
        tf::Matrix3x3(q_wind).getRPY(roll, pitch, psi_w);
        //regulation and publication

        double  tf = ros::Time::now().toSec();

        if (tf-t0<0.3)
        {
            ROS_INFO("Do anything");
        }
        
        else{
            
            next_point1=near_point(actualposition1, v);
            next_point2=near_point(actualposition2, v);
            next_point3=near_point(actualposition3, v);
            next_point4=near_point(actualposition4, v);
            //ROS_INFO("next_point1 x=%f, y=%f", next_point1[0], next_point1[1]);
            //ROS_INFO("next_point2 x=%f, y=%f", next_point2[0], next_point2[1]);
            //ROS_INFO("next_point3 x=%f, y=%f", next_point3[0], next_point3[1]);
            //ROS_INFO("next_point4 x=%f, y=%f", next_point4[0], next_point4[1]);
            controler_line(m1, theta1, psi_w, actualposition1, next_point1, ur1, us1, q1);
            controler_line(m2, theta2, psi_w, actualposition2, next_point2, ur2, us2, q2);
            controler_line(m3, theta3, psi_w, actualposition3, next_point3, ur3, us3, q3);
            controler_line(m4, theta4, psi_w, actualposition4, next_point4, ur4, us4, q4);

            if (norme(m1-next_point1)<0.5)
            {
                point_valide.push_back(next_point1);
                //ROS_INFO("next_point1 x=%f, y=%f", next_point1[0], next_point1[1]);
                actualposition1=next_point1;
                v.erase(std::remove(v.begin(), v.end(), next_point1), v.end());
            }

             if (norme(m2-next_point2)<0.5)
            {
                point_valide.push_back(next_point2);
                actualposition2=next_point2;
                v.erase(std::remove(v.begin(), v.end(), next_point2), v.end());
            }

            if (norme(m3-next_point3)<0.5)
            {
                point_valide.push_back(next_point3);
                actualposition3=next_point3;
                v.erase(std::remove(v.begin(), v.end(), next_point3), v.end());
            }

            if (norme(m4-next_point4)<0.5)
            {
                point_valide.push_back(next_point4);
                actualposition4=next_point4;
                v.erase(std::remove(v.begin(), v.end(), next_point4), v.end());
            }

            //ROS_INFO("angle_sail us=%f", us);
            publication_command(msg1, ur1, us1);
            publication_command(msg2, ur2, us2);
            publication_command(msg3, ur3, us3);
            publication_command(msg4, ur4, us4);
            com_servo1.publish(msg1);
            com_servo2.publish(msg2);
            com_servo3.publish(msg3);
            com_servo4.publish(msg4);

            
        }
    	ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}