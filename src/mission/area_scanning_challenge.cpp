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
double r=2, zeta=M_PI/4, urmax=M_PI/4;// rayon de couloir, l'angle de près, l'angle maximale du gouvernail
double theta1, theta2, theta3, theta4, psi_w; // cap de la voile, l'angle du vent
double q1=1, q2=1, q3=1, q4=1; // valeur d'hystérésis and the speed of the wind
double q=0;
double ur1,us1, ur2,us2, ur3,us3, ur4,us4;
double ks=1.0; // constante k pour regler l'angle de la sail
double Gamma=M_PI/4; // Constance pour rendre la ligne plus attractive
tf::Quaternion q_sail1, q_sail2, q_sail3, q_sail4, q_wind;
int i1=0, i2=0, i3=0, i4=0;
Eigen::Vector2d actualposition1={-47.5, 67.5}, actualposition2={47.5, 67.5}, actualposition3={-47.5, -67.5}, actualposition4={47.5,-67.5};
Eigen::Vector2d a={-27.5,-47.5}, b={37.5,47.5};
Eigen::Vector2d actpos1={-47.5, 67.5}, actpos2={47.5, 67.5}, actpos3={-47.5, -67.5},  actpos4={47.5,-67.5};
vector<Eigen::Vector2d> v1, v2;

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

double scalaire_prod(Eigen::Vector2d a, Eigen::Vector2d b)
{
    return a[0]*b[0]+a[1]*b[1];
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

void point_attract(Eigen::Vector2d m, double theta, double psi_w, Eigen::Vector2d a, double& ur, double& us)
{
    Eigen::Vector2d vect=-2*(m-a);
    double theta_bar=angle(vect);

    ROS_INFO("diff=%f", theta_bar);
    //double zeta=M_PI/10;

    //if ((cos(psi_w-theta_bar)+cos(zeta))<0)
        //theta_bar=M_PI+psi_w - zeta;
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

void near_list(Eigen::Vector2d& actualposition, vector<Eigen::Vector2d> v, vector<Eigen::Vector2d>& L)
{
    Eigen::Vector2d next_point;
    double distMinimal=5000000000;
    for (int i=0; i<v.size(); i++)
    {
        if (distMinimal>norme(actualposition-v[i])){
            distMinimal=norme(actualposition-v[i]);
        }
    }

    for (int i=0; i<v.size(); i++)
    {
        if (norme(actualposition-v[i])==distMinimal){
            L.push_back(v[i]);
        }
    }
}


Eigen::Vector2d best_near_point_vert(Eigen::Vector2d actualposition, vector<Eigen::Vector2d> v)
{

    vector<Eigen::Vector2d> L;
    near_list(actualposition, v, L);
    if (L.size()==2){
        for (int i=0; i<L.size(); i++)
        {
            if ((actualposition-L[i])[1]!=0){
                return L[i];
            }
        }
    }

    if (L.size()==3)
    {
        for (int i=0; i<L.size(); i++)
        {
            if ((actualposition-L[i])[1]>0 ){
                return L[i];
            }
        }
    }
    else{
        return L[0];
    }    
}

Eigen::Vector2d best_near_point_hori(Eigen::Vector2d actualposition, vector<Eigen::Vector2d> v)
{

    vector<Eigen::Vector2d> L;
    near_list(actualposition, v, L);
    if (L.size()==2){
        for (int i=0; i<L.size(); i++)
        {
            if ((actualposition-L[i])[0]!=0){
                return L[i];
            }
        }
    }

    if (L.size()==3)
    {
        for (int i=0; i<L.size(); i++)
        {
            if ((actualposition-L[i])[0]>0 ){
                return L[i];
            }
        }
    }
    else{
        return L[0];
    }    
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
    ros::Publisher pub_valid1 = n.advertise<geometry_msgs::Point>("center_point1", 1000);
    ros::Publisher pub_valid2 = n.advertise<geometry_msgs::Point>("center_point2", 1000);
    ros::Publisher pub_valid3 = n.advertise<geometry_msgs::Point>("center_point3", 1000);
    ros::Publisher pub_valid4 = n.advertise<geometry_msgs::Point>("center_point4", 1000);


    for (int j=0; j<20; j++){
        for (int i=0; i<20; i++)
        {
            Eigen::Vector2d c={-50+i*5+5.0/2.0, 50-j*5-5.0/2.0};
            v1.push_back(c);
            v2.push_back(c);
        }
    }

    ros::Rate loop_rate(100);
    double t0 = ros::Time::now().toSec();
    while(ros::ok()){
    	geometry_msgs::Vector3 msg1, msg2, msg3, msg4;
        geometry_msgs::Point msgCenter1, msgCenter2, msgCenter3, msgCenter4;
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
            
            Eigen::Vector2d nextpoint1=best_near_point_vert(actpos1, v1);
            Eigen::Vector2d nextpoint2=best_near_point_hori(actpos2, v1);
           
            Eigen::Vector2d nextpoint3=best_near_point_hori(actpos3, v1);
            Eigen::Vector2d nextpoint4=best_near_point_vert(actpos4, v1);
            controler_line(m1, theta1, psi_w, actpos1, nextpoint1, ur1, us1, q1);
            controler_line(m2, theta2, psi_w, actpos2, nextpoint2, ur2, us2, q2);
            controler_line(m3, theta3, psi_w, actpos3, nextpoint3, ur3, us3, q3);
            controler_line(m4, theta4, psi_w, actpos4, nextpoint4, ur4, us4, q4);
            //point_attract(m3, theta3, psi_w, a, ur3, us3);
            //point_attract(m2, m1, m3, m4, theta2, psi_w, nextpoint2, ur2, us2);
            //point_attract(m3, m2, m1, m4, theta3, psi_w, nextpoint3, ur3, us3);
            //point_attract(m4, m2, m3, m1, theta4, psi_w, nextpoint4, ur4, us4);

            if (scalaire_prod(nextpoint1-actpos1, nextpoint1-m1)<0)
            {
                actpos1=nextpoint1;
                v1.erase(std::remove(v1.begin(), v1.end(), nextpoint1), v1.end());
                /*if (norme(m1-nextpoint1)>2. )
                {
                    ROS_INFO("zap1");
                    nextPointValide.push_back(nextpoint1);
                }*/

            }


            if (scalaire_prod(nextpoint2-actpos2, nextpoint2-m2)<0)
            {
                actpos2=nextpoint2;
                v1.erase(std::remove(v1.begin(), v1.end(), nextpoint2), v1.end());
                /*if (norme(m2-nextpoint2)>2.)
                {
                    ROS_INFO("zap2");
                    nextPointValide.push_back(nextpoint2);
                }*/
            }
            
            if (scalaire_prod(nextpoint3-actpos3, nextpoint3-m3)<0)
            {
                actpos3=nextpoint3;
                v1.erase(std::remove(v1.begin(), v1.end(), nextpoint3), v1.end());
                /*if (norme(m3-nextpoint3)>2.)
                {
                    ROS_INFO("zap3");
                    nextPointValide.push_back(nextpoint3);
                }*/
            }

            if (scalaire_prod(nextpoint4-actpos4, nextpoint4-m4)<0)
            {
                actpos4=nextpoint4;
                v1.erase(std::remove(v1.begin(), v1.end(), nextpoint4), v1.end());
                /*if (norme(m4-nextpoint4)>2.)
                {
                    ROS_INFO("zap4");
                    nextPointValide.push_back(nextpoint4);
                }*/
            }

            //ROS_INFO("yes");

            for (int i=0; i<v2.size(); i++)
            {
                if (norme(m1-v2[i])<2.)
                {
                    ROS_INFO("valider x=%f, y=%f", v2[i][0], v2[i][1]);
                    msgCenter1.x=v2[i][0];
                    msgCenter1.y=v2[i][1];
                    msgCenter1.z=1.0;                                      
                    pub_valid1.publish(msgCenter1);
                    //v1.erase(std::remove(v1.begin(), v1.end(), v1[i]), v1.end());
                    v2.erase(std::remove(v2.begin(), v2.end(), v2[i]), v2.end());
                    //v3.erase(std::remove(v3.begin(), v3.end(), v1[i]), v3.end());
                    //v4.erase(std::remove(v4.begin(), v4.end(), v1[i]), v4.end());
                    break;
                }

                if (norme(m2-v2[i])<2.)
                {
                    msgCenter2.x=v2[i][0];
                    msgCenter2.y=v2[i][1];
                    msgCenter2.z=2.0;
                    pub_valid2.publish(msgCenter2);
                    //v1.erase(std::remove(v1.begin(), v1.end(), v1[i]), v1.end());
                    v2.erase(std::remove(v2.begin(), v2.end(), v2[i]), v2.end());
                    //v3.erase(std::remove(v3.begin(), v3.end(), v1[i]), v3.end());
                    //v4.erase(std::remove(v4.begin(), v4.end(), v1[i]), v4.end());
                    break;
                }

                if (norme(m3-v2[i])<2.)
                {
                    ROS_INFO("valider x=%f, y=%f", v2[i][0], v2[i][1]);
                    msgCenter3.x=v2[i][0];
                    msgCenter3.y=v2[i][1];
                    msgCenter3.z=3.0;
                    pub_valid3.publish(msgCenter3);
                    //v1.erase(std::remove(v1.begin(), v1.end(), v1[i]), v1.end());
                    v2.erase(std::remove(v2.begin(), v2.end(), v2[i]), v2.end());
                    //v3.erase(std::remove(v3.begin(), v3.end(), v1[i]), v3.end());
                    //v4.erase(std::remove(v4.begin(), v4.end(), v1[i]), v4.end());
                    break;
                }

                if (norme(m4-v2[i])<2.)
                {
                    msgCenter4.x=v2[i][0];
                    msgCenter4.y=v2[i][1];
                    msgCenter4.z=4.0;
                    pub_valid4.publish(msgCenter4);
                    //v1.erase(std::remove(v1.begin(), v1.end(), v1[i]), v1.end());
                    v2.erase(std::remove(v2.begin(), v2.end(), v2[i]), v2.end());
                    //v3.erase(std::remove(v3.begin(), v3.end(), v1[i]), v3.end());
                    //v4.erase(std::remove(v4.begin(), v4.end(), v1[i]), v4.end());
                    break;
                }
                
            }

            
            
            //nextPointValide.clear();

            ROS_INFO("ur1=%f us1=%f", ur1, us1);
            ROS_INFO("ur2=%f us2=%f", ur2, us2);
            ROS_INFO("ur3=%f us3=%f", ur3, us3);
            ROS_INFO("ur4=%f us4=%f", ur4, us4);
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