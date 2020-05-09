// This file is the simulation code of a sailboat. 

#include "ros/ros.h"
#include <vector>
#include <math.h>
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/tf.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"


using namespace std;

double dt=0.01;// dt is the simulation step 
double ur=0,us=0; //the input of the sailboat
double a=1; // velocity of the "true" wind
double psi_w=0; //the angle of the "true" wind
double p1=0.1, p2=1,p3=6000, p4=1000, p5=2000,p6=1,p7=1,p8=2,p9=300,p10=10000;
vector<double> x={30.0,-18.0,-1.72,0.0,0.0}; // the state vector of the sailboat
vector<double> xdot={0.0,0.0,0.0,0.0,0.0};


double sign(double x)
{
    if (x<0)
        return -1;
    else
        return 1;
}

double norme(vector<double> x)
{
    return sqrt(pow(x[0],2)+pow(x[1],2));
}

double angle(vector<double> x)
{
    return atan2(x[1],x[0]);
}

void Euler_integration(vector<double>& x, vector<double> xdot)
{
    x[0]+=dt*xdot[0];
    x[1]+=dt*xdot[1];
    x[2]+=dt*xdot[2];
    x[3]+=dt*xdot[3];
    x[4]+=dt*xdot[4];

}

// Dynamique of the sailboat
void f(double u1, double u2, vector<double> x, vector<double>& xdot, double psi_w)
{
    vector<double> wap={a*cos(psi_w-x[2])-x[3],a*sin(psi_w)-x[2]};
    double psi_wap=angle(wap);
    double norm_wap=norme(wap);
    double sigma=cos(psi_wap)+cos(u2);
    double delta_s;
    if (sigma <=0)
        delta_s= M_PI + psi_wap;
    else
        delta_s=-sign(sin(psi_wap))*u2;
    double fr=p5*x[3]*sin(u1);
    double fs=p4*norm_wap*sin(delta_s)-psi_wap;
    xdot[0]=x[3]*cos(x[2])+ p1*a*cos(psi_w);
    xdot[1]=x[3]*sin(x[2])+ p1*a*sin(psi_w);
    xdot[2]=x[4];
    xdot[3]=(fs*sin(delta_s)-fr*sin(u1)-p2*x[3]*x[3])/p9;
    xdot[4]=(fs*(p6-p7*cos(delta_s))-p8*fr*cos(u1)-p3*x[4]*x[3])/p10;
}

void commCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    ur = msg->x;
    us = msg->y;
}


int main(int argc, char **argv)
{   
    
    ros::init(argc, argv, "node_simulation");
    ros::NodeHandle n;
    ros::Publisher pose_state = n.advertise<geometry_msgs::Point>("boat_pose", 1000);
    ros::Publisher twist_state = n.advertise<geometry_msgs::Twist>("boat_twist", 1000);
    ros::Publisher wind = n.advertise<geometry_msgs::Quaternion>("wind_angle", 1000);
    ros::Publisher cap = n.advertise<geometry_msgs::Quaternion>("heading_boat", 1000);
    ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker",0 );
    ros::Publisher wind_pub = n.advertise<visualization_msgs::Marker>( "visualization_wind",0 );
    ros::Subscriber commande = n.subscribe("actuators", 1000, commCallback);
    ros::Rate loop_rate(50);

    while (ros::ok()) {
        geometry_msgs::Point msg_pose;
        geometry_msgs::Twist msg_twist;
        geometry_msgs::Quaternion msg_wind;
        geometry_msgs::Quaternion msg_cap;
        visualization_msgs::Marker marker;
        visualization_msgs::Marker wind_marker;
        geometry_msgs::PoseStamped msgs;

        
        // Publication of the wind angle
        tf::Quaternion q_wind;
        q_wind.setRPY(0, 0, psi_w);
        tf::quaternionTFToMsg(q_wind, msg_wind);
        wind.publish(msg_wind);

        // Publication of the position
        f(ur,us,x,xdot,psi_w);
        Euler_integration(x,xdot);
        msg_pose.x=x[0];
        msg_pose.y=x[1];
        msg_pose.z=0;
        pose_state.publish(msg_pose);

        //Publication of the angle of the boat
        tf::Quaternion q_cap;
        q_cap.setRPY(0, 0, x[2]);
        tf::quaternionTFToMsg(q_cap, msg_cap);
        ROS_INFO("Quaternion: x=%f,y=%f,z=%f,w=%f",msg_cap.x,msg_cap.y,msg_cap.z,msg_cap.w);
        cap.publish(msg_cap);

        //Publication of the velocity
        msg_twist.linear.x=x[3];
        msg_twist.linear.y=0;
        msg_twist.linear.z=0;
        msg_twist.angular.x=x[4];
        msg_twist.angular.y=0;
        msg_twist.angular.z=0;
        twist_state.publish(msg_twist);

        //visualisation of the boat
        msgs.pose.position.x=x[0];
        msgs.pose.position.y=x[1];
        msgs.pose.position.z=0;
        tf::Quaternion q;
        q.setRPY(0, 0, x[2]);
        tf::quaternionTFToMsg(q, msgs.pose.orientation);
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "sail";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose =msgs.pose;
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.mesh_resource = "package://sailboat/meshs/boat.dae";

        //visualisation wind
        wind_marker.header.frame_id = "map";
        wind_marker.header.stamp = ros::Time::now();
        wind_marker.ns = "wind";
        wind_marker.id = 0;
        wind_marker.type = visualization_msgs::Marker::ARROW;
        wind_marker.action = visualization_msgs::Marker::ADD;
        wind_marker.pose.position.z=0;
        wind_marker.scale.y = 0.5;
        wind_marker.scale.z = 0.5;
        wind_marker.color.a = 1.0;
        wind_marker.pose.position.x = x[0] + 10*cos(x[2]);//10*cos(psi_w);
        wind_marker.pose.position.y = x[1] + 10*sin(x[2]);//10*sin(psi_w);
        tf::quaternionTFToMsg(q_wind, wind_marker.pose.orientation);
        wind_marker.scale.x = 2.0;
        wind_marker.color.r = 1.0f;
        wind_marker.color.g = 0.0f;
        wind_marker.color.b = 0.0f;
        wind_pub.publish( wind_marker );
        vis_pub.publish( marker );
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}