// This file is the simulation code of a sailboat. 

#include "ros/ros.h"
#include <vector>
#include <math.h>
#include <algorithm>
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/tf.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include "std_msgs/Float64.h"


using namespace std;

double dt=0.01;// dt is the simulation step 
double ur=0,us=0; //the input of the sailboat
double a=5; // velocity of the "true" wind
double psi_w; //the angle of the "true" wind
double delta_s=0;
//p1=0.125; p2=29.99; p3=96.43; p4=58.07; p5=120.65; p6=0.1; p7=0; p8=0.5; p9=10; p10=29.87; p11=0; % Plymouth
//p1=0.03; p2=40; p3=6000; p4=200; p5=1500; p6=0.5; p7=0.5; p8=2; p9=300; p10=400; p11=0.2; % Aland
double p1=0.05, p2=0.2, p3=6000, p4=1000, p5=2000, p6=1, p7=1, p8=2, p9=300, p10=10000, p11=1; // Vamos
//double p1=0.1, p2=1,p3=6000, p4=1000, p5=2000,p6=1,p7=1,p8=2,p9=300,p10=10000;
vector<double> x={0,0,0,0.0,0.0}; // the state vector of the sailboat
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
void f(double u1, double u2, vector<double> x, vector<double>& xdot, double psi_w, double& delta_s)
{
    vector<double> wap={a*cos(psi_w-x[2])-x[3],a*sin(psi_w-x[2])};
    double psi_wap=angle(wap);
    double norm_wap=norme(wap);
    double sigma=cos(psi_wap)+cos(u2);
    if (sigma <=0)
        delta_s= M_PI + psi_wap;
    else
        delta_s=-sign(sin(psi_wap))*u2;
    double fr=p5*x[3]*sin(u1);
    double fs=p4*norm_wap*sin(delta_s-psi_wap);
    xdot[0]=x[3]*cos(x[2])+ p1*a*cos(psi_w);
    xdot[1]=x[3]*sin(x[2])+ p1*a*sin(psi_w);
    xdot[2]=x[4];
    xdot[3]=(fs*sin(delta_s)-fr*p11*sin(u1)-p2*pow(x[3],2))/p9;
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
    ros::Publisher wind = n.advertise<geometry_msgs::Quaternion>("wind_angle", 1000);
    ros::Publisher cap = n.advertise<geometry_msgs::Quaternion>("heading_boat", 1000);
    ros::Publisher boat_pub = n.advertise<visualization_msgs::Marker>( "visualization_boat",0 );
    ros::Publisher sail_pub = n.advertise<visualization_msgs::Marker>( "visualization_sail",0 );
    ros::Publisher rudder_pub = n.advertise<visualization_msgs::Marker>( "visualization_rudder",0 );
    ros::Publisher wind_pub = n.advertise<visualization_msgs::Marker>( "visualization_wind",0 );
    //ros::Publisher pose_boat_pub = n.advertise<visualization_msgs::Marker>( "visualization_pos_boat",0 );
    ros::Subscriber commande = n.subscribe("actuators", 1000, commCallback);

    geometry_msgs::Point msg_pose;
    geometry_msgs::Quaternion msg_wind;
    geometry_msgs::Quaternion msg_cap;

    geometry_msgs::PoseStamped msgs_boat;
    geometry_msgs::PoseStamped msgs_sail;
    geometry_msgs::PoseStamped msgs_rudder;

    visualization_msgs::Marker marker_boat;
    visualization_msgs::Marker marker_sail;
    visualization_msgs::Marker marker_rudder;
    visualization_msgs::Marker marker_wind;
    visualization_msgs::Marker marker_B;
    

    geometry_msgs::TransformStamped transformStamped_boat;
    tf2_ros::TransformBroadcaster br_boat;

    geometry_msgs::TransformStamped transformStamped_sail;
    tf2_ros::TransformBroadcaster br_sail;

    geometry_msgs::TransformStamped transformStamped_rudder;
    tf2_ros::TransformBroadcaster br_rudder;

    transformStamped_boat.header.frame_id = "map";
    transformStamped_boat.child_frame_id = "boat";

    transformStamped_sail.header.frame_id = "boat";
    transformStamped_sail.child_frame_id = "sail";

    transformStamped_rudder.header.frame_id = "boat";
    transformStamped_rudder.child_frame_id = "rudder";

    n.param<double>("psi_wind", psi_w, 0);
    n.param<double>("boat_x", x[0], 0);
    n.param<double>("boat_y", x[1], 0);
    n.param<double>("boat_head", x[2], 0);
    ros::Rate loop_rate(100);
    while (ros::ok()) {
        

        //ROS_INFO("psi_w=%f", psi_w);
        //ROS_INFO("boat_head=%f", x[2]);
        // Publication of the wind angle
        tf::Quaternion q_wind;
        q_wind.setRPY(0, 0, psi_w);
        tf::quaternionTFToMsg(q_wind, msg_wind);
        wind.publish(msg_wind);

        // Publication of the position
        f(ur,us,x,xdot,psi_w, delta_s);
        //ROS_INFO("boat_x=%f", x[0]);
        Euler_integration(x,xdot);
        //ROS_INFO("delta_s=%f", delta_s);
        msg_pose.x=x[0];
        msg_pose.y=x[1];
        msg_pose.z=0;
        pose_state.publish(msg_pose);

        //Publication of the angle of the boat
        tf::Quaternion q_cap;
        q_cap.setRPY(0, 0, x[2]);
        tf::quaternionTFToMsg(q_cap, msg_cap);
        //ROS_INFO("Quaternion: x=%f,y=%f,z=%f,w=%f",msg_cap.x,msg_cap.y,msg_cap.z,msg_cap.w);
        cap.publish(msg_cap);

        tf::Quaternion q;
        q.setRPY(0, 0, x[2]);
        transformStamped_boat.header.stamp = ros::Time::now();
        transformStamped_boat.transform.translation.x =x[0];
        transformStamped_boat.transform.translation.y = x[1];
        transformStamped_boat.transform.translation.z = 0.0;
        tf::quaternionTFToMsg(q,transformStamped_boat.transform.rotation);
        br_boat.sendTransform(transformStamped_boat);
        //visualisation of the boat
        q.setRPY(M_PI/2, 0, M_PI/2);
        msgs_boat.pose.position.x=-1.0;
        msgs_boat.pose.position.y=-0.5;
        msgs_boat.pose.position.z=-0.5;
        tf::quaternionTFToMsg(q, msgs_boat.pose.orientation);
        marker_boat.header.frame_id = "boat";
        marker_boat.header.stamp = ros::Time::now();
        marker_boat.ns = "boat";
        marker_boat.id = 0;
        marker_boat.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker_boat.action = visualization_msgs::Marker::ADD;
        marker_boat.pose =msgs_boat.pose;
        marker_boat.scale.x = 0.0004;
        marker_boat.scale.y = 0.0004;
        marker_boat.scale.z = 0.0004;
        marker_boat.color.a = 1.0;
        marker_boat.color.r = 1.0;
        marker_boat.color.g = 1.0;
        marker_boat.color.b = 1.0;
        marker_boat.mesh_resource = "package://sailboat/meshs/boat.STL";

        //--------------------------------------//
        tf::Quaternion q_sail;
        q_sail.setRPY(0, 0, delta_s);
        transformStamped_sail.header.stamp = ros::Time::now();
        transformStamped_sail.transform.translation.x =0.35;//2.43;
        transformStamped_sail.transform.translation.y = 0.0;
        transformStamped_sail.transform.translation.z = 0.35;
        tf::quaternionTFToMsg(q_sail,transformStamped_sail.transform.rotation);
        br_sail.sendTransform(transformStamped_sail);

        //visualisation of the sail
        msgs_sail.pose.position.x=-1.15;
        msgs_sail.pose.position.y=0;
        msgs_sail.pose.position.z=0;
        q.setRPY(M_PI/2, 0, M_PI/2);
        tf::quaternionTFToMsg(q, msgs_sail.pose.orientation);
        marker_sail.header.frame_id = "sail";
        marker_sail.header.stamp = ros::Time::now();
        marker_sail.ns = "sail";
        marker_sail.id = 0;
        marker_sail.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker_sail.action = visualization_msgs::Marker::ADD;
        marker_sail.pose =msgs_sail.pose;
        marker_sail.scale.x = 0.0005;
        marker_sail.scale.y = 0.0004;
        marker_sail.scale.z = 0.00038;
        marker_sail.color.a = 5.0;
        marker_sail.color.r = 0.0;
        marker_sail.color.g = 1.0;
        marker_sail.color.b = 0.0;
        marker_sail.mesh_resource = "package://sailboat/meshs/sail.STL";

        
        transformStamped_rudder.header.stamp = ros::Time::now();
        transformStamped_rudder.transform.translation.x =-0.7;
        transformStamped_rudder.transform.translation.y = 0.0;
        transformStamped_rudder.transform.translation.z = 0;
        q.setRPY(0, 0, ur);
        tf::quaternionTFToMsg(q,transformStamped_rudder.transform.rotation);
        br_rudder.sendTransform(transformStamped_rudder);
        //visualisation of the rudder
        msgs_rudder.pose.position.x=-0.5;
        msgs_rudder.pose.position.y=0;
        msgs_rudder.pose.position.z=-0.3;
        q.setRPY(M_PI/2, 0, M_PI/2);
        tf::quaternionTFToMsg(q, msgs_rudder.pose.orientation);
        marker_rudder.header.frame_id = "rudder";
        marker_rudder.header.stamp = ros::Time::now();
        marker_rudder.ns = "rudder";
        marker_rudder.id = 0;
        marker_rudder.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker_rudder.action = visualization_msgs::Marker::ADD;
        marker_rudder.pose =msgs_rudder.pose;
        marker_rudder.scale.x = 0.0007;
        marker_rudder.scale.y = 0.0004;
        marker_rudder.scale.z = 0.0004;
        marker_rudder.color.a = 20.0;
        marker_rudder.color.r = 1.0;
        marker_rudder.color.g = 1.0;
        marker_rudder.color.b = 0.0;
        marker_rudder.mesh_resource = "package://sailboat/meshs/rudder.STL";

        //visualisation wind
        marker_wind.header.frame_id = "map";
        marker_wind.header.stamp = ros::Time();
        marker_wind.ns = "wind";
        marker_wind.id = 0;
        marker_wind.type = visualization_msgs::Marker::ARROW;
        marker_wind.action = visualization_msgs::Marker::ADD;
        marker_wind.pose.position.z=0;
        marker_wind.scale.y = 0.5;
        marker_wind.scale.z = 0.5;
        marker_wind.color.a = 1.5;
        marker_wind.pose.position.x = 35;
        marker_wind.pose.position.y = -10;
        tf::quaternionTFToMsg(q_wind, marker_wind.pose.orientation);
        marker_wind.scale.x = 5.0;
        marker_wind.color.r = 1.0f;
        marker_wind.color.g = 0.0f;
        marker_wind.color.b = 0.0f;

        //pose_boat_pub.publish(marker_B);
        wind_pub.publish( marker_wind );
        boat_pub.publish( marker_boat );
        rudder_pub.publish( marker_rudder );
        sail_pub.publish( marker_sail );
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
