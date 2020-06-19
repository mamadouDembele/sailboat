// This file is the simulation code of a sailboat. 

#include "ros/ros.h"
#include <vector>
#include <math.h>
#include <string>
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
double ur1=0, ur2=0,us1=0, us2=0; //the input of the sailboat
double ur3=0, ur4=0, us3=0, us4=0;
double a=1; // velocity of the "true" wind
double psi_w; //the angle of the "true" wind
double delta_s1=0, delta_s2=M_PI/2;
double delta_s3=0, delta_s4=M_PI/3;
//p1=0.125; p2=29.99; p3=96.43; p4=58.07; p5=120.65; p6=0.1; p7=0; p8=0.5; p9=10; p10=29.87; p11=0; % Plymouth
//p1=0.03; p2=40; p3=6000; p4=200; p5=1500; p6=0.5; p7=0.5; p8=2; p9=300; p10=400; p11=0.2; % Aland
double p1=0.05, p2=0.2, p3=6000, p4=1000, p5=2000, p6=1, p7=1, p8=2, p9=300, p10=10000, p11=1; // Vamos
//double p1=0.1, p2=1,p3=6000, p4=1000, p5=2000,p6=1,p7=1,p8=2,p9=300,p10=10000;
vector<double> x1={0,0,-M_PI/2,0.0,0.0}; // the state vector of the sailboat
vector<double> x2={0,0,-M_PI/2,0.5,0.0};
vector<double> x3={0,0,M_PI/2,0.0,0.0};
vector<double> x4={0,0,M_PI/2,0,0};
vector<double> xdot={0.0,0.0,0.0,0.0,0.0};
vector<double> couleur1={1.0, 0.0, 0.0}, couleur2={0.0, 1.0, 0.0}, couleur3={1.0, 1.0, 0.0}, couleur4={1.0, 1.0, 1.0};


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



void print_boat(visualization_msgs::Marker& marker, geometry_msgs::TransformStamped& transformStamped, tf2_ros::TransformBroadcaster& br, geometry_msgs::PoseStamped& msgs, string name, vector<double>& x, vector<double> coleur, int i)
{
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = name;
    tf::Quaternion q;
    q.setRPY(0, 0, x[2]);
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.transform.translation.x =x[0];
    transformStamped.transform.translation.y = x[1];
    transformStamped.transform.translation.z = 0.0;
    tf::quaternionTFToMsg(q,transformStamped.transform.rotation);
    br.sendTransform(transformStamped);
    //visualisation of the boat
    q.setRPY(M_PI/2, 0, M_PI/2);
    msgs.pose.position.x=-1.0;
    msgs.pose.position.y=-0.5;
    msgs.pose.position.z=-0.5;
    tf::quaternionTFToMsg(q, msgs.pose.orientation);
    marker.header.frame_id = name;
    marker.header.stamp = ros::Time::now();
    marker.ns = name;
    marker.id = i;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose =msgs.pose;
    marker.scale.x = 0.0004;
    marker.scale.y = 0.0004;
    marker.scale.z = 0.0004;
    marker.color.a = 1.0;
    marker.color.r = coleur[0];
    marker.color.g = coleur[1];
    marker.color.b = coleur[2];
    marker.mesh_resource = "package://sailboat/meshs/boat.STL";
}

void print_sail(visualization_msgs::Marker& marker, geometry_msgs::TransformStamped& transformStamped, tf2_ros::TransformBroadcaster& br, geometry_msgs::PoseStamped& msgs, string name1, string name2, double delta_s, int i)
{

    transformStamped.header.frame_id = name1;
    transformStamped.child_frame_id = name2;
    tf::Quaternion q;
    q.setRPY(0, 0, delta_s);
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.transform.translation.x =0.35;
    transformStamped.transform.translation.y = 0.0;//-0.2;
    transformStamped.transform.translation.z = 0.35;
    tf::quaternionTFToMsg(q,transformStamped.transform.rotation);
    br.sendTransform(transformStamped);

    //visualisation of the sail
    msgs.pose.position.x=-1.15;
    msgs.pose.position.y=0;
    msgs.pose.position.z=0;
    q.setRPY(M_PI/2, 0, M_PI/2);
    tf::quaternionTFToMsg(q, msgs.pose.orientation);
    marker.header.frame_id = name2;
    marker.header.stamp = ros::Time::now();
    marker.ns = name2;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose =msgs.pose;
    marker.scale.x = 0.0005;
    marker.scale.y = 0.0004;
    marker.scale.z = 0.00038;
    marker.color.a = 5.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.mesh_resource = "package://sailboat/meshs/sail.STL";
}

void print_rudder(visualization_msgs::Marker& marker, geometry_msgs::TransformStamped& transformStamped, tf2_ros::TransformBroadcaster& br, geometry_msgs::PoseStamped& msgs, string name1, string name2, double ur, int i)
{
    transformStamped.header.frame_id = name1;
    transformStamped.child_frame_id = name2;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.transform.translation.x =-0.7;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0;
    tf::Quaternion q;
    q.setRPY(0, 0, ur);
    tf::quaternionTFToMsg(q,transformStamped.transform.rotation);
    br.sendTransform(transformStamped);
    //visualisation of the rudder
    msgs.pose.position.x=-0.5;
    msgs.pose.position.y=0;
    msgs.pose.position.z=-0.3;
    q.setRPY(M_PI/2, 0, M_PI/2);
    tf::quaternionTFToMsg(q, msgs.pose.orientation);
    marker.header.frame_id = name2;
    marker.header.stamp = ros::Time::now();
    marker.ns = name2;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose =msgs.pose;
    marker.scale.x = 0.0007;
    marker.scale.y = 0.0004;
    marker.scale.z = 0.0004;
    marker.color.a = 20.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.mesh_resource = "package://sailboat/meshs/rudder.STL";

}

void publication_pose(geometry_msgs::Point& msg_pose, vector<double> x){
    msg_pose.x=x[0];
    msg_pose.y=x[1];
    msg_pose.z=0;
}

void comm1Callback(const geometry_msgs::Vector3::ConstPtr& msg) {
    ur1 = msg->x;
    us1 = msg->y;
}

void comm2Callback(const geometry_msgs::Vector3::ConstPtr& msg) {
    ur2 = msg->x;
    us2 = msg->y;
}

void comm3Callback(const geometry_msgs::Vector3::ConstPtr& msg) {
    ur3 = msg->x;
    us3 = msg->y;
}

void comm4Callback(const geometry_msgs::Vector3::ConstPtr& msg) {
    ur4 = msg->x;
    us4 = msg->y;
}



int main(int argc, char **argv)
{   
    
    ros::init(argc, argv, "node_simulation_area_scanning");
    ros::NodeHandle n;
    ros::Publisher pose1_state = n.advertise<geometry_msgs::Point>("boat1_pose", 1000);
    ros::Publisher pose2_state = n.advertise<geometry_msgs::Point>("boat2_pose", 1000);
    ros::Publisher pose3_state = n.advertise<geometry_msgs::Point>("boat3_pose", 1000);
    ros::Publisher pose4_state = n.advertise<geometry_msgs::Point>("boat4_pose", 1000);
    ros::Publisher wind = n.advertise<geometry_msgs::Quaternion>("wind_angle", 1000);
    ros::Publisher cap1 = n.advertise<geometry_msgs::Quaternion>("heading_boat1", 1000);
    ros::Publisher cap2 = n.advertise<geometry_msgs::Quaternion>("heading_boat2", 1000);
    ros::Publisher cap3 = n.advertise<geometry_msgs::Quaternion>("heading_boat3", 1000);
    ros::Publisher cap4 = n.advertise<geometry_msgs::Quaternion>("heading_boat4", 1000);
    ros::Publisher boat_pub = n.advertise<visualization_msgs::Marker>( "visualization_boat",0 );
    ros::Publisher sail_pub = n.advertise<visualization_msgs::Marker>( "visualization_sail",0 );
    ros::Publisher rudder_pub = n.advertise<visualization_msgs::Marker>( "visualization_rudder",0 );
    ros::Publisher wind_pub = n.advertise<visualization_msgs::Marker>( "visualization_wind",0 );
    ros::Subscriber commande1 = n.subscribe("actuators1", 1000, comm1Callback);
    ros::Subscriber commande2 = n.subscribe("actuators2", 1000, comm2Callback);
    ros::Subscriber commande3 = n.subscribe("actuators3", 1000, comm3Callback);
    ros::Subscriber commande4 = n.subscribe("actuators4", 1000, comm4Callback);

    geometry_msgs::Point msg_pose1, msg_pose2, msg_pose3, msg_pose4;
    geometry_msgs::Quaternion msg_wind;
    geometry_msgs::Quaternion msg_cap1, msg_cap2, msg_cap3, msg_cap4;

    geometry_msgs::PoseStamped msgs_boat;
    geometry_msgs::PoseStamped msgs_sail;
    geometry_msgs::PoseStamped msgs_rudder;

    visualization_msgs::Marker marker_boat;
    visualization_msgs::Marker marker_sail;
    visualization_msgs::Marker marker_rudder;
    visualization_msgs::Marker marker_wind;
    

    geometry_msgs::TransformStamped transformStamped_boat;
    tf2_ros::TransformBroadcaster br_boat;

    geometry_msgs::TransformStamped transformStamped_sail;
    tf2_ros::TransformBroadcaster br_sail;

    geometry_msgs::TransformStamped transformStamped_rudder;
    tf2_ros::TransformBroadcaster br_rudder;

    n.param<double>("boatx1", x1[0], 0);
    n.param<double>("boaty1", x1[1], 0);
    n.param<double>("boatx2", x2[0], 0);
    n.param<double>("boaty2", x2[1], 0);
    n.param<double>("boatx3", x3[0], 0);
    n.param<double>("boaty3", x3[1], 0);
    n.param<double>("boatx4", x4[0], 0);
    n.param<double>("boaty4", x4[1], 0);
    n.param<double>("psi_wind", psi_w, 0);
    ros::Rate loop_rate(100);
    while (ros::ok()) {
        

        //ROS_INFO("psi_w=%f", psi_w);
        //ROS_INFO("boat_x=%f", x[0]);
        //ROS_INFO("boat_head=%f", x[2]);
        // Publication of the wind angle
        tf::Quaternion q_wind;
        q_wind.setRPY(0, 0, psi_w);
        tf::quaternionTFToMsg(q_wind, msg_wind);
        wind.publish(msg_wind);

        // Publication of the position
        f(ur1,us1,x1,xdot,psi_w, delta_s1);
        Euler_integration(x1,xdot);
        f(ur2,us2,x2,xdot,psi_w, delta_s2);
        Euler_integration(x2,xdot);
        f(ur3,us3,x3,xdot,psi_w, delta_s3);
        Euler_integration(x3,xdot);
        f(ur4,us4,x4,xdot,psi_w, delta_s4);
        Euler_integration(x4,xdot);
        
        publication_pose(msg_pose1, x1);
        publication_pose(msg_pose2, x2);
        publication_pose(msg_pose3, x3);
        publication_pose(msg_pose4, x4);
        pose1_state.publish(msg_pose1);
        pose2_state.publish(msg_pose2);
        pose3_state.publish(msg_pose3);
        pose4_state.publish(msg_pose4);

        //Publication of the angle of the boat
        tf::Quaternion q_cap;
        q_cap.setRPY(0, 0, x1[2]);
        tf::quaternionTFToMsg(q_cap, msg_cap1);
        cap1.publish(msg_cap1);
        q_cap.setRPY(0, 0, x2[2]);
        tf::quaternionTFToMsg(q_cap, msg_cap2);
        cap2.publish(msg_cap2);
        q_cap.setRPY(0, 0, x3[2]);
        tf::quaternionTFToMsg(q_cap, msg_cap3);
        cap3.publish(msg_cap3);
        q_cap.setRPY(0, 0, x4[2]);
        tf::quaternionTFToMsg(q_cap, msg_cap4);
        cap4.publish(msg_cap4);

        print_boat(marker_boat, transformStamped_boat, br_boat, msgs_boat, "boat1", x1, couleur1, 0);
        print_sail(marker_sail, transformStamped_sail, br_sail, msgs_sail, "boat1", "sail1", delta_s1, 0);
        print_rudder(marker_rudder, transformStamped_rudder, br_rudder, msgs_rudder,"boat1", "rudder1", ur1, 0);
        boat_pub.publish( marker_boat );
        sail_pub.publish( marker_sail );
        rudder_pub.publish( marker_rudder );
        print_boat(marker_boat, transformStamped_boat, br_boat, msgs_boat, "boat2", x2, couleur2, 0);
        print_sail(marker_sail, transformStamped_sail, br_sail, msgs_sail, "boat2", "sail2", delta_s2, 0);
        print_rudder(marker_rudder, transformStamped_rudder, br_rudder, msgs_rudder, "boat2", "rudder2", ur2, 0);
        sail_pub.publish( marker_sail );
        rudder_pub.publish( marker_rudder );
        boat_pub.publish( marker_boat );
        print_boat(marker_boat, transformStamped_boat, br_boat, msgs_boat, "boat3", x3, couleur3, 0);
        print_sail(marker_sail, transformStamped_sail, br_sail, msgs_sail, "boat3", "sail3",delta_s3, 0);
        print_rudder(marker_rudder, transformStamped_rudder, br_rudder, msgs_rudder, "boat3" ,"rudder3", ur3, 0);
        sail_pub.publish( marker_sail );
        rudder_pub.publish( marker_rudder );
        boat_pub.publish( marker_boat );
        print_boat(marker_boat, transformStamped_boat, br_boat, msgs_boat, "boat4", x4, couleur4, 0);
        print_sail(marker_sail, transformStamped_sail, br_sail, msgs_sail, "boat4", "sail4", delta_s4, 0);
        print_rudder(marker_rudder, transformStamped_rudder, br_rudder, msgs_rudder, "boat4", "rudder4", ur4, 0);
        boat_pub.publish( marker_boat );
        sail_pub.publish( marker_sail );
        rudder_pub.publish( marker_rudder );
        

        //visualisation wind
        marker_wind.header.frame_id = "map";
        marker_wind.header.stamp = ros::Time();
        marker_wind.ns = "wind";
        marker_wind.id = 0;
        marker_wind.type = visualization_msgs::Marker::ARROW;
        marker_wind.action = visualization_msgs::Marker::ADD;
        marker_wind.pose.position.z=0;
        marker_wind.scale.y = 1.0;
        marker_wind.scale.z = 1.0;
        marker_wind.color.a = 1.5;
        marker_wind.pose.position.x = 0;//10*cos(psi_w);
        marker_wind.pose.position.y = 0;//10*sin(psi_w);
        tf::quaternionTFToMsg(q_wind, marker_wind.pose.orientation);
        marker_wind.scale.x = 4.0;
        marker_wind.color.r = 1.0f;
        marker_wind.color.g = 1.0f;
        marker_wind.color.b = 0.0f;

        //pose_boat_pub.publish(marker_B);
        wind_pub.publish( marker_wind );

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
