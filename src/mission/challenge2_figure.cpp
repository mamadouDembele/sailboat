#include "ros/ros.h"
#include "eigen3/Eigen/Dense"
#include <math.h>
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "tf/tf.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Float64.h"


using namespace std;

Eigen::Vector2d SK1, SK2={0,0};
Eigen::Vector2d m;
bool printSk2=false;
double r, r_true=0;

double norme(Eigen::Vector2d x)
{
    return sqrt(pow(x[0],2)+pow(x[1],2));
}


void rCallback(const std_msgs::Float64::ConstPtr& msg) {
    r=msg->data;
}

void skCallback(const geometry_msgs::Point::ConstPtr& msg) {
    SK1[0]=msg->x;
    SK1[1]=msg->y;
}



void poseCallback(const geometry_msgs::Point::ConstPtr& msg) {
    m[0]=msg->x;
    m[1]=msg->y;
}

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "node_sk_figure");
    ros::NodeHandle n;
    ros::Publisher circle1_pub = n.advertise<visualization_msgs::Marker>( "visualization_circleSK1",0 );
    ros::Publisher circle2_pub = n.advertise<visualization_msgs::Marker>( "visualization_circleSK2",0 );
    ros::Publisher cube_pub = n.advertise<visualization_msgs::Marker>( "visualization_cube",0 );
    ros::Publisher arrow_pub = n.advertise<visualization_msgs::Marker>( "visualization_arrow",0 );
    ros::Subscriber sub_r= n.subscribe("radius_r", 1000, rCallback);
    ros::Subscriber sub_sl= n.subscribe("sk", 1000, skCallback);
    ros::Subscriber pos_sail= n.subscribe("boat_pose", 1000, poseCallback);

    ros::Rate loop_rate(100);
    double t0 = ros::Time::now().toSec();
    while(ros::ok()){
        visualization_msgs::Marker points1, points2, marker_A, marker_c;

        double  tf = ros::Time::now().toSec();

        if (tf-t0<0.5)
        {
            ROS_INFO("Do anything");
            SK2=SK1;
        }
        
        else{

            if (SK1!=SK2)
                printSk2=true;
            for (int i=0; i<2; i++)
            {

                marker_A.header.frame_id = "map";
                marker_A.header.stamp = ros::Time::now();
                marker_A.ns = "arrow";
                marker_A.id = i;
                marker_A.action = visualization_msgs::Marker::ADD; 
                marker_A.type = visualization_msgs::Marker::ARROW;
                marker_A.pose.position.z=1;
                tf::Quaternion q;
                double phi=-M_PI/2;
                q.setRPY(0, phi, 0);
                tf::quaternionTFToMsg(q, marker_A.pose.orientation);
                marker_A.scale.x = 0.5;
                marker_A.scale.y = 0.5;
                marker_A.scale.z = 0.5;
                marker_A.color.a = 1.0;
                marker_A.color.r = 1.0f;
                marker_A.color.g = 1.0f;
                marker_A.color.b = 1.0f;

                marker_c.header.frame_id = "map";
                marker_c.header.stamp = ros::Time::now();
                marker_c.ns = "cube";
                marker_c.id = i;
                marker_c.action = visualization_msgs::Marker::ADD; 
                marker_c.type = visualization_msgs::Marker::CUBE;
                marker_c.pose.position.z=0.5;
                marker_c.pose.orientation.x=0;
                marker_c.pose.orientation.y=0;
                marker_c.pose.orientation.z=0;
                marker_c.pose.orientation.w=1;
                marker_c.scale.x = 0.5;
                marker_c.scale.y = 0.5;
                marker_c.scale.z = 1.0;
                marker_c.color.a = 1.0;
                marker_c.color.r = 1.0f;
                marker_c.color.g = 1.0f;
                marker_c.color.b = 1.0f;

                if (i==0)
                {
                    marker_A.pose.position.x = SK1[0];
                    marker_A.pose.position.y = SK1[1];
                    marker_c.pose.position.x = SK1[0];
                    marker_c.pose.position.y = SK1[1];
                }
                
                if (i==1 && printSk2==true)
                {
                    marker_A.pose.position.x = SK2[0];
                    marker_A.pose.position.y = SK2[1];
                    marker_c.pose.position.x = SK2[0];
                    marker_c.pose.position.y = SK2[1];
                }

                arrow_pub.publish(marker_A);
                cube_pub.publish(marker_c);
                
                
            }

            //Circle1
            points1.header.frame_id ="map";
            points1.header.stamp = ros::Time::now();
            points1.id = 0;
            points1.ns="Circlesk1";
            points1.action = visualization_msgs::Marker::ADD; 
            points1.type = visualization_msgs::Marker::LINE_STRIP;
            points1.pose.orientation.w=1;
            points1.scale.x = 0.2;
            //points.scale.y = 0.2;
            points1.color.a = 1.0;
            points1.color.r = 1.0f;
            points1.color.g = 0.0f;
            points1.color.b = 0.0f;
            if (norme(m-SK1)<r)
            {
                points1.color.r = 0.0f;
                points1.color.g = 1.0f;
                points1.color.b = 0.0f;
            }
            for (double i = 0.0; i < 1.1; i+=0.01)
            {
                double abs_x = SK1[0] + r * cos(2*i*M_PI);
                double ord_y = SK1[1] + r * sin(2*i*M_PI);
                geometry_msgs::Point p;
                p.x = abs_x;
                p.y = ord_y;
                p.z=0;
                points1.points.push_back(p);

            }

            //Circle2
            points2.header.frame_id ="map";
            points2.header.stamp = ros::Time::now();
            points2.id = 0;
            points2.ns="Circlesk2";
            points2.action = visualization_msgs::Marker::ADD; 
            points2.type = visualization_msgs::Marker::LINE_STRIP;
            points2.pose.orientation.w=1;
            points2.scale.x = 0.2;
            points2.color.a = 1.0;
            points2.color.r = 0.0f;
            points2.color.g = 1.0f;
            points2.color.b = 0.0f;
            for (double i = 0.0; i < 1.1; i+=0.01)
            {
                if (printSk2==true)
                    r_true=r;
                double abs_x = SK2[0] + r_true * cos(2*i*M_PI);
                double ord_y = SK2[1] + r_true * sin(2*i*M_PI);
                geometry_msgs::Point p;
                p.x = abs_x;
                p.y = ord_y;
                p.z=0;
                points2.points.push_back(p);

                }

            //ROS_INFO(" circle2 appear");
            circle2_pub.publish(points2);
            circle1_pub.publish(points1);
        }
        ros::spinOnce();
        loop_rate.sleep();
        
    }
    return 0;
}