#include "ros/ros.h"
#include <math.h>
#include "eigen3/Eigen/Dense"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float64.h"
#include "sailboat/Fonction.h"
double ks=1.0;
double Gamma=M_PI/4;
double zeta=M_PI/4, urmax=M_PI/4;

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
