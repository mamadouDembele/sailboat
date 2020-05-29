#ifndef __FONCTION_H__
#define __FONCTION_H__

#include <math.h>
#include "eigen3/Eigen/Dense"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float64.h"

double sign(double x);
double norme(Eigen::Vector2d x);
double angle(Eigen::Vector2d x);
void publication_point(geometry_msgs::Point& msg, Eigen::Vector2d a);
void publication_float(std_msgs::Float64& msg, double a);
void f0(double p1, double p2, Eigen::Vector2d& v);
void f(Eigen::Vector2d m, Eigen::Matrix2d D, Eigen::Vector2d c, Eigen::Vector2d& v);
void controller_circle(Eigen::Vector2d m, double theta, double psi_w, Eigen::Matrix2d D, Eigen::Vector2d c, double& ur, double& us);
void controler_line(Eigen::Vector2d m, double theta, double psi_w, Eigen::Vector2d a, Eigen::Vector2d b, double& ur, double& us);

#endif