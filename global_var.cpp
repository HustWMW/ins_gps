#include"global_var.h"
#include <math.h>
#include <iostream>
using namespace std;



double degree2arc(double degree)
{
	return (degree*PI/180);
}

double arc2degree(double arc)
{
	double degree = arc/PI*180;
	if(degree > 180)
		degree -= 360;
	if(degree < -180)
		degree += 360;
	return degree;
}
Eigen::Vector3d arc2degreeVector3d(Eigen::Vector3d arc)
{
  Eigen::Vector3d degree;
  degree << arc2degree(arc(0)),arc2degree(arc(1)),arc2degree(arc(2));
  return degree;
}

double arc2degree2(double arc)
{
	double degree = arc/PI*180;
	if(degree > 360)
		degree -= 360;
	if(degree < 0)
		degree += 360;
	return degree;
}

Eigen::Matrix3d rotationVector2X(Eigen::Vector3d rotation_vector)
{
  Eigen::Matrix3d matX;
  matX <<                   0,    -rotation_vector(2),   rotation_vector(1),
	   rotation_vector(2),                      0,  -rotation_vector(0),
	  -rotation_vector(1),     rotation_vector(0),                    0;
  return matX;
}


Eigen::Matrix3d rotationVector2Matrix(Eigen::Vector3d rotation_vector)
{
//   //捷联惯导算法与组合导航原理中的罗德里格旋转公式
//   double VecNorm = rotation_vector.norm();
//   double a,b;
//   if(VecNorm < 1.0E-8)
//   {
//     a = 1-VecNorm*(1/6-VecNorm/120);
//     b = 0.5-VecNorm*(1/24-VecNorm/720);
//   }
//   else
//   {
//     double sqrt_VecNorm = sqrt(VecNorm);
//     a = sin(sqrt_VecNorm);//sqrt_VecNorm;
//     b = (1-cos(sqrt_VecNorm));//VecNorm;
//   }
//   Eigen::Matrix3d VX = rotationVector2X(rotation_vector);
//   Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity()+a*VX+b*VX*VX;
//   return rotation_matrix;
  //SLAM十四讲中的罗德里格斯公式
  double VecNorm = rotation_vector.norm();
  Eigen::Vector3d vec_n = rotation_vector/VecNorm;
  double cos_a = cos(VecNorm);
  Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity()*cos_a+(1-cos_a)*vec_n*vec_n.transpose()+sin(VecNorm)*rotationVector2X(vec_n);
  return rotation_matrix;
}

Eigen::Matrix3d attitude2Matrix(Eigen::Vector3d attitude)
{
  Eigen::Matrix3d rotation_matrix;
  double sin_i = sin(attitude(1));
  double cos_i = cos(attitude(1));
  double sin_j = sin(attitude(2));
  double cos_j = cos(attitude(2));
  double sin_k = sin(attitude(0));
  double cos_k = cos(attitude(0));
  rotation_matrix <<cos_j*cos_k-sin_i*sin_j*sin_k,         -cos_i*sin_k,        sin_j*cos_k+sin_i*cos_j*sin_k,
		    cos_j*sin_k+sin_i*sin_j*cos_k,          cos_i*cos_k,        sin_j*sin_k-sin_i*cos_j*cos_k,
		    -cos_i*sin_j,                                 sin_i,                          cos_i*cos_j;
  return rotation_matrix;
}

Eigen::Quaterniond attitude2Qua(Eigen::Vector3d attitude)
{
  double sin_i = sin(attitude(1)/2);
  double cos_i = cos(attitude(1)/2);
  double sin_j = sin(attitude(2)/2);
  double cos_j = cos(attitude(2)/2);
  double sin_k = sin(attitude(0)/2);
  double cos_k = cos(attitude(0)/2);
  Eigen::Quaterniond Qnb(cos_k*cos_i*cos_j-sin_k*sin_i*sin_j,
			 cos_k*sin_i*cos_j-sin_k*cos_i*sin_j,
			 sin_k*sin_i*sin_j+cos_k*cos_i*sin_j,
			 sin_k*cos_i*cos_j+cos_k*sin_i*sin_j);
  return Qnb;
}
 
 
Eigen::Quaterniond rotationVector2Qua(Eigen::Vector3d rotation_vector)
{
  double VecNorm = rotation_vector.norm();
  double VecNorm2= VecNorm*VecNorm;
  double qx,q1,q2,q3,temp;
  if(VecNorm < 1E-8)
  {
    qx = 1-VecNorm2*(1/8.0 - VecNorm2/384.0);
    temp = 1/2.0 - VecNorm2/48.0;
  }
  else
  {
    qx = cos(VecNorm/2.0);
    temp = (sin(VecNorm/2.0))/VecNorm;
  }
  Eigen::Quaterniond Qnn(qx,rotation_vector(0)*temp,rotation_vector(1)*temp,rotation_vector(2)*temp);
  return Qnn;
}

Eigen::Vector3d Matrix2attitude(Eigen::Matrix3d Cnb)
{
  //寒假写的
  Eigen::Vector3d attitude;
  if(abs(Cnb(2,1))<=0.999999)
  {
    attitude << asin(Cnb(2,1)),-atan2(Cnb(2,0),Cnb(2,2)),-atan2(Cnb(0,1),Cnb(1,1));
  }
  else
    attitude << asin(Cnb(2,1)),atan2(Cnb(0,2),Cnb(0,0)),0;
  return attitude;

  //王新龙仿真方法
  //double psi = atan2((-Cnb(0,1)),Cnb(1,1));
}
    
Eigen::Vector3d Qua2attitude(Eigen::Quaterniond Qnb)
{
  Eigen::Vector3d attitude;
  attitude = Matrix2attitude(Qnb.matrix());
  return attitude;
}
  
  

