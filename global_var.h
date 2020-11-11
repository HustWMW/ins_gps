#ifndef _global_variables_H
#define _global_variables_H

// Eigen 部分
#include <Eigen/Core>
// 稠密矩阵的代数运算（逆，特征值等）
#include <Eigen/Dense>
using namespace std;

// ins parameters
#define PI 3.1415926535898
#define LIGHT_V  2.99792458E+08
#define Wie  7.2921151467E-5
#define EARTH_a  6378136.998405
#define EARTH_b  6356752.3142
//#define EARTH_f  1 - EARTH_b / EARTH_a  
#define EARTH_f  1.0/298.257223563
#define EARTH_e (sqrt(EARTH_a*EARTH_a - EARTH_b*EARTH_b)/EARTH_a)
#define GM 3.986004418e+14
#define F_WGS84 1.0/298.257224
#define Ga 9.780325333434361
#define Gb 9.832184935381024

// GNSS parameters
#define EARTH_R 6378137.
#define GPS_NUM 32
#define GPS_L1 1.57542E+09
#define GPS_L2 1.2276E+09
#define LAMDA_Lw (LIGHT_V/(GPS_L1 - GPS_L2))
#define ELEV_MASK 20.0

#define _priorSigma 0.6
#define BAD_DCB_FLAG 70.0
#define Residual_Flag 5.0
#define BAD_P4_FLAG 150.0
#define ARC_LENGTH 40

#define INTERVAL 0.01

extern Eigen::Vector3d Weie;

//transform degree to arc 
double degree2arc(double degree);
Eigen::Vector3d degree2arcVector3d(Eigen::Vector3d arc);

//degree [-180, 180]
double arc2degree(double arc);

//degree [0, 360]
double arc2degree2(double arc);
Eigen::Vector3d arc2degreeVector3d(Eigen::Vector3d arc);

//roration_vector to antisymmetric_matrix
Eigen::Matrix3d rotationVector2X(Eigen::Vector3d rotation_vector);

//rotation_vector to rotation_matrix
Eigen::Matrix3d rotationVector2Matrix(Eigen::Vector3d rotation_vector);

//attitude (ZXY) to rotation_matrix
Eigen::Matrix3d attitude2Matrix(Eigen::Vector3d attitude);

//attitude (ZXY) to Quaterniond
Eigen::Quaterniond attitude2Qua(Eigen::Vector3d attitude);

//rotation_vector to Quaterniond
Eigen::Quaterniond rotationVector2Qua(Eigen::Vector3d rotation_vector);

//Quaterniond to attitude (ZXY)
Eigen::Vector3d Qua2attitude(Eigen::Quaterniond Qnb);

//Matrix to attitude(ZXY)
Eigen::Vector3d Matrix2attitude(Eigen::Matrix3d Cnb);

//龙格库塔方法
double RungeKutta2(double deltaT,double preF,double dF0,double dF1);

#endif
