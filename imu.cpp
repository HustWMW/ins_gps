#include <iostream>
using namespace std;
// Eigen 部分
#include <Eigen/Core>
// 稠密矩阵的代数运算（逆，特征值等）
#include <Eigen/Dense>
#include "logger.h"
#include "global_var.h"
#include "imu.h"
#include <math.h>
#include <sys/time.h>


INS::INS(InitSystemState init_system_state)
{
  cur_state.t = init_system_state.t;
  cur_state.V_enu = init_system_state.V_init;
  cur_state.Pos_phiLamdaH = init_system_state.Pos_init;
  cur_state.attitude = init_system_state.attitude_init;
  pre_state = cur_state;
  //INSData cur_data;
}

 void INS::SetCurImuData(Eigen::Vector3d acce,Eigen::Vector3d gyro)
 {
   cur_imu_data.delta_A = acce;
   cur_imu_data.delta_G = gyro;
   gettimeofday(&cur_time,NULL);
   cur_imu_data.t = cur_time.tv_usec;
 }

 void INS::SetPreImuData(Eigen::Vector3d acce,Eigen::Vector3d gyro,long int time)
 {
   pre_imu_data.delta_A = acce;
   pre_imu_data.delta_G = gyro;
   pre_imu_data.t = time;
 }

void INS::GetRudeCebMethod1(const INSData &average_imu_data)
{
  Eigen::Vector3d acc = cur_earth.Gcc(2)*average_imu_data.delta_A;
  rude_Ceb << 0.0,    0.0,   1.0/(cur_earth.Gcc(2)*Wie*cur_earth.cos_phi),
         cur_earth.sin_phi/(cur_earth.cos_phi*cur_earth.Gcc(2)),   1.0/(Wie*cur_earth.cos_phi),  0.0,
         -1.0/cur_earth.Gcc(2),   0.0,  0.0;
  //Ceb = Eigen::MatrixXd::Identity(3,3);
  Eigen::Vector3d rb;
  rb = acc.cross(average_imu_data.delta_G);
  Eigen::Matrix3d temp = Eigen::MatrixXd::Identity(3,3);
  temp.row(0) = acc.transpose();
  //cout<< " delta_A: "<< temp.row(0) <<endl;
  temp.row(1) = average_imu_data.delta_G.transpose();
  temp.row(2) = rb.transpose();
  rude_Ceb = rude_Ceb*temp;
  LOG(INFO)<<"GetRudeCeb 1";
}

void INS::GetRudeCebMethod2(const INSData &average_imu_data)
{
  Eigen::Vector3d acc = cur_earth.Gcc(2)*average_imu_data.delta_A;
  Eigen::Vector3d fxw;
  fxw = acc.cross(average_imu_data.delta_G);
  Eigen::Vector3d fxwxf = fxw.cross(acc);
  rude_Ceb.row(0) = -(fxw/(fxw.norm())).transpose();
  //cout<< " delta_A: "<< temp.row(0) <<endl;
  rude_Ceb.row(1) = (fxwxf/(fxwxf.norm())).transpose();
  rude_Ceb.row(2) = (acc/(acc.norm())).transpose();
  LOG(INFO)<<"GetRudeCeb 2";
}

void INS::OneFixCeb(const INSData &average_imu_data)
{
  Eigen::Vector3d phi;
  Eigen::Vector3d Fn = rude_Ceb*average_imu_data.delta_A;
  //cout<<" Fn: "<<Fn.transpose()<<" delta_A: "<<average_imu_data.delta_A.transpose()<<endl;
  Eigen::Vector3d Wnib = rude_Ceb*average_imu_data.delta_G;
  phi(0) = -Fn(1)/cur_earth.Gcc(2);
  phi(1) = Fn(0)/cur_earth.Gcc(2);
  phi(2) = Wnib(0)/(Wie*cur_earth.cos_phi)+Fn(0)*cur_earth.sin_phi/(cur_earth.cos_phi*cur_earth.Gcc(2));
  cout<<"OneFixPhi:"<<phi.transpose()<<endl;
  rude_Ceb = (Eigen::MatrixXd::Identity(3,3)+rotationVector2X(phi))*rude_Ceb;
  LOG(INFO)<<"OneFixCeb";
}

void INS::CalculateEarthParamaters()
{  
   cur_earth.Pos_phiLamdaH = cur_state.Pos_phiLamdaH;
   double e2=EARTH_e*EARTH_e;
   cur_earth.sin_phi = sin(cur_state.Pos_phiLamdaH(0));
   cur_earth.cos_phi = cos(cur_state.Pos_phiLamdaH(0));
   double tan_phi = cur_earth.sin_phi/cur_earth.cos_phi;
   double sin2_phi = cur_earth.sin_phi * cur_earth.sin_phi;
   double temp = 1-e2*cur_earth.sin_phi*cur_earth.sin_phi;
   double sqrt_temp = sqrt(temp);
   cur_earth.Rnh = EARTH_a / sqrt_temp + cur_state.Pos_phiLamdaH(2);
   cur_earth.Rmh = EARTH_a*(1-e2)/temp + cur_state.Pos_phiLamdaH(2);
   //cout<<EARTH_a / sqrt_temp<<"  "<<EARTH_a*(1-e2)/temp<<endl;
   cur_earth.Wnie << 0.0,Wie*cur_earth.cos_phi,Wie*cur_earth.sin_phi;
   cur_earth.Wnen << -cur_state.V_enu(1)/cur_earth.Rmh,
		       cur_state.V_enu(0)/cur_earth.Rnh,
		       cur_state.V_enu(0)*tan_phi/cur_earth.Rnh;
   cur_earth.Wnin = cur_earth.Wnie + cur_earth.Wnen;
   cur_earth.Wnien= cur_earth.Wnin + cur_earth.Wnie;
   double G_phi_h = Ga*(1+5.27094e-3*sin2_phi+2.32718e-5*sin2_phi*sin2_phi)-3.086e-6*cur_state.Pos_phiLamdaH(2);
   cur_earth.Gn << 0.0,0.0,G_phi_h;
   cur_earth.Gcc= cur_earth.Gn-cur_earth.Wnien.cross(cur_state.V_enu);
}

// int INS::INS_update(INSData cur_data,INSData pre_data,InsState *cur_state,InsState pre_state,EarthPara &cur_earth)
// {
//   cout.width(12);
//   cout.unsetf(ios::left);
//   double interval = INTERVAL;
//   pre_data.delta_A *=cur_earth.Gn(2);
//   //cout<<cur_earth.Gn(2)<<"  ";
//   get_earth_para(pre_state,&cur_earth);
//   cur_data.delta_A *=cur_earth.Gn(2);
//   //cout<<cur_earth.Gn(2)<<endl;
//   //cout<<cur_data.delta_A.transpose()<<"  "<<pre_data.delta_A.transpose()<<endl;
  
//   Eigen::Vector3d pre_delte_v =pre_data.delta_A*interval;
//   Eigen::Vector3d cur_delte_v =cur_data.delta_A*interval;
//   Eigen::Vector3d pre_delte_theta =pre_data.delta_G*interval;
//   Eigen::Vector3d cur_delte_theta =cur_data.delta_G*interval;
//   //速度更新
//   //Eigen::Matrix3d Mwnin = rotationVector2Matrix(interval*cur_earth.Wnin);
//   Eigen::Quaterniond Qwnin = rotationVector2Qua(cur_earth.Wnin*interval/2.0);
//   //cout<<(interval*cur_earth.Wnin).transpose()<<endl;
//   //Eigen::Matrix3d Cnb = attitude2Matrix(pre_state.attitude);
//   Eigen::Vector3d Vbb = cur_delte_v + cur_delte_theta.cross(cur_delte_v)/2.0+(pre_delte_theta.cross(cur_delte_v)+pre_delte_v.cross(cur_delte_theta))/12.0;
//   //cout<<cur_data.delta_A.transpose()<<"##"<<(cur_delte_theta.cross(cur_delte_v)/2.0).transpose()<<"##"<<((pre_delte_theta.cross(cur_delte_v)+pre_delte_v.cross(cur_delte_theta))/12.0).transpose()<<endl;;
//   //Eigen::Vector3d Vnb = Mwnin*Cnb*Vbb;
//   Eigen::Vector3d Vnb = Qwnin*(pre_state.Qnb*Vbb);
//   //cout<<Vnb.transpose()<<endl;
//   cur_state->V_delta = Vnb + cur_earth.Gcc*interval;
//   cur_state->V_enu = pre_state.V_enu + cur_state->V_delta;
//   //cout<<cur_state->V_enu.transpose()<<endl;
  
//   //位置更新
//   Eigen::Vector3d Vave = (pre_state.V_enu + cur_state->V_enu)/2.0;
//   Eigen::Matrix3d Mpv;
//   Mpv << 0,1/cur_earth.Rmh,0,
// 	 1.0/(cur_earth.Rnh*cur_earth.cos_phi),0,0,
// 	 0,0,1;
//   cur_state->Pos_phiLamdaH = pre_state.Pos_phiLamdaH + Mpv*Vave*interval;
  
//   //四元数和姿态角更新
//   //Eigen::Quaterniond Qnb = attitude2Qua(pre_state.attitude);
//   Eigen::Vector3d phi = (cur_data.delta_G + pre_data.delta_G.cross(cur_data.delta_G)/12.0)*interval;
//   Eigen::Quaterniond Qbb = rotationVector2Qua(phi);
//   Eigen::Quaterniond Qnn = rotationVector2Qua(-1.0*cur_earth.Wnin*interval);
//   cur_state->Qnb = (Qnn*(pre_state.Qnb)*Qbb).normalized();
//   cur_state->attitude = Qua2attitude(cur_state->Qnb);
//   //输出积分结果
//   //cout<<"att:"<<arc2degree(cur_state->attitude).transpose()<<" V_enu:"<<cur_state->V_enu.transpose()<<" Pos:"<<arc2degree(cur_state->Pos_phiLamdaH (0))<<" "<<arc2degree(cur_state->Pos_phiLamdaH (1))<<" "<<cur_state->Pos_phiLamdaH(2)<<endl; 
// }
  