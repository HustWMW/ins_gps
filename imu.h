#ifndef IMU_H
#define IMU_H

// Eigen 部分
#include <Eigen/Core>
// 稠密矩阵的代数运算（逆，特征值等）
#include <Eigen/Dense>

#include "global_var.h"



typedef struct init_system_state_tag
{
  double t;
  Eigen::Vector3d V_init;     //东北天坐标系（地理坐标系）中的速度
  Eigen::Vector3d Pos_init;   //纬度、经度、海拔
  Eigen::Vector3d attitude_init;  //初始姿态角
  init_system_state_tag()
  {
    t = 0;
    V_init << 0.0,0.0,0.0;
    Pos_init<<degree2arc(30.518711),degree2arc(114.424937),69.0;
    attitude_init<<0.0,0.0,0.0;
  }
}InitSystemState;


typedef struct INS_state_tag
{
  double t;
  Eigen::Vector3d V_enu;     //东北天坐标系（地理坐标系）中的速度
  Eigen::Vector3d V_delta;
  Eigen::Vector3d Pos_phiLamdaH;   //纬度、经度、海拔
  Eigen::Vector3d attitude;  //姿态角
  Eigen::Quaterniond Qnb;  
  INS_state_tag()
  {
    t = 0.0;
    V_enu << 0.0,0.0,0.0;
    V_delta << 0.0,0.0,0.0;
    Pos_phiLamdaH<<0.0,0.0,0.0;
    attitude<<0.0,0.0,0.0;
    //Qnb = Eigen::Vector3d(0,0,0);
  }
}InsState;

typedef struct INS_data_tag
{
  double t;
  Eigen::Vector3d delta_A;
  Eigen::Vector3d delta_G;
}INSData;

typedef struct EarthPara_tag
{
  double Rmh;
  double Rnh;
  double sin_phi;
  double cos_phi;
  Eigen::Vector3d Pos_phiLamdaH;   //纬度、经度、海拔
  Eigen::Vector3d Win;
  Eigen::Vector3d Wnie;
  Eigen::Vector3d Wnen;
  Eigen::Vector3d Wnin;
  Eigen::Vector3d Wnien;
  Eigen::Vector3d Gn;
  Eigen::Vector3d Gcc;
  EarthPara_tag()
  {
    Rmh=Rnh=0;
  }
}EarthPara;

typedef struct GPS_data_tag
{
  Eigen::Vector3d Pos_phiLamdaH;
  Eigen::Vector3d V_enu;
}GPSData;



class INS
{
public:
  //INS(){};
  INS(InitSystemState init_system_state);
  //void SetImuData(Eigen::Vector3d acce,Eigen::Vector3d gyro);
  //利用ＩＭＵ原始数据和纬度信息，进行粗对准　　机械大楼经纬度：114.424937,30.518711
  static void GetRudeCeb(Eigen::Matrix3d &Ceb,const INSData &cur_data,const EarthPara &cur_earth);
  //更新有关的地球参数
  static void get_earth_para(InsState ins_state,EarthPara *earth_para);
  //纯惯性导航
  static int INS_update(INSData cur_data,INSData pre_data,InsState *cur_state,InsState pre_state,EarthPara &cur_earth);

  InsState GetCurState() { return cur_state;}
private:
  //Eigen::Matrix3d Ceb;
  INSData cur_data;
  INSData pre_data;
  InsState cur_state;
  InsState pre_state;
  EarthPara cur_earth;
  
};


#endif