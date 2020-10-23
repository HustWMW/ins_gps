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
  long int t;
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
  INS(){};

  //利用init_system_state初始化cur_state和pre_state
  INS(InitSystemState init_system_state);

  //利用ＩＭＵ原始数据和纬度信息，进行粗对准　　机械大楼经纬度：114.424937,30.518711
  void GetRudeCebMethod1(const INSData &average_imu_data);
  void GetRudeCebMethod2(const INSData &average_imu_data);

  //一次修正粗对准
  void OneFixCeb(const INSData &average_imu_data);

  //更新有关的地球参数(使用cur_state计算，结果保存到cur_earth中)
  void CalculateEarthParamaters();

  //纯惯性导航
  //int INS_update(INSData cur_data,INSData pre_data,InsState *cur_state,InsState pre_state,EarthPara &cur_earth);

  void SetCurImuData(Eigen::Vector3d acce,Eigen::Vector3d gyro);
  void SetPreImuData(Eigen::Vector3d acce,Eigen::Vector3d gyro,long int time);
  void SetCurInsState(InsState CurState) { cur_state = CurState;}
  void SetPreInsState(InsState PreState) { pre_state = PreState;}
  void SetCurEarthParamaters(EarthPara CurEarth) { cur_earth = CurEarth;}
  void SetRudeCeb(Eigen::Matrix3d Ceb) { rude_Ceb = Ceb;}

  INSData GetCurImuData() { return cur_imu_data;}
  INSData GetPreImuData() { return pre_imu_data;}
  InsState GetCurState() { return cur_state;}
  InsState GetPreState() { return pre_state;}
  EarthPara GetCurEarthParamaters() { return cur_earth;}
  Eigen::Matrix3d GetRudeCeb() { return rude_Ceb;}

private:
  INSData cur_imu_data;  //加速度单位为Ｇ，使用时需要再乘以cur_earth.GCC(2)
  INSData pre_imu_data;
  InsState cur_state;
  InsState pre_state;
  EarthPara cur_earth;
  struct timeval cur_time;
  Eigen::Matrix3d rude_Ceb;
};


#endif