#include "wmwSP.h"
#include "global_var.h"
#include "imu.h"
#include<stdio.h>      /*标准输入输出定义*/
#include<string.h>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "logger.h"

using namespace std;
//using namespace Eigen;
int main()
{
  string LogFile;
  LogFile = "./Log/InfoLog.log";
  initLogger(LogFile);
  GetCurrentWorkDir();

  int fd;                            //文件描述符
  int err;                           //返回调用函数的状态                      
  char rcv_buf[100];  
  long int main_counter = 0;
  Eigen::Vector3d gyro_degree;
  INSData average_imu_data;
  average_imu_data.delta_A << 0.0,0.0,0.0;
  average_imu_data.delta_G << 0.0,0.0,0.0; 
  RunMode cur_mode;

  fd = UART0_Open(fd); //打开串口，返回文件描述符
  do {
    err = UART0_Init(fd,460800,0,8,1,'N');
    printf("Set Port Exactly!\n");
    LOG(INFO)<<"Set Port Exactly!";
  }while(FALSE == err || FALSE == fd);

  //初始化ＩＭＵ校准参数　（先取无校准的参数，后面再修改）
  Eigen::Vector3d AB(0.0,0.0,0.0),AS(1.0,1.0,1.0),GB(0.0,0.0,0.0),GS(1.0,1.0,1.0);
  SetAccAndGyroBiasAndScale(AB,AS,GB,GS);
  cout.width(10);
  cout.unsetf(ios::left);

  InitSystemState init_system;
  INS InsUpdate(init_system);
  InsUpdate.CalculateEarthParamaters();
  Eigen::Vector3d attitude;
  Eigen::Vector2d alig_variance_EN_speed_(0.01,0.01);                                                          //alig_Pk中的方差项(也是Qk中的方差项)
  Eigen::Vector3d alig_variance_ENU_platform_error_angle_(degree2arc(1.0),degree2arc(1.0),degree2arc(1.0));    //alig_Pk中的方差项
  Eigen::Vector2d alig_variance_EN_gyro_bias_(0.001,0.001);                                                    //alig_Pk中的方差项
  Eigen::Vector3d alig_variance_ENU_acce_bias_(0.001,0.001,0.001);                                             //alig_Pk中的方差项
  Eigen::Vector2d alig_noise_EN_acce_(0.001,0.001);                //alig_Qk中的噪声项
  Eigen::Vector3d alig_noise_ENU_gyro_(0.001,0.001,0.001);         //alig_Qk中的噪声项
  Eigen::Vector2d alig_noise_EN_speed_(0.01,0.01);                 //alig_Qk中的噪声项
  InsUpdate.SetAligVarianceENSpeed(alig_variance_EN_speed_);
  InsUpdate.SetAligVarianceENUPlatformErrorAngle(alig_variance_ENU_platform_error_angle_);
  InsUpdate.SetAligVarianceENGyroBias(alig_variance_EN_gyro_bias_);
  InsUpdate.SetAligVarianceENUAcceBias(alig_variance_ENU_acce_bias_);
  InsUpdate.SetAligNoiseENAcce(alig_noise_EN_acce_);
  InsUpdate.SetAligNoiseENUGyro(alig_noise_ENU_gyro_);
  InsUpdate.SetAligNoiseENSpeed(alig_noise_EN_speed_);
  bool elaborate_alignment_init_finish = false;

  while (1) { //循环读取数据
    int read_out = UART0_Recv(fd, rcv_buf);
    if (read_out ==1) {
      CorrectImuData(acce,gyro);

      //取５００组平均数据，用于粗对准
      if (main_counter < 500) {
        average_imu_data.delta_A += acce;
        average_imu_data.delta_G += gyro;
        main_counter ++;
        cur_mode = GetAverageImuData;
        std::cout<<"getting average data "<<main_counter<<std::endl;
      } 

      //拿到平均数据之后，先进行粗对准，再对初始化精对准的一些变量
      else if (( main_counter >= 500)&&(!elaborate_alignment_init_finish)) {
        cur_mode = CursoryAlignment;
        average_imu_data.delta_A /= 500;
        average_imu_data.delta_G /= 500;
        LOG(INFO)<<" average_imu_data: "<< average_imu_data.delta_A.transpose()<<", "<<average_imu_data.delta_G.transpose();
        //直接对陀螺仪的数据进行校准，计算出的欧拉角，取用ＸＹ值，Ｚ值直接置零。
        InsUpdate.GetRudeCebMethod1(average_imu_data);  //计算出来的Ceb需要处理后才等同于旋转矩阵
        attitude = Matrix2attitude(InsUpdate.GetRudeCeb());
        attitude(0) = 0.0;
        LOG(INFO)<<"main_counter == 500!!!   attitude1 :"<<attitude.transpose();
        std::cout<<std::endl<<"attitude(radian):"<<attitude.transpose()<<std::endl;
        InsUpdate.SetRudeCeb(attitude2Matrix(attitude));  //把偏航角置零，重新计算Ceb，再赋值给rude_Ceb,方便精对准

        InsUpdate.OneFixCeb(average_imu_data);                         
        attitude = Matrix2attitude(InsUpdate.GetRudeCeb());
        std::cout<<"attitude after one fixed(radian):"<<attitude.transpose()<<std::endl;

        GB = average_imu_data.delta_G - InsUpdate.GetRudeCeb().inverse()*(-InsUpdate.GetCurEarthParamaters().Wnie);
        SetAccAndGyroBiasAndScale(AB,AS,GB,GS);
        elaborate_alignment_init_finish = InsUpdate.ElaborateAlignmentInit();
        //elaborate_alignment_init_finish = true;
        std::cout<<"already get 500 group data! ElaborateAlignmentInit"<<std::endl;
        if(elaborate_alignment_init_finish) {
          std::cout<<"########### ElaborateAlignmentInit finshed , ElaborateAlignment will begin ! ############"<<std::endl;
          LOG(INFO)<<"########### ElaborateAlignmentInit finshed , ElaborateAlignment will begin ! ############";
        }
        main_counter ++;
      }

      //粗对准并进行了一阶校准之后，进行精对准（程序中并没有对偏航角的数据进行低通滤波，而采取直接置零，后面准备直接取用ＧＰＳ的偏航角）
      else if ((main_counter >= 500)&&(main_counter < 1000)&&(elaborate_alignment_init_finish)) {
        cur_mode = ElaborateAlignment;
        InsUpdate.SetCurImuData(acce,gyro);
        InsUpdate.ElaborateAlignment();
        std::cout<<main_counter<<" speed: "<<InsUpdate.GetAligXk()(0)<<","<<InsUpdate.GetAligXk()(1)<<" platform_error_angle:"<<
        InsUpdate.GetAligXk()(2)<<","<<InsUpdate.GetAligXk()(3)<<","<<InsUpdate.GetAligXk()(4)<<std::endl;
        main_counter ++;
        //精对准结束后，把精对准的结果保存下来，为组合导航做准备
        if(main_counter == 1000) {
          //attitude << InsUpdate.GetAligXk()(2),InsUpdate.GetAligXk()(3),InsUpdate.GetAligXk()(4);  //注意此处的欧拉角顺序为ＸＹＺ
          attitude << InsUpdate.GetAligXk()(2),InsUpdate.GetAligXk()(3),0;
          InsUpdate.SetRudeCeb((Eigen::MatrixXd::Identity(3,3)+rotationVector2X(attitude))*InsUpdate.GetRudeCeb());
          attitude = Matrix2attitude(InsUpdate.GetRudeCeb());
          LOG(INFO)<<"ElaborateAlignment finished,   attitude (radian) :"<<attitude.transpose();
          std::cout<<"############ ElaborateAlignment finished,combined navigation will begin ! ##############"<<std::endl;
          std::cout<<"attitude(radian):"<<attitude.transpose()<<std::endl;
        }
      }

      //精对准之后，开始进行组合导航
      else if ((main_counter >= 1000)) {
        std::cout<<"combined navigation  !"<<std::endl;
      }
    }
  }
  UART0_Close(fd);
           
}
