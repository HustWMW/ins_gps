#include "wmwSP.h"
#include "global_var.h"
#include "imu.h"
#include<stdio.h>      /*标准输入输出定义*/
#include<string.h>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <unistd.h>    /*getcwd()函数的头文件，为了输出程序运行的路径。*/
#include "logger.h"

using namespace std;
//using namespace Eigen;
int main()
{
  //输出当前程序运行目录
  char *CurrentWorkDir;
  CurrentWorkDir = getcwd(NULL, 0);
  string LogFile;
  LogFile = "./Log/InfoLog.log";
  initLogger(LogFile);
  LOG(INFO)<< "当前文件运行路径：" << CurrentWorkDir;

  int fd;                            //文件描述符
  int err;                           //返回调用函数的状态                      
  char rcv_buf[100];  
  long int main_counter = 0;
  Eigen::Vector3d gyro_degree;

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

  while (1) { //循环读取数据
    int read_out = UART0_Recv(fd, rcv_buf);
    if (read_out ==1) {
      CorrectImuData(acce,gyro);
      InsUpdate.SetCurImuData(acce,gyro);
      InsUpdate.GetRudeCebMethod1(InsUpdate.GetCurImuData());  //计算出来的Ceb需要处理后才等同于旋转矩阵
      attitude = Matrix2attitude(InsUpdate.GetRudeCeb());
      std::cout<<"attitude1 :"<<attitude.transpose();
      InsUpdate.SetRudeCeb(attitude2Matrix(attitude));  //不知道怎么整理旋转矩阵，就先换成姿态角，再转换成旋转矩阵吧。
      attitude = Matrix2attitude(InsUpdate.GetRudeCeb());
      std::cout<<",  attitude2 : "<<attitude.transpose()<<std::endl;    ////?????????????两个姿态角的顺序换了次序，还是得看一下global_var文件中的转换函数
      //Ceb = attitude2Matrix(attitude);
      //cout<<"Ceb 行列式：　"<<Ceb.determinant()<<" RRT = I ?"<<endl;
      //cout<<Ceb*Ceb.transpose()<<endl;
      //printf("attitude: %-10f,%-10f,%-10f\n ",attitude[0],attitude[1],attitude[2]);
      //InsUpdate.OneFixCeb(Ceb,imu,earth);  //暂时先不加一次修正，ＩＭＵ未校正数据误差太大，修正效果还不如没有。。。
      //attitude = Matrix2attitude(Ceb);
      gyro_degree = arc2degreeVector3d(gyro);
      main_counter ++;
      //cout<<main_counter<<" "<<" Att:"<<attitude[0]<<" " <<attitude[1]<<" "<<attitude[2];
      // printf(" Acc:%-10f,%-10f,%-10f, Gyr:%-10f,%-10f,%-10f,  attitude: %-10f,%-10f,%-10f\n ",
      // InsUpdate.GetCurImuData().delta_A[0],InsUpdate.GetCurImuData().delta_A[1],InsUpdate.GetCurImuData().delta_A[2],
      // gyro_degree[0],gyro_degree[1],gyro_degree[2],attitude[0],attitude[1],attitude[2]);
    }
  }  
  UART0_Close(fd); 
           
}
