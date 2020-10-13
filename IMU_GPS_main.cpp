#include "wmwSP.h"
#include "global_var.h"
#include "imu.h"
#include<stdio.h>      /*标准输入输出定义*/
#include<string.h>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
//using namespace Eigen;
int main()
{
    int fd;                            //文件描述符
    int err;                           //返回调用函数的状态                      
    char rcv_buf[100];  
    long int main_counter = 0;
    double gyro_degree[3];

    fd = UART0_Open(fd); //打开串口，返回文件描述符
    do {
	    err = UART0_Init(fd,460800,0,8,1,'N');
	    printf("Set Port Exactly!\n");
	  }while(FALSE == err || FALSE == fd);

    //初始化ＩＭＵ校准参数　（先取无校准的参数，后面再修改）
    Eigen::Vector3d AB(0.0,0.0,0.0),AS(1.0,1.0,1.0),GB(0.0,0.0,0.0),GS(1.0,1.0,1.0);
    SetAccAndGyroBiasAndScale(AB,AS,GB,GS);
    cout.width(10);
    cout.unsetf(ios::left);

    // InsState init_system_state;
    // init_system_state.Pos_phiLamdaH << degree2arc(30.518711),degree2arc(114.424937),69.0;
    // init_system_state.V_enu<< 0,0,0;
    // init_system_state.attitude << degree2arc(0.1),degree2arc(0.1),degree2arc(0.1);
    // init_system_state.Qnb = attitude2Qua(init_system_state.attitude);
    // INS InsUpdate(init_system_state);

    InitSystemState init_system;
    INS InsUpdate(init_system);

    Eigen::Matrix3d Ceb = Eigen::MatrixXd::Identity(3,3);
    INSData imu;
    EarthPara earth;
    InsUpdate.get_earth_para(InsUpdate.GetCurState(),&earth);
    Eigen::Vector3d attitude;

    while (1) { //循环读取数据
      int read_out = UART0_Recv(fd, rcv_buf);
      if (read_out ==1) {
        //CorrectImuData(acce,gyro);
        imu.delta_A = acce;
        imu.delta_G = gyro;
        InsUpdate.GetRudeCeb(Ceb,imu,earth);
        attitude = Matrix2attitude(Ceb);
        gyro_degree[0] = degree2arc(gyro[0]);
        gyro_degree[1] = degree2arc(gyro[1]);
        gyro_degree[2] = degree2arc(gyro[2]);
        main_counter ++;
        //cout<<main_counter<<" "<<" Att:"<<attitude[0]<<" " <<attitude[1]<<" "<<attitude[2];
        printf(" Acc:%-10f,%-10f,%-10f, Gyr:%-10f,%-10f,%-10f,  attitude: %-10f,%-10f,%-10f\n ",
        acce[0],acce[1],acce[2],gyro_degree[0],gyro_degree[1],gyro_degree[2],attitude[0],attitude[1],attitude[2]);
      }
    }  
    UART0_Close(fd); 
           
}
