#include "wmwSP.h"
#include "global_var.h"
#include<stdio.h>      /*标准输入输出定义*/
#include<string.h>
#include <iostream>
//#include <Eigen/Core>
// 稠密矩阵的代数运算（逆，特征值等）
//#include <Eigen/Dense>

using namespace std;
//using namespace Eigen;
int main()
{
    int fd;                            //文件描述符
    int err;                           //返回调用函数的状态
    int len;                        
    char rcv_buf[100];  
    long int main_counter = 0;
    fd = UART0_Open(fd); //打开串口，返回文件描述符
    do {
	    err = UART0_Init(fd,460800,0,8,1,'N');
	    printf("Set Port Exactly!\n");
	  }while(FALSE == err || FALSE == fd);
    //double Bias_x = 0.00081894045;  //寒假在家自己标定的，误差很大，需要更精确的平台来获取标定数据。
    //double Bias_y = -0.0172993811;
    //double Bias_z = -0.043428385;
    //double Scale_x = 1.00293068445;
    //double Scale_y = 1.0012066569;
    //double Scale_z = 1.003699648;
    double AB[3] = {0.00081894045,-0.0172993811,-0.043428385},AS[3] = {2.00293068445,1.0012066569,1.003699648},GB[3] = {0.0,0.0,0.0},GS[3] = {1.0,1.0,1.0};
    SetAccAndGyroBiasAndScale(AB,AS,GB,GS);
    cout.width(10);
    cout.unsetf(ios::left);
    while (1) { //循环读取数据
      int read_out = UART0_Recv(fd, rcv_buf);
      if (read_out ==1) {
        CorrectImuData(acce,gyro);
        gyro_degree[0] = degree2arc(gyro[0]);
        gyro_degree[1] = degree2arc(gyro[1]);
        gyro_degree[2] = degree2arc(gyro[2]);
        main_counter ++;
        cout<<main_counter<<" ";
        printf(" Acc:%-10f,%-10f,%-10f, Gyr:%-10f,%-10f,%-10f\n ",acce[0],acce[1],acce[2],gyro_degree[0],gyro_degree[1],gyro_degree[2]);
        //InsUpdate.INS_update(cur_data,pre_data,&cur_state,pre_state, cur_earth);
        //pre_data = cur_data;
        //pre_state = cur_state;
        //double distance_E = (cur_state.Pos_phiLamdaH(1)-init_system_state.Pos_phiLamdaH(1))*cur_earth.Rnh;
        //double distance_N = (cur_state.Pos_phiLamdaH(0)-init_system_state.Pos_phiLamdaH(0))*cur_earth.Rmh;
        //double distance_U = cur_state.Pos_phiLamdaH(2)-init_system_state.Pos_phiLamdaH(2);
        //cout<<"E-N-U distance:"<<distance_E<<" "<<distance_N<<" "<<distance_U<<endl;
        //printf("%ld Venu:%-10f,%-10f,%-10f, PhiLamdaH:%-10f,%-10f,%-10f\n ",main_counter,cur_state.V_enu(0),cur_state.V_enu(1),cur_state.V_enu(2),
        //arc2degree(cur_state.Pos_phiLamdaH(0)),arc2degree(cur_state.Pos_phiLamdaH(1)),cur_state.Pos_phiLamdaH(2));
        
      }
    }  
    UART0_Close(fd); 
           
}
