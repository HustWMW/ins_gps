#include<stdio.h>      /*标准输入输出定义*/
#include<stdlib.h>     /*标准函数库定义*/
#include<unistd.h>     /*Unix 标准函数定义*/
#include<sys/types.h> 
#include<sys/stat.h>   
#include<fcntl.h>      /*文件控制定义*/
#include<termios.h>    /*PPSIX 终端控制定义*/
#include<errno.h>      /*错误号定义*/
#include<string.h>
#include<math.h>


//宏定义
#define FALSE  -1
#define TRUE   0

 extern double acce[3],gyro[3],gyro_degree[3];;
 extern double q0, q1 , q2 , q3 ;
 extern double p_w[3];     //position in world frame
 extern double v_w[3];      //velocity in world frame 
 extern int order;
 extern signed int acc_uint[3],gyr_uint[3];
 extern double AccBias[3],AccScale[3],GyroBias[3],GyroScale[3];

 
 /*******************************************************************
* 名称：                  UART0_Open
* 功能：                打开串口并返回串口设备文件描述
* 入口参数：        fd    :文件描述符     port :串口号(ttyS0,ttyS1,ttyS2)
* 出口参数：        正确返回为1，错误返回为0
*******************************************************************/
int UART0_Open(int fd);

/*******************************************************************
* 名称：                UART0_Close
* 功能：                关闭串口并返回串口设备文件描述
* 入口参数：        fd    :文件描述符     port :串口号(ttyS0,ttyS1,ttyS2)
* 出口参数：        void
*******************************************************************/
 void UART0_Close(int fd);
 
 /*******************************************************************
* 名称：                UART0_Set
* 功能：                设置串口数据位，停止位和效验位
* 入口参数：        fd        串口文件描述符
*                              speed     串口速度
*                              flow_ctrl   数据流控制
*                           databits   数据位   取值为 7 或者8
*                           stopbits   停止位   取值为 1 或者2
*                           parity     效验类型 取值为N,E,O,,S
*出口参数：          正确返回为1，错误返回为0
*******************************************************************/
int UART0_Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity);

/*******************************************************************
* 名称：                UART0_Init()
* 功能：                串口初始化
* 入口参数：        fd       :  文件描述符   
*               speed  :  串口速度
*                              flow_ctrl  数据流控制
*               databits   数据位   取值为 7 或者8
*                           stopbits   停止位   取值为 1 或者2
*                           parity     效验类型 取值为N,E,O,,S
*                      
* 出口参数：        正确返回为1，错误返回为0
*******************************************************************/
int UART0_Init(int fd, int speed,int flow_ctrl,int databits,int stopbits,int parity);

/*******************************************************************
* 名称：                  UART0_Recv
* 功能：                接收串口数据
* 入口参数：        fd                  :文件描述符    
*                              rcv_buf     :接收串口中数据存入rcv_buf缓冲区中
*                              data_len    :一帧数据的长度
* 出口参数：        正确返回为1，错误返回为0
*******************************************************************/
int UART0_Recv(int fd, char *rcv_buf);

/********************************************************************
* 名称：                  UART0_Send
* 功能：                发送数据
* 入口参数：        fd                  :文件描述符    
*                              send_buf    :存放串口发送数据
*                              data_len    :一帧数据的个数
* 出口参数：        正确返回为1，错误返回为0
*******************************************************************/
int UART0_Send(int fd, char *send_buf,int data_len);

/******************************************************************
 * 名称：　SetAccAndGyroBiasAndScale
 * 功能：　设置加速度计和陀螺仪的零偏
 * 入口参数：　看参数变量名
 * 出口参数：　成功返回１，失败返回０
 * 作者：　王明旺，2020/10/6
 ******************************************************************/
int SetAccAndGyroBiasAndScale(double accBias[3],double accScale[3],double gyroBias[3],double gyroScale[3]);

/******************************************************************
 * CorrectImuData
 * 功能：　使用加速度计和陀螺仪的零偏以及比例误差纠正原始
 * 入口参数：　看参数变量名
 * 出口参数：　成功返回１，失败返回０
 * 作者：　王明旺，2020/10/6
 ******************************************************************/
int CorrectImuData(double *accData,double *gyroData);