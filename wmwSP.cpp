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
#include"wmwSP.h"
#include"logger.h"

// double acce[3],gyro[3],gyro_degree[3];;
// double q0, q1 , q2 , q3 ;
// double p_w[3];     //position in world frame
// double v_w[3];      //velocity in world frame 
int order =0;
signed int acc_uint[3],gyr_uint[3];
Eigen::Vector3d AccBias,AccScale,GyroBias,GyroScale;

Eigen::Vector3d acce, gyro;
bool first_flag = true, find_dollar = false;

int UART0_Open(int fd)
{
   
     fd = open( "/dev/ttyUSB0", O_RDWR|O_NOCTTY);//|O_NDELAY  没加O_NDELAY就是非阻塞
     if (FALSE == fd)
     {
        perror("Can't Open Serial Port");
        return(FALSE);
     }
     //恢复串口为阻塞状态                               
     if(fcntl(fd, F_SETFL, 0) < 0)
     {
        printf("fcntl failed!\n");
        return(FALSE);
     }     
     else
     {
        printf("fcntl=%d\n",fcntl(fd, F_SETFL,0));
     }
      //测试是否为终端设备    
     if(0 == isatty(STDIN_FILENO))
     {
        printf("standard input is not a terminal device\n");
        return(FALSE);
     }
     else
     {
	printf("isatty success!\n");
     }              
     printf("fd->open=%d\n",fd);
     return fd;
}

void UART0_Close(int fd)
{
    close(fd);
}

int UART0_Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity)
{
   
    int   i;
    int   status;
    int   speed_arr[] = {B460800,B115200, B19200, B9600, B4800, B2400, B1200, B300};
    int   name_arr[] = {460800,  115200,  19200,  9600,  4800,  2400,  1200,  300};
         
    struct termios options;
   
    /*tcgetattr(fd,&options)得到与fd指向对象的相关参数，并将它们保存于options,该函数还可以测试配置是否正确，该串口是否可用等。若调用成功，函数返回值为0，若调用失败，函数返回值为1.
    */
    if  ( tcgetattr( fd,&options)  !=  0)
    {
        perror("SetupSerial 1");    
        return(FALSE); 
    }
  
    //设置串口输入波特率和输出波特率
    for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++)
    {
       if  (speed == name_arr[i])
       {             
          cfsetispeed(&options, speed_arr[i]); 
          cfsetospeed(&options, speed_arr[i]);  
       }
    }     
   
    //修改控制模式，保证程序不会占用串口
    options.c_cflag |= CLOCAL;
    //修改控制模式，使得能够从串口中读取输入数据
    options.c_cflag |= CREAD;
  
    //设置数据流控制
    switch(flow_ctrl)
    {
      
       case 0 ://不使用流控制
              options.c_cflag &= ~CRTSCTS;
              break;   
      
       case 1 ://使用硬件流控制
              options.c_cflag |= CRTSCTS;
              break;
       case 2 ://使用软件流控制
              options.c_cflag |= IXON | IXOFF | IXANY;
              break;
    }
    //设置数据位
    //屏蔽其他标志位
    options.c_cflag &= ~CSIZE;
    switch (databits)
    {  
       case 5    :
                     options.c_cflag |= CS5;
                     break;
       case 6    :
                     options.c_cflag |= CS6;
                     break;
       case 7    :    
                 options.c_cflag |= CS7;
                 break;
       case 8:    
                 options.c_cflag |= CS8;
                 break;  
       default:   
                 fprintf(stderr,"Unsupported data size\n");
                 return (FALSE); 
    }
    //设置校验位
    switch (parity)
    {  
       case 'n':
       case 'N': //无奇偶校验位。
                 options.c_cflag &= ~PARENB; 
                 options.c_iflag &= ~INPCK;    
                 break; 
       case 'o':  
       case 'O'://设置为奇校验    
                 options.c_cflag |= (PARODD | PARENB); 
                 options.c_iflag |= INPCK;             
                 break; 
       case 'e': 
       case 'E'://设置为偶校验  
                 options.c_cflag |= PARENB;       
                 options.c_cflag &= ~PARODD;       
                 options.c_iflag |= INPCK;      
                 break;
       case 's':
       case 'S': //设置为空格 
                 options.c_cflag &= ~PARENB;
                 options.c_cflag &= ~CSTOPB;
                 break; 
       default:  
                 fprintf(stderr,"Unsupported parity\n");    
                 return (FALSE); 
    } 
    // 设置停止位 
    switch (stopbits)
    {  
       case 1:   
                 options.c_cflag &= ~CSTOPB; break; 
       case 2:   
                 options.c_cflag |= CSTOPB; break;
       default:   
                       fprintf(stderr,"Unsupported stop bits\n"); 
                       return (FALSE);
    }
    
    options.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
   
    //修改输出模式，原始数据输出
    options.c_oflag &= ~OPOST;
  
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);//我加的
    //options.c_lflag &= ~(ISIG | ICANON);
   
    //设置等待时间和最小接收字符
    options.c_cc[VTIME] = 1; /* 读取一个字符等待1*(1/10)s */  
    options.c_cc[VMIN] = 1; /* 读取字符的最少个数为1 */
   
    //如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读
    tcflush(fd,TCIFLUSH);
   
    //激活配置 (将修改后的termios数据设置到串口中）
    if (tcsetattr(fd,TCSANOW,&options) != 0)  
           {
               perror("com set error!\n");  
              return (FALSE); 
           }
    return (TRUE); 
}

int UART0_Init(int fd, int speed,int flow_ctrl,int databits,int stopbits,int parity)
{
    int err;
    //设置串口数据帧格式
    if (UART0_Set(fd,speed,flow_ctrl,databits,stopbits,parity) == FALSE)
    {                                                         
        return FALSE;
    }
    else
    {
        return  TRUE;
    }
}

int UART0_Recv(int fd, char *rcv_buf)
{
   int len,fs_sel;
   fd_set fs_read;
   short int one,two,three,four;
   int data[24];
   unsigned int temp[50];
   short sum;
   memset(rcv_buf,'\0',sizeof(rcv_buf));
   find_dollar = false;
   struct timeval time;

   FD_ZERO(&fs_read);
   FD_SET(fd,&fs_read);

   time.tv_sec = 0;
   time.tv_usec = 20000;

   //使用select实现串口的多路通信
   fs_sel = select(fd+1,&fs_read,NULL,NULL,&time);
   if (fs_sel) {
      while(first_flag)
      {
         len = read(fd,rcv_buf,1);
         if ( (unsigned char) rcv_buf[0] == 0x24) {
            len = read(fd,rcv_buf,3);                     //若发现$表示符，则再往后读三位，判断是否是ＡＤＩ
            if (((unsigned char) rcv_buf[0] == 0x41) &&
               ((unsigned char) rcv_buf[1] == 0x44)&&
               ((unsigned char) rcv_buf[2] == 0x49)) {
                  first_flag = false;
                  find_dollar = true;
                  printf("first try----Find $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\n");
            }
         }

      }
      if (!first_flag) {
         len = read(fd,rcv_buf,1);
         if ( (unsigned char) rcv_buf[0] == 0x24) {
            len = read(fd,rcv_buf,3);                     //若发现$表示符，则再往后读三位，判断是否是ＡＤＩ
            if (((unsigned char) rcv_buf[0] == 0x41) &&
               ((unsigned char) rcv_buf[1] == 0x44)&&
               ((unsigned char) rcv_buf[2] == 0x49)) {
               find_dollar = true;
               //printf("Find $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\n");
            }
         }
         else {
            find_dollar = false;
         }
      }
      if (find_dollar) {
         len=read(fd,rcv_buf,28);
         if (len == 28) {
         for(int i=0;i<len;i++) {
            temp[i]=(unsigned char)rcv_buf[i];
            //printf(" %x ",temp[i]);
            //printf(" %d ",temp[i]);
         }
         //LOG(INFO)<<std::hex<<temp;
         //printf("   %d\n ",len);
         acc_uint[0] = (((temp[3]&0xFF)<<24)|((temp[2]&0xFF)<<16)|((temp[1]&0xFF)<<8)|(temp[0]));
         acc_uint[1] = (((temp[7]&0xFF)<<24)|((temp[6]&0xFF)<<16)|((temp[5]&0xFF)<<8)|(temp[4]));
         acc_uint[2] = (((temp[11]&0xFF)<<24)|((temp[10]&0xFF)<<16)|((temp[9]&0xFF)<<8)|(temp[8]));
         //printf("  %f\n ",acce[0]);
         acce[0] = -acc_uint[0]/52428800.0;
         acce[1] = -acc_uint[1]/52428800.0;
         acce[2] = -acc_uint[2]/52428800.0;

         gyr_uint[0] = (((temp[15]&0xFF)<<24)|((temp[14]&0xFF)<<16)|((temp[13]&0xFF)<<8)|(temp[12]));
         gyr_uint[1] = (((temp[19]&0xFF)<<24)|((temp[18]&0xFF)<<16)|((temp[17]&0xFF)<<8)|(temp[16]));
         gyr_uint[2] = (((temp[23]&0xFF)<<24)|((temp[22]&0xFF)<<16)|((temp[21]&0xFF)<<8)|(temp[20]));
         gyro[0] = -gyr_uint[0]/655360.0;
         gyro[1] = -gyr_uint[1]/655360.0;
         gyro[2] = -gyr_uint[2]/655360.0;
         return 1;
         //printf("%5d  Acc:%-10f,%-10f,%-10f, Gyr:%-10f,%-10f,%-10f\n ",order,acce[0],acce[1],acce[2],gyro[0],gyro[1],gyro[2]);
         //order++;
         }
      }      
   } 
   else {
	   printf("select wrong");
      return 0;
   }     
}

int UART0_Send(int fd, char *send_buf,int data_len)
{
   int len = 0;
   
   len = write(fd,send_buf,data_len);
   if (len == data_len) {
      return len;
   }     
   else {
      tcflush(fd,TCOFLUSH);
      return FALSE;
   }
   
}


int SetAccAndGyroBiasAndScale(Eigen::Vector3d accBias,Eigen::Vector3d accScale,Eigen::Vector3d gyroBias,Eigen::Vector3d gyroScale)
{
      AccBias   = accBias;
      AccScale  = accScale;
      GyroBias  = gyroBias;
      GyroScale = gyroScale;
   //printf("SetAccAndGyroBiasAndScale ***********************************\n");
   //printf("AccBias:%-10f,%-10f,%-10f;  AccScale:%-10f,%-10f,%-10f; GyroBias:%-10f,%-10f,%-10f; GyroScale:%-10f,%-10f,%-10f;\n",
   //        AccBias[0],AccBias[1],AccBias[2],AccScale[0],AccScale[1],AccScale[2],
   //        GyroBias[0],GyroBias[1],GyroBias[2],GyroScale[0],GyroScale[1],GyroScale[2]);
   LOG(INFO) << "SetBiasScale: AccBias:"<<AccBias.transpose()<<" ,AccScale:"<<AccScale.transpose();
   LOG(INFO) << "SetBiasScale: GyroBias:"<<GyroBias.transpose()<<" ,GyroScale:"<<GyroScale.transpose();
   return 1;
}

int CorrectImuData(Eigen::Vector3d &accData,Eigen::Vector3d &gyroData)
{
   for( int i=0; i<3; i++) {
      accData[i] = (accData[i] - AccBias[i])/AccScale[i];
      gyroData[i] = (gyroData[i] - GyroBias[i])/GyroScale[i];
   }
   //printf("CorrectImuData***********************************************\n");
   LOG(INFO) << "CorrectImuData: Acc:"<<accData.transpose()<<" ,Gyro:"<<gyroData.transpose();
   return 1;
}