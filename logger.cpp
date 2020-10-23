#include "logger.h"
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <string.h>
 
std::ofstream Logger::log_file;
std::string LogLevel[4] = { "INFO","WARNING","REEOR","FATAL"};
 
void initLogger(const std::string& log_filename){
   Logger::log_file.open(log_filename.c_str());
}
 
std::ostream& Logger::getStream(log_rank_t log_rank){
   return (log_file.is_open() ?log_file : std::cout) ;
}
 
std::ostream& Logger::start(log_rank_t log_rank,
                               const int line,
                               const std::string& function,const std::string& file) {
   time_t tm;
   time(&tm);
   char time_string[128];
   ctime_r(&tm, time_string);
   time_string[24] = ' ';  //把time_string末尾的换行符去掉
   //std::cout<<"time_string: "<<strlen(time_string)<<std::endl;
   std::string FileNameWithoutPath = GetFileNameWithoutPath(file);
   return getStream(log_rank) << time_string 
                               << "[" << LogLevel[log_rank] << "] file:"
                               << "[" << FileNameWithoutPath << "] func:"
                               << "[" << function << "] line:"
                               << "[" << line << "] "
                               << std::flush;
}

std::string Logger::GetFileNameWithoutPath(std::string fullName)
   {
      size_t position = fullName.find_last_of('/') + 1;
      return fullName.substr(position , fullName.length() - position );
   }
 
Logger::~Logger(){
   getStream(m_log_rank) << std::endl << std::flush;
   
   if (FATAL == m_log_rank) {
       log_file.close();
       abort();
    }
}