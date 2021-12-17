#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>                 
#include <iostream>
#include <stdio.h>
#include <unistd.h>         
#include <wiringPi.h>　　
#include <wiringSerial.h> 
using namespace std;

int fd;

void Callback(const std_msgs::String::ConstPtr& vel,fd)
{
cout << "Linear x:" << vel->linear.x << endl;
cout << "Linear y:" << vel->linear.y << endl;
cout << "Linear z:" << vel->linear.z << endl;
cout << "Angular z:" << vel->angular.z << endl;
char vx,vy,vz,rz;
vx = char(vel->linear.x);
vy = char(vel->linear.y);
vz = char(vel->linear.z);
rz = char(vel->angular.z);
serialPutchar(fd,vx);
serialPutchar(fd,vy);
serialPutchar(fd,vz);
serialPutchar(fd,rz);
}

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "motor_test");
  ros::NodeHandle n;
  
  // シリアルポートオープン，ボーレート（19200）はESP32側と統一すること
  fd = serialOpen("/dev/serial0",19200);    	
  wiringPiSetup();
  fflush(stdout);
  //シリアルポートを開くことが出来たか否かの確認
  if(fd<0) printf("can not open serialport\n");    
  else       printf("serialport opened\n");
  
  ros::Subscriber sub = n.subscribe("/cmd_vel", 10, Callback);

  ros::spin();

  return 0;
}
