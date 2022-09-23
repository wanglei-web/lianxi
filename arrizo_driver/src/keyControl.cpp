#include<iostream>
#include <unistd.h>
#include <linux/input.h>
#include <fcntl.h>
#include"arrizo_msgs/VehicleCtrlCmd.h"

#include<ros/ros.h>


using namespace std;

int main(int argc ,char **argv)
{
	int keys_fd ,mouse_fd;
	char ret[2];
	struct input_event keyEvent,mouseEvent;
	keys_fd=open("/dev/input/event4",O_RDONLY|O_NONBLOCK);//只读&非阻塞MODE，获取设备文件权限
	//  mouse_fd=open("/dev/input/event2",O_RDONLY|O_NONBLOCK);//只读&非阻塞 MODE
	ros::init(argc,argv,"key_control");
	
	ros::NodeHandle nh;
	
	ros::Publisher pub = nh.advertise<arrizo_msgs::VehicleCtrlCmd>("/VehicleCtrlCmd",10);
	
	double speed=0, steer=0, steer_right=0, brake=0;
	int didi=0, left_light=0, right_light=0, D_change=12, hand_brake=1;
	int keyboard=65535;
	arrizo_msgs::VehicleCtrlCmd cmd;
	
    	while(1)
	{   
		pub.publish(cmd);
		usleep(1000);
		read(keys_fd,&keyEvent,sizeof(struct input_event));
		// read(mouse_fd,&mouseEvent,sizeof(struct input_event));
		if(keyEvent.value==1)
		{
			//printf(" value %i type=%d \t code =%d\n", keyEvent.value,keyEvent.type,keyEvent.code);
			keyboard = keyEvent.code;
		}
		
		else
			continue;
		   
		 
		std::cout << keyboard << std::endl;
		if(keyboard==57) //设置自动驾驶模式
		{
			cmd.set_driverlessMode=2;

		}
		if(keyboard == 48)//B-hand_brake //设置手刹
		{
			if (hand_brake == 0)
				hand_brake = 1;
			else
				hand_brake = 0;
		}
		else if(keyboard==47)//V-gear//设置前进档
		{
			if (hand_brake == 0)
			{
				if (D_change == 12)
					D_change = 11;
				else
					D_change = 12;
			}
		}

		if (hand_brake == 1 || D_change == 12)
		{
			speed = 0;D_change = 12;

		}

		if (hand_brake == 0 && D_change == 11)
		{
			if (keyboard == 103)//up
			{
				if (speed + 1 > 30)
					speed = 30;
				else
					speed = speed + 1;
			}

			if (keyboard == 108)//down
			{
				if (speed - 5 < 0)
					speed = 0;
				else
					speed = speed -5;
			}

		}
		if (keyboard == 105)
		{
			if (steer + 20 > 500)
				steer = 500;
			else
				steer = steer + 20;
		}
		if (keyboard == 106)
		{
			if (steer - 20 < -500)
				steer = -500;
			else
				steer = steer - 20;
		}

		cmd.set_horn = didi;
		cmd.set_handBrake = hand_brake;
		cmd.set_turnLight_R = right_light;
		cmd.set_turnLight_L = left_light;
		cmd.set_gear = D_change;
		cmd.set_brake = brake;
		cmd.set_speed = speed;
		cmd.set_roadWheelAngle = steer;

		usleep(300000);
		if(keyboard==1)//对应Esc按键
		{
			close(keys_fd);
			return 0;
		}	
	}
		
	close(keys_fd);
	return 0;
}
