#include<iostream>
#include <unistd.h>
#include <linux/input.h>
#include <fcntl.h>
#include"arrizo_msgs/VehicleCtrlCmd.h"
#include"arrizo_msgs/VehicleState.h"//
#include<ros/ros.h>


using namespace std;

int main(int argc ,char **argv)
{
	int keys_fd ,mouse_fd;
	char ret[2];
	struct input_event keyEvent,mouseEvent;
	keys_fd=open("/dev/input/event1",O_RDONLY|O_NONBLOCK);//只读&非阻塞MODE
	//  mouse_fd=open("/dev/input/event2",O_RDONLY|O_NONBLOCK);//只读&非阻塞 MODE
	ros::init(argc,argv,"key_control");
	
	ros::NodeHandle nh;
	
	ros::Publisher pub1 = nh.advertise<arrizo_msgs::VehicleCtrlCmd>("/VehicleCtrlCmd",10);
    ros::Publisher pub2 = nh.advertise<arrizo_msgs::VehicleState>("/VehicleState",10);//
	
	double speed=0, steer=0, steer_right=0, brake=0;
	int didi=0, left_light=0, right_light=0, D_change=12, hand_brake=1;
	int keyboard=65535;
	arrizo_msgs::VehicleCtrlCmd cmd1;
    arrizo_msgs::VehicleState cmd2;//
	
    	while(1)
	{   
		pub1.publish(cmd1);
        pub2.publish(cmd2);//
		usleep(1000);
		read(keys_fd,&keyEvent,sizeof(struct input_event));
		// read(mouse_fd,&mouseEvent,sizeof(struct input_event));
		if(keyEvent.type==1 && keyEvent.value==1)
		{
			//printf(" value %i type=%d \t code =%d\n", keyEvent.value,keyEvent.type,keyEvent.code);
			keyboard = keyEvent.code;
		}
		
		else
			continue;
		   
		 
		std::cout << keyboard << std::endl;
		if(keyboard==57)
		{
			cmd2.set_driverlessMode=1;
	

		}
		if(keyboard == 48)//B-hand_brake
		{
			if (hand_brake == 0)
				hand_brake = 1;
			else
				hand_brake = 0;
		}
		else if(keyboard==47)//V-gear
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
//

		cmd1.set_horn = didi;
		cmd1.set_handBrake = hand_brake;
		cmd1.set_turnLight_R = right_light;
		cmd1.set_turnLight_L = left_light;
		cmd1.set_gear = D_change;
		cmd1.set_brake = brake;
		cmd1.set_speed = speed;
		cmd1.set_roadWheelAngle = steer;



		usleep(300000);
		if(keyboard==1)
		{
			close(keys_fd);
			return 0;
		}	
	}
		//ros::spin();
	close(keys_fd);
	return 0;
}
