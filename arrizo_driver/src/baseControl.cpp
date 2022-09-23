#include <ros/ros.h>
#include <iostream>
#include <assert.h>

#include <can2serial/can2serial.h>
#include <serial/serial.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <std_msgs/UInt64.h>
#include <std_msgs/String.h>

#include <arrizo_msgs/VehicleCtrlCmd.h>
#include <arrizo_msgs/VehicleState.h>

#include "testSo.h"
#include "controlcan.h"
/*
	档位状态描述:
	0x0: Neutral
	0x1: Driving
	0x2: Reverse
	
	控制模式描述:
	0x0: 手动驾驶模式
	0x1: 自定义驾驶模式
	0x2: 自动驾驶模式
*/



#define ID_STATE_can3			   0x206
#define ID_CMD_can3			       0x88

#define ID_CMD_can1			       0x200


#define STEERANGLE_ROADWHEELANGLE_RATIO  25.0
#define NEUTRAL_GEAR               0x0
#define DRIVE_GEAR                 0x1
#define REVERSE_GEAR               0x2

#define SPEED_CONTROL_MODE         0x0
#define THROTTLE_CONTROL_MODE      0x1
#define DEFAULT_CONTROL_MODE       THROTTLE_CONTROL_MODE

float g_cmdspeed, g_cmdangle;
uint8_t g_gear,g_set_driverlessMode;

using namespace std;

class BaseControl
{
public:
	BaseControl();
	~BaseControl();
	bool init(int ,char**);
	void run();
	
	void parse_CanMsg();
	void timer_callBack(const ros::TimerEvent& event);
	void vehicleCtrlCmd_callback(const arrizo_msgs::VehicleCtrlCmd::ConstPtr msg);
	
private:
	Can2serial can2serial3,can2serial1;
	
	bool default_drive_gear_;
	boost::shared_ptr<boost::thread> readFromStm32_thread_ptr_; 
	
	ros::Subscriber vehicleCtrlCmd_sub_;
	ros::Publisher vehicleState_pub_;
	ros::Timer timer_;
	
	std::string obd_can3_port_name_;
	std::string obd_can1_port_name_;

	float max_steering_speed_;  //Front and rear frame maximun steering angle difference
	int steering_offset_; 
	
	CanMsg_t canMsg_cmd3_;
	CanMsg_t canMsg_cmd1_;

	
	arrizo_msgs::VehicleState vehicleState_;
	//ant_msgs::ControlCmd1 vehicleCtrlCmd_;
	
	boost::mutex mutex_;
	
	float steer_angle;
	uint8_t curGearState;
	
	float curDeceleration;

	
	float curSteerWheelAngle;
	float curSteerWheelAngleSpeed;
	
	uint8_t curControlMode;
	
};

BaseControl::BaseControl()
{
    canMsg_cmd3_.ID = ID_CMD_can3;
    canMsg_cmd3_.len = 8;
    canMsg_cmd3_.type = Can2serial::STD_DATA_FRAME; //STD frame;
    
    *(long *)canMsg_cmd3_.data = 0;
	
    canMsg_cmd1_.ID = ID_CMD_can1;
    canMsg_cmd1_.len = 8;
    canMsg_cmd1_.type = Can2serial::STD_DATA_FRAME; //STD frame;
    
    *(long *)canMsg_cmd1_.data = 0;

}

BaseControl::~BaseControl()
{
	
}

bool BaseControl::init(int argc,char**argv)
{
	ros::init(argc,argv,"base_control");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	vvprints();
	nh_private.param<std::string>("obd_can3_port_name", obd_can3_port_name_, "/dev/ttyUSB0");
	nh_private.param<std::string>("obd_can1_port_name", obd_can1_port_name_, "/dev/ttyUSB0");
	nh_private.param<float>("max_steering_speed",max_steering_speed_,2.0); //通过限制前后帧转角命令差值 控制转向最大速度**
	nh_private.param<int>("steering_offset",steering_offset_,0);
	nh_private.param<bool>("default_drive_gear", default_drive_gear_, true);//是否默认为前进档
	
	//assert(!obd_can3_port_name_.empty());
	//assert(!obd_can1_port_name_.empty());//条件返回错误，则终止程序执行
	assert(max_steering_speed_>0);

	vehicleCtrlCmd_sub_ = nh.subscribe("/VehicleCtrlCmd",1,&BaseControl::vehicleCtrlCmd_callback,this);
	vehicleState_pub_ = nh.advertise<arrizo_msgs::VehicleState>("/vehicleStateSet",10);
	timer_ = nh.createTimer(ros::Duration(0.03), &BaseControl::timer_callBack, this);//定时器实时向上发布状态
	
	/*if(!can2serial3.configure_port(obd_can3_port_name_.c_str()))
	{
		ROS_INFO("open port %s failed",obd_can3_port_name_.c_str());
		return false;
	}
	else
		ROS_INFO("open port %s successfully",obd_can3_port_name_.c_str());
	if(!can2serial1.configure_port(obd_can1_port_name_.c_str()))
	{
		ROS_INFO("open port %s failed",obd_can1_port_name_.c_str());
		return false;
	}
	else
		ROS_INFO("open port %s successfully",obd_can1_port_name_.c_str());	
	can2serial3.clearCanFilter();
	can2serial3.StartReading();
	can2serial1.clearCanFilter();
	can2serial1.StartReading();*/

	ROS_INFO("System initialization completed");
	return true;
}

void BaseControl::run()
{
	 //这里使用多线程技术循环解析串口状态消息
	boost::thread parse_msg(boost::bind(&BaseControl::parse_CanMsg,this)); 
	ros::spin();
}

void BaseControl::parse_CanMsg()
{
	CanMsg_t canMsg;
	while(ros::ok())
	{
		usleep(3000);
		if(!can2serial3.getCanMsg(canMsg))
		{
			continue;
		}
		bool key = false;
	
		switch(canMsg.ID)
		{
			case ID_STATE_can3:
				//can2serial.showCanMsg(canMsg);
				
				curGearState = canMsg.data[1]&0x0f;//data[1]取后4位
				curControlMode = (canMsg.data[2]&0xc0) >> 6;//data[2]取前两位，再右移6位				
				
				
				vehicleState_.roadwheel_angle = (int16_t(((canMsg.data[3]&0xff) << 8)+ canMsg.data[4]) )* 0.1;//data[3]左移8位+data[4]
				vehicleState_.speed = (int16_t(((canMsg.data[0]&0xff) << 4) + ((canMsg.data[1]&0xf0 )>> 4) ) ) *0.1;//data[0]左移4位+data[1]取前4位再右移4位
				vehicleState_.driverless = (curControlMode == 0x02) ? true : false;
				
				if(curGearState == 11)
					vehicleState_.gear = arrizo_msgs::VehicleState::GEAR_DRIVE;
				else if(curGearState == 13)
					vehicleState_.gear = arrizo_msgs::VehicleState::GEAR_REVERSE;
				else 
					vehicleState_.gear = arrizo_msgs::VehicleState::GEAR_NEUTRAL;
				printf("当前状态/控制指令：车速：%0.1f / %0.1f, 转角：%0.1f / %0.1f，档位：%d / %d，模式：%d / %d \n",vehicleState_.speed,g_cmdspeed,vehicleState_.roadwheel_angle,g_cmdangle,curGearState,g_gear,curControlMode,g_set_driverlessMode);
				
				break;		
			default:
				break;
		}

	}
}

void BaseControl::timer_callBack(const ros::TimerEvent& event)
//回调函数自动带实参，和消息回调函数一样
{
	vehicleState_pub_.publish(vehicleState_);
}

void BaseControl::vehicleCtrlCmd_callback(const arrizo_msgs::VehicleCtrlCmd::ConstPtr msg)
{

    float set_speed  =(msg->set_speed)*10;
	uint8_t set_gear =msg->set_gear;
	float set_roadWheelAngle =(msg->set_roadWheelAngle)*10;
	uint8_t set_driverlessMode = msg->set_driverlessMode;
	int16_t a,b;
	

	canMsg_cmd1_.data[0] = ((set_driverlessMode) << 4) & 0x30;

	    canMsg_cmd3_.data[0] = uint16_t(set_speed)>> 4;
    	canMsg_cmd3_.data[1] = ((uint16_t(set_speed)&0x0f)<<4) + (uint8_t(set_gear)&0x0f) ;
    	canMsg_cmd3_.data[3] = uint8_t((int16_t(set_roadWheelAngle))>>8);
    	canMsg_cmd3_.data[4] = uint8_t(int16_t(set_roadWheelAngle));
        
        /*a= uint8_t((int16_t(set_roadWheelAngle))>>8);
        b = uint8_t(int16_t(set_roadWheelAngle));
        cout << a<<","<< b << endl;
        uint16_t c =(uint16_t (a))<<8;
        cout <<  int16_t(c+(uint16_t (b)))<< endl;*/ //负数拆分成两个无符号字节，再组合回去。目的是为了验证有符号情况下的拆分效果。
	g_cmdspeed = msg->set_speed; g_cmdangle = msg->set_roadWheelAngle;
	g_gear = set_gear;
	g_set_driverlessMode=set_driverlessMode;
	printf("当前状态/控制指令：车速：%0.1f / %0.1f, 转角：%0.1f / %0.1f，档位：%d / %d, 模式：%d / %d\n",g_cmdspeed,g_cmdspeed,g_cmdangle,g_cmdangle,g_gear,g_gear,g_set_driverlessMode,g_set_driverlessMode);
	//ROS_INFO("speed=%f,  gear:%d,  Angle:%f",set_speed,set_gear,set_roadWheelAngle);

	//can2serial1.sendCanMsg(canMsg_cmd1_);
        //can2serial3.sendCanMsg(canMsg_cmd3_);
	//vvprints();
}


int main(int argc,char**argv)
{

	BaseControl base_control;

	/*uint16_t a = 2;
	uint16_t b = (a << 4)& 0x30;
	if (b == 0x0020 ) cout<< "!~!" << endl;
	cout << b<< ".." << endl;
	*/ //

	/*uint8_t a = 100;
	float b = a<<8 ;
	cout << b << endl;
	ROS_INFO("b = %f",b);*/

	if(base_control.init(argc,argv))
		base_control.run();
	
	printf("base_control_node has exited");
	return 0;
}

