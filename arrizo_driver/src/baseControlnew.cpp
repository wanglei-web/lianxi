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
//{以下为usbcan使用：
#include "testSo.h"
#include "controlcan.h"
#include <pthread.h>

#define msleep(ms)  usleep((ms)*1000)
#define min(a,b)  (((a) < (b)) ? (a) : (b))

#define MAX_CHANNELS  2
#define CHECK_POINT  200
#define RX_WAIT_TIME  100
#define RX_BUFF_SIZE  1000
//}

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
uint8_t g_gear;
//{以下为usbcan使用：
unsigned gDevType = 4;
unsigned gDevIdx = 0;
unsigned gChMask = 3;
unsigned gBaud = 0x1c00;
unsigned gTxType = 2;
unsigned gTxSleep = 0;
unsigned gTxFrames = 3;
unsigned gTxCount = 1000;
//}
using namespace std;

unsigned s2n(const char *s)
{
    unsigned l = strlen(s);
    unsigned v = 0;
    unsigned h = (l > 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X'));
    unsigned char c;
    unsigned char t;
    if (!h) return atoi(s);
    if (l > 10) return 0;
    for (s += 2; c = *s; s++)
    {
        if (c >= 'A' && c <= 'F') c += 32;
        if (c >= '0' && c <= '9') t = c - '0';
        else if (c >= 'a' && c <= 'f') t = c - 'a' + 10;
        else return 0;
        v = (v << 4) | t;
    }
    return v;
}

void generate_frame(VCI_CAN_OBJ *can)
{
    memset(can, 0, sizeof(VCI_CAN_OBJ));
    can->SendType = gTxType;
    can->DataLen = 1 + (rand() % 8); // random data length: 1~8
    unsigned i;
    for (i = 0; i < can->DataLen; i++)
    {
        can->Data[i] = rand() & 0xff; // random data
        can->ID ^= can->Data[i]; // id: bit0~7, checksum of data0~N
    }
    can->ID |= ((unsigned)can->DataLen - 1) << 8; // id: bit8~bit10 = data_length-1
    can->ExternFlag = rand() % 2; // random frame format
    if (!can->ExternFlag)
        return;
    can->ID |= can->ID << 11; // id: bit11~bit21 == bit0~bit10
    can->ID |= can->ID << 11; // id: bit22~bit28 == bit0~bit7
}

int verify_frame(VCI_CAN_OBJ *can)
{
    if (can->DataLen > 8) return 0; // error: data length
    unsigned bcc = 0;
    unsigned i;
    for (i = 0; i < can->DataLen; i++)
        bcc ^= can->Data[i];
    if ((can->ID & 0xff) != bcc) return 0; // error: data checksum
    if (((can->ID >> 8) & 7) != (can->DataLen - 1)) return 0; // error: data length
    if (!can->ExternFlag) return 1; // std-frame ok
    if (((can->ID >> 11) & 0x7ff) != (can->ID & 0x7ff)) return 0; // error: frame id
    if (((can->ID >> 22) & 0x7f) != (can->ID & 0x7f)) return 0; // error: frame id
    return 1; // ext-frame ok
}

typedef struct {
    unsigned channel; // channel index, 0~1
    unsigned stop; // stop RX-thread
    unsigned total; // total received
    unsigned error; // error(s) detected
} RX_CTX;

void * rx_thread(void *data)
{
    RX_CTX *ctx = (RX_CTX *)data;
    ctx->total = 0; // reset counter

    VCI_CAN_OBJ can[RX_BUFF_SIZE]; // buffer
    int cnt; // current received
    int i;

    unsigned check_point = 0;
    while (!ctx->stop && !ctx->error)
    {
        cnt = VCI_Receive(gDevType, gDevIdx, ctx->channel, can, RX_BUFF_SIZE, RX_WAIT_TIME);
        if (!cnt)
            continue;

        for (i = 0; i < cnt; i++) {
            if (verify_frame(&can[i]))
                continue;
            printf("CAN%d: verify_frame() failed\n", ctx->channel);
            ctx->error = 1;
            break;
        }
        if (ctx->error) break;

        ctx->total += cnt;
        if (ctx->total / CHECK_POINT >= check_point) {
            printf("CAN%d: %d frames received & verified\n", ctx->channel, ctx->total);
            check_point++;
        }
    }

    printf("CAN%d RX thread terminated, %d frames received & verified: %s\n",
        ctx->channel, ctx->total, ctx->error ? "error(s) detected" : "no error");
    
    pthread_exit(0);

}
int test()
{
    // ----- init & start -------------------------------------------------

    VCI_INIT_CONFIG config;
    config.AccCode = 0;
    config.AccMask = 0xffffffff;
    config.Filter = 1;
    config.Mode = 0;
    config.Timing0 = gBaud & 0xff;
    config.Timing1 = gBaud >> 8;

    int i, j;
    for (i = 0; i < MAX_CHANNELS; i++)
    {
        if ((gChMask & (1 << i)) == 0) continue;

        if (!VCI_InitCAN(gDevType, gDevIdx, i, &config))
        {
            printf("VCI_InitCAN(%d) failed\n", i);
            return 0;
        }
        printf("VCI_InitCAN(%d) succeeded\n", i);

        if (!VCI_StartCAN(gDevType, gDevIdx, i))
        {
            printf("VCI_StartCAN(%d) failed\n", i);
            return 0;
        }
        printf("VCI_StartCAN(%d) succeeded\n", i);
    }

    // ----- RX-timeout test ----------------------------------------------

    VCI_CAN_OBJ can;
    time_t tm1, tm2;
    for (i = 0; i < 3; i++)
    {
        time(&tm1);
        VCI_Receive(gDevType, gDevIdx, 0, &can, 1, (i + 1) * 1000/*ms*/);
        time(&tm2);
        printf("VCI_Receive returned: time ~= %ld seconds\n", tm2 - tm1);
    }

    // ----- create RX-threads --------------------------------------------

    RX_CTX rx_ctx[MAX_CHANNELS];
    pthread_t rx_threads[MAX_CHANNELS];
    for (i = 0; i < MAX_CHANNELS; i++)
    {
        if ((gChMask & (1 << i)) == 0) continue;

        rx_ctx[i].channel = i;
        rx_ctx[i].stop = 0;
        rx_ctx[i].total = 0;
        rx_ctx[i].error = 0;

        pthread_create(&rx_threads[i], NULL, rx_thread, &rx_ctx[i]);

    }

    // ----- wait --------------------------------------------------------

    printf("<ENTER> to start TX: %d*%d frames/channel, baud: t0=0x%02x, t1=0x%02x...\n",
        gTxFrames, gTxCount, config.Timing0, config.Timing1);
    getchar();

    // ----- start transmit -----------------------------------------------

    VCI_CAN_OBJ *buff = (VCI_CAN_OBJ *)malloc(sizeof(VCI_CAN_OBJ) * gTxFrames);
    time(&tm1);
    int err = 0;
    unsigned tx;
    for (tx = 0; !err && tx < gTxCount; tx++)
    {
        for (i = 0; i < MAX_CHANNELS; i++)
        {
            if ((gChMask & (1 << i)) == 0) continue;

            for (j = 0; j < gTxFrames; j++)
            	generate_frame(&buff[j]);
            if (gTxFrames != VCI_Transmit(gDevType, gDevIdx, i, &buff[0], gTxFrames))
            {
                printf("CAN%d TX failed: ID=%08x\n", i, can.ID);
                err = 1;
                break;
            }
        }
        if (gTxSleep) msleep(gTxSleep);
    }
    time(&tm2);
    free(buff);

    // ----- stop TX & RX -------------------------------------------------

    msleep(1000);
    printf("TX stopped, <ENTER> to terminate RX-threads...\n");
    getchar();

    for (i = 0; i < MAX_CHANNELS; i++)
    {
        if ((gChMask & (1 << i)) == 0) continue;

        rx_ctx[i].stop = 1;

        pthread_join(rx_threads[i], NULL);

        if (rx_ctx[i].error)
            err = 1;
    }

    // ----- report -------------------------------------------------------

    if (err) {
        printf("error(s) detected, test failed\n");
        return 0;
    }

    printf("\n ***** %d frames/channel transferred, %ld seconds elapsed *****\n",
        gTxFrames * gTxCount, tm2 - tm1);
    if (tm2 - tm1)
        printf("        performance: %ld frames/channel/second\n", gTxFrames * gTxCount / (tm2 - tm1));

    return 1;
}


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
	//nh_private.param<std::string>("obd_can3_port_name", obd_can3_port_name_, "/dev/ttyUSB0");
	//nh_private.param<std::string>("obd_can1_port_name", obd_can1_port_name_, "/dev/ttyUSB0");
	nh_private.param<float>("max_steering_speed",max_steering_speed_,2.0); //通过限制前后帧转角命令差值 控制转向最大速度**
	nh_private.param<int>("steering_offset",steering_offset_,0);
	nh_private.param<bool>("default_drive_gear", default_drive_gear_, true);//是否默认为前进档
	
	//assert(!obd_can3_port_name_.empty());
	//assert(!obd_can1_port_name_.empty());
	assert(max_steering_speed_>0);

	vehicleCtrlCmd_sub_ = nh.subscribe("/VehicleCtrlCmd",1,&BaseControl::vehicleCtrlCmd_callback,this);
	vehicleState_pub_ = nh.advertise<arrizo_msgs::VehicleState>("/vehicleStateSet",10);
	
	timer_ = nh.createTimer(ros::Duration(0.03), &BaseControl::timer_callBack, this);
	
	/*if(!can2serial3.configure_port(obd_can3_port_name_.c_str()))
	{
		ROS_INFO("open port %s failed",obd_can3_port_name_.c_str());
		return false;
	}
	else
		ROS_INFO("open port %s successfully",obd_can3_port_name_.c_str());*/
	/*if(!can2serial1.configure_port(obd_can1_port_name_.c_str()))
	{
		ROS_INFO("open port %s failed",obd_can1_port_name_.c_str());
		return false;
	}
	else
		ROS_INFO("open port %s successfully",obd_can1_port_name_.c_str());*/	
	//can2serial3.clearCanFilter();
	//can2serial3.StartReading();
	//can2serial1.clearCanFilter();
	//can2serial1.StartReading();

	ROS_INFO("System initialization completed");
	return true;
}

void BaseControl::run()
{
	//boost::thread parse_msg(boost::bind(&BaseControl::parse_CanMsg,this)); 
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
				printf("当前状态/控制指令：车速：%0.1f / %0.1f, 转角：%0.1f / %0.1f，档位：%d / %d \n",vehicleState_.speed,g_cmdspeed,vehicleState_.roadwheel_angle,g_cmdangle,curGearState,g_gear);
				
			
				break;		
			default:
				break;
		}

	}
}

void BaseControl::timer_callBack(const ros::TimerEvent& event)
{
	vehicleState_pub_.publish(vehicleState_);
}


void BaseControl::vehicleCtrlCmd_callback(const arrizo_msgs::VehicleCtrlCmd::ConstPtr msg)
{

        float set_speed  =(msg->set_speed)*10;
	uint8_t set_gear =msg->set_gear;
	float set_roadWheelAngle =(msg->set_roadWheelAngle)*10;
	uint16_t set_driverlessMode = msg->set_driverlessMode;
	int16_t a,b;
	
	if (set_driverlessMode == 1)
	{
		set_driverlessMode = 2;
	}
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
	//printf("当前状态/控制指令：车速：%0.1f / %0.1f, 转角：%0.1f / %0.1f，档位：%d / %d \n",g_cmdspeed,g_cmdspeed,g_cmdangle,g_cmdangle,g_gear,g_gear);
	//ROS_INFO("speed=%f,  gear:%d,  Angle:%f",set_speed,set_gear,set_roadWheelAngle);

	//can2serial1.sendCanMsg(canMsg_cmd1_);
    	//can2serial3.sendCanMsg(canMsg_cmd3_);
	//vvprints();
}


int main(int argc,char**argv)
{

	BaseControl base_control;

	if(base_control.init(argc,argv))
	{
		if (!VCI_OpenDevice(gDevType, gDevIdx, 0)) {
        		printf("VCI_OpenDevice failed\n");
        		return 0;
    		}
		printf("VCI_OpenDevice succeeded\n");

		test();
		base_control.run();
	}
	printf("base_control_node has exited");
	VCI_CloseDevice(gDevType, gDevIdx);
    	printf("VCI_CloseDevice\n");
	return 0;
}

