#include <iostream>
#include <cmath>
#include <string>

#include <aris.hpp>
#include "json.hpp"


#include <chrono>
#include <thread>
#include <algorithm>

#include "plan.h"
#include "kinematic.h"

#include "../lib/MCDLL_Define.h"
#include "../lib/MCDLL_NET.h"
#include "../lib/MCDLL_Return.h"
using namespace std;

#define AxisMax  18

const double A_Dangliang = 26497; // 6400 * 52 / 6.28
const double X_Dangliang = 3840;   // 6400 脉冲 = 2.5MM  6400/2.5 * 3
const double Y_Dangliang = 3840;


//参数定义
const double body[16] =
{
 1,0,0,0,
 0,1,0,0,
 0,0,1,0,
 0,0,0,1
};
const double h = 380;
const double d = 670;
const double xyz[18] =
{
 0,                       d,                        -h,
 -d * sin(3.1415926 / 3), d * cos(3.1415926 / 3),   -h,
 -d * sin(3.1415926 / 3),-d * cos(3.1415926 / 3),   -h,
 0,                      -d,                        -h,
 d * sin(3.1415926 / 3), -d * cos(3.1415926 / 3),   -h,
 d * sin(3.1415926 / 3),  d * cos(3.1415926 / 3),   -h
};

double input[18];
double input_c[18];
WalkParam param;
int ret = 1;
std::string str_command;
int step = 0;

auto sendToBoard(const double *input)
{
	const double A_Dangliang = 26497; // 6400 * 52 / 6.28
	const double X_Dangliang = 3840;   // 6400 脉冲 = 2.5MM  6400/2.5 * 3
	const double Y_Dangliang = 3840;
	
	double input_c[18];

	input_c[0] = -input[0] * A_Dangliang;
	input_c[1] = -input[1] * X_Dangliang;
	input_c[2] = -input[2] * Y_Dangliang;

	input_c[3] = -input[3] * A_Dangliang;
	input_c[4] = -input[4] * X_Dangliang;
	input_c[5] = -input[5] * Y_Dangliang;

	input_c[6] = -input[6] * A_Dangliang;
	input_c[7] = -input[7] * X_Dangliang;
	input_c[8] = -input[8] * Y_Dangliang;

	input_c[9] = -input[9] * A_Dangliang;
	input_c[10] = -input[10] * X_Dangliang;
	input_c[11] = -input[11] * Y_Dangliang;

	input_c[12] = -input[12] * A_Dangliang;
	input_c[13] = -input[13] * X_Dangliang;
	input_c[14] = -input[14] * Y_Dangliang;

	input_c[15] = -input[15] * A_Dangliang;
	input_c[16] = -input[16] * X_Dangliang;
	input_c[17] = -input[17] * Y_Dangliang;

	MCF_Uniaxial_dDist_Change_Net(Axis_13, input_c[0], Position_Absolute, 0);
	MCF_Uniaxial_dDist_Change_Net(Axis_1, input_c[1], Position_Absolute, 0);
	MCF_Uniaxial_dDist_Change_Net(Axis_2, input_c[2], Position_Absolute, 0);


	MCF_Uniaxial_dDist_Change_Net(Axis_14, input_c[3], Position_Absolute, 0);
	MCF_Uniaxial_dDist_Change_Net(Axis_3, input_c[4], Position_Absolute, 0);
	MCF_Uniaxial_dDist_Change_Net(Axis_4, input_c[5], Position_Absolute, 0);

	MCF_Uniaxial_dDist_Change_Net(Axis_15, input_c[6], Position_Absolute, 0);
	MCF_Uniaxial_dDist_Change_Net(Axis_5, input_c[7], Position_Absolute, 0);
	MCF_Uniaxial_dDist_Change_Net(Axis_6, input_c[8], Position_Absolute, 0);

	MCF_Uniaxial_dDist_Change_Net(Axis_16, input_c[9], Position_Absolute, 0);
	MCF_Uniaxial_dDist_Change_Net(Axis_7, input_c[10], Position_Absolute, 0);
	MCF_Uniaxial_dDist_Change_Net(Axis_8, input_c[11], Position_Absolute, 0);

	MCF_Uniaxial_dDist_Change_Net(Axis_17, input_c[12], Position_Absolute, 0);
	MCF_Uniaxial_dDist_Change_Net(Axis_9, input_c[13], Position_Absolute, 0);
	MCF_Uniaxial_dDist_Change_Net(Axis_10, input_c[14], Position_Absolute, 0);

	MCF_Uniaxial_dDist_Change_Net(Axis_18, input_c[15], Position_Absolute, 0);
	MCF_Uniaxial_dDist_Change_Net(Axis_11, input_c[16], Position_Absolute, 0);
	MCF_Uniaxial_dDist_Change_Net(Axis_12, input_c[17], Position_Absolute, 0);
};

void Prepair()
{


	//运动学参数初始化
	param.d = 300;   // MM
	param.h = 50;   // MM
	param.alpha = 1.57;
	param.beta = 0.0;
	//	 param.total_count = 2000;  //1ms
	param.total_count = 200;   //10ms
	param.n = 2;
	param.begin_pee_wrt_ground = xyz;
	param.begin_pm_wrt_ground = body;
	param.mot_pos = input;

	// TODO: 在此添加控件通知处理程序代码
	ret = walk_plan(0, &param);
	// 弧度 中间为 0    input[0]                        0.2弧度
	// X轴 往外为正数据 input[1]  长方向  单位 ：毫米   0 - 30mm 
	// Y轴 往下为正数据 input[2]  长方向  单位 ：毫米   0 - 30mm
	input_c[0] = -input[0] * A_Dangliang;
	input_c[1] = -input[1] * X_Dangliang;
	input_c[2] = -input[2] * Y_Dangliang;

	input_c[3] = -input[3] * A_Dangliang;
	input_c[4] = -input[4] * X_Dangliang;
	input_c[5] = -input[5] * Y_Dangliang;

	input_c[6] = -input[6] * A_Dangliang;
	input_c[7] = -input[7] * X_Dangliang;
	input_c[8] = -input[8] * Y_Dangliang;

	input_c[9] = -input[9] * A_Dangliang;
	input_c[10] = -input[10] * X_Dangliang;
	input_c[11] = -input[11] * Y_Dangliang;

	input_c[12] = -input[12] * A_Dangliang;
	input_c[13] = -input[13] * X_Dangliang;
	input_c[14] = -input[14] * Y_Dangliang;

	input_c[15] = -input[15] * A_Dangliang;
	input_c[16] = -input[16] * X_Dangliang;
	input_c[17] = -input[17] * Y_Dangliang;

	for (int j = 0; j < 18; j++)
	{
		MCF_Set_Axis_Profile_Net(j, 0, 50000, 10000000, 600000000, 0, 0, 0);
	}

	MCF_Uniaxial_dDist_Change_Net(Axis_13, input_c[0], Position_Absolute, 0);
	MCF_Uniaxial_dDist_Change_Net(Axis_1, input_c[1], Position_Absolute, 0);
	MCF_Uniaxial_dDist_Change_Net(Axis_2, input_c[2], Position_Absolute, 0);


	MCF_Uniaxial_dDist_Change_Net(Axis_14, input_c[3], Position_Absolute, 0);
	MCF_Uniaxial_dDist_Change_Net(Axis_3, input_c[4], Position_Absolute, 0);
	MCF_Uniaxial_dDist_Change_Net(Axis_4, input_c[5], Position_Absolute, 0);

	MCF_Uniaxial_dDist_Change_Net(Axis_15, input_c[6], Position_Absolute, 0);
	MCF_Uniaxial_dDist_Change_Net(Axis_5, input_c[7], Position_Absolute, 0);
	MCF_Uniaxial_dDist_Change_Net(Axis_6, input_c[8], Position_Absolute, 0);

	MCF_Uniaxial_dDist_Change_Net(Axis_16, input_c[9], Position_Absolute, 0);
	MCF_Uniaxial_dDist_Change_Net(Axis_7, input_c[10], Position_Absolute, 0);
	MCF_Uniaxial_dDist_Change_Net(Axis_8, input_c[11], Position_Absolute, 0);

	MCF_Uniaxial_dDist_Change_Net(Axis_17, input_c[12], Position_Absolute, 0);
	MCF_Uniaxial_dDist_Change_Net(Axis_9, input_c[13], Position_Absolute, 0);
	MCF_Uniaxial_dDist_Change_Net(Axis_10, input_c[14], Position_Absolute, 0);

	MCF_Uniaxial_dDist_Change_Net(Axis_18, input_c[15], Position_Absolute, 0);
	MCF_Uniaxial_dDist_Change_Net(Axis_11, input_c[16], Position_Absolute, 0);
	MCF_Uniaxial_dDist_Change_Net(Axis_12, input_c[17], Position_Absolute, 0);
}

int command = 0, state = 0;
enum  //状态常量
{
	INIT = 0,		//初始
	HOMED = 1,		//回原点
	HOMEFINISH = 2, //回原点结束
	PREPAIRD = 3,	 //准备
	RUNNING = 4,		//运行
	ERRO = 5,		 //错误
};

enum  //命令常量
{
	HOME = 0,      //回原点
	PREPAIR = 1,   //准备
	FORWARD = 2,   //前进
	BACK = 3,      //后退
	LEFT = 4,      //左移
	RIGHT = 5,      //右移
	TURNL = 6,      //左旋
	TURNR = 7		//右旋
};
int table_state[6][8] =       //状态表
{
   //       home    prepair  forward    back      lest     right   turnl    turnr
	{      HOMED,        -1,      -1,      -1,      -1,      -1,      -1,      -1},		//init0
	{ HOMEFINISH,        -1,      -1,      -1,      -1,      -1,      -1,      -1},		//homed1
	{ HOMEFINISH,  PREPAIRD,      -1,      -1,      -1,      -1,      -1,      -1},		//homefinish2
	{         -1,  PREPAIRD, RUNNING, RUNNING, RUNNING, RUNNING, RUNNING, RUNNING},		//prepaird3
	{         -1,        -1, RUNNING, RUNNING, RUNNING, RUNNING, RUNNING, RUNNING},		//running4
	{       ERRO,      ERRO,    ERRO,    ERRO,    ERRO,    ERRO,    ERRO, ERRO}		//error5
};

auto execute(int command,int state)->std::tuple<int, std::string>
{
	static unsigned short Home_State = 0;
	int n = step;
	std::cout << n<<std::endl;

	switch (command)
	{
		
		case HOME:
		{
			if (state == HOMED || state == HOMEFINISH)
			{
				unsigned short Runcount[AxisMax] = { 1 };
				unsigned short Reason[AxisMax] = { 1 };
				unsigned short Reason1[AxisMax] = { 0 };

				for (int i = 0; i < AxisMax; i++)			//设置18轴触发
				{
					MCF_Get_Home_Net(i, &Home_State, 0);
					if (Home_State == 1)
					{
						MCF_Set_Home_Trigger_Net(i, Trigger_Low_IMD, 0);
						MCF_JOG_Net(i, 10000, 10000000, 0);//往正找原点
						Runcount[i] = 1;
					}
				}
				bool Home_Number = true;
				while (Home_Number)
				{
					for (int i = 0; i < AxisMax; i++)
					{
						if (Runcount[i] == 1)
						{
							MCF_Get_Axis_State_Net(i, &Reason[i], 0);
							if (Reason[i] == IMD_STOP_AT_Home)
							{

								MCF_Clear_Axis_State_Net(i, 0);

								MCF_Set_Home_Trigger_Net(i, Trigger_Close, 0);
								MCF_Set_Axis_Profile_Net(i, 0, 20000, 1000000, 10000000, 0, Profile_S, 0);
								if (i < Axis_13)
								{
									MCF_Uniaxial_Net(i, -10000, Position_Opposite, 0);//机械臂往外升出10000,脉冲	
		//						    MCF_Uniaxial_Net(i, -5000, Position_Opposite, 0);//机械臂往外升出10000,脉冲	
								}
								else
								{
									MCF_Uniaxial_Net(i, -50000, Position_Opposite, 0);//机械臂往外升出10000,脉冲	
		//						    MCF_Uniaxial_Net(i, -25000, Position_Opposite, 0);//机械臂往外升出10000,脉冲
								}

								Runcount[i] = 0;
							}
							else if (Reason[i] == ERR_Axis_Busy)
							{
							}
						}
						else if (Runcount[i] == 0)
						{
							MCF_Get_Axis_State_Net(i, &Reason1[i], 0);
							if (Reason1[i] == 0)
							{
								Runcount[i] = 2;
							}
							else
							{
								Runcount[i] = 0;
							}
						}

						if (std::all_of(Runcount, Runcount + 18, [](auto v) {return v == 2; }))
						{
							for (i = 0; i < AxisMax; i++)
							{
								MCF_Set_Position_Net(i, 0, 0);				//设置当前位置为0
								MCF_Set_Encoder_Net(i, 0, 0);			//设置当前编码器为0 
																			//清零
							}

							Home_Number = false;
						}
					}
				}

				std::cout << "home finished" << std::endl;
				std::this_thread::sleep_for(std::chrono::seconds(1));
			}
			return std::make_tuple<int, std::string>(0, "home finished");
			break;
		}
		case PREPAIR:
		{
			if (state == HOMEFINISH || state == PREPAIRD)
			{
				Prepair();
				std::cout << "prepair finish" << std::endl;
			}
			return std::make_tuple<int, std::string>(0, "prepair finish");
			break;
		}
		case FORWARD:
		{
			if (state == PREPAIRD || state == RUNNING)
			{
				Prepair();


				param.d = 400; //步距 MM
				param.h = 90;  //腿高度 MM
				param.alpha = 3.1415926 / 3; //方向
				param.beta = 0.0;
				//	 param.total_count = 2000;  //1ms
				param.total_count = 50;   //10ms
				param.n =  n;
				param.begin_pee_wrt_ground = xyz;
				param.begin_pm_wrt_ground = body;
				param.mot_pos = input;

				for (int j = 0; j < 18; j++)
				{
					MCF_Set_Axis_Profile_Net(j, 0, 100000, 10000000, 600000000, 0, 0, 0);
				}
				//MCF_Set_Axis_Profile_Net(3, 0, 100000, 10000000, 600000000, 0, 0, 0);
				//MCF_Set_Axis_Profile_Net(4, 0, 100000, 10000000, 600000000, 0, 0, 0);
				//MCF_Set_Axis_Profile_Net(14, 0, 100000, 10000000, 600000000, 0, 0, 0);

				std::chrono::high_resolution_clock c;
				auto begin_time = c.now();
				for (int i = 0; ret; ++i)
				{
					std::this_thread::sleep_until(begin_time + i * std::chrono::milliseconds(5));
					ret = walk_plan(i, &param);
					// 弧度 中间为 0    input[0]                        0.2弧度
					// X轴 往外为正数据 input[1]  长方向  单位 ：毫米   0 - 30mm 
					// Y轴 往下为正数据 input[2]  长方向  单位 ：毫米   0 - 30mm

					sendToBoard(input);
				}
				std::cout << "forward finish" << std::endl;
			}
			return std::make_tuple<int, std::string>(0, "forward finish");
			break;
		}
		case BACK:
		{
			if (state == PREPAIRD || state == RUNNING)
			{
				Prepair();

				param.d = -400; //步距 MM
				param.h = 90;  //腿高度 MM
				param.alpha = 3.1415926 / 3; //方向
				param.beta = 0.0;
				//	 param.total_count = 2000;  //1ms
				param.total_count = 50;   //10ms
				param.n = n;
				param.begin_pee_wrt_ground = xyz;
				param.begin_pm_wrt_ground = body;
				param.mot_pos = input;

				for (int j = 0; j < 18; j++)
				{
					MCF_Set_Axis_Profile_Net(j, 0, 100000, 10000000, 600000000, 0, 0, 0);
				}

				std::chrono::high_resolution_clock c;
				auto begin_time = c.now();
				for (int i = 0; ret; ++i)
				{
					std::this_thread::sleep_until(begin_time + i * std::chrono::milliseconds(5));
					ret = walk_plan(i, &param);
					// 弧度 中间为 0    input[0]                        0.2弧度
					// X轴 往外为正数据 input[1]  长方向  单位 ：毫米   0 - 30mm 
					// Y轴 往下为正数据 input[2]  长方向  单位 ：毫米   0 - 30mm
					sendToBoard(input);
				}
				std::cout << "back finish" << std::endl;
			}
			return std::make_tuple<int, std::string>(0, "back finish");
			break;
		}
		case LEFT:
		{
			if (state == PREPAIRD || state == RUNNING)
			{
				Prepair();

				param.d = 200;
				param.h = 90;
				param.alpha = 3.1415926 * 5 / 6;
				param.beta = 0.0;
				//	 param.total_count = 2000;  //1ms
				param.total_count = 50;   //10ms
				param.n = n;
				param.begin_pee_wrt_ground = xyz;
				param.begin_pm_wrt_ground = body;
				param.mot_pos = input;

				for (int j = 0; j < 18; j++)
				{
					MCF_Set_Axis_Profile_Net(j, 0, 100000, 10000000, 600000000, 0, 0, 0);
				}

				std::chrono::high_resolution_clock c;
				auto begin_time = c.now();
				for (int i = 0; ret; ++i)
				{
					std::this_thread::sleep_until(begin_time + i * std::chrono::milliseconds(5));

					ret = walk_plan(i, &param);
					// 弧度 中间为 0    input[0]                        0.2弧度
					// X轴 往外为正数据 input[1]  长方向  单位 ：毫米   0 - 30mm 
					// Y轴 往下为正数据 input[2]  长方向  单位 ：毫米   0 - 30mm
					sendToBoard(input);
				}
				
				std::cout << "left finish" << std::endl;
			}
			return std::make_tuple<int, std::string>(0, "left finish");
			break;
		}
		case RIGHT:
		{
			if (state == PREPAIRD || state == RUNNING)
			{
				Prepair();

				param.d = -200;
				param.h = 90;
				param.alpha = 3.1415926 * 5 / 6;
				param.beta = 0.0;
				//	 param.total_count = 2000;  //1ms
				param.total_count = 50;   //10ms
				param.n = n;
				param.begin_pee_wrt_ground = xyz;
				param.begin_pm_wrt_ground = body;
				param.mot_pos = input;

				for (int j = 0; j < 18; j++)
				{
					MCF_Set_Axis_Profile_Net(j, 0, 100000, 10000000, 600000000, 0, 0, 0);
				}

				std::chrono::high_resolution_clock c;
				auto begin_time = c.now();
				for (int i = 0; ret; ++i)
				{
					std::this_thread::sleep_until(begin_time + i * std::chrono::milliseconds(5));
					ret = walk_plan(i, &param);
					// 弧度 中间为 0    input[0]                        0.2弧度
					// X轴 往外为正数据 input[1]  长方向  单位 ：毫米   0 - 30mm 
					// Y轴 往下为正数据 input[2]  长方向  单位 ：毫米   0 - 30mm
					sendToBoard(input);
				}
				std::cout << "right finish" << std::endl;
			}
			return std::make_tuple<int, std::string>(0, "right finish");
			break;
		}
		case TURNL:
		{
			if (state == PREPAIRD || state == RUNNING)
			{
				Prepair();

				param.d = 0;
				param.h = 90;
				param.alpha = 0.0;
				param.beta = -0.5;
				//	 param.total_count = 2000;  //1ms
				param.total_count = 50;   //10ms
				param.n = n;
				param.begin_pee_wrt_ground = xyz;
				param.begin_pm_wrt_ground = body;
				param.mot_pos = input;

				for (int j = 0; j < 18; j++)
				{
					MCF_Set_Axis_Profile_Net(j, 0, 100000, 10000000, 600000000, 0, 0, 0);
				}

				std::chrono::high_resolution_clock c;
				auto begin_time = c.now();
				for (int i = 0; ret; ++i)
				{
					std::this_thread::sleep_until(begin_time + i * std::chrono::milliseconds(5));
					ret = walk_plan(i, &param);
					// 弧度 中间为 0    input[0]                        0.2弧度
					// X轴 往外为正数据 input[1]  长方向  单位 ：毫米   0 - 30mm 
					// Y轴 往下为正数据 input[2]  长方向  单位 ：毫米   0 - 30mm
					sendToBoard(input);
				}
				std::cout << "turn left finish" << std::endl;
			}
			return std::make_tuple<int, std::string>(0, "turn left finish");
			break;
		}
		case TURNR:
		{
			if (state == PREPAIRD || state == RUNNING)
			{
				Prepair();

				param.d = 0;
				param.h = 90;
				param.alpha = 0.0;
				param.beta = 0.5;
				//	 param.total_count = 2000;  //1ms
				param.total_count = 50;   //10ms
				param.n = n;
				param.begin_pee_wrt_ground = xyz;
				param.begin_pm_wrt_ground = body;
				param.mot_pos = input;



				for (int j = 0; j < 18; j++)
				{
					MCF_Set_Axis_Profile_Net(j, 0, 100000, 10000000, 600000000, 0, 0, 0);
				}

				std::chrono::high_resolution_clock c;
				auto begin_time = c.now();
				for (int i = 0; ret; ++i)
				{
					std::this_thread::sleep_until(begin_time + i * std::chrono::milliseconds(5));
					ret = walk_plan(i, &param);
					// 弧度 中间为 0    input[0]                        0.2弧度
					// X轴 往外为正数据 input[1]  长方向  单位 ：毫米   0 - 30mm 
					// Y轴 往下为正数据 input[2]  长方向  单位 ：毫米   0 - 30mm
					sendToBoard(input);
				}
				std::cout << "turn right finish" << std::endl;
			}
			return std::make_tuple<int, std::string>(0, "turn right finish");
			break;
		}
	}
}

auto current_state(std::string& cmd) ->int
{
	std::map<std::string, int> int2str=
	{
		std::make_pair(std::string("home"),    HOME),
		std::make_pair(std::string("prepair"), PREPAIR),
		std::make_pair(std::string("forward"), FORWARD),
		std::make_pair(std::string("back"),    BACK),
		std::make_pair(std::string("left"),    LEFT),
		std::make_pair(std::string("right"),   RIGHT),
		std::make_pair(std::string("turnl"),   TURNL),
		std::make_pair(std::string("turnr"),   TURNR),
	};
	command = int2str.at(cmd);
	return table_state[state][command];
}


auto StringParser(std::string& cmd )->std::string 
{
	std::string str1;

	int len = cmd.size();  //cmd为网页发送字符串,计算字符长度)
	if (len < 8)
	{
		str1 = cmd;  //str_command为抽取命令字符串
		step = 0;
	}
	else if((cmd[len-1] >= 48 && cmd[len-1] <= 57) && (cmd[len - 2] < 48 || cmd[len - 2] > 57))
	{
		str1.assign(cmd, 0, len - 6);  //str_command为抽取命令字符串
		step = cmd[len-1];     //a为步数
		step = step - 48;
	}
	else if ((cmd[len - 1] >= 48 && cmd[len - 1] <= 57) && (cmd[len - 2] >= 48 || cmd[len - 2] <= 57))
	{
		str1.assign(cmd, 0, len - 7);  //str_command为抽取命令字符串
		step = (cmd[len - 1]-48)+ (cmd[len - 2] - 48)*10;     //a为步数
		
	}
	return str1;
}


void main()
{

	//1. 打开卡
	state = table_state[state][command];  //状态初始化
	
	unsigned short Station_Number[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };//设置站点顺序
	unsigned short Station_Type[8] = { 4 };	//设置站号类型
	MCF_Open_Net(1, &Station_Number[0], &Station_Type[0]);		//打开运动控制卡
	std::this_thread::sleep_for(std::chrono::seconds(1));

	//网络
	aris::core::Socket socket("sock", "", "5866", aris::core::Socket::WEB);

	socket.setOnReceivedMsg([](aris::core::Socket* socket, aris::core::Msg& msg)->int
	{
		auto send_ret = [socket](aris::core::Msg& ret_msg)->void
		{
			try
			{
				socket->sendMsg(ret_msg);
			}
			catch (std::exception & e)
			{
				std::cout << e.what() << std::endl;
				LOG_ERROR << e.what() << std::endl;
			}
		};
		auto send_code_and_msg = [send_ret, msg](int code, const std::string& ret_msg_str)
		{
			nlohmann::json js;
			js["return_code"] = code;
			js["return_message"] = ret_msg_str;

			aris::core::Msg ret_msg = msg;
			ret_msg.copy(js.dump(2));
			send_ret(ret_msg);
		};

		auto msg_data = std::string_view(msg.data(), msg.size());
		if (msg_data == "get") return 0;

		std::string  str = msg.toString();
			
			

			

		//收到网页命令后
		str_command = StringParser(str);   //字符串解析，得到命令和步数

		try
		{
			if (current_state(str_command) == -1)    //根据命令判断下一步状态，若状态正常则更新状态，否则继续为上一步状态并反馈错误信息
			{
				state = state;
				send_code_and_msg(-1, "fail");
 			}
			else
			{
				state = current_state(str_command);
			}
			auto [code, ret_str] = execute(command, state);    //执行状态更新后的命令
			send_code_and_msg(code, ret_str);               
		}
		catch (std::exception&e)
		{
			send_code_and_msg(-1, e.what());
		}

		return 0;
	});
	socket.setOnReceivedConnection([](aris::core::Socket* sock, const char* ip, int port)->int
	{
		std::cout << "socket receive connection" << std::endl;
		LOG_INFO << "socket receive connection:\n"
			<< std::setw(aris::core::LOG_SPACE_WIDTH) << "|" << "  ip:" << ip << "\n"
			<< std::setw(aris::core::LOG_SPACE_WIDTH) << "|" << "port:" << port << std::endl;
		return 0;
	});
	socket.setOnLoseConnection([](aris::core::Socket* socket)->int
	{
		std::cout << "socket lose connection" << std::endl;
		LOG_INFO << "socket lose connection" << std::endl;
		for (;;)
		{
			try
			{
				socket->startServer(socket->port());
				break;
			}
			catch (std::runtime_error & e)
			{
				std::cout << e.what() << std::endl << "will try to restart server socket in 1s" << std::endl;
				LOG_ERROR << e.what() << std::endl << "will try to restart server socket in 1s" << std::endl;
				std::this_thread::sleep_for(std::chrono::seconds(1));
			}
		}
		std::cout << "socket restart successful" << std::endl;
		LOG_INFO << "socket restart successful" << std::endl;

		return 0;
	});

	socket.startServer();







	for (;;)std::this_thread::sleep_for(std::chrono::milliseconds(1));








}