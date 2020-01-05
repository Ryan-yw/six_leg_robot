/********************************************************************************************************************************************************************
                                                       0 全局宏定义
********************************************************************************************************************************************************************/	
//0.0 Axis
#define Axis_1                   0    //第1轴
#define Axis_2                   1    //第2轴
#define Axis_3                   2    //第3轴
#define Axis_4                   3    //第4轴
#define Axis_5                   4    //第5轴
#define Axis_6                   5    //第6轴
#define Axis_7                   6    //第7轴
#define Axis_8                   7    //第8轴
#define Axis_9                   8    //第9轴
#define Axis_10                  9    //第10轴
#define Axis_11                  10   //第11轴
#define Axis_12                  11   //第12轴
#define Axis_13                  12   //第13轴
#define Axis_14                  13   //第14轴
#define Axis_15                  14   //第15轴
#define Axis_16                  15   //第16轴
#define Axis_17                  16   //第17轴
#define Axis_18                  17   //第18轴
#define Axis_19                  18   //第19轴
#define Axis_20                  19   //第20轴
#define Axis_21                  20   //第21轴
#define Axis_22                  21   //第22轴
#define Axis_23                  22   //第23轴
#define Axis_24                  23   //第14轴
//0.1 Coordinate
#define Coordinate_0             0    //第0坐标系
#define Coordinate_1             1    //第1坐标系
//0.2 Buffer_Number
#define Buffer_Number_0          0    //第0缓冲区
//0.3 Position_Mode
#define Position_Absolute        0    //绝对位置模式
#define Position_Opposite        1    //相对位置模式
//0.4 Profile
#define Profile_T                0    //T型曲线
#define Profile_S                1    //S型曲线
//0.5 Direction
#define Clock_Wise               0    //顺弧
#define Counter_Clock_Wise       1    //逆弧
/********************************************************************************************************************************************************************
                                                       1 控制卡打开函数
********************************************************************************************************************************************************************/
//1.1   Station_Type[]
#define Station_Type_24I16O      0    //24输入16输出模块        
#define Station_Type_48I32O      1    //48输入32输出模块          
#define Station_Type_4D          2    //4轴运动控制卡
#define Station_Type_8D          3    //8轴运动控制卡
/********************************************************************************************************************************************************************
                                                      2 输入输出函数
********************************************************************************************************************************************************************/
//2.3.1 Bit_Output_Number
#define Bit_Output_0             0    //输出0
#define Bit_Output_1             1    //输出1
#define Bit_Output_2             2    //输出2
#define Bit_Output_3             3    //输出3
#define Bit_Output_4             4    //输出4
#define Bit_Output_5             5    //输出5
#define Bit_Output_6             6    //输出6
#define Bit_Output_7             7    //输出7
#define Bit_Output_8             8    //输出8
#define Bit_Output_9             9    //输出9
#define Bit_Output_10            10   //输出10
#define Bit_Output_11            11   //输出11
#define Bit_Output_12            12   //输出12
#define Bit_Output_13            13   //输出13
#define Bit_Output_14            14   //输出14
#define Bit_Output_15            15   //输出15
//2.3.2 Bit_Output_Logic
#define Bit_Output_Close         0    //触点闭合,硬件灯亮
#define Bit_Output_Open          1    //触点打开,硬件灯灭
//2.4.1 Bit_Input_Number
#define Bit_Input_0              0    //输入0
#define Bit_Input_1              1    //输入1
#define Bit_Input_2              2    //输入2
#define Bit_Input_3              3    //输入3
#define Bit_Input_4              4    //输入4
#define Bit_Input_5              5    //输入5
#define Bit_Input_6              6    //输入6
#define Bit_Input_7              7    //输入7
#define Bit_Input_8              8    //输入8
#define Bit_Input_9              9    //输入9
#define Bit_Input_10             10   //输入10
#define Bit_Input_11             11   //输入11
#define Bit_Input_12             12   //输入12
#define Bit_Input_13             13   //输入13
#define Bit_Input_14             14   //输入14
#define Bit_Input_15             15   //输入15
//2.4.2 Bit_Input_Logic
#define Bit_Input_Close          0    //触点闭合,硬件灯亮
#define Bit_Input_Open           1    //触点打开,硬件灯灭
//2.5   EMG_Mode
#define EMG_Trigger_Close        0    //不使用紧急停止功能
#define EMG_Trigger_Low_IMD      1    //低电平触发紧急停止
#define EMG_Trigger_Low_DEC      2    //低电平触发减速停止
#define EMG_Trigger_High_IMD     3    //高电平触发紧急停止
#define EMG_Trigger_High_DEC     4    //高电平触发减速停止
/********************************************************************************************************************************************************************
                                                      3 轴设置函数
********************************************************************************************************************************************************************/
//3.1 Pulse_Mode
#define Pulse_Dir_H              0    //脉冲方向(默认)
#define Pulse_Dir_L              1    //脉冲方向
#define Pulse_CW_CCW             2    //双脉冲
#define Pulse_CCW_CW             3    //双脉冲
#define Pulse_AB                 4    //AB相位
#define Pulse_BA                 5    //AB相位
//3.3 Trigger_Mode    
#define Soft_Limit_Close         0    //软件限位关闭
#define Soft_Limit_Open          1    //软件限位打开
//3.4 Trigger_Mode    
#define Trigger_Close            0    //关闭电平触发(默认)
#define Trigger_Low_IMD          1    //低电平触发紧急停止
#define Trigger_Low_DEC          2    //低电平触发减速停止
#define Trigger_High_IMD         3    //高电平触发紧急停止
#define Trigger_High_DEC         4    //高电平触发减速停止
//3.9.2 Limit_Logic      
//3.9.3 Home_Logic      
//3.9.4 Index_Logic      
#define Low_Logic                0    //低电平触发
#define High_Logic               1    //高电平触发 
/********************************************************************************************************************************************************************
                                                      4 点位运动控制函数
********************************************************************************************************************************************************************/
//4.7 Axis_Stop_Mode
#define Axis_Stop_IMD            0    //紧急停止
#define Axis_Stop_DEC            1    //减速停止 
/********************************************************************************************************************************************************************
                                                      5 插补运动控制函数
********************************************************************************************************************************************************************/
//5.6 Coordinate_Stop_Mode
#define Coordinate_Stop_IMD      0    //紧急停止
#define Coordinate_Stop_DEC      1    //减速停止
/********************************************************************************************************************************************************************
                                                      6 轴状态监控函数
********************************************************************************************************************************************************************/
//6.1 Servo_Logic
#define Servo_Close              0    //触点闭合 
#define Servo_Open               1    //触点打开
//6.2 Alarm_Logic
#define Alarm_Close              0    //触点闭合 
#define Alarm_Open               1    //触点打开
//6.3 Servo_Alarm_State
#define Servo_Alarm_Close        0    //触点闭合
#define Servo_Alarm_Open         1    //触点打开
//6.4 Servo_INP_State
#define Servo_INP_Close          0    //触点闭合
#define Servo_INP_Open           1    //触点打开
//6.5 Z_State
#define Z_Logic_L                0    //Z相低电平
#define Z_Logic_H                1    //Z相高电平
//6.6 Home_State
#define Home_Close               0    //触点闭合,硬件灯亮
#define Home_Open                1    //触点打开,硬件灯灭
//6.7 Positive_Limit_State
#define Positive_Limit_Close     0    //触点闭合,硬件灯亮
#define Positive_Limit_Open      1    //触点打开,硬件灯灭
//6.8 Negative_Limit_State
#define Negative_Limit_Close     0    //触点闭合,硬件灯亮
#define Negative_Limit_Open      1    //触点打开,硬件灯灭
/********************************************************************************************************************************************************************
                                                      7 电子齿轮控制函数
********************************************************************************************************************************************************************/
//7.1.1 Follow_Source
#define Follow_Command           0    //跟随命令
#define Follow_Encode            1    //跟随编码器
//7.1.2 Dir
#define Dir_P_T_P                0    //跟随正同方向走 
#define Dir_N_T_N                1    //跟随负同方向走
#define Dir_PN_T_PN              2    //跟随正负同方向走 
#define Dir_PN_T_P               3    //跟随正负都往正方向走
#define Dir_PN_T_N               4    //跟随正负都往负方向走 
//7.2 Gear_Enable
#define Gear_Close               0    //电子齿轮关闭
#define Gear_Open                1    //电子齿轮打开
/********************************************************************************************************************************************************************
                                                      8 缓冲区函数
********************************************************************************************************************************************************************/
//8.2 Coordinate_Stop_Mode
#define Buffer_Stop_IMD          0    //紧急停止
#define Buffer_Stop_DEC          1    //减速停止
//8.5 Velocity_Ratio_Enable
#define Velocity_Ratio_Clsoe     0    //速度倍率关闭
#define Velocity_Ratio_Open      1    //速度倍率打开
//8.16 Execute_Mode 
#define Execute_Mode_0           0    //按照用户曲线参数执行
#define Execute_Mode_1           1    //根据两轴运动指令间的拐角做速度规划
/********************************************************************************************************************************************************************
                                                      9 数据捕捉函数
********************************************************************************************************************************************************************/
//9.2 Capture_State
#define Data_Keep              0    //数据更新
#define Data_Updata            1    //数据保持



