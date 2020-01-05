#if !defined(_GMC_4D_PCI_H_1C474833_BDCF_826DF5_INCLUDED_)
#define _GMC_4D_PCI_H_1C474833_BDCF_826DF5_INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif //_MSC_VER > 1000

#if _WIN32_WINNT < _WIN32_WINNT_WIN2K
#error MXMPAC need _WIN32_WINNT >= _WIN32_WINNT_WIN2K.
#endif

#include <wtypes.h>

#ifdef __cplusplus
extern "C" {
#endif
/********************************************************************************************************************************************************************
                                                       1 控制卡打开函数
********************************************************************************************************************************************************************/
//1.1 初始化函数                                       [1,100]                          [0,99]                          宏定义1.1                     
short WINAPI MCF_Open_Net                             (unsigned short Connection_Number,unsigned short *Station_Number, unsigned short *Station_Type);  
short WINAPI MCF_Close_Net                            ();                                                                                                
/********************************************************************************************************************************************************************
                                                      2 输入输出函数
********************************************************************************************************************************************************************/
//2.1 通用IO全部输出函数                               [OUT15,OUT0]                     [0,99]               
short WINAPI MCF_Set_Output_Net                       (unsigned long  All_Output_Logic, unsigned short StationNumber = 0);                                                    
short WINAPI MCF_Get_Output_Net                       (unsigned long *All_Output_Logic, unsigned short StationNumber = 0);                                                    
//2.2 通用IO全部输入函数                               [Input31,Input0]                 [Input48,Input32]               [0,99] 
short WINAPI MCF_Get_Input_Net                        (unsigned long *All_Input_Logic1, unsigned long *All_Input_Logic2,unsigned short StationNumber = 0); 
//2.3 通用IO按位输出函数                               宏定义2.3.1                      宏定义2.3.2                      [0,99]  
short WINAPI MCF_Set_Output_Bit_Net                   (unsigned short Bit_Output_Number,unsigned short Bit_Output_Logic, unsigned short StationNumber = 0);                                                           
short WINAPI MCF_Get_Output_Bit_Net                   (unsigned short Bit_Output_Number,unsigned short *Bit_Output_Logic,unsigned short StationNumber = 0);  
//2.4 通用IO按位输入函数                               宏定义2.4.1                      宏定义2.4.2                     [0,99] 
short WINAPI MCF_Get_Input_Bit_Net                    (unsigned short Bit_Input_Number, unsigned short *Bit_Input_Logic,unsigned short StationNumber = 0);   
//2.5 通用IO做为紧急停止函数                           宏定义2.4.1                      宏定义2.4.2              [0,99] 
short WINAPI MCF_Set_EMG_Bit_Net                      (unsigned short EMG_Input_Number, unsigned short  EMG_Mode,unsigned short StationNumber = 0); 
/********************************************************************************************************************************************************************
                                                      3 轴设置函数
********************************************************************************************************************************************************************/
//3.1 脉冲通道输出设置函数                             宏定义0.0           宏定义3.1                 [0,99] 
short WINAPI MCF_Set_Pulse_Mode_Net                   (unsigned short Axis,unsigned long  Pulse_Mode,unsigned short StationNumber = 0);                                                       
short WINAPI MCF_Get_Pulse_Mode_Net                   (unsigned short Axis,unsigned long *Pulse_Mode,unsigned short StationNumber = 0);    
//3.2 软件限位触发运动停止函数                         宏定义0.0           [-2^31,2^31]P     >     [-2^31,2^31]P          [0,99] 
short WINAPI MCF_Set_Soft_Limit_Net                   (unsigned short Axis,long  Positive_Position,long  Negative_Position,unsigned short StationNumber = 0);                           
short WINAPI MCF_Get_Soft_Limit_Net                   (unsigned short Axis,long *Positive_Position,long *Negative_Position,unsigned short StationNumber = 0);
//3.3 软件限位触发运动停止开关函数                     宏定义0.0           宏定义3.3                        [0,99] 
short WINAPI MCF_Set_Soft_Limit_Enable_Net            (unsigned short Axis,unsigned long  Soft_Limit_Enable,unsigned short StationNumber = 0);
short WINAPI MCF_Get_Soft_Limit_Enable_Net            (unsigned short Axis,unsigned long *Soft_Limit_Enable,unsigned short StationNumber = 0);
//3.4 伺服报警触发运动停止函数                         宏定义0.0           宏定义3.4                   [0,99] 
short WINAPI MCF_Set_Alarm_Trigger_Net                (unsigned short Axis,unsigned long  Trigger_Mode,unsigned short StationNumber = 0);
short WINAPI MCF_Get_Alarm_Trigger_Net                (unsigned short Axis,unsigned long *Trigger_Mode,unsigned short StationNumber = 0);
//3.5 Index触发运动停止函数                            宏定义0.0           宏定义3.4                   [0,99] 
short WINAPI MCF_Set_Index_Trigger_Net                (unsigned short Axis,unsigned long  Trigger_Mode,unsigned short StationNumber = 0);
short WINAPI MCF_Get_Index_Trigger_Net                (unsigned short Axis,unsigned long *Trigger_Mode,unsigned short StationNumber = 0);
//3.6 原点触发运动停止函数                             宏定义0.0           宏定义3.4                   [0,99] 
short WINAPI MCF_Set_Home_Trigger_Net                 (unsigned short Axis,unsigned long  Trigger_Mode,unsigned short StationNumber = 0);
short WINAPI MCF_Get_Home_Trigger_Net                 (unsigned short Axis,unsigned long *Trigger_Mode,unsigned short StationNumber = 0);
//3.7 正限位触发运动停止函数                           宏定义0.0           宏定义3.4                   [0,99] 
short WINAPI MCF_Set_ELP_Trigger_Net                  (unsigned short Axis,unsigned long  Trigger_Mode,unsigned short StationNumber = 0);
short WINAPI MCF_Get_ELP_Trigger_Net                  (unsigned short Axis,unsigned long *Trigger_Mode,unsigned short StationNumber = 0);
//3.8 负限位触发运动停止函数                           宏定义0.0           宏定义3.4                   [0,99] 
short WINAPI MCF_Set_ELN_Trigger_Net                  (unsigned short Axis,unsigned long  Trigger_Mode,unsigned short StationNumber = 0);
short WINAPI MCF_Get_ELN_Trigger_Net                  (unsigned short Axis,unsigned long *Trigger_Mode,unsigned short StationNumber = 0);
//3.9 设置回零参数                                     宏定义0.0           [1,35]                          宏定义3.9.1                宏定义3.9.2               宏定义3.9.3                (0,10M]P/S     (0,10M]P/S     [-2^31,(2^31-1)]
short WINAPI MCF_Search_Home_Set_Net                  (unsigned short Axis,unsigned short Search_Home_Mode,unsigned short Limit_Logic,unsigned short Home_Logic,unsigned short Index_Logic,double H_dMaxV,double L_dMaxV,long Offset_Positio = 1000,unsigned short Trigger_Source = 0, unsigned short StationNumber = 0);
//3.10设置回零启动                                     宏定义0.0           [0,99] 
short WINAPI MCF_Search_Home_Start_Net                (unsigned short Axis,unsigned short StationNumber = 0);
//3.11设置回零停止                                     宏定义0.0           [0,99] 
short WINAPI MCF_Search_Home_Stop_Net                 (unsigned short Axis,unsigned short StationNumber = 0);
//3.12获取回零状态                                     宏定义0.0           MC_Retrun.h{0,31,32}       [0,99]    
short WINAPI MCF_Search_Home_Get_State_Net            (unsigned short Axis,unsigned short *Home_State,unsigned short StationNumber = 0);
/********************************************************************************************************************************************************************
                                                      4 点位运动控制函数
********************************************************************************************************************************************************************/
//4.1 速度控制函数                                     宏定义0.0           (0,10M]P/S    (0,1T]P^2/S  [0,99] 
short WINAPI MCF_JOG_Net                              (unsigned short Axis,double dMaxV, double dMaxA,unsigned short StationNumber = 0);                                                  
//4.2 单轴运动位置改变函数                             宏定义0.0           [-2^31,(2^31-1)]  宏定义0.3                    [0,99] 
short WINAPI MCF_Uniaxial_dDist_Change_Net            (unsigned short Axis,long dDist,       unsigned short Position_Mode,unsigned short StationNumber = 0);   
//4.3 单轴运动速度改变函数                             宏定义0.0           (0,10M]P/S   [0,99] 
short WINAPI MCF_Uniaxial_dMaxV_Change_Net            (unsigned short Axis,double dMaxV,unsigned short StationNumber = 0);                                                             
//4.4 单轴曲线函数                                     宏定义0.0           [0,dMaxV]      (0,10M]P/S    (0,1T]P^2/S   (0,100T]P^3/S [0,dMaxV]      宏定义0.4               [0,99] 
short WINAPI MCF_Set_Axis_Profile_Net                 (unsigned short Axis,double  dV_ini,double dMaxV, double  dMaxA,double  dJerk,double  dV_end,unsigned short  Profile,unsigned short StationNumber = 0);     
short WINAPI MCF_Get_Axis_Profile_Net                 (unsigned short Axis,double *dV_ini,double *dMaxV,double *dMaxA,double *dJerk,double *dV_end,unsigned short *Profile,unsigned short StationNumber = 0);  
//4.5 单轴运动函数                                     宏定义0.0           [-2^31,(2^31-1)]  宏定义0.3                    [0,99] 
short WINAPI MCF_Uniaxial_Net                         (unsigned short Axis,long dDist,       unsigned short Position_Mode,unsigned short StationNumber = 0);                                                           
//4.6 单轴停止曲线函数                                 宏定义0.0           (0,1T]P^2/S   (0,100T]P^3/S 宏定义0.4               [0,99] 
short WINAPI MCF_Set_Axis_Stop_Profile_Net            (unsigned short Axis,double  dMaxA,double  dJerk,unsigned short  Profile,unsigned short StationNumber = 0);                    
short WINAPI MCF_Get_Axis_Stop_Profile_Net            (unsigned short Axis,double *dMaxA,double *dJerk,unsigned short *Profile,unsigned short StationNumber = 0);
//4.7 轴停止函数                                       宏定义0.0           宏定义4.7                     [0,99] 
short WINAPI MCF_Axis_Stop_Net                        (unsigned short Axis,unsigned short Axis_Stop_Mode,unsigned short StationNumber = 0);                                                       
/********************************************************************************************************************************************************************
                                                      5 插补运动控制函数
********************************************************************************************************************************************************************/
//5.1 坐标系曲线函数                                   宏定义0.1                 [0,dMaxV]      (0,10M]P/S    (0,1T]P^2/S   (0,100T]P^3/S [0,dMaxV]      宏定义0.4               [0,99]     
short WINAPI MCF_Set_Coordinate_Profile_Net           (unsigned short Coordinate,double  dV_ini,double  dMaxV,double  dMaxA,double  dJerk,double  dV_end,unsigned short Profile, unsigned short StationNumber = 0); 
short WINAPI MCF_Get_Coordinate_Profile_Net           (unsigned short Coordinate,double *dV_ini,double *dMaxV,double *dMaxA,double *dJerk,double *dV_end,unsigned short *Profile,unsigned short StationNumber = 0);
//5.2 圆半径插补运动函数                               宏定义0.1                 宏定义0.0                 [-2^31,(2^31-1)] [-2^31,(2^31-1)]  宏定义0.5                 宏定义0.3                    [0,99] 
short WINAPI MCF_Arc2_Radius_Net                      (unsigned short Coordinate,unsigned short *Axis_List,long *dDist_List,long Arc_Radius,  unsigned short Direction, unsigned short Position_Mode,unsigned short StationNumber = 0); 
//5.3 圆圆心插补运动函数                               宏定义0.1                 宏定义0.0                 [-2^31,(2^31-1)] [-2^31,(2^31-1)]  宏定义0.5                 宏定义0.3                    [0,99] 
short WINAPI MCF_Arc2_Centre_Net                      (unsigned short Coordinate,unsigned short *Axis_List,long *dDist_List,long *Center_List,unsigned short Direction, unsigned short Position_Mode,unsigned short StationNumber = 0); 
//5.4 直线插补运动函数                                 宏定义0.1                 宏定义0.0                 [-2^31,(2^31-1)] 宏定义0.3                   [0,99] 
short WINAPI MCF_Line2_Net                            (unsigned short Coordinate,unsigned short *Axis_List,long *dDist_List,unsigned short Position_Mode,unsigned short StationNumber = 0);
short WINAPI MCF_Line3_Net                            (unsigned short Coordinate,unsigned short *Axis_List,long *dDist_List,unsigned short Position_Mode,unsigned short StationNumber = 0);
short WINAPI MCF_Line4_Net                            (unsigned short Coordinate,unsigned short *Axis_List,long *dDist_List,unsigned short Position_Mode,unsigned short StationNumber = 0);
//5.5 坐标系停止曲线函数                               宏定义0.1                 (0,1T]P^2/S   (0,100T]P^3/S  宏定义0.4               [0,99] 
short WINAPI MCF_Set_Coordinate_Stop_Profile_Net      (unsigned short Coordinate,double  dMaxA, double  dJerk,unsigned short  Profile,unsigned short StationNumber = 0);                  
short WINAPI MCF_Get_Coordinate_Stop_Profile_Net      (unsigned short Coordinate,double *dMaxA, double *dJerk,unsigned short *Profile,unsigned short StationNumber = 0); 
//5.6 坐标系停止函数                                   宏定义0.1                 宏定义5.6                           [0,99] 
short WINAPI MCF_Coordinate_Stop_Net                  (unsigned short Coordinate,unsigned short Coordinate_Stop_Mode,unsigned short StationNumber = 0);                                                  
/********************************************************************************************************************************************************************
                                                      6 轴状态监控函数
********************************************************************************************************************************************************************/
//6.1 伺服使能设置函数                                 宏定义0.0           宏定义6.1                   [0,99] 
short WINAPI MCF_Set_Servo_Enable_Net                 (unsigned short Axis,unsigned short  Servo_Logic,unsigned short StationNumber = 0); 
short WINAPI MCF_Get_Servo_Enable_Net                 (unsigned short Axis,unsigned short *Servo_Logic,unsigned short StationNumber = 0);
//6.2 伺服报警复位设置函数                             宏定义0.0           宏定义6.2                   [0,99] 
short WINAPI MCF_Set_Servo_Alarm_Reset_Net            (unsigned short Axis,unsigned short  Alarm_Logic,unsigned short StationNumber = 0);    
short WINAPI MCF_Get_Servo_Alarm_Reset_Net            (unsigned short Axis,unsigned short *Alarm_Logic,unsigned short StationNumber = 0); 
//6.3 伺服报警输入获取函数                             宏定义0.0           宏定义6.3                            [0,99]
short WINAPI MCF_Get_Servo_Alarm_Net                  (unsigned short Axis,unsigned short *Servo_Alarm_State,   unsigned short StationNumber = 0);
//6.4 伺服定位完成输入获取函数                         宏定义0.0           宏定义6.4                            [0,99]
short WINAPI MCF_Get_Servo_INP_Net                    (unsigned short Axis,unsigned short *Servo_INP_State,     unsigned short StationNumber = 0);
//6.5 编码器Z相输入获取函数                            宏定义0.0           宏定义6.5                            [0,99]
short WINAPI MCF_Get_Z_Net                            (unsigned short Axis,unsigned short *Z_State,   unsigned short StationNumber = 0);
//6.6 原点输入获取函数                                 宏定义0.0           宏定义6.6                            [0,99]
short WINAPI MCF_Get_Home_Net                         (unsigned short Axis,unsigned short *Home_State,          unsigned short StationNumber = 0);
//6.7 正限位输入获取函数                               宏定义0.0           宏定义6.7                            [0,99]
short WINAPI MCF_Get_Positive_Limit_Net               (unsigned short Axis,unsigned short *Positive_Limit_State,unsigned short StationNumber = 0); 
//6.8 负限位输入获取函数                               宏定义0.0           宏定义6.8                            [0,99]
short WINAPI MCF_Get_Negative_Limit_Net               (unsigned short Axis,unsigned short *Negative_Limit_State,unsigned short StationNumber = 0);                                                 
//6.9 位置设置函数                                     宏定义0.0           [-2^31,(2^31-1)] [0,99] 
short WINAPI MCF_Set_Position_Net                     (unsigned short Axis,long  Position,  unsigned short StationNumber = 0);                                                         
short WINAPI MCF_Get_Position_Net                     (unsigned short Axis,long *Position,  unsigned short StationNumber = 0);   
//6.10 编码器设置函数                                  宏定义0.0           [-2^31,(2^31-1)] [0,99] 
short WINAPI MCF_Set_Encoder_Net                      (unsigned short Axis,long  Encoder,   unsigned short StationNumber = 0);                                                            
short WINAPI MCF_Get_Encoder_Net                      (unsigned short Axis,long *Encoder,   unsigned short StationNumber = 0);   
//6.11 速度获取                                        宏定义0.0           [-2^15,(2^15-1)]    [-2^15,(2^15-1)]   [0,99] 
short WINAPI MCF_Get_Vel_Net                          (unsigned short Axis,double *Command_Vel,double *Encode_Vel,unsigned short StationNumber = 0);                                    
//6.12 轴转态清除函数                                  宏定义0.0           [0,99] 
short WINAPI MCF_Clear_Axis_State_Net                 (unsigned short Axis,unsigned short StationNumber = 0);  
//6.13 轴转态查询函数                                  宏定义0.0           MC_Retrun.h[0,28]      [0,99] 
short WINAPI MCF_Get_Axis_State_Net                   (unsigned short Axis,unsigned short *Reason,unsigned short StationNumber = 0);                                                      
//6.14 原点触发位置记录函数	                           宏定义0.0           [-2^31,(2^31-1)]  [0,99] 
short WINAPI MCF_Get_Home_Rise_Position_Net           (unsigned short Axis,long *Position,unsigned short StationNumber = 0);  
short WINAPI MCF_Get_Home_Fall_Position_Net           (unsigned short Axis,long *Position,unsigned short StationNumber = 0);   
short WINAPI MCF_Get_Home_Rise_Encoder_Net            (unsigned short Axis,long *Encoder,unsigned short StationNumber = 0);  
short WINAPI MCF_Get_Home_Fall_Encoder_Net            (unsigned short Axis,long *Encoder,unsigned short StationNumber = 0);   
/********************************************************************************************************************************************************************
                                                      7 电子齿轮控制函数
********************************************************************************************************************************************************************/
//7.1 电子齿轮设置函数                                 宏定义0.0           宏定义0.0                   (0,(2^31-1)]               (0,(2^31-1)]            宏定义7.1.1                   宏定义7.1.2         [0,99]
short WINAPI MCF_Set_Gear_Net                         (unsigned short Axis,unsigned short  Follow_Axis,unsigned long  Denominator,unsigned long  Molecule,unsigned short  Follow_Source,unsigned short  Dir,unsigned short StationNumber = 0); //关闭使能再使能数据有效
short WINAPI MCF_Get_Gear_Net                         (unsigned short Axis,unsigned short *Follow_Axis,unsigned long *Denominator,unsigned long *Molecule,unsigned short *Follow_Source,unsigned short *Dir,unsigned short StationNumber = 0);
//7.2 电子齿轮开关函数                                 宏定义0.0           宏定义7.2                  [0,99] 
short WINAPI MCF_Set_Gear_Enable_Net                  (unsigned short Axis,unsigned short  Gear_Enable,unsigned short StationNumber = 0); 
short WINAPI MCF_Get_Gear_Enable_Net                  (unsigned short Axis,unsigned short *Gear_Enable,unsigned short StationNumber = 0);
//7.3 电子齿轮运动距离后自动关闭                       宏定义0.0           [-2^31,(2^31-1)] [0,99] 
short WINAPI MCF_Set_Gear_Auto_Disable_Net            (unsigned short Axis,long dDist,      unsigned short StationNumber = 0); 
/********************************************************************************************************************************************************************
                                                      8 缓冲区函数
********************************************************************************************************************************************************************/
//8.1 缓冲区停止曲线函数                               宏定义0.2                    (0,1T]P^2/S    (0,100T]P^3/S  宏定义0.4              [0,99]
short WINAPI MCF_Buffer_Set_Stop_Profile_Net          (unsigned short Buffer_Number,double  dMaxA, double  dJerk, unsigned short Profile,unsigned short StationNumber = 0);
//8.2 缓冲区停止函数                                   宏定义0.2                    宏定义8.2                       [0,99] 
short WINAPI MCF_Buffer_Stop_Net                      (unsigned short Buffer_Number,unsigned short Buffer_Stop_Mode,unsigned short StationNumber = 0);
//8.3 缓冲区在线改变速度倍率                           宏定义0.2                    (0,10]                [0,99] 
short WINAPI MCF_Buffer_Change_Velocity_Ratio_Net     (unsigned short Buffer_Number,double Velocity_Ratio,unsigned short StationNumber = 0);
//8.4 缓冲区建立开始函数                               宏定义0.2                    [0,99] 
short WINAPI MCF_Buffer_Start_Net                     (unsigned short Buffer_Number,unsigned short StationNumber = 0);
//8.5 缓冲区速度倍率                                   宏定义0.2                    宏定义8.5                                [0,99] 
short WINAPI MCF_Buffer_Set_Velocity_Ratio_Enable_Net (unsigned short Buffer_Number,unsigned short Velocity_Ratio_Enable = 0,unsigned short StationNumber = 0);
//8.6 缓冲区前瞻处理降速比                             宏定义0.2                    (0,1]                   [0,99]
short WINAPI MCF_Buffer_Set_Reduce_Ratio_Net          (unsigned short Buffer_Number,double Reduce_Ratio = 1,unsigned short StationNumber = 0);
//8.7 缓冲区曲线函数                                   宏定义0.2                    [0,dMaxV]     (0,10M]P/S    (0,1T]P^2/S  (0,100T]P^3/S [0,dMaxV]     宏定义0.4               [0,99]  
short WINAPI MCF_Buffer_Set_Profile_Net               (unsigned short Buffer_Number,double dV_ini,double dMaxV, double dMaxA,double dJerk, double dV_end,unsigned short Profile ,unsigned short StationNumber = 0);
//8.8 缓冲区单轴运动                                   宏定义0.2                    宏定义0.0           [-2^31,(2^31-1)]  宏定义0.3                    [0,99] 
short WINAPI MCF_Buffer_Uniaxial_Net                  (unsigned short Buffer_Number,unsigned short Axis,long dDist,       unsigned short Position_Mode,unsigned short StationNumber = 0);
//8.9 缓冲区直线插补运动                               宏定义0.2                    宏定义0.0                 [-2^31,(2^31-1)] 宏定义0.3                    [0,99] 
short WINAPI MCF_Buffer_Line2_Net                     (unsigned short Buffer_Number,unsigned short *Axis_List,long *dDist_List,unsigned short Position_Mode,unsigned short StationNumber = 0);
short WINAPI MCF_Buffer_Line3_Net                     (unsigned short Buffer_Number,unsigned short *Axis_List,long *dDist_List,unsigned short Position_Mode,unsigned short StationNumber = 0);
short WINAPI MCF_Buffer_Line4_Net                     (unsigned short Buffer_Number,unsigned short *Axis_List,long *dDist_List,unsigned short Position_Mode,unsigned short StationNumber = 0);
//8.10 缓冲区圆半径插补运动函数                        宏定义0.2                    宏定义0.0                 [-2^31,(2^31-1)] [-2^31,(2^31-1)]  宏定义0.5                宏定义0.3                    [0,99] 
short WINAPI MCF_Buffer_Arc_Radius_Net                (unsigned short Buffer_Number,unsigned short *Axis_List,long *dDist_List,long Arc_Radius,  unsigned short Direction,unsigned short Position_Mode,unsigned short StationNumber = 0);
//8.11 缓冲区圆圆心插补运动函数                        宏定义0.2                    宏定义0.0                 [-2^31,(2^31-1)] [-2^31,(2^31-1)]  宏定义0.5                宏定义0.3                    [0,99] 
short WINAPI MCF_Buffer_Arc_Centre_Net                (unsigned short Buffer_Number,unsigned short *Axis_List,long *dDist_List,long *Center_List,unsigned short Direction,unsigned short Position_Mode,unsigned short StationNumber = 0);
//8.12 缓冲区延时函数                                  宏定义0.2                    [0,2^31-1]           [0,99]
short WINAPI MCF_Buffer_Delay_Net                     (unsigned short Buffer_Number,unsigned long number,unsigned short StationNumber = 0);
//8.13 缓冲区IO输出函数                                宏定义0.2                    宏定义2.3.1               宏定义2.3.2          [0,99]     
short WINAPI MCF_Buffer_Set_Output_Bit_Net            (unsigned short Buffer_Number,unsigned short Bit_Number,unsigned short output,unsigned short StationNumber = 0);
//8.14 缓冲区IO等待函数                                宏定义0.2                    宏定义2.4.1               宏定义2.4.2          (0,2^15-1]              [0,99] 
short WINAPI MCF_Buffer_Wait_Input_Bit_Net            (unsigned short Buffer_Number,unsigned short Bit_Number,unsigned short Logic,unsigned short Time_Out,unsigned short StationNumber = 0);
//8.15 缓冲区建立结束                                  宏定义0.2                    [1,2^31-1]                    [0,99] 
short WINAPI MCF_Buffer_End_Net                       (unsigned short Buffer_Number,unsigned long *Command_Number,unsigned short StationNumber = 0);
//8.16 缓冲区执行函数                                  宏定义0.2                    宏定义8.16                  [0,99]
short WINAPI MCF_Buffer_Execute_Net                   (unsigned short Buffer_Number,unsigned short Execute_Mode,unsigned short StationNumber = 0);
//8.17 缓冲区断点启动函数                              宏定义0.2                    [0,99]
short WINAPI MCF_Buffer_Execute_BreakPoint_Net        (unsigned short Buffer_Number,unsigned short StationNumber = 0);
//8.18 缓冲区状态查询函数                              宏定义0.2                    MC_Retrun.h{0,29,30}                   [0,2^15-1]
short WINAPI MCF_Buffer_Get_State_Net                 (unsigned short Buffer_Number,unsigned short *Execute_State,unsigned short *Execute_Number,unsigned short StationNumber = 0);
/********************************************************************************************************************************************************************
                                                      9 数据捕捉函数
********************************************************************************************************************************************************************/
//9.1 数据捕捉打开函数(必须在MCF_Open_Net前面提前调用)                                    
short WINAPI MCF_Capture_Open_Net                     ();
//9.2 数据捕捉检查数据更新函数                         宏定义9.2            
short WINAPI MCF_Capture_State_Net                    (unsigned short *Capture_State);
//9.3 数据捕捉读取函数                                 宏定义0.0           &Array[1000] 
short WINAPI MCF_Capture_Read_Command_Net             (unsigned short Axis,long *Command);
short WINAPI MCF_Capture_Read_Encoder_Net             (unsigned short Axis,long *Encoder);
short WINAPI MCF_Capture_Read_AD_Net                  (unsigned short Axis,long *AD);
//9.4 ADC采样滤波                                      宏定义0.0           [0,1]
short WINAPI MCF_Capture_Filter_AD_Net                (unsigned short Axis,double Filter_Coefficient = 1);
/********************************************************************************************************************************************************************
                                                       10  系统函数
********************************************************************************************************************************************************************/
//10.1 模块版本号                                     [0x00000000,0xFFFFFFFF] [0,99] 
short WINAPI MCF_Get_Version_Net                      (unsigned long *Version,unsigned short StationNumber = 0);                                                  

#ifdef __cplusplus
} 
#endif

#endif // _MXMPAC_H_1C474833_BDCF_826DF5_INCLUDED_






