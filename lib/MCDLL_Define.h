/********************************************************************************************************************************************************************
                                                       0 ȫ�ֺ궨��
********************************************************************************************************************************************************************/	
//0.0 Axis
#define Axis_1                   0    //��1��
#define Axis_2                   1    //��2��
#define Axis_3                   2    //��3��
#define Axis_4                   3    //��4��
#define Axis_5                   4    //��5��
#define Axis_6                   5    //��6��
#define Axis_7                   6    //��7��
#define Axis_8                   7    //��8��
#define Axis_9                   8    //��9��
#define Axis_10                  9    //��10��
#define Axis_11                  10   //��11��
#define Axis_12                  11   //��12��
#define Axis_13                  12   //��13��
#define Axis_14                  13   //��14��
#define Axis_15                  14   //��15��
#define Axis_16                  15   //��16��
#define Axis_17                  16   //��17��
#define Axis_18                  17   //��18��
#define Axis_19                  18   //��19��
#define Axis_20                  19   //��20��
#define Axis_21                  20   //��21��
#define Axis_22                  21   //��22��
#define Axis_23                  22   //��23��
#define Axis_24                  23   //��14��
//0.1 Coordinate
#define Coordinate_0             0    //��0����ϵ
#define Coordinate_1             1    //��1����ϵ
//0.2 Buffer_Number
#define Buffer_Number_0          0    //��0������
//0.3 Position_Mode
#define Position_Absolute        0    //����λ��ģʽ
#define Position_Opposite        1    //���λ��ģʽ
//0.4 Profile
#define Profile_T                0    //T������
#define Profile_S                1    //S������
//0.5 Direction
#define Clock_Wise               0    //˳��
#define Counter_Clock_Wise       1    //�满
/********************************************************************************************************************************************************************
                                                       1 ���ƿ��򿪺���
********************************************************************************************************************************************************************/
//1.1   Station_Type[]
#define Station_Type_24I16O      0    //24����16���ģ��        
#define Station_Type_48I32O      1    //48����32���ģ��          
#define Station_Type_4D          2    //4���˶����ƿ�
#define Station_Type_8D          3    //8���˶����ƿ�
/********************************************************************************************************************************************************************
                                                      2 �����������
********************************************************************************************************************************************************************/
//2.3.1 Bit_Output_Number
#define Bit_Output_0             0    //���0
#define Bit_Output_1             1    //���1
#define Bit_Output_2             2    //���2
#define Bit_Output_3             3    //���3
#define Bit_Output_4             4    //���4
#define Bit_Output_5             5    //���5
#define Bit_Output_6             6    //���6
#define Bit_Output_7             7    //���7
#define Bit_Output_8             8    //���8
#define Bit_Output_9             9    //���9
#define Bit_Output_10            10   //���10
#define Bit_Output_11            11   //���11
#define Bit_Output_12            12   //���12
#define Bit_Output_13            13   //���13
#define Bit_Output_14            14   //���14
#define Bit_Output_15            15   //���15
//2.3.2 Bit_Output_Logic
#define Bit_Output_Close         0    //����պ�,Ӳ������
#define Bit_Output_Open          1    //�����,Ӳ������
//2.4.1 Bit_Input_Number
#define Bit_Input_0              0    //����0
#define Bit_Input_1              1    //����1
#define Bit_Input_2              2    //����2
#define Bit_Input_3              3    //����3
#define Bit_Input_4              4    //����4
#define Bit_Input_5              5    //����5
#define Bit_Input_6              6    //����6
#define Bit_Input_7              7    //����7
#define Bit_Input_8              8    //����8
#define Bit_Input_9              9    //����9
#define Bit_Input_10             10   //����10
#define Bit_Input_11             11   //����11
#define Bit_Input_12             12   //����12
#define Bit_Input_13             13   //����13
#define Bit_Input_14             14   //����14
#define Bit_Input_15             15   //����15
//2.4.2 Bit_Input_Logic
#define Bit_Input_Close          0    //����պ�,Ӳ������
#define Bit_Input_Open           1    //�����,Ӳ������
//2.5   EMG_Mode
#define EMG_Trigger_Close        0    //��ʹ�ý���ֹͣ����
#define EMG_Trigger_Low_IMD      1    //�͵�ƽ��������ֹͣ
#define EMG_Trigger_Low_DEC      2    //�͵�ƽ��������ֹͣ
#define EMG_Trigger_High_IMD     3    //�ߵ�ƽ��������ֹͣ
#define EMG_Trigger_High_DEC     4    //�ߵ�ƽ��������ֹͣ
/********************************************************************************************************************************************************************
                                                      3 �����ú���
********************************************************************************************************************************************************************/
//3.1 Pulse_Mode
#define Pulse_Dir_H              0    //���巽��(Ĭ��)
#define Pulse_Dir_L              1    //���巽��
#define Pulse_CW_CCW             2    //˫����
#define Pulse_CCW_CW             3    //˫����
#define Pulse_AB                 4    //AB��λ
#define Pulse_BA                 5    //AB��λ
//3.3 Trigger_Mode    
#define Soft_Limit_Close         0    //�����λ�ر�
#define Soft_Limit_Open          1    //�����λ��
//3.4 Trigger_Mode    
#define Trigger_Close            0    //�رյ�ƽ����(Ĭ��)
#define Trigger_Low_IMD          1    //�͵�ƽ��������ֹͣ
#define Trigger_Low_DEC          2    //�͵�ƽ��������ֹͣ
#define Trigger_High_IMD         3    //�ߵ�ƽ��������ֹͣ
#define Trigger_High_DEC         4    //�ߵ�ƽ��������ֹͣ
//3.9.2 Limit_Logic      
//3.9.3 Home_Logic      
//3.9.4 Index_Logic      
#define Low_Logic                0    //�͵�ƽ����
#define High_Logic               1    //�ߵ�ƽ���� 
/********************************************************************************************************************************************************************
                                                      4 ��λ�˶����ƺ���
********************************************************************************************************************************************************************/
//4.7 Axis_Stop_Mode
#define Axis_Stop_IMD            0    //����ֹͣ
#define Axis_Stop_DEC            1    //����ֹͣ 
/********************************************************************************************************************************************************************
                                                      5 �岹�˶����ƺ���
********************************************************************************************************************************************************************/
//5.6 Coordinate_Stop_Mode
#define Coordinate_Stop_IMD      0    //����ֹͣ
#define Coordinate_Stop_DEC      1    //����ֹͣ
/********************************************************************************************************************************************************************
                                                      6 ��״̬��غ���
********************************************************************************************************************************************************************/
//6.1 Servo_Logic
#define Servo_Close              0    //����պ� 
#define Servo_Open               1    //�����
//6.2 Alarm_Logic
#define Alarm_Close              0    //����պ� 
#define Alarm_Open               1    //�����
//6.3 Servo_Alarm_State
#define Servo_Alarm_Close        0    //����պ�
#define Servo_Alarm_Open         1    //�����
//6.4 Servo_INP_State
#define Servo_INP_Close          0    //����պ�
#define Servo_INP_Open           1    //�����
//6.5 Z_State
#define Z_Logic_L                0    //Z��͵�ƽ
#define Z_Logic_H                1    //Z��ߵ�ƽ
//6.6 Home_State
#define Home_Close               0    //����պ�,Ӳ������
#define Home_Open                1    //�����,Ӳ������
//6.7 Positive_Limit_State
#define Positive_Limit_Close     0    //����պ�,Ӳ������
#define Positive_Limit_Open      1    //�����,Ӳ������
//6.8 Negative_Limit_State
#define Negative_Limit_Close     0    //����պ�,Ӳ������
#define Negative_Limit_Open      1    //�����,Ӳ������
/********************************************************************************************************************************************************************
                                                      7 ���ӳ��ֿ��ƺ���
********************************************************************************************************************************************************************/
//7.1.1 Follow_Source
#define Follow_Command           0    //��������
#define Follow_Encode            1    //���������
//7.1.2 Dir
#define Dir_P_T_P                0    //������ͬ������ 
#define Dir_N_T_N                1    //���渺ͬ������
#define Dir_PN_T_PN              2    //��������ͬ������ 
#define Dir_PN_T_P               3    //��������������������
#define Dir_PN_T_N               4    //�������������������� 
//7.2 Gear_Enable
#define Gear_Close               0    //���ӳ��ֹر�
#define Gear_Open                1    //���ӳ��ִ�
/********************************************************************************************************************************************************************
                                                      8 ����������
********************************************************************************************************************************************************************/
//8.2 Coordinate_Stop_Mode
#define Buffer_Stop_IMD          0    //����ֹͣ
#define Buffer_Stop_DEC          1    //����ֹͣ
//8.5 Velocity_Ratio_Enable
#define Velocity_Ratio_Clsoe     0    //�ٶȱ��ʹر�
#define Velocity_Ratio_Open      1    //�ٶȱ��ʴ�
//8.16 Execute_Mode 
#define Execute_Mode_0           0    //�����û����߲���ִ��
#define Execute_Mode_1           1    //���������˶�ָ���Ĺս����ٶȹ滮
/********************************************************************************************************************************************************************
                                                      9 ���ݲ�׽����
********************************************************************************************************************************************************************/
//9.2 Capture_State
#define Data_Keep              0    //���ݸ���
#define Data_Updata            1    //���ݱ���



