#ifndef __ROBSTRIDE_C_H__
#define __ROBSTRIDE_C_H__

#include "main.h"
#include "string.h"
#include <stdint.h>
#include <stdbool.h>
#include "Dji3508.h"

#define Set_mode 'j'
#define Set_parameter 'p'

// ????
#define move_control_mode 0
#define Pos_control_mode 1
#define Speed_control_mode 2
#define Elect_control_mode 3
#define Set_Zero_mode 4
#define CSP_control_mode 5

// ????
#define Communication_Type_Get_ID 0x00
#define Communication_Type_MotionControl 0x01
#define Communication_Type_MotorRequest 0x02
#define Communication_Type_MotorEnable 0x03
#define Communication_Type_MotorStop 0x04
#define Communication_Type_SetPosZero 0x06
#define Communication_Type_Can_ID 0x07
#define Communication_Type_Control_Mode 0x12
#define Communication_Type_GetSingleParameter 0x11
#define Communication_Type_SetSingleParameter 0x12
#define Communication_Type_ErrorFeedback 0x15
#define Communication_Type_MotorDataSave 0x16
#define Communication_Type_BaudRateChange 0x17
#define Communication_Type_ProactiveEscalationSet 0x18
#define Communication_Type_MotorModeSet 0x19

typedef struct
{
    uint16_t index;
    float data;
} data_read_write_one;

static const uint16_t Index_List[] = {
    0x7005, 0x7006, 0x700A, 0x700B, 0x7010, 0x7011, 0x7014, 0x7016,
    0x7017, 0x7018, 0x7019, 0x701A, 0x701B, 0x701C, 0x701D};

typedef struct
{
    data_read_write_one run_mode;
    data_read_write_one iq_ref;
    data_read_write_one spd_ref;
    data_read_write_one imit_torque;
    data_read_write_one cur_kp;
    data_read_write_one cur_ki;
    data_read_write_one cur_filt_gain;
    data_read_write_one loc_ref;
    data_read_write_one limit_spd;
    data_read_write_one limit_cur;
    data_read_write_one mechPos;
    data_read_write_one iqf;
    data_read_write_one mechVel;
    data_read_write_one VBUS;
    data_read_write_one rotation;
} data_read_write;

typedef struct
{
    float Angle;
    float Speed;
    float Torque;
    float Temp;
    int pattern;
} Motor_Pos_RobStride_Info;

typedef struct
{
    int set_motor_mode;
    float set_current;
    float set_speed;
    float set_acceleration;
    float set_Torque;
    float set_angle;
    float set_limit_cur;
    float set_limit_speed;
    float set_Kp;
    float set_Ki;
    float set_Kd;
} Motor_Set;

typedef enum
{
    operationControl = 0,
    positionControl = 1,
    speedControl = 2
} MIT_TYPE;

typedef struct
{
    uint8_t CAN_ID;
    uint64_t Unique_ID;
    uint16_t Master_CAN_ID;
    float (*Motor_Offset_MotoFunc)(float Motor_Tar);

    Motor_Set Motor_Set_All;
    uint8_t error_code;
    bool MIT_Mode;
    MIT_TYPE MIT_Type;

    float output;
    int Can_Motor;
    Motor_Pos_RobStride_Info Pos_Info;
    data_read_write drw;
} RobStride_Motor;
void RS02_UserInit(void); // RS02用户初始化函数
void RS02_Task(void);     // RS02任务处理函数
void RS02_PosPID_Init(void);
static float RS02_PID_Compute(PID_t *pid, float target, float current);
void RobStride_Motor_init(RobStride_Motor *motor, uint8_t CAN_Id, bool MIT_Mode);
void RobStride_Motor_move_control(RobStride_Motor *motor, float Torque, float Angle, float Speed, float Kp, float Kd);
void RobStride_Motor_MIT_Enable(RobStride_Motor *motor);
void RobStride_Motor_MIT_Disable(RobStride_Motor *motor);
void RobStride_Motor_MIT_ClearOrCheckError(RobStride_Motor *motor, uint8_t F_CMD);
void RobStride_Motor_MIT_SetMotorType(RobStride_Motor *motor, uint8_t F_CMD);
void RobStride_Motor_MIT_SetMotorId(RobStride_Motor *motor, uint8_t F_CMD);
void RobStride_Motor_MIT_Control(RobStride_Motor *motor, float Angle, float Speed, float Kp, float Kd, float Torque);
void RobStride_Motor_MIT_PositionControl(RobStride_Motor *motor, float position_rad, float speed_rad_per_s);
void RobStride_Motor_MIT_SpeedControl(RobStride_Motor *motor, float speed_rad_per_s, float current_limit);
void RobStride_Motor_MIT_SetZeroPos(RobStride_Motor *motor);
void RobStride_Motor_Pos_control(RobStride_Motor *motor, float Speed, float Angle);
void RobStride_Motor_CSP_control(RobStride_Motor *motor, float Angle, float limit_spd);
void RobStride_Motor_Speed_control(RobStride_Motor *motor, float Speed, float limit_cur);
void RobStride_Motor_current_control(RobStride_Motor *motor, float current);
void RobStride_Motor_Set_Zero_control(RobStride_Motor *motor);
void Enable_Motor(RobStride_Motor *motor);
void Disenable_Motor(RobStride_Motor *motor, uint8_t clear_error);
void Set_RobStride_Motor_parameter(RobStride_Motor *motor, uint16_t Index, float Value, char Value_mode);
void Get_RobStride_Motor_parameter(RobStride_Motor *motor, uint16_t Index);
void Set_CAN_ID(RobStride_Motor *motor, uint8_t Set_CAN_ID);
void Set_ZeroPos(RobStride_Motor *motor);
void RobStride_Motor_MotorDataSave(RobStride_Motor *motor);
void RobStride_Motor_BaudRateChange(RobStride_Motor *motor, uint8_t F_CMD);
void RobStride_Motor_ProactiveEscalationSet(RobStride_Motor *motor, uint8_t F_CMD);
void RobStride_Motor_MotorModeSet(RobStride_Motor *motor, uint8_t F_CMD);
void RobStride_Motor_Analysis(RobStride_Motor *motor, uint8_t *DataFrame, uint32_t ID_ExtId);
void RobStride_Get_CAN_ID(RobStride_Motor *motor);
#endif
