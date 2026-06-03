#include "RS02.h"
#include "main.h"
#include "string.h"
#include "dm_motor_drv.h"

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -44.0f
#define V_MAX 44.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -17.0f
#define T_MAX 17.0f
#define RS02_TARGET_POS_RAD      6.00f   //目标位置
#define RS02_LIMIT_SPEED_RAD_S   2.0f    // 位置控制最大速度

extern CAN_HandleTypeDef hcan1;
uint32_t Mailboxc;
#define RS02_CAN_ID 0x01U            // RS02电机CAN ID
#define RS02_MASTER_ID 0xFDU         // 主控CAN ID
#define RS02_TARGET_SPEED_RAD_S 1.0f // 目标速度（弧度/秒）
#define RS02_LIMIT_CURRENT_A 2.0f    // 限制电流（安培）
#define RS02_CMD_PERIOD_MS 10U       // 控制命令发送周期（毫秒）

extern RobStride_Motor motor1;          // RS02电机实例
static uint32_t rs02_last_cmd_tick = 0; // 上次发送命令的时间戳
static uint8_t rs02_inited = 0;         // RS02初始化标志位
void RS02_UserInit(void)
{
    bsp_can_init(); // 初始化CAN底层驱动

    RobStride_Motor_init(&motor1, RS02_CAN_ID, false); // 初始化RS02电机对象，ID=1，非MIT模式
    motor1.Master_CAN_ID = RS02_MASTER_ID;             // 设置主控CAN ID
    Disenable_Motor(&motor1, 1);                       // 禁用电机
    HAL_Delay(20);                                     // 延时等待

    Get_RobStride_Motor_parameter(&motor1, 0x7005); // 读取当前控制模式参数
    HAL_Delay(20);                                  // 延时等待响应

//    Set_RobStride_Motor_parameter(&motor1, 0x7005, Speed_control_mode, Set_mode); // 设置为速度控制模式
	Set_RobStride_Motor_parameter(&motor1, 0x7005, Pos_control_mode, Set_mode);
    HAL_Delay(5);                                                                 // 延时等待

    Set_RobStride_Motor_parameter(&motor1, 0x7018, RS02_LIMIT_CURRENT_A, Set_parameter); // 设置电流限制为2A
    HAL_Delay(5);                                                                        // 延时等待

    Enable_Motor(&motor1); // 使能电机
    HAL_Delay(20);         // 延时等待电机就绪

    rs02_last_cmd_tick = HAL_GetTick(); // 记录初始时间戳
    rs02_inited = 1;                    // 标记初始化完成
}

/**
 * @brief RS02电机周期性任务函数
 * @note 按照设定周期发送速度控制指令
 */
void RS02_Task(void)
{
    if (!rs02_inited) // 检查是否已完成初始化
    {
        return; // 未初始化则直接返回
    }

    // 检查是否到达控制周期
    if ((HAL_GetTick() - rs02_last_cmd_tick) >= RS02_CMD_PERIOD_MS)
    {
        rs02_last_cmd_tick = HAL_GetTick(); // 更新时间戳
        // 发送速度控制指令：目标速度1rad/s，电流限制2A
        //RobStride_Motor_Speed_control(&motor1, RS02_TARGET_SPEED_RAD_S, RS02_LIMIT_CURRENT_A);
		RobStride_Motor_Pos_control(&motor1, RS02_LIMIT_SPEED_RAD_S, RS02_TARGET_POS_RAD);
    }
}
// -------------------- 数据转换工具 --------------------
float uint16_to_float(uint16_t x, float x_min, float x_max, int bits)
{
    uint32_t span = (1U << bits) - 1U;
    x &= span;
    float offset = x_max - x_min;
    return offset * x / span + x_min;
}

float Byte_to_float(uint8_t *bytedata)
{
    uint32_t data = ((uint32_t)bytedata[7] << 24) |
                    ((uint32_t)bytedata[6] << 16) |
                    ((uint32_t)bytedata[5] << 8) |
                    ((uint32_t)bytedata[4]);
    float data_float;
    memcpy(&data_float, &data, sizeof(float));
    return data_float;
}
// MIT错位码转换至私有模式错位码
uint8_t mapFaults(uint16_t fault16)
{
    uint8_t fault8 = 0;

    if (fault16 & (1U << 14))
        fault8 |= (1U << 4); // 过载故障
    if (fault16 & (1U << 7))
        fault8 |= (1U << 5); // 未标定
    if (fault16 & (1U << 3))
        fault8 |= (1U << 3); // 磁编码故障
    if (fault16 & (1U << 2))
        fault8 |= (1U << 0); // 欠压故障
    if (fault16 & (1U << 1))
        fault8 |= (1U << 1); // 驱动故障
    if (fault16 & (1U << 0))
        fault8 |= (1U << 2); // 过温

    return fault8;
}

// -------------------- 初始化 --------------------
void RobStride_Motor_init(RobStride_Motor *motor, uint8_t CAN_Id, bool MIT_Mode)
{
    if (motor == NULL)
        return;

    memset(motor, 0, sizeof(RobStride_Motor));

    motor->CAN_ID = CAN_Id;
    motor->Master_CAN_ID = 0xFD;
    motor->Motor_Set_All.set_motor_mode = move_control_mode;
    motor->MIT_Mode = MIT_Mode;
    motor->MIT_Type = operationControl;

    motor->drw.run_mode.index = Index_List[0];
    motor->drw.iq_ref.index = Index_List[1];
    motor->drw.spd_ref.index = Index_List[2];
    motor->drw.imit_torque.index = Index_List[3];
    motor->drw.cur_kp.index = Index_List[4];
    motor->drw.cur_ki.index = Index_List[5];
    motor->drw.cur_filt_gain.index = Index_List[6];
    motor->drw.loc_ref.index = Index_List[7];
    motor->drw.limit_spd.index = Index_List[8];
    motor->drw.limit_cur.index = Index_List[9];
    motor->drw.mechPos.index = Index_List[10];
    motor->drw.iqf.index = Index_List[11];
    motor->drw.mechVel.index = Index_List[12];
    motor->drw.VBUS.index = Index_List[13];
    motor->drw.rotation.index = Index_List[14];
}

// -------------------- 接收解析 --------------------
void RobStride_Motor_Analysis(RobStride_Motor *motor, uint8_t *DataFrame, uint32_t ID_ExtId)
{
    if (motor == NULL || DataFrame == NULL)
        return;

    if (motor->MIT_Mode)
    {
        if ((ID_ExtId & 0xFFU) == 0xFDU)
        {
            if (DataFrame[3] == 0x00 && DataFrame[4] == 0x00 && DataFrame[5] == 0x00 && DataFrame[6] == 0x00 && DataFrame[7] == 0x00)
            {
                uint16_t fault16 = 0;
                memcpy(&fault16, &DataFrame[1], 2);
                motor->error_code = mapFaults(fault16);
            }
            else
            {
                motor->Pos_Info.Angle = uint16_to_float(((uint16_t)DataFrame[1] << 8) | DataFrame[2], P_MIN, P_MAX, 16);
                motor->Pos_Info.Speed = uint16_to_float(((uint16_t)DataFrame[3] << 4) | (DataFrame[4] >> 4), V_MIN, V_MAX, 12);
                motor->Pos_Info.Torque = uint16_to_float(((uint16_t)DataFrame[4] << 8) | DataFrame[5], T_MIN, T_MAX, 12);
                motor->Pos_Info.Temp = (float)(((uint16_t)DataFrame[6] << 8) | DataFrame[7]) * 0.1f;
            }
        }
        else
        {
            memcpy(&motor->Unique_ID, DataFrame, 8);
        }
    }
    else
    {
        if ((uint8_t)((ID_ExtId & 0xFF00U) >> 8) == motor->CAN_ID)
        {
            int comm_type = (int)((ID_ExtId & 0x3F000000U) >> 24);

            if (comm_type == 2)
            {
                motor->Pos_Info.Angle = uint16_to_float(((uint16_t)DataFrame[0] << 8) | DataFrame[1], P_MIN, P_MAX, 16);
                motor->Pos_Info.Speed = uint16_to_float(((uint16_t)DataFrame[2] << 8) | DataFrame[3], V_MIN, V_MAX, 16);
                motor->Pos_Info.Torque = uint16_to_float(((uint16_t)DataFrame[4] << 8) | DataFrame[5], T_MIN, T_MAX, 16);
                motor->Pos_Info.Temp = (float)(((uint16_t)DataFrame[6] << 8) | DataFrame[7]) * 0.1f;
                motor->error_code = (uint8_t)((ID_ExtId & 0x3F0000U) >> 16);
                motor->Pos_Info.pattern = (int)((ID_ExtId & 0xC00000U) >> 22);
            }
            else if (comm_type == 17)
            {
                uint16_t index = ((uint16_t)DataFrame[1] << 8) | DataFrame[0];

                for (int index_num = 0; index_num <= 14; index_num++)
                {
                    if (index == Index_List[index_num])
                    {
                        switch (index_num)
                        {
                        case 0:
                            motor->drw.run_mode.data = (float)DataFrame[4];
                            break;
                        case 1:
                            motor->drw.iq_ref.data = Byte_to_float(DataFrame);
                            break;
                        case 2:
                            motor->drw.spd_ref.data = Byte_to_float(DataFrame);
                            break;
                        case 3:
                            motor->drw.imit_torque.data = Byte_to_float(DataFrame);
                            break;
                        case 4:
                            motor->drw.cur_kp.data = Byte_to_float(DataFrame);
                            break;
                        case 5:
                            motor->drw.cur_ki.data = Byte_to_float(DataFrame);
                            break;
                        case 6:
                            motor->drw.cur_filt_gain.data = Byte_to_float(DataFrame);
                            break;
                        case 7:
                            motor->drw.loc_ref.data = Byte_to_float(DataFrame);
                            break;
                        case 8:
                            motor->drw.limit_spd.data = Byte_to_float(DataFrame);
                            break;
                        case 9:
                            motor->drw.limit_cur.data = Byte_to_float(DataFrame);
                            break;
                        case 10:
                            motor->drw.mechPos.data = Byte_to_float(DataFrame);
                            break;
                        case 11:
                            motor->drw.iqf.data = Byte_to_float(DataFrame);
                            break;
                        case 12:
                            motor->drw.mechVel.data = Byte_to_float(DataFrame);
                            break;
                        case 13:
                            motor->drw.VBUS.data = Byte_to_float(DataFrame);
                            break;
                        case 14:
                            motor->drw.rotation.data = (float)((int16_t)(((uint16_t)DataFrame[5] << 8) | DataFrame[4]));
                            break;
                        default:
                            break;
                        }
                    }
                }
            }
            else if ((uint8_t)(ID_ExtId & 0xFFU) == 0xFEU)
            {
                motor->CAN_ID = (uint8_t)((ID_ExtId & 0xFF00U) >> 8);
                memcpy(&motor->Unique_ID, DataFrame, 8);
            }
        }
    }
}

// -------------------- 基础 CAN 指令 --------------------
void RobStride_Get_CAN_ID(RobStride_Motor *motor)
{
    if (motor == NULL)
        return;

    uint8_t txdata[8] = {0};
    CAN_TxHeaderTypeDef TxMessage;

    TxMessage.IDE = CAN_ID_EXT;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 8;
    TxMessage.ExtId = ((uint32_t)Communication_Type_Get_ID << 24) |
                      ((uint32_t)motor->Master_CAN_ID << 8) |
                      motor->CAN_ID;

    HAL_CAN_AddTxMessage(&hcan1, &TxMessage, txdata, &Mailboxc);
}

void Enable_Motor(RobStride_Motor *motor)
{
    if (motor == NULL)
        return;

    if (motor->MIT_Mode)
    {
        RobStride_Motor_MIT_Enable(motor);
    }
    else
    {
        uint8_t txdata[8] = {0};
        CAN_TxHeaderTypeDef TxMessage;

        TxMessage.IDE = CAN_ID_EXT;
        TxMessage.RTR = CAN_RTR_DATA;
        TxMessage.DLC = 8;
        TxMessage.ExtId = ((uint32_t)Communication_Type_MotorEnable << 24) |
                          ((uint32_t)motor->Master_CAN_ID << 8) |
                          motor->CAN_ID;

        HAL_CAN_AddTxMessage(&hcan1, &TxMessage, txdata, &Mailboxc);
    }
}

void Disenable_Motor(RobStride_Motor *motor, uint8_t clear_error)
{
    if (motor == NULL)
        return;

    if (motor->MIT_Mode)
    {
        RobStride_Motor_MIT_Disable(motor);
    }
    else
    {
        uint8_t txdata[8] = {0};
        CAN_TxHeaderTypeDef TxMessage;

        txdata[0] = clear_error;
        TxMessage.IDE = CAN_ID_EXT;
        TxMessage.RTR = CAN_RTR_DATA;
        TxMessage.DLC = 8;
        TxMessage.ExtId = ((uint32_t)Communication_Type_MotorStop << 24) |
                          ((uint32_t)motor->Master_CAN_ID << 8) |
                          motor->CAN_ID;

        HAL_CAN_AddTxMessage(&hcan1, &TxMessage, txdata, &Mailboxc);
        Set_RobStride_Motor_parameter(motor, 0x7005, move_control_mode, Set_mode);
    }
}

void Set_RobStride_Motor_parameter(RobStride_Motor *motor, uint16_t Index, float Value, char Value_mode)
{
    if (motor == NULL)
        return;

    uint8_t txdata[8] = {0};
    CAN_TxHeaderTypeDef TxMessage;

    TxMessage.IDE = CAN_ID_EXT;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 8;
    TxMessage.ExtId = ((uint32_t)Communication_Type_SetSingleParameter << 24) |
                      ((uint32_t)motor->Master_CAN_ID << 8) |
                      motor->CAN_ID;

    txdata[0] = (uint8_t)Index;
    txdata[1] = (uint8_t)(Index >> 8);
    txdata[2] = 0x00;
    txdata[3] = 0x00;

    if (Value_mode == Set_parameter)
    {
        memcpy(&txdata[4], &Value, 4);
    }
    else if (Value_mode == Set_mode)
    {
        motor->Motor_Set_All.set_motor_mode = (int)Value;
        txdata[4] = (uint8_t)Value;
        txdata[5] = 0x00;
        txdata[6] = 0x00;
        txdata[7] = 0x00;
    }

    HAL_CAN_AddTxMessage(&hcan1, &TxMessage, txdata, &Mailboxc);
}

void Get_RobStride_Motor_parameter(RobStride_Motor *motor, uint16_t Index)
{
    if (motor == NULL)
        return;

    uint8_t txdata[8] = {0};
    CAN_TxHeaderTypeDef TxMessage;

    txdata[0] = (uint8_t)Index;
    txdata[1] = (uint8_t)(Index >> 8);

    TxMessage.IDE = CAN_ID_EXT;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 8;
    TxMessage.ExtId = ((uint32_t)Communication_Type_GetSingleParameter << 24) |
                      ((uint32_t)motor->Master_CAN_ID << 8) |
                      motor->CAN_ID;

    HAL_CAN_AddTxMessage(&hcan1, &TxMessage, txdata, &Mailboxc);
}

// -------------------- 私有协议控制 --------------------
void RobStride_Motor_move_control(RobStride_Motor *motor, float Torque, float Angle, float Speed, float Kp, float Kd)
{
    if (motor == NULL)
        return;

    uint8_t txdata[8] = {0};
    CAN_TxHeaderTypeDef TxMessage;

    motor->Motor_Set_All.set_Torque = Torque;
    motor->Motor_Set_All.set_angle = Angle;
    motor->Motor_Set_All.set_speed = Speed;
    motor->Motor_Set_All.set_Kp = Kp;
    motor->Motor_Set_All.set_Kd = Kd;

    if (motor->drw.run_mode.data != 0)
    {
        Set_RobStride_Motor_parameter(motor, 0x7005, move_control_mode, Set_mode);
        Get_RobStride_Motor_parameter(motor, 0x7005);
        Enable_Motor(motor);
        motor->Motor_Set_All.set_motor_mode = move_control_mode;
    }

    if (motor->Pos_Info.pattern != 2)
    {
        Enable_Motor(motor);
    }

    TxMessage.IDE = CAN_ID_EXT;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 8;
    TxMessage.ExtId = ((uint32_t)Communication_Type_MotionControl << 24) |
                      ((uint32_t)float_to_uint(motor->Motor_Set_All.set_Torque, T_MIN, T_MAX, 16) << 8) |
                      motor->CAN_ID;

    txdata[0] = (uint8_t)(float_to_uint(motor->Motor_Set_All.set_angle, P_MIN, P_MAX, 16) >> 8);
    txdata[1] = (uint8_t)(float_to_uint(motor->Motor_Set_All.set_angle, P_MIN, P_MAX, 16));
    txdata[2] = (uint8_t)(float_to_uint(motor->Motor_Set_All.set_speed, V_MIN, V_MAX, 16) >> 8);
    txdata[3] = (uint8_t)(float_to_uint(motor->Motor_Set_All.set_speed, V_MIN, V_MAX, 16));
    txdata[4] = (uint8_t)(float_to_uint(motor->Motor_Set_All.set_Kp, KP_MIN, KP_MAX, 16) >> 8);
    txdata[5] = (uint8_t)(float_to_uint(motor->Motor_Set_All.set_Kp, KP_MIN, KP_MAX, 16));
    txdata[6] = (uint8_t)(float_to_uint(motor->Motor_Set_All.set_Kd, KD_MIN, KD_MAX, 16) >> 8);
    txdata[7] = (uint8_t)(float_to_uint(motor->Motor_Set_All.set_Kd, KD_MIN, KD_MAX, 16));

    HAL_CAN_AddTxMessage(&hcan1, &TxMessage, txdata, &Mailboxc);
}

void RobStride_Motor_Pos_control(RobStride_Motor *motor, float Speed, float Angle)
{
    if (motor == NULL)
        return;

    motor->Motor_Set_All.set_speed = Speed;
    motor->Motor_Set_All.set_angle = Angle;

    if (motor->drw.run_mode.data != 1)
    {
        Set_RobStride_Motor_parameter(motor, 0x7005, Pos_control_mode, Set_mode);
        Get_RobStride_Motor_parameter(motor, 0x7005);
        motor->Motor_Set_All.set_motor_mode = Pos_control_mode;
        Enable_Motor(motor);
        Set_RobStride_Motor_parameter(motor, 0x7024, motor->Motor_Set_All.set_limit_speed, Set_parameter);
        Set_RobStride_Motor_parameter(motor, 0x7025, motor->Motor_Set_All.set_acceleration, Set_parameter);
    }

    HAL_Delay(1);
    Set_RobStride_Motor_parameter(motor, 0x7016, motor->Motor_Set_All.set_angle, Set_parameter);
}

void RobStride_Motor_CSP_control(RobStride_Motor *motor, float Angle, float limit_spd)
{
    if (motor == NULL)
        return;

    if (motor->MIT_Mode)
    {
        RobStride_Motor_MIT_PositionControl(motor, Angle, limit_spd);
    }
    else
    {
        motor->Motor_Set_All.set_angle = Angle;
        motor->Motor_Set_All.set_limit_speed = limit_spd;

        if (motor->drw.run_mode.data != 1)
        {
            Set_RobStride_Motor_parameter(motor, 0x7005, CSP_control_mode, Set_mode);
            Get_RobStride_Motor_parameter(motor, 0x7005);
            Enable_Motor(motor);
            Set_RobStride_Motor_parameter(motor, 0x7017, motor->Motor_Set_All.set_limit_speed, Set_parameter);
        }

        HAL_Delay(1);
        Set_RobStride_Motor_parameter(motor, 0x7016, motor->Motor_Set_All.set_angle, Set_parameter);
    }
}

void RobStride_Motor_Speed_control(RobStride_Motor *motor, float Speed, float limit_cur)
{
    if (motor == NULL)
        return;

    motor->Motor_Set_All.set_speed = Speed;
    motor->Motor_Set_All.set_limit_cur = limit_cur;

    if (motor->drw.run_mode.data != 2)
    {
        Set_RobStride_Motor_parameter(motor, 0x7005, Speed_control_mode, Set_mode);
        Get_RobStride_Motor_parameter(motor, 0x7005);
        Enable_Motor(motor);
        motor->Motor_Set_All.set_motor_mode = Speed_control_mode;
        Set_RobStride_Motor_parameter(motor, 0x7018, motor->Motor_Set_All.set_limit_cur, Set_parameter);
        Set_RobStride_Motor_parameter(motor, 0x7022, 10.0f, Set_parameter);
    }

    Set_RobStride_Motor_parameter(motor, 0x700A, motor->Motor_Set_All.set_speed, Set_parameter);
}

void RobStride_Motor_current_control(RobStride_Motor *motor, float current)
{
    if (motor == NULL)
        return;

    motor->Motor_Set_All.set_current = current;
    motor->output = motor->Motor_Set_All.set_current;

    if (motor->Motor_Set_All.set_motor_mode != Elect_control_mode)
    {
        Set_RobStride_Motor_parameter(motor, 0x7005, Elect_control_mode, Set_mode);
        Get_RobStride_Motor_parameter(motor, 0x7005);
        motor->Motor_Set_All.set_motor_mode = Elect_control_mode;
        Enable_Motor(motor);
    }

    Set_RobStride_Motor_parameter(motor, 0x7006, motor->Motor_Set_All.set_current, Set_parameter);
}

void RobStride_Motor_Set_Zero_control(RobStride_Motor *motor)
{
    if (motor == NULL)
        return;
    Set_RobStride_Motor_parameter(motor, 0x7005, Set_Zero_mode, Set_mode);
}

void Set_CAN_ID(RobStride_Motor *motor, uint8_t Set_CAN_ID_Value)
{
    if (motor == NULL)
        return;

    Disenable_Motor(motor, 0);

    uint8_t txdata[8] = {0};
    CAN_TxHeaderTypeDef TxMessage;

    TxMessage.IDE = CAN_ID_EXT;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 8;
    TxMessage.ExtId = ((uint32_t)Communication_Type_Can_ID << 24) |
                      ((uint32_t)Set_CAN_ID_Value << 16) |
                      ((uint32_t)motor->Master_CAN_ID << 8) |
                      motor->CAN_ID;

    HAL_CAN_AddTxMessage(&hcan1, &TxMessage, txdata, &Mailboxc);
}

void Set_ZeroPos(RobStride_Motor *motor)
{
    if (motor == NULL)
        return;

    Disenable_Motor(motor, 0);

    uint8_t txdata[8] = {0};
    CAN_TxHeaderTypeDef TxMessage;

    TxMessage.IDE = CAN_ID_EXT;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 8;
    TxMessage.ExtId = ((uint32_t)Communication_Type_SetPosZero << 24) |
                      ((uint32_t)motor->Master_CAN_ID << 8) |
                      motor->CAN_ID;
    txdata[0] = 1;

    HAL_CAN_AddTxMessage(&hcan1, &TxMessage, txdata, &Mailboxc);
    Enable_Motor(motor);
}

void RobStride_Motor_MotorDataSave(RobStride_Motor *motor)
{
    if (motor == NULL)
        return;

    uint8_t txdata[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    CAN_TxHeaderTypeDef TxMessage;

    TxMessage.IDE = CAN_ID_EXT;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 8;
    TxMessage.ExtId = ((uint32_t)Communication_Type_MotorDataSave << 24) |
                      ((uint32_t)motor->Master_CAN_ID << 8) |
                      motor->CAN_ID;

    HAL_CAN_AddTxMessage(&hcan1, &TxMessage, txdata, &Mailboxc);
}

void RobStride_Motor_BaudRateChange(RobStride_Motor *motor, uint8_t F_CMD)
{
    if (motor == NULL)
        return;

    uint8_t txdata[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, F_CMD, 0x08};
    CAN_TxHeaderTypeDef TxMessage;

    TxMessage.IDE = CAN_ID_EXT;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 8;
    TxMessage.ExtId = ((uint32_t)Communication_Type_BaudRateChange << 24) |
                      ((uint32_t)motor->Master_CAN_ID << 8) |
                      motor->CAN_ID;

    HAL_CAN_AddTxMessage(&hcan1, &TxMessage, txdata, &Mailboxc);
}

void RobStride_Motor_ProactiveEscalationSet(RobStride_Motor *motor, uint8_t F_CMD)
{
    if (motor == NULL)
        return;

    uint8_t txdata[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, F_CMD, 0x08};
    CAN_TxHeaderTypeDef TxMessage;

    TxMessage.IDE = CAN_ID_EXT;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 8;
    TxMessage.ExtId = ((uint32_t)Communication_Type_ProactiveEscalationSet << 24) |
                      ((uint32_t)motor->Master_CAN_ID << 8) |
                      motor->CAN_ID;

    HAL_CAN_AddTxMessage(&hcan1, &TxMessage, txdata, &Mailboxc);
}

void RobStride_Motor_MotorModeSet(RobStride_Motor *motor, uint8_t F_CMD)
{
    if (motor == NULL)
        return;

    uint8_t txdata[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, F_CMD, 0x08};
    CAN_TxHeaderTypeDef TxMessage;

    TxMessage.IDE = CAN_ID_EXT;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 8;
    TxMessage.ExtId = ((uint32_t)Communication_Type_MotorModeSet << 24) |
                      ((uint32_t)motor->Master_CAN_ID << 8) |
                      motor->CAN_ID;

    HAL_CAN_AddTxMessage(&hcan1, &TxMessage, txdata, &Mailboxc);
}

// -------------------- MIT 协议控制 --------------------
void RobStride_Motor_MIT_Enable(RobStride_Motor *motor)
{
    if (motor == NULL)
        return;

    uint8_t txdata[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    CAN_TxHeaderTypeDef txMsg;

    txMsg.StdId = motor->CAN_ID;
    txMsg.IDE = CAN_ID_STD;
    txMsg.RTR = CAN_RTR_DATA;
    txMsg.DLC = 8;

    HAL_CAN_AddTxMessage(&hcan1, &txMsg, txdata, &Mailboxc);
}

void RobStride_Motor_MIT_Disable(RobStride_Motor *motor)
{
    if (motor == NULL)
        return;

    uint8_t txdata[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    CAN_TxHeaderTypeDef txMsg;

    txMsg.StdId = motor->CAN_ID;
    txMsg.IDE = CAN_ID_STD;
    txMsg.RTR = CAN_RTR_DATA;
    txMsg.DLC = 8;

    HAL_CAN_AddTxMessage(&hcan1, &txMsg, txdata, &Mailboxc);
}

void RobStride_Motor_MIT_ClearOrCheckError(RobStride_Motor *motor, uint8_t F_CMD)
{
    if (motor == NULL)
        return;

    uint8_t txdata[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, F_CMD, 0xFB};
    CAN_TxHeaderTypeDef txMsg;

    txMsg.StdId = motor->CAN_ID;
    txMsg.IDE = CAN_ID_STD;
    txMsg.RTR = CAN_RTR_DATA;
    txMsg.DLC = 8;

    HAL_CAN_AddTxMessage(&hcan1, &txMsg, txdata, &Mailboxc);
}

void RobStride_Motor_MIT_SetMotorType(RobStride_Motor *motor, uint8_t F_CMD)
{
    if (motor == NULL)
        return;

    uint8_t txdata[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, F_CMD, 0xFC};
    CAN_TxHeaderTypeDef txMsg;

    txMsg.StdId = motor->CAN_ID;
    txMsg.IDE = CAN_ID_STD;
    txMsg.RTR = CAN_RTR_DATA;
    txMsg.DLC = 8;

    HAL_CAN_AddTxMessage(&hcan1, &txMsg, txdata, &Mailboxc);
}

void RobStride_Motor_MIT_SetMotorId(RobStride_Motor *motor, uint8_t F_CMD)
{
    if (motor == NULL)
        return;

    uint8_t txdata[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, F_CMD, 0x01};
    CAN_TxHeaderTypeDef txMsg;

    txMsg.StdId = motor->CAN_ID;
    txMsg.IDE = CAN_ID_STD;
    txMsg.RTR = CAN_RTR_DATA;
    txMsg.DLC = 8;

    HAL_CAN_AddTxMessage(&hcan1, &txMsg, txdata, &Mailboxc);
}

void RobStride_Motor_MIT_Control(RobStride_Motor *motor, float Angle, float Speed, float Kp, float Kd, float Torque)
{
    if (motor == NULL)
        return;

    uint8_t txdata[8] = {0};
    CAN_TxHeaderTypeDef txMsg;

    txMsg.StdId = motor->CAN_ID;
    txMsg.IDE = CAN_ID_STD;
    txMsg.RTR = CAN_RTR_DATA;
    txMsg.DLC = 8;

    txdata[0] = (uint8_t)(float_to_uint(Angle, P_MIN, P_MAX, 16) >> 8);
    txdata[1] = (uint8_t)(float_to_uint(Angle, P_MIN, P_MAX, 16));
    txdata[2] = (uint8_t)(float_to_uint(Speed, V_MIN, V_MAX, 12) >> 4);
    txdata[3] = (uint8_t)((float_to_uint(Speed, V_MIN, V_MAX, 12) << 4) | (float_to_uint(Kp, KP_MIN, KP_MAX, 12) >> 8));
    txdata[4] = (uint8_t)(float_to_uint(Kp, KP_MIN, KP_MAX, 12));
    txdata[5] = (uint8_t)(float_to_uint(Kd, KD_MIN, KD_MAX, 12) >> 4);
    txdata[6] = (uint8_t)((float_to_uint(Kd, KD_MIN, KD_MAX, 12) << 4) | (float_to_uint(Torque, T_MIN, T_MAX, 12) >> 8));
    txdata[7] = (uint8_t)(float_to_uint(Torque, T_MIN, T_MAX, 12));

    HAL_CAN_AddTxMessage(&hcan1, &txMsg, txdata, &Mailboxc);
}

void RobStride_Motor_MIT_PositionControl(RobStride_Motor *motor, float position_rad, float speed_rad_per_s)
{
    if (motor == NULL)
        return;

    uint8_t txdata[8] = {0};
    CAN_TxHeaderTypeDef txMsg;

    txMsg.StdId = ((uint32_t)1 << 8) | motor->CAN_ID;
    txMsg.IDE = CAN_ID_STD;
    txMsg.RTR = CAN_RTR_DATA;
    txMsg.DLC = 8;

    memcpy(&txdata[0], &position_rad, 4);
    memcpy(&txdata[4], &speed_rad_per_s, 4);

    HAL_CAN_AddTxMessage(&hcan1, &txMsg, txdata, &Mailboxc);
}

void RobStride_Motor_MIT_SpeedControl(RobStride_Motor *motor, float speed_rad_per_s, float current_limit)
{
    if (motor == NULL)
        return;

    uint8_t txdata[8] = {0};
    CAN_TxHeaderTypeDef txMsg;

    txMsg.StdId = ((uint32_t)2 << 8) | motor->CAN_ID;
    txMsg.IDE = CAN_ID_STD;
    txMsg.RTR = CAN_RTR_DATA;
    txMsg.DLC = 8;

    memcpy(&txdata[0], &speed_rad_per_s, 4);
    memcpy(&txdata[4], &current_limit, 4);

    HAL_CAN_AddTxMessage(&hcan1, &txMsg, txdata, &Mailboxc);
}

void RobStride_Motor_MIT_SetZeroPos(RobStride_Motor *motor)
{
    if (motor == NULL)
        return;

    uint8_t txdata[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
    CAN_TxHeaderTypeDef txMsg;

    txMsg.StdId = motor->CAN_ID;
    txMsg.IDE = CAN_ID_STD;
    txMsg.RTR = CAN_RTR_DATA;
    txMsg.DLC = 8;

    HAL_CAN_AddTxMessage(&hcan1, &txMsg, txdata, &Mailboxc);
}
