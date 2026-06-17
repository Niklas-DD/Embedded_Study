#include "Dji3508.h"
#include "math.h"
#include <stdlib.h>
#include "can.h"
#include "main.h"
#define ALPHA 0.3f
#define M3508_ENCODER_RESOLUTION 8192            //????
#define M3508_REDUCTION_RATIO (3591.0f / 187.0f) // ??? � 19.19

extern CAN_HandleTypeDef hcan1;

CAN_TxHeaderTypeDef chassis_tx_message;
uint8_t data_current[8] = {0};

motor_measure_t moto_chassis[8] = {0};

static float PID_Compute(PID3508_t *pid, float setpoint, float measured)
{
  float error = setpoint - measured;

  // 积分限幅，防止越转越快
  pid->integral += error;
  if (pid->integral > pid->output_limit)
    pid->integral = pid->output_limit;
  if (pid->integral < -pid->output_limit)
    pid->integral = -pid->output_limit;

  float derivative = error - pid->last_error;
  pid->last_error = error;

  float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

  // 输出限幅
  if (output > pid->output_limit)
    output = pid->output_limit;
  if (output < -pid->output_limit)
    output = -pid->output_limit;

  return output;
}

// 初始化三环控制器
void Motor3Loop_Init(Motor3LoopCtrl_t *ctrl)
{
  ctrl->pos_pid.Kp = 2.0f;
  ctrl->pos_pid.Ki = 0.0f;
  ctrl->pos_pid.Kd = 0.2f;
  ctrl->pos_pid.integral = 0;
  ctrl->pos_pid.last_error = 0;
  ctrl->pos_pid.output_limit = 2000;

  ctrl->speed_pid.Kp = 0.8f;
  ctrl->speed_pid.Ki = 0.05f; // 降低Ki以减少积分累积
  ctrl->speed_pid.Kd = 0.05f;
  ctrl->speed_pid.integral = 0;
  ctrl->speed_pid.last_error = 0;
  ctrl->speed_pid.output_limit = 2000;

  ctrl->current_pid.Kp = 1.0f;
  ctrl->current_pid.Ki = 0.0f;
  ctrl->current_pid.Kd = 0.0f;
  ctrl->current_pid.integral = 0;
  ctrl->current_pid.last_error = 0;
  ctrl->current_pid.output_limit = 3000;

  ctrl->target_pos = 0;
  ctrl->target_speed = 0;
}

// 三环控制更新
int16_t Motor3Loop_Update(Motor3LoopCtrl_t *ctrl, motor_measure_t *m, int speed_mode)
{
  if (speed_mode)
  {
    // 恒速模式：直接速度环输出
    float cur_ref = PID_Compute(&ctrl->speed_pid, ctrl->target_speed, m->speed_rpm_filtered);
    if (cur_ref > 3000)
      cur_ref = 3000;
    if (cur_ref < -3000)
      cur_ref = -3000;
    return (int16_t)cur_ref;
  }
  else
  {
    // 三环位置控制
    float speed_ref = PID_Compute(&ctrl->pos_pid, ctrl->target_pos, m->total_angle_output);
    float cur_ref = PID_Compute(&ctrl->speed_pid, speed_ref, m->speed_rpm_filtered);
    float cur_final = PID_Compute(&ctrl->current_pid, cur_ref, m->real_current);
    if (cur_final > 3000)
      cur_final = 3000;
    if (cur_final < -3000)
      cur_final = -3000;
    return (int16_t)cur_final;
  }
}
// CAN??????
void CAN_Filter_Init(void)
{

  CAN_FilterTypeDef can_filter_st;
  can_filter_st.FilterActivation = ENABLE;
  can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter_st.FilterIdHigh = 0x0000;
  can_filter_st.FilterIdLow = 0x0000;
  can_filter_st.FilterMaskIdHigh = 0x0000;
  can_filter_st.FilterMaskIdLow = 0x0000;
  can_filter_st.FilterBank = 0;
  can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
  can_filter_st.SlaveStartFilterBank = 14;

  HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

// 3508??????????
void send_chassis_cur1_4(int16_t motor1, int16_t motor2, int16_t motor3)
{
  uint32_t send_mail_box; //????

  chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;

  data_current[0] = motor1 >> 8;
  data_current[1] = motor1;
  data_current[2] = motor2 >> 8;
  data_current[3] = motor2;
  data_current[4] = motor3 >> 8;
  data_current[5] = motor3;
  //	data_current[6] = motor4 >> 8;
  //	data_current[7] = motor4;

  HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, data_current, &send_mail_box);
}

//?????
void get_total_angle(motor_measure_t *p)
{
  int res1, res2, delta;

  if (p->angle < p->last_angle)
  {
    res1 = p->angle - p->last_angle + M3508_ENCODER_RESOLUTION; //?
    res2 = p->angle - p->last_angle;                            //?
  }
  else
  {
    res1 = p->angle - p->last_angle - M3508_ENCODER_RESOLUTION; //?
    res2 = p->angle - p->last_angle;                            //?
  }
  if (abs(res1) < abs(res2))
  {
    delta = res1;
  }
  else
  {
    delta = res2;
  }

  p->total_angle += delta;
  p->last_angle = p->angle;

  p->total_angle_output = (float)p->total_angle / M3508_REDUCTION_RATIO;
}

// ??3508??????
void get_motor_measure(motor_measure_t *ptr, uint8_t *Data)
{
  ptr->last_angle = ptr->angle;
  ptr->angle = (uint16_t)(Data[0] << 8 | Data[1]);

  int16_t raw_speed = (int16_t)(Data[2] << 8 | Data[3]);
  ptr->speed_rpm = raw_speed;

  if (ptr->speed_rpm_filtered == 0)
  {
    ptr->speed_rpm_filtered = raw_speed;
  }
  else
  {
    ptr->speed_rpm_filtered = (int16_t)(ALPHA * raw_speed + (1 - ALPHA) * ptr->speed_rpm_filtered);
  }

  int16_t current_raw = (int16_t)(Data[4] << 8 | Data[5]);
  ptr->real_current = current_raw * 5.0f / 16384.0f;
}

extern motor_t motor[10];
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
  /* Converts a float to an unsigned int, given range and number of bits */
  float span = x_max - x_min;
  float offset = x_min;
  return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}
/**
************************************************************************
* @brief:      	can_bsp_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN 'G
************************************************************************
**/
void bsp_can_init(void)
{
  can_filter_init();
  HAL_CAN_Start(&hcan1);

  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}
/**
************************************************************************
* @brief:      	can_filter_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN?��?��'��
************************************************************************
**/
void can_filter_init(void)
{
  CAN_FilterTypeDef can_filter_st;
  can_filter_st.FilterActivation = ENABLE;
  can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter_st.FilterIdHigh = 0x0000;
  can_filter_st.FilterIdLow = 0x0000;
  can_filter_st.FilterMaskIdHigh = 0x0000;
  can_filter_st.FilterMaskIdLow = 0x0000;
  can_filter_st.FilterBank = 0;
  can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
  HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}
/**
************************************************************************
* @brief:      	canx_bsp_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
* @param:       hcan: CAN�?
* @param:       id: 	CAN?��ID
* @param:       data: ��_�C��?* @param:       len:  ��_�C��?��?* @retval:     	void
* @details:    	��_?�?************************************************************************
**/
uint8_t canx_send_data(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len)
{
  CAN_TxHeaderTypeDef tx_header;

  tx_header.StdId = id;
  tx_header.ExtId = 0;
  tx_header.IDE = 0;
  tx_header.RTR = 0;
  tx_header.DLC = len;
  /*?���?k�_??���?��?�_��?*/
  if (HAL_CAN_AddTxMessage(hcan, &tx_header, data, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK)
  {
    if (HAL_CAN_AddTxMessage(hcan, &tx_header, data, (uint32_t *)CAN_TX_MAILBOX1) != HAL_OK)
    {
      HAL_CAN_AddTxMessage(hcan, &tx_header, data, (uint32_t *)CAN_TX_MAILBOX2);
    }
  }
  return 0;
}
/**
************************************************************************
* @brief:      	canx_bsp_receive(CAN_HandleTypeDef *hcan, uint8_t *buf)
* @param:       hcan: CAN�?
* @param[out]:  rec_id: 	�??�?�?aAN?��ID
* @param:       buf���??��?��? @retval:     	�??C��?��?* @details:    	�??��?************************************************************************
**/

uint8_t canx_receive(hcan_t *hcan, uint16_t *rec_id, uint8_t *buf)
{
  CAN_RxHeaderTypeDef rx_header;
  *rec_id = rx_header.StdId;
  return rx_header.DLC; // �??��?�
}

/**
************************************************************************
* @brief:      	fdcan1_rx_callback: CAN1�???���?
* @param:      	void
* @retval:     	void
* @details:    	���?N1�?????������???��cD�?��?�????�J��??*               ���??�ID?0?����?dm4310_fbdata��?��?Motor�k���?�?�
************************************************************************
**/

//????
// void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
// {
//   CAN_RxHeaderTypeDef rx_header;
//   uint8_t rx_data[8] = {0};
//   uint16_t rec_id[2];

//   HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

//   if (hcan->Instance == CAN1)
//   {
//     rec_id[0] = rx_header.StdId;
//     // CAN1 -> ????
//     switch (rec_id[0])
//     {
//     case CAN_2006_M1_ID:
//     case CAN_2006_M2_ID:
//     case CAN_2006_M3_ID:
//     case CAN_2006_M4_ID:
//     {
//       uint8_t i = rx_header.StdId - CAN_2006_M1_ID;
//       get_motor_measure(&moto_chassis[i], rx_data);
//       get_total_angle(&moto_chassis[i]);
//       break;
//     }

//     default:
//       break;
//     }
//   }
//   else if (hcan->Instance == CAN2)
//   {
//     rec_id[1] = rx_header.StdId;

//     switch (rec_id[1])
//     {
//     case 0x0000:
//       dm_motor_fbdata(&motor[Motor1], rx_data);
//       receive_motor_data(&motor[Motor1], rx_data);
//       break;
//     }
//   }
// }
