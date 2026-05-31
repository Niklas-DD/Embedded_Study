#ifndef _BSP_CAN_H
#define _BSP_CAN_H

#include "main.h"
#include "can.h"

//3508??
typedef struct{
	uint16_t angle;//????????
	uint16_t last_angle;//???????????
	int32_t total_angle;//?????
	float last_current;//????????
	float real_current;//??
	int16_t speed_rpm;//??????	
	int16_t speed_rpm_filtered;
	float total_angle_output;
}motor_measure_t;

//ID
typedef enum{
	
	CAN_CHASSIS_ALL_ID = 0x200,
	CAN_2006_M1_ID = 0x201,
	CAN_2006_M2_ID = 0x202,
	CAN_2006_M3_ID = 0x203,
	CAN_2006_M4_ID = 0x204,
	
	CAN_6020_ALL_ID = 0x1FF,
    CAN_2006_M5_ID = 0x205,
	CAN_2006_M6_ID = 0x206,
	CAN_2006_M7_ID = 0x207,
	CAN_2006_M8_ID = 0x208,    // 6020??4??ID
    CAN_6020_ANGLE_MAX = 8192, 
	CAN_6020_MAX_CURRENT =3000,
}ID;

typedef struct
{
    uint16_t can_id;//??ID
    int16_t  set_voltage;//??????
    uint16_t rotor_angle;//????
    int16_t  rotor_speed;//??
    int16_t  torque_current;//????
    uint8_t  temp;//??
}moto_info_t;


// ??????(????)
extern CAN_TxHeaderTypeDef chassis_tx_message;
extern motor_measure_t moto_chassis[8];

//// ??PID???
//extern PID_t Motor1, Motor2, Motor3; // 3508??PID

// ????
void CAN_Filter_Init(void);
void send_chassis_cur1_4(int16_t motor1, int16_t motor2, int16_t motor3);
void get_total_angle(motor_measure_t *p);
void get_motor_measure(motor_measure_t *ptr, uint8_t *Data);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void send_GM6020_voltage(int16_t v1, int16_t v2, int16_t v3, int16_t v4);
void CAN2_Filter_Init(void);
void send_GM6020_voltage_CAN2(int16_t v1, int16_t v2, int16_t v3, int16_t v4);

typedef CAN_HandleTypeDef hcan_t;

void bsp_can_init(void);
void can_filter_init(void);
uint8_t canx_send_data(hcan_t *hcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t canx_receive(hcan_t *hcan, uint16_t *recid, uint8_t *buf);
void can1_rx_callback(void);
void can2_rx_callback(void);



#endif
