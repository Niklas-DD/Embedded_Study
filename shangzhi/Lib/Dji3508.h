#ifndef _BSP_CAN_H
#define _BSP_CAN_H

#include "main.h"
#include "can.h"
// 3508??
typedef CAN_HandleTypeDef hcan_t;
typedef struct
{
    uint16_t angle;      //????????
    uint16_t last_angle; //???????????
    int32_t total_angle; //?????
    float last_current;  //????????
    float real_current;  //??
    int16_t speed_rpm;   //??????
    int16_t speed_rpm_filtered;
    float total_angle_output;
} motor_measure_t;
typedef struct
{
    int id;
    int state;
    int p_int;
    int v_int;
    int t_int;
    int kp_int;
    int kd_int;
    float pos;
    float vel;
    float tor;
    float Kp;
    float Kd;
    float Tmos;
    float Tcoil;
} motor_fbpara_t;
typedef struct
{
    uint8_t mode;
    float pos_set;
    float vel_set;
    float tor_set;
    float cur_set;
    float kp_set;
    float kd_set;
} motor_ctrl_t;
typedef struct
{
    uint8_t read_flag;
    uint8_t write_flag;
    uint8_t save_flag;

    float UV_Value;   // µÍÑ¹±£»¤Öµ
    float KT_Value;   // Å¤¾ØÏµÊý
    float OT_Value;   // ¹ýÎÂ±£»¤Öµ
    float OC_Value;   // ¹ýÁ÷±£»¤Öµ
    float ACC;        // ¼ÓËÙ¶È
    float DEC;        // ¼õËÙ¶È
    float MAX_SPD;    // ×î´óËÙ¶È
    uint32_t MST_ID;  // ·´À¡ID
    uint32_t ESC_ID;  // ½ÓÊÕID
    uint32_t TIMEOUT; // ³¬Ê±¾¯±¨Ê±¼ä
    uint32_t cmode;   // ¿ØÖÆÄ£Ê½
    float Damp;       // µç»úÕ³ÖÍÏµÊý
    float Inertia;    // µç»ú×ª¶¯¹ßÁ¿
    uint32_t hw_ver;  // ±£Áô
    uint32_t sw_ver;  // Èí¼þ°æ±¾ºÅ
    uint32_t SN;      // ±£Áô
    uint32_t NPP;     // µç»ú¼«¶ÔÊý
    float Rs;         // µç×è
    float Ls;         // µç¸Ð
    float Flux;       // ´ÅÁ´
    float Gr;         // ³ÝÂÖ¼õËÙ±È
    float PMAX;       // Î»ÖÃÓ³Éä·¶Î§
    float VMAX;       // ËÙ¶ÈÓ³Éä·¶Î§
    float TMAX;       // Å¤¾ØÓ³Éä·¶Î§
    float I_BW;       // µçÁ÷»·¿ØÖÆ´ø¿í
    float KP_ASR;     // ËÙ¶È»·Kp
    float KI_ASR;     // ËÙ¶È»·Ki
    float KP_APR;     // Î»ÖÃ»·Kp
    float KI_APR;     // Î»ÖÃ»·Ki
    float OV_Value;   // ¹ýÑ¹±£»¤Öµ
    float GREF;       // ³ÝÂÖÁ¦¾ØÐ§ÂÊ
    float Deta;       // ËÙ¶È»·×èÄáÏµÊý
    float V_BW;       // ËÙ¶È»·ÂË²¨´ø¿í
    float IQ_cl;      // µçÁ÷»·ÔöÇ¿ÏµÊý
    float VL_cl;      // ËÙ¶È»·ÔöÇ¿ÏµÊý
    uint32_t can_br;  // CAN²¨ÌØÂÊ´úÂë
    uint32_t sub_ver; // ×Ó°æ±¾ºÅ
    float u_off;      // uÏàÆ«ÖÃ
    float v_off;      // vÏàÆ«ÖÃ
    float k1;         // ²¹³¥Òò×Ó1
    float k2;         // ²¹³¥Òò×Ó2
    float m_off;      // ½Ç¶ÈÆ«ÒÆ
    float dir;        // ·½Ïò
    float p_m;        // µç»úÎ»ÖÃ
    float x_out;      // Êä³öÖáÎ»ÖÃ
} esc_inf_t;
typedef struct
{
    uint16_t id;
    uint16_t mst_id;
    motor_fbpara_t para;
    motor_ctrl_t ctrl;
    esc_inf_t tmp;
} motor_t;

// ID
typedef enum
{

    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_2006_M1_ID = 0x201,
    CAN_2006_M2_ID = 0x202,
    CAN_2006_M3_ID = 0x203,
    CAN_2006_M4_ID = 0x204,

    CAN_6020_ALL_ID = 0x1FF,
    CAN_2006_M5_ID = 0x205,
    CAN_2006_M6_ID = 0x206,
    CAN_2006_M7_ID = 0x207,
    CAN_2006_M8_ID = 0x208, // 6020??4??ID
    CAN_6020_ANGLE_MAX = 8192,
    CAN_6020_MAX_CURRENT = 3000,
} ID;

typedef struct
{
    uint16_t can_id;        //??ID
    int16_t set_voltage;    //??????
    uint16_t rotor_angle;   //????
    int16_t rotor_speed;    //??
    int16_t torque_current; //????
    uint8_t temp;           //??
} moto_info_t;

// 三环PID结构体
typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float last_error;
    float output_limit;
} PID3508_t;

// 电机三环控制器
typedef struct
{
    PID3508_t pos_pid;     // 位置环
    PID3508_t speed_pid;   // 速度环
    PID3508_t current_pid; // 电流环
    float target_pos;      // 目标位置
    float target_speed;    // 目标速度
} Motor3LoopCtrl_t;

// 对外接口
void Motor3Loop_Init(Motor3LoopCtrl_t *ctrl);
int16_t Motor3Loop_Update(Motor3LoopCtrl_t *ctrl, motor_measure_t *m, int speed_mode);
// ??????(????)
extern CAN_TxHeaderTypeDef chassis_tx_message;
extern motor_measure_t moto_chassis[8];

//// ??PID???
// extern PID_t Motor1, Motor2, Motor3; // 3508??PID

// ????
void CAN_Filter_Init(void);
void send_chassis_cur1_4(int16_t motor1, int16_t motor2, int16_t motor3);
void get_total_angle(motor_measure_t *p);
void get_motor_measure(motor_measure_t *ptr, uint8_t *Data);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void send_GM6020_voltage(int16_t v1, int16_t v2, int16_t v3, int16_t v4);
void CAN2_Filter_Init(void);
void send_GM6020_voltage_CAN2(int16_t v1, int16_t v2, int16_t v3, int16_t v4);

void bsp_can_init(void);
void can_filter_init(void);
uint8_t canx_send_data(hcan_t *hcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t canx_receive(hcan_t *hcan, uint16_t *recid, uint8_t *buf);
void can1_rx_callback(void);
void can2_rx_callback(void);

#endif
