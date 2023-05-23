#ifndef HIGH_H
#define  HIGH_H

#include "remote_control.h"
#include "can.h"
#include "CAN_receive.h"
#include "struct_typedef.h"
#include "Motor.h"

//遥控器状态命名
    #define left_switch_is_up           (top_RC->rc.s[0] == 1)
    #define left_switch_is_mid          (top_RC->rc.s[0] == 3)
    #define left_switch_is_down         (top_RC->rc.s[0] == 2)
    #define right_switch_is_up          (top_RC->rc.s[1] == 1)
    #define right_switch_is_mid         (top_RC->rc.s[1] == 3)
    #define right_switch_is_down        (top_RC->rc.s[1] == 2)

    #define left_rocker_up              (top_RC->rc.ch[3] > 0)
    #define left_rocker_down            (top_RC->rc.ch[3] < 0)
    #define left_rocker_mid             (top_RC->rc.ch[3] == 0)

    #define right_rocker_up              (top_RC->rc.ch[1] > 0)
    #define right_rocker_down            (top_RC->rc.ch[1] < 0)
    #define right_rocker_mid             (top_RC->rc.ch[1] == 0)

    //电机状态命名
    #define state_is_stop               (top.lift_state == stop)
    #define state_is_up                 (top.lift_state == up)
    #define state_is_down               (top.lift_state == down)
    
    //m3508转化成抬升速度(m/s)的比例，
    #define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
    #define CHASSIS_HIGH_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//键盘状态
#define KEY_TOP_Z           if_key_pessed(top_RC, 'Z')
#define HIGH_OPEN_RC_SCALE 100\
//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_DEADBAND 0

typedef struct 
{
    int16_t Reset_key;
    int16_t Reset_last_key;
    int16_t Reset_flag;
    int16_t Reset_last_flag;
}reset_t;

class TOP{
public:
    const RC_ctrl_t *top_RC; //底盘使用的遥控器指针
    RC_ctrl_t *last_top_RC; //底盘使用的遥控器指针

    uint16_t top_last_key_v;  //遥控器上次按键

    
    M3508_motor chassis_high_motor[2]; //抬升电机数据

    fp32 chassis_high_motor_start_angle[2];
    bool_t motor_status[2];
    //抬升
    
    float lift_lenth;
    // const auto_t *auto_behave;
    int lift_state;
    // const reset_t *reset_key;
    int8_t save_data;


    void init();
    void feedback_update();

    void save_task();

    void lift_set_mode(void);
    void lift_control(fp32 *vlift_set);
    void behaviour_control_set(fp32 *vlift_set);
    void set_control();
    void solve();
    void output();
};

extern TOP top;

//--------------------------------救援-----------------------------------------------

//-----------------------------------------------------------------------------------
//--------------------------------抬升-----------------------------------------------

// 电控限位值
#define lift_down  -10.0f
#define lift_up  -520.0f

//一号电机PID
#define LIFT_LEFT_KP       20.0f
#define LIFT_LEFT_KI      0.0f
#define LIFT_LEFT_KD      1.0f
#define LIFT_LEFT_MAX_OUT      4000.0f
#define LIFT_LEFT_MAX_IOUT    1.0f
//二号电机PID
#define LIFT_RIGHT_KP       20.0f
#define LIFT_RIGHT_KI       0.0f
#define LIFT_RIGHT_KD       1.0f
#define LIFT_RIGHT_MAX_OUT     4000.0f
#define LIFT_RIGHT_MAX_IOUT     1.0f

//角度环
#define MOTIVE_MOTOR_ANGLE_PID_KP 4000.0f 
#define MOTIVE_MOTOR_ANGLE_PID_KI 0.0f
#define MOTIVE_MOTOR_ANGLE_PID_KD 1.0f
#define MOTIVE_MOTOR_ANGLE_PID_MAX_IOUT 1.0f
#define MOTIVE_MOTOR_ANGLE_PID_MAX_OUT 15000.0f
#define ANGLE_ERR_TOLERANT 5000

// 各种档位的赋值，方便模式设置
extern enum
{
    stop    =   0,
    up,
    down,
    in,
    out,
    
}state_type;

extern enum
{
    READY,
    WAIT,
}motor_status;

 //任务流程
    void feedback_update();

    //救援电机控制
    void save_task();

    //抬升模式改变
    void lift_set_mode();

    //抬升赋值
   void lift_control();

    //电流输出
    void output();


//-----------------------------------------------------------------------------------
#endif