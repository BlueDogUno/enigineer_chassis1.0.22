#include "high.h"
#include "tim.h"
#include "remote_control.h"
#include "Communicate.h"
#include "Motor.h"
#include "Can_receive.h"
//还要接受上板自动模式的数据 或者直接在这里计算
TOP top;

/**
 * @brief          遥控器的死区判断，因为遥控器的拨杆在中位的时候，不一定为0，
 * @param          输入的遥控器值
 * @param          输出的死区处理后遥控器值
 * @param          死区值
 */
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }
void TOP::init()
{
    //获取遥控器指针
    top_RC = remote_control.get_remote_control_point();
    last_top_RC = remote_control.get_last_remote_control_point();
    top_last_key_v = 0;

    //速度环控制
    fp32 lift_left_pid_parm[5] = {LIFT_LEFT_KP, LIFT_LEFT_KI, LIFT_LEFT_KD, LIFT_LEFT_MAX_IOUT, LIFT_LEFT_MAX_OUT};
    fp32 lift_right_pid_parm[5] = {LIFT_RIGHT_KP, LIFT_RIGHT_KI, LIFT_RIGHT_KD, LIFT_RIGHT_MAX_IOUT, LIFT_RIGHT_MAX_OUT};
    chassis_high_motor[0].speed_pid.init(PID_SPEED, lift_left_pid_parm, &chassis_high_motor[0].speed, &chassis_high_motor[0].speed_set, NULL);
    chassis_high_motor[1].speed_pid.init(PID_SPEED, lift_right_pid_parm, &chassis_high_motor[1].speed, &chassis_high_motor[1].speed_set, NULL);


    //角度环控制
    fp32 lift_angle_pid_parm[5] = {MOTIVE_MOTOR_ANGLE_PID_KP, MOTIVE_MOTOR_ANGLE_PID_KI, MOTIVE_MOTOR_ANGLE_PID_KD, MOTIVE_MOTOR_ANGLE_PID_MAX_IOUT, MOTIVE_MOTOR_ANGLE_PID_MAX_OUT};
    chassis_high_motor[0].speed_pid.init(PID_SPEED, lift_angle_pid_parm, &chassis_high_motor[0].total_angle, &chassis_high_motor[0].angle_set, NULL);
    chassis_high_motor[1].speed_pid.init(PID_SPEED, lift_angle_pid_parm, &chassis_high_motor[1].total_angle, &chassis_high_motor[1].angle_set, NULL);
    //初始化抬升电机
    for (uint8_t i = 0; i < 2; ++i)
    {
       //抬升电机数据更新
        chassis_high_motor[i].init(can_receive.get_chassis_high_motor_measure_point(i));//在Can_receive里面加上返回电机数据的函数
        //初始化pid
        
        chassis_high_motor[i].speed_pid.pid_clear();
        chassis_high_motor[i].angle_pid.pid_clear();
        chassis_high_motor_start_angle[i] = chassis_high_motor[i].total_angle;
        chassis_high_motor[i].angle_error = chassis_high_motor[i].total_angle - chassis_high_motor[i].angle_set;
    }

    top.lift_state = stop;
    top.save_data = stop;
    //更新一下数据
    feedback_update();
}



void TOP::feedback_update()
{
    //记录上一次遥控器值
    top_last_key_v = top_RC->key.v;

    //更新电机数据
    for (uint8_t i = 0; i < 2; ++i)
    {
        //更新抬升电机速度
        chassis_high_motor[i].speed = CHASSIS_HIGH_RPM_TO_VECTOR_SEN * chassis_high_motor[i].motor_measure->speed_rpm;
        chassis_high_motor[i].total_angle = chassis_high_motor[i].motor_measure->total_angle;
        if (chassis_high_motor[i].angle_error < ANGLE_ERR_TOLERANT && chassis_high_motor[i].angle_error > -ANGLE_ERR_TOLERANT)
            motor_status[i] = READY;
        else
            motor_status[i] = WAIT;
    
    }

}

//--------------------------------救援-----------------------------------------------


void TOP::save_task(){
			if (left_switch_is_down)
			{
				if (right_rocker_down)
				{
          top.save_data = 0;
					__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 970);
					__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 2030);
					//HAL_GPIO_WritePin(SERVO_GPIO_Port, SERVO_Pin, GPIO_PIN_RESET);
				}
				else if(right_rocker_up)
				{
          top.save_data = 1;
					__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 2500);
					__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 500);
					//HAL_GPIO_WritePin(SERVO_GPIO_Port, SERVO_Pin, GPIO_PIN_SET);
				}
			}
			//else if(top_RC->mouse.z > 0&&top_RC->key.v == KEY_PRESSED_OFFSET_F)
			//{
				//save_data = 1;
			//}else if(top_RC->mouse.z < 0&&top_RC->key.v == KEY_PRESSED_OFFSET_F)
			//{
				//save_data = 0;
			//}
			
			
			// if(top.save_data == 0)
			// {
			// 	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 2500);
			// 	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 500);
			// }else
			// {
			// 	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1030);//1450
			// 	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1980);//2300
			// }
}

//-----------------------------------------------------------------------------------
//--------------------------------抬升-----------------------------------------------

void TOP::lift_set_mode()
{

        //遥控器控制
        if(right_switch_is_down)//右拨杆向下     
        {
            if (left_rocker_up)
            {
                top.lift_state = up;
            }
            else if(left_rocker_down)
            {
                top.lift_state = down;
            }
            else
            {
                top.lift_state = stop;
            }
        }else if(KEY_TOP_Z){ //键盘控制
            if (top_RC->mouse.y < 0)
            {
                top.lift_state = up;
            }
            if(top_RC->mouse.y > 0)
            {
                top.lift_state = down;
            }
            if(top_RC->mouse.y == 0)
            {
                top.lift_state = stop;
            }
        }else {
            top.lift_state = stop;
        }

}


/*
 * @brief          设置控制量.根据不同底盘控制模式，三个参数会控制不同运动.在这个函数里面，会调用不同的控制函数.
 * @param[out]     vmine_set, 通常控制纵向移动.
 * @param[out]     vstretch_set, 通常控制横向移动.
 * @param[out]     angle_set, 通常控制旋转运动.
 * @param[in]      包括底盘所有信息.
 * @retval         none
 */
void TOP::behaviour_control_set(fp32 *vlift_add)
{

    if (vlift_add == NULL)
    {
        return;
    }
    // lift_control(vlift_set);
    lift_control(vlift_add);
    last_top_RC->key.v = top_RC->key.v;
}


void TOP::lift_control(fp32 *vlift_set)
{   

    if (vlift_set == NULL)
    {
        return;
    }
    static int16_t lift_channel = 0;

    rc_deadband_limit(top_RC->rc.ch[3], lift_channel, RC_DEADBAND);

    *vlift_set = top_RC->rc.ch[3];

    chassis_high_motor[0].angle_set +=  *vlift_set * 10;
    chassis_high_motor[1].angle_set -=  *vlift_set * 10;

}

void TOP::set_control()
{
    fp32 tvlift_set = 0.0f;
    fp32 angle_set = 0;   

    behaviour_control_set(&angle_set);

    chassis_high_motor[0].angle_set =angle_set;
    chassis_high_motor[1].angle_set =-1*angle_set;//?


}

void TOP::solve(){

    for (int i=1 ;i<2; i++){
        chassis_high_motor[i].speed_set = chassis_high_motor[i].angle_pid.pid_calc();
        chassis_high_motor[i].current_give = chassis_high_motor[i].speed_pid.pid_calc();
    }
    
}

void TOP::output()
{
    //赋值电流值
    for (int i = 0; i < 2; i++)
    {
        chassis_high_motor[i].current_give = (int16_t)(chassis_high_motor[i].current_set);
    }

    //电流输出控制,通过调整宏定义控制
    can_receive.can_cmd_chassis_high_motor(chassis_high_motor[0].current_give, chassis_high_motor[1].current_give);
}
//-----------------------------------------------------------------------------------