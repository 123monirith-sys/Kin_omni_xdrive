/*
 * kinematic.c
 *
 *  Created on: Dec 9, 2025
 *      Author: rith
 */


#include "kinematic.h"
#include "math.h"

void orien_init(ORIEN *orien){
	orien->vx_orien=0.0f;
	orien->vy_orien=0.0f;
	orien->omega_orien =0.0f;
	orien->x_orien=0.0f;
	orien->y_orien=0.0f;
	orien->theta_orien=0.0f;
}// for odom

void omni_init(OMNIWHEEL *omni)
{
	omni->r = 0.065f;//unit as m
	omni->l = 0.57f;//unit as m
	omni->motor_max_speed = 600.0f;//full speed  rpm
	omni->wheel_speed_max = 90.0f;//robot max speed rpm

}
//omni_ 90
void test_omni_set_motor_speed_90(OMNIWHEEL *omni, CONTROLLER *data,float *wheel_speed)
{
	wheel_speed[0] = (-data->vy + data->omega*omni->l)/(omni->r) *(omni->motor_max_speed /omni->wheel_speed_max);
	wheel_speed[1] = (-data->vx + data->omega*omni->l)/(omni->r) *(omni->motor_max_speed /omni->wheel_speed_max);
	wheel_speed[2] = (data->vy  + data->omega*omni->l) / (omni->r) *(omni->motor_max_speed /omni->wheel_speed_max);
	wheel_speed[3] = (data->vx + data->omega*omni->l)/(omni->r) *(omni->motor_max_speed /omni->wheel_speed_max);

}

// for konolomic xdrive
void omni45_kinematic(CONTROLLER *data,OMNIWHEEL *omni, float *wheel_speed) // remeber this
{
        wheel_speed[0] = (((0.707106781 * (data->vx + data->vy)) - ((data->omega) * omni->l)) / omni->r)*60.0f / (2.0f * M_PI) ;//fl
	    wheel_speed[1] = (((0.707106781 * (data->vx - data->vy)) + ((data->omega) * omni->l)) / omni->r)*60.0f / (2.0f * M_PI) ;//fr
	    wheel_speed[2] = (((0.707106781 * (-data->vx + data->vy)) + ((data->omega) * omni->l)) / omni->r)*60.0f / (2.0f * M_PI);//rl
	    wheel_speed[3] = -(((0.707106781 * (-data->vx - data->vy)) - ((data->omega) * omni->l)) / omni->r)*60.0f / (2.0f * M_PI);//rr
}
void omni45_kinematic_pose(CONTROL_POSE *data_pose,OMNIWHEEL *omni, float *wheel_speed){

	wheel_speed[0] = ((0.35*data_pose->x) + (0.35*data_pose->y) - (0.25*data_pose->theta))*60.0f / (2.0f * M_PI) ;
	wheel_speed[1] = ((0.35*data_pose->x) - (0.35*data_pose->y) + (0.25*data_pose->theta))*60.0f / (2.0f * M_PI) ;
	wheel_speed[2] = ((-0.35*data_pose->x) + (0.35*data_pose->y) + (0.25*data_pose->theta))*60.0f / (2.0f * M_PI);
	wheel_speed[3] = -((-0.35*data_pose->x) - (0.35*data_pose->y) - (0.25*data_pose->theta))*60.0f / (2.0f * M_PI);
}

// for konolomic xdrive test
void omni45_kinematic2(CONTROLLER *data,OMNIWHEEL *omni, float *wheel_speed) // remeber this
{

        wheel_speed[0] = (((0.707106781 * (data->vx + data->vy)) - ((data->omega) * omni->l)) / omni->r)*(omni->motor_max_speed/omni->wheel_speed_max);//fl
	    wheel_speed[1] = (((0.707106781 * (data->vx - data->vy)) + ((data->omega) * omni->l)) / omni->r)*(omni->motor_max_speed/omni->wheel_speed_max) ;//fr
	    wheel_speed[2] = (((0.707106781 * (-data->vx + data->vy)) + ((data->omega) * omni->l)) / omni->r)*(omni->motor_max_speed/omni->wheel_speed_max);//rl
	    wheel_speed[3] = -(((0.707106781 * (-data->vx - data->vy)) - ((data->omega) * omni->l)) / omni->r)*(omni->motor_max_speed/omni->wheel_speed_max);//rr
}
