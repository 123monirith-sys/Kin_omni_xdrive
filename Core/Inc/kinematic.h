/*
 * kinematic.h
 *
 *  Created on: Dec 9, 2025
 *      Author: rith
 */

#ifndef INC_KINEMATIC_H_
#define INC_KINEMATIC_H_

#include <stdint.h>
#include <stdio.h>

typedef struct{
	float vx_orien;
	float vy_orien;
	float omega_orien;
	float x_orien;
	float y_orien;
	float theta_orien;

}ORIEN;

typedef struct{
	float r;
	float l;
	float motor_max_speed;
	float wheel_speed_max;
}OMNIWHEEL;

typedef struct{
	float vx;
	float vy;
	float omega;

}CONTROLLER;
typedef struct{
	float x;
	float y;
	float theta;
}CONTROL_POSE;
void orien_init(ORIEN *orien);
void omni_init(OMNIWHEEL *omni);
void test_omni_set_motor_speed_90(OMNIWHEEL *omni, CONTROLLER *data, float *wheel_speed);
void omni45_kinematic(CONTROLLER *data ,OMNIWHEEL *omni, float *wheel_speed); // remeber this
void omni45_kinematic2(CONTROLLER *data ,OMNIWHEEL *omni, float *wheel_speed);
void omni45_kinematic_pose(CONTROL_POSE *data_pose,OMNIWHEEL *omni, float *wheel_speed); //givedirection

#endif /* INC_KINEMATIC_H_ */
