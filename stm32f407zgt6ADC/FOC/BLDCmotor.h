#ifndef BLDCMotor_H
#define BLDCMotor_H

#include "CurrentSense.h"
#include "FOCMotor.h"
#include "pid.h"

#define SHUTDOWN_PIN                  GPIO_PIN_6
#define SHUTDOWN_GPIO_PORT            GPIOE

#define SHUTDOWN_PIN2                  GPIO_PIN_0
#define SHUTDOWN_GPIO_PORT2            GPIOE


#define M1_Enable  HAL_GPIO_WritePin(SHUTDOWN_GPIO_PORT, SHUTDOWN_PIN, GPIO_PIN_SET)     
#define M1_Disable HAL_GPIO_WritePin(SHUTDOWN_GPIO_PORT, SHUTDOWN_PIN, GPIO_PIN_RESET)    

#define M2_Enable  HAL_GPIO_WritePin(SHUTDOWN_GPIO_PORT2, SHUTDOWN_PIN2, GPIO_PIN_SET)      
#define M2_Disable HAL_GPIO_WritePin(SHUTDOWN_GPIO_PORT2, SHUTDOWN_PIN2, GPIO_PIN_RESET)   

typedef enum
{
    CW      = 1,  //clockwise
    CCW     = -1, // counter clockwise
    UNKNOWN = 0   //not yet known or invalid state
} Direction;

extern long sensor_direction;
extern long sensor_direction2;

//extern float current_sp;
//extern float shaft_angle;
//extern float electrical_angle;
void Motor_init(void);
void Motor_init2(void);

void Motor_initFOC(float zero_electric_offset, Direction _sensor_direction);
void Motor_initFOC2(float zero_electric_offset, Direction _sensor_direction);

int alignSensor(void);
int alignSensor2(void);

uint8_t setPhaseVoltage(float Uq, float Ud, float angle_el);
uint8_t setPhaseVoltage2(float Uq, float Ud, float angle_el);

void move(float new_target);
void move2(float new_target);


float move_velocity(float new_target);
float move_position(float position_target);
//float loopFOCtest(uint8_t sec);

void loopFOCtest(void);
void loopFOCtest_second(void);

void loopFOC(void);


float move_position2(float OPP_shaft_angle);
float mymove(float new_target);


void move_torque1(void);
void move_torque2(void);

#endif


