#include "FOCMotor.h"

float shaft_angle;//!< current motor angle
float shaft_angle2;

float electrical_angle;
float electrical_angle2;

float shaft_velocity;
float shaft_velocity2;

float current_sp;
float current_sp2;


float shaft_velocity_sp;
float shaft_angle_sp;

float shaft_velocity_sp2;
float shaft_angle_sp2;

extern int  pole_pairs;

DQVoltage_s voltage;
DQVoltage_s voltage2;


DQCurrent_s current;
//DQCurrent_s current2;

TorqueControlType torque_controller;
MotionControlType controller;

float sensor_offset;   //编码器0度和电机三相驱动0度偏差
float zero_angle;
float Electrical_Angele_Sensor_Offset;//编码器电角度0度和电机三相驱动0度偏差
float zero_electric_angle;
float zero_electric_angle2;

/******************************************************************************/
// shaft angle calculation
float shaftAngle(void)
{
  // if no sensor linked return previous value ( for open loop )
  //if(!sensor) return shaft_angle;
  return sensor_direction*getAngle() - sensor_offset;
}
float shaftAngle2(void)
{
  // if no sensor linked return previous value ( for open loop )
  //if(!sensor) return shaft_angle;
  return sensor_direction2*getAngle2() - sensor_offset;
}


// shaft velocity calculation
float shaftVelocity(void)
{
  // if no sensor linked return previous value ( for open loop )
  //if(!sensor) return shaft_velocity;
  return sensor_direction*LPFoperator(&LPF_velocity,getVelocity());
}

float shaftVelocity2(void)
{
  // if no sensor linked return previous value ( for open loop )
  //if(!sensor) return shaft_velocity;
  return sensor_direction2*LPFoperator(&LPF_velocity2,getVelocity2());
}
///******************************************************************************/
//电角度
float electricalAngle(void)
{
  return _normalizeAngle((shaft_angle + sensor_offset) * pole_pairs - zero_electric_angle);
//	return _normalizeAngle(_2PI-(((AS5600_ReadRawAngle()/ cpr) * _2PI) * pole_pairs - Electrical_Angele_Sensor_Offset - zero_electric_angle));
}

//电角度
float electricalAngle2(void)
{
  return _normalizeAngle((shaft_angle2 + sensor_offset) * pole_pairs - zero_electric_angle2);
//	return _normalizeAngle(_2PI-(((AS5600_ReadRawAngle()/ cpr) * _2PI) * pole_pairs - Electrical_Angele_Sensor_Offset - zero_electric_angle));
}

