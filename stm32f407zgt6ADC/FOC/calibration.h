#ifndef __CALIBRATION_H__
#define __CALIBRATION_H__

#include "main.h"

//#define PWM_ARR					3000       
//#define DT						(1.0f/20000.0f) // 20KHz
//#define CURRENT_MEASURE_HZ		20000

#define DT						(1.0f/20000.0f) // 20KHz
#define CURRENT_MEASURE_HZ		20000

typedef enum eCalibrateStep{
	CS_NULL = 0,
	
	CS_MOTOR_R_START,
	CS_MOTOR_R_LOOP,
	CS_MOTOR_R_END,
	
	CS_MOTOR_L_START,
	CS_MOTOR_L_LOOP,
	CS_MOTOR_L_END,
	
	CS_ENCODER_DIR_PP_START,
	CS_ENCODER_DIR_PP_LOCK,
	CS_ENCODER_DIR_PP_LOOP,
	CS_ENCODER_DIR_PP_END,
	
	CS_ENCODER_OFFSET_START,
	CS_ENCODER_OFFSET_CW_LOOP,
	CS_ENCODER_OFFSET_CCW_LOOP,
	CS_ENCODER_OFFSET_END,
	
	CS_ERROR,
}tCalibrationStep;

typedef enum eCalibrationError{
	CE_NULL = 0,
	CE_PHASE_RESISTANCE_OUT_OF_RANGE,
	CE_MOTOR_POLE_PAIRS_OUT_OF_RANGE
}tCalibrationError;

typedef struct {
	uint16_t adc_vbus;
    uint16_t adc_phase_a, adc_phase_b, adc_phase_c;
	float v_bus;                                            // DC link voltage
    float i_a, i_b, i_c;                                    // Phase currents
	float dtc_a, dtc_b, dtc_c;
	float i_d_filt, i_q_filt, i_bus_filt;                   // D/Q currents
	
	float current_ctrl_integral_d, current_ctrl_integral_q;	// Current error integrals
	int adc_phase_a_offset;
	int adc_phase_b_offset;
	int adc_phase_c_offset;
} FOCStruct;

void CALIBRATION_loop(void);
void CALIBRATION_end(void);
void CALIBRATION_start(void);

//void FOC_update_current_gain(void);

static inline void int_to_data(int val, uint8_t *data);
static inline int data_to_int(uint8_t *data);
static inline void float_to_data(float val, uint8_t *data);
static inline float data_to_float(uint8_t *data);



#endif


