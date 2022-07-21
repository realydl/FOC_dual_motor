#include "calibration.h"
#include "foc_utils.h"

#define CALB_SPEED 								_2PI		// Calibration speed in rad/s
#define MOTOR_POLE_PAIRS_MAX			30

#define OFFSET_LUT_NUM						128
#define SAMPLES_PER_PPAIR					OFFSET_LUT_NUM

#define ENCODER_CPR 							AS5600_CPR 

extern int pole_pairs;//电机极对数

float voltages[2];

float calib_current =0.50f;//为啥是10？？？？
float calib_max_voltage = 3.0f;
float motor_phase_resistance = 0.0f;
float motor_phase_inductance = 0.0f;

int encoder_offset = 0;

float current_ctrl_bandwidth =1000.0f;
float current_ctrl_p_gain =0.0f;
float current_ctrl_i_gain = 0.0f;

static int *p_error_arr = 0;

static tCalibrationStep mCalibStep = CS_NULL;

//mCalibStep = CS_MOTOR_R_START;

static tCalibrationError mCalibError = CE_NULL;

PhaseCurrent_s cailbCerrents;


static inline void int_to_data(int val, uint8_t *data)
{
		data[0] = *(((uint8_t*)(&val)) + 0);
    data[1] = *(((uint8_t*)(&val)) + 1);
    data[2] = *(((uint8_t*)(&val)) + 2);
    data[3] = *(((uint8_t*)(&val)) + 3);
}

static inline int data_to_int(uint8_t *data)
{
	int tmp_int;
    *(((uint8_t*)(&tmp_int)) + 0) = data[0];
    *(((uint8_t*)(&tmp_int)) + 1) = data[1];
    *(((uint8_t*)(&tmp_int)) + 2) = data[2];
    *(((uint8_t*)(&tmp_int)) + 3) = data[3];
    return tmp_int;
}
static inline void float_to_data(float val, uint8_t *data)
{
    data[0] = *(((uint8_t*)(&val)) + 0);
    data[1] = *(((uint8_t*)(&val)) + 1);
    data[2] = *(((uint8_t*)(&val)) + 2);
    data[3] = *(((uint8_t*)(&val)) + 3);
}
static inline float data_to_float(uint8_t *data)
{
    float tmp_float;
    *(((uint8_t*)(&tmp_float)) + 0) = data[0];
    *(((uint8_t*)(&tmp_float)) + 1) = data[1];
    *(((uint8_t*)(&tmp_float)) + 2) = data[2];
    *(((uint8_t*)(&tmp_float)) + 3) = data[3];
    return tmp_float;
}

void FOC_update_current_gain(void)
{
	//电流环p参数=相电感*电流环带宽
	current_ctrl_p_gain = motor_phase_inductance * current_ctrl_bandwidth;
	//电流环i参数=相电阻*电流环带宽
	current_ctrl_i_gain = motor_phase_resistance * current_ctrl_bandwidth;
}

void CALIBRATION_start(void)
{
//	if(p_error_arr == NULL){
//		p_error_arr = HEAP_malloc(SAMPLES_PER_PPAIR*MOTOR_POLE_PAIRS_MAX*4);
//	}
	mCalibStep = CS_MOTOR_R_START;
}


void CALIBRATION_end(void)
{
	mCalibStep = CS_NULL;
//	HEAP_free(p_error_arr);
	p_error_arr = NULL;
}


void CALIBRATION_loop(void)
{
	static uint32_t loop_count;
	
	// R
	static const float kI = 1.0f;
  static const int num_R_cycles = CURRENT_MEASURE_HZ * 2;
	
	// L
  static float Ialphas[2];
	static const uint32_t num_L_cycles = CURRENT_MEASURE_HZ / 2; //为啥要/2
	
	static int raw_offset;
	static int sample_count;
	static float next_sample_time;
	static float theta_ref;
	static float start_count;
	static float end_count;

	float time = (float)(loop_count*DT);
	
	while((motor_phase_resistance == 0)||(motor_phase_inductance == 0)){

	switch(mCalibStep){
		case CS_NULL:
			break;
		
		//相电阻辨识
		case CS_MOTOR_R_START:
			loop_count = 0;
			voltages[0] = 0.0f;
		
//			FOC_arm();
		
			mCalibStep = CS_MOTOR_R_LOOP;
			break;
		
		case CS_MOTOR_R_LOOP:
//			if((AS5600_ReadRawAngle()>=0)&&(AS5600_ReadRawAngle()<=10))
//				cailbCerrents =	getSelectionPhaseCurrents(1);
//			else
//				cailbCerrents =	getSelectionPhaseCurrents(6);
		
		cailbCerrents = getPhaseCurrents();
//			voltages[0] += (kI * DT) * (calib_current - foc->i_a);
			voltages[0] += (kI * DT) * (calib_current - cailbCerrents.a);
			
			if (voltages[0] > calib_max_voltage || voltages[0] < -calib_max_voltage){		
//				FOC_disarm();
				mCalibError = CE_PHASE_RESISTANCE_OUT_OF_RANGE;
				mCalibStep = CS_ERROR;
				break;
			}
			
			// Test voltage along phase A
//			apply_voltage_timings(foc->v_bus, voltages[0], 0.0f, 0.0f);
			setPhaseVoltage(0.0f, voltages[0], 0);
			
			// Test runs for 2s
			if(loop_count >= num_R_cycles){		
//				FOC_disarm();				
				mCalibStep = CS_MOTOR_R_END;
			}
			break;
			
		case CS_MOTOR_R_END:
//			motor_phase_resistance = (voltages[0] / UsrConfig.calib_current) * (2.0f / 3.0f);
			motor_phase_resistance = (voltages[0] / calib_current) * (2.0f / 3.0f);
//			motor_phase_resistance = voltages[0]/cailbCerrents.a;
			printf("Motor phase resistance = %f\n\r", motor_phase_resistance);
		
			{
				uint8_t data[4];
				float_to_data(motor_phase_resistance, data);
//				CAN_report_calibration(1, data);
			}
			
			mCalibStep = CS_MOTOR_L_START;
			break;
			
		//相电感辨识
		case CS_MOTOR_L_START:
			Ialphas[0] = 0.0f;
			Ialphas[1] = 0.0f;
			voltages[0] = -calib_max_voltage;
			voltages[1] = +calib_max_voltage;
			loop_count = 0;
//			FOC_arm();
//			apply_voltage_timings(foc->v_bus, voltages[0], 0, 0);
			setPhaseVoltage(0.0f, voltages[0], 0);
			mCalibStep = CS_MOTOR_L_LOOP;
			break;
		
		case CS_MOTOR_L_LOOP:
			{
				int i = loop_count & 1;
//				Ialphas[i] += foc->i_a;
			if((AS5600_ReadRawAngle()>=0)&&(AS5600_ReadRawAngle()<=10))
				cailbCerrents =	getSelectionPhaseCurrents(1);
			else
				cailbCerrents =	getSelectionPhaseCurrents(6);
			
				Ialphas[i] += cailbCerrents.a;
				// Test voltage along phase A
//				apply_voltage_timings(foc->v_bus, voltages[i], 0.0f, 0.0f);
				setPhaseVoltage(0.0f, voltages[i], 0);
				
				if(loop_count >= (num_L_cycles<<1)){
//					FOC_disarm();
					mCalibStep = CS_MOTOR_L_END;
				}
			}
			break;
			
		case CS_MOTOR_L_END:
			{
				float dI_by_dt = (Ialphas[1] - Ialphas[0]) / (float)(DT * num_L_cycles);
				float L = calib_max_voltage / dI_by_dt;
				motor_phase_inductance = L * (2.0f / 3.0f);
				printf("Motor phase inductance = %f\n\r", motor_phase_inductance);
				
				{
					uint8_t data[4];
					float_to_data(motor_phase_inductance, data);
//					CAN_report_calibration(2, data);
				}
		
				FOC_update_current_gain();
				
				mCalibStep = CS_ENCODER_OFFSET_START;
			}
			break;
			
//		case CS_ENCODER_DIR_PP_START:
//			voltages[0] = UsrConfig.calib_current * UsrConfig.motor_phase_resistance * 3.0f / 2.0f;;
//			FOC_arm();
//			loop_count = 0;
//			mCalibStep = CS_ENCODER_DIR_PP_LOCK;
//			break;
//			
//		case CS_ENCODER_DIR_PP_LOCK:
//			apply_voltage_timings(foc->v_bus, (voltages[0] * time / 2.0f), 0, 0);
//			if (time >= 2){
//				start_count = Encoder.shadow_count_;
//				loop_count = 0;
//				mCalibStep = CS_ENCODER_DIR_PP_LOOP;
//				break;
//			}
//			break;
//		
//		case CS_ENCODER_DIR_PP_LOOP:
//			apply_voltage_timings(foc->v_bus, voltages[0], 0, CALB_SPEED*time);
//			if (time >= 4.0f*M_PI/CALB_SPEED){
//				end_count = Encoder.shadow_count_;
//				apply_voltage_timings(foc->v_bus, voltages[0], 0, 0);
//				mCalibStep = CS_ENCODER_DIR_PP_END;
//				break;
//			}
//			break;
//			
//		case CS_ENCODER_DIR_PP_END:
//			{	
//				// Check motor pole pairs
//				int diff = end_count - start_count;
//				UsrConfig.motor_pole_pairs = round(2.0f/fabsf(diff/(float)ENCODER_CPR));
//				DEBUG("Motor pole pairs = %d\n\r", UsrConfig.motor_pole_pairs);
//				if(UsrConfig.motor_pole_pairs > MOTOR_POLE_PAIRS_MAX){
//					mCalibError = CE_MOTOR_POLE_PAIRS_OUT_OF_RANGE;
//					mCalibStep = CS_ERROR;
//					break;
//				}
//				
//				uint8_t data[4];
//				int_to_data(UsrConfig.motor_pole_pairs, data);
//				CAN_report_calibration(3, data);
//				
//				// Check response and direction
//				if(diff > +10){
//					UsrConfig.encoder_dir = 1;
//					DEBUG("Encoder dir = +1\n\r");
//				}else if(diff < -10){
//					UsrConfig.encoder_dir = -1;
//					DEBUG("Encoder dir = -1\n\r");
//				}
//				
//				int_to_data(UsrConfig.encoder_dir, data);
//				CAN_report_calibration(4, data);
//			}
//			mCalibStep = CS_ENCODER_OFFSET_START;
//			break;

#if 0
		case CS_ENCODER_OFFSET_START:
			theta_ref = 0;
			sample_count = 0;
			next_sample_time = 0;
			
//			raw_offset = Encoder.raw;
			raw_offset = AS5600_ReadRawAngle();
		
			loop_count = 0;
			mCalibStep = CS_ENCODER_OFFSET_CW_LOOP;
			break;
		
		case CS_ENCODER_OFFSET_CW_LOOP:
			// rotate voltage vector through one mechanical rotation in the positive direction
			theta_ref += CALB_SPEED * DT;
			//apply_voltage_timings(foc->v_bus, voltages[0], 0, theta_ref);
			setPhaseVoltage(0,voltage[0],theta_ref);
		
			// sample SAMPLES_PER_PPAIR times per pole-pair
			if(time > next_sample_time){
				int count_ref = ENCODER_CPR * (theta_ref/(float)(_2PI*pole_pairs));
				int error;
				if(UsrConfig.encoder_dir == +1){
//					error = Encoder.raw - count_ref;
					error = AS5600_ReadRawAngle() - count_ref;
				}else if(UsrConfig.encoder_dir == -1){
//					error = (ENCODER_CPR - Encoder.raw) - count_ref;
					error = (ENCODER_CPR - AS5600_ReadRawAngle()) - count_ref;
				}
				p_error_arr[sample_count] = error + ENCODER_CPR * (error<0);
				sample_count ++;
				if(sample_count >= pole_pairs*SAMPLES_PER_PPAIR){
					sample_count --;
				}
				next_sample_time += _2PI/(CALB_SPEED*SAMPLES_PER_PPAIR);
			}

			if (time >= _2PI*pole_pairs/CALB_SPEED){
				next_sample_time = 0;
				loop_count = 0;
				mCalibStep = CS_ENCODER_OFFSET_CCW_LOOP;
				break;
			}
			break;
			
		case CS_ENCODER_OFFSET_CCW_LOOP:
			// rotate voltage vector through one mechanical rotation in the negative direction
			theta_ref -= CALB_SPEED * DT;
//			apply_voltage_timings(foc->v_bus, voltages[0], 0, theta_ref);
			setPhaseVoltage(0,voltage[0],theta_ref);
		
			// sample SAMPLES_PER_PPAIR times per pole-pair
			if(time > next_sample_time){
				int count_ref = ENCODER_CPR * (theta_ref/(float)(_2PI*pole_pairs));
				int error;
				if(UsrConfig.encoder_dir == +1){
//					error = Encoder.raw - count_ref;
					error = AS5600_ReadRawAngle() - count_ref;
				}else if(UsrConfig.encoder_dir == -1){
//					error = (ENCODER_CPR - Encoder.raw) - count_ref;
					error = (ENCODER_CPR - AS5600_ReadRawAngle()) - count_ref;
				}
				error = error + ENCODER_CPR * (error<0);
				p_error_arr[sample_count] = (p_error_arr[sample_count] + error)/2;
				sample_count --;
				if(sample_count <= 0){
					sample_count = 0;
				}
				next_sample_time += _2PI/(CALB_SPEED*SAMPLES_PER_PPAIR);
			}

			if (time >= _2PI*pole_pairs/CALB_SPEED){
//				raw_offset += Encoder.raw;
				raw_offset += AS5600_ReadRawAngle();
				raw_offset /= 2;
//				FOC_disarm();
				mCalibStep = CS_ENCODER_OFFSET_END;
				break;
			}
			break;
			
		case CS_ENCODER_OFFSET_END:
			{
				// Calculate average offset
				int64_t ezero_mean = 0;
				for(int i = 0; i<(pole_pairs*SAMPLES_PER_PPAIR); i++){
					ezero_mean += p_error_arr[i];
				}
				encoder_offset = ezero_mean/(pole_pairs*SAMPLES_PER_PPAIR);
				printf("Encoder offset = %d\n\r", encoder_offset);
				
				uint8_t data[4];
				int_to_data(encoder_offset, data);
//				CAN_report_calibration(5, data);
		
				// Moving average to filter out cogging ripple
				int window = SAMPLES_PER_PPAIR;
				raw_offset = OFFSET_LUT_NUM * raw_offset / ENCODER_CPR;
				for(int i = 0; i<OFFSET_LUT_NUM; i++){
					int moving_avg = 0;
					for(int j = (-window)/2; j<(window)/2; j++){
						int index = i*pole_pairs*SAMPLES_PER_PPAIR/OFFSET_LUT_NUM + j;
						if(index < 0){
							index += (SAMPLES_PER_PPAIR*pole_pairs);
						}else if(index > (SAMPLES_PER_PPAIR*pole_pairs-1)){
							index -= (SAMPLES_PER_PPAIR*pole_pairs);
						}
						moving_avg += p_error_arr[index];
					}
					moving_avg = moving_avg/window;
					int lut_index = raw_offset + i;
					if(lut_index > (OFFSET_LUT_NUM-1)){
						lut_index -= OFFSET_LUT_NUM;
					}
					UsrConfig.offset_lut[lut_index] = moving_avg - encoder_offset;
				}
				
				// CAN report
				for(int i=0; i<OFFSET_LUT_NUM; i++){
					int_to_data(UsrConfig.offset_lut[i], data);
//					CAN_report_calibration(i+10, data);
//					SYSTICK_delay_ms(10);
					my_delay_ms(10);
				}
				
//				UsrConfig.calib_valid = true;
//				FSM_input(CMD_MENU);
			}
			break;
#endif
		case CS_ERROR:
			printf("Calib error %d\n\r", mCalibError);
			{
				uint8_t data[4];
				int_to_data(mCalibError, data);
//				CAN_report_calibration(-1, data);
			}
			CALIBRATION_end();
			mCalibStep = CS_NULL;
			break;
		
		default:
			break;
	}
	
	loop_count ++;
	
	}
}
