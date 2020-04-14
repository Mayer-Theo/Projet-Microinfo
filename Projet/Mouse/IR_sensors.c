#include <stdio.h>
#include <string.h>
#include "ch.h"
#include "hal.h"
#include "IR_sensors.h"
#include <main.h>
#include "sensors/proximity.h"
#include "motors.h"

#define DETECT_T_JUNCTION 400 //valeur expérimentale à vérifier-------------------------------------------------------------------------------------------

static THD_WORKING_AREA(proximity_scan_wa, 256);
static THD_FUNCTION(proximity_scan, arg){
	(void) arg;
	chRegSetThreadName(__FUNCTION__);

	uint8_t stop_loop = 0;
	systime_t time;

	messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
	proximity_msg_t prox_values;
	int16_t leftSpeed = 0, rightSpeed = 0;

	uint8_t rgb_state = 0, rgb_counter = 0;
	uint16_t melody_state = 0, melody_counter = 0;

	uint8_t IR_max_channel = 0;
	unsigned int IR_max_value = 0;
	unsigned int IR_temp_value=0;

	while(stop_loop == 0) {
		time = chVTGetSystemTime();

		messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));
		while(prox_values.delta[1]<DETECT_T_JUNCTION && prox_values.delta[6]<DETECT_T_JUNCTION){
			leftSpeed = MOTOR_SPEED_LIMIT - prox_values.delta[0]*2 - prox_values.delta[1];
			rightSpeed = MOTOR_SPEED_LIMIT - prox_values.delta[7]*2 - prox_values.delta[6];
			right_motor_set_speed(rightSpeed);
			left_motor_set_speed(leftSpeed);
		}



		chThdSleepUntilWindowed(time, time + MS2ST(100)); // Refresh @ 10 Hz.
	}
}

void proximity_check(void){
	chThdCreateStatic(proximity_scan_wa, sizeof(proximity_scan_wa), NORMALPRIO, proximity_scan, NULL);
}
