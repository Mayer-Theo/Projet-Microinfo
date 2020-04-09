#include <stdio.h>
#include <string.h>
#include "ch.h"
#include "hal.h"
#include "IR_sensors.h"
#include <main.h>
#include "sensors/proximity.h"

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

		IR_max_channel=0;
		IR_temp_value=0;
		 IR_max_value = 0;

		for (uint8_t i = 0; i < PROXIMITY_NB_CHANNELS; i++) {
			IR_temp_value=get_prox(i);
			if(IR_temp_value>IR_max_value){
				IR_max_value = IR_temp_value;
				IR_max_channel=i+1;
			}
		}

	   if(IR_max_channel==1 || IR_max_channel==2){
				set_rgb_led(0, 10, 0, 0);
				set_rgb_led(1, 0, 0, 0);
				set_rgb_led(2, 0, 0, 0);
				set_rgb_led(3, 0, 0, 0);
	   }

	   if(IR_max_channel==3 || IR_max_channel==4){
				set_rgb_led(0, 0, 0, 0);
				set_rgb_led(1, 10, 0, 0);
				set_rgb_led(2, 0, 0, 0);
				set_rgb_led(3, 0, 0, 0);
	   }

	   if(IR_max_channel==5){
				set_rgb_led(0, 0, 0, 0);
				set_rgb_led(1, 0, 0, 0);
				set_rgb_led(2, 10, 0, 0);
				set_rgb_led(3, 0, 0, 0);
	   }

	   if(IR_max_channel==6 || IR_max_channel==7 || IR_max_channel==8){
				set_rgb_led(0, 0, 0, 0);
				set_rgb_led(1, 0, 0, 0);
				set_rgb_led(2, 0, 0, 0);
				set_rgb_led(3, 10, 0, 0);
	   }

	   /* simple avoidance function
		messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));
		leftSpeed = MOTOR_SPEED_LIMIT - prox_values.delta[0]*2 - prox_values.delta[1];
		rightSpeed = MOTOR_SPEED_LIMIT - prox_values.delta[7]*2 - prox_values.delta[6];
		right_motor_set_speed(rightSpeed);
		left_motor_set_speed(leftSpeed);
		*/

		chThdSleepUntilWindowed(time, time + MS2ST(100)); // Refresh @ 10 Hz.
	}
}

void proximity_check(void){
	chThdCreateStatic(proximity_scan_wa, sizeof(proximity_scan_wa), NORMALPRIO, proximity_scan, NULL);
}
