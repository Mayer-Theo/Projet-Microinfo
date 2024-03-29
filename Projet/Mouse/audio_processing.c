#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <audio/play_melody.h>
#include <fft.h>
#include <arm_math.h>

#include "control.h"

//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micFront_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micFront_output[FFT_SIZE];
static float micBack_output[FFT_SIZE];

#define MIN_VALUE_THRESHOLD	35000

#define MIN_FREQ	10	//we don't analyze before this index to not use resources for nothing
#define FREQ_180DEG	12	//188Hz
#define FREQ_PAUSE	14	//219Hz
#define FREQ_LEFT	16	//250Hz
#define FREQ_RIGHT	18	//281Hz
#define FREQ_PLAY	20	//312Hz
#define	FREQ_FASTER	22	//344Hz
#define FREQ_SLOWER	24	//375Hz
#define MAX_FREQ	30	//we don't analyze after this index to not use resources for nothing

#define FREQ_180DEG_L	(FREQ_180DEG-1)
#define FREQ_180DEG_H	(FREQ_180DEG+1)
#define FREQ_PAUSE_L	(FREQ_PAUSE-1)
#define FREQ_PAUSE_H	(FREQ_PAUSE+1)
#define FREQ_LEFT_L		(FREQ_LEFT-1)
#define FREQ_LEFT_H		(FREQ_LEFT+1)
#define FREQ_RIGHT_L	(FREQ_RIGHT-1)
#define FREQ_RIGHT_H	(FREQ_RIGHT+1)
#define FREQ_PLAY_L		(FREQ_PLAY-1)
#define FREQ_PLAY_H		(FREQ_PLAY+1)
#define FREQ_FASTER_L (FREQ_FASTER-1)
#define FREQ_FASTER_H (FREQ_FASTER+1)
#define FREQ_SLOWER_L (FREQ_SLOWER-1)
#define FREQ_SLOWER_H (FREQ_SLOWER+1)

#define TURN_LEFT			-1	//permet de modifier le sens de rotation des moteurs pour tourner dans le bon sens
#define TURN_RIGHT			1	//idem que pour TURN_LEFT
#define SLOW_SPEED			400
#define FAST_SPEED			800

static Commandstate status=WAIT_COMMAND;

static THD_WORKING_AREA(Mic_thread_wa, 1024);
static THD_FUNCTION(Mic_thread, arg){
	(void) arg;
	chRegSetThreadName(__FUNCTION__);
	systime_t time;

	while(1){
		mic_start(&processAudioData);
		chThdSleepUntilWindowed(time, time + MS2ST(100)); // Refresh @ 10 Hz.
	}

}

/*
*	Simple function used to detect the highest value in a buffer
*	and to execute a motor command depending on it
*/
void sound_remote(float* data){
	float max_norm = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index = -1;

	//search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(data[i] > max_norm){
			max_norm = data[i];
			max_norm_index = i;
		}
	}

	//Switch permettant d'effectuer les actions nécessaires selon la commande
	switch(status){
	case WAIT_COMMAND:
			if(max_norm_index >= FREQ_180DEG_L && max_norm_index <= FREQ_180DEG_H){
				status=COMMAND_180DEG;
			}
			else if(max_norm_index >= FREQ_PAUSE_L && max_norm_index <= FREQ_PAUSE_H){
				status=COMMAND_PAUSE;
			}
			else if(max_norm_index >= FREQ_LEFT_L && max_norm_index <= FREQ_LEFT_H){
				status=COMMAND_TURN_LEFT;
			}
			else if(max_norm_index >= FREQ_RIGHT_L && max_norm_index <= FREQ_RIGHT_H){
				status=COMMAND_TURN_RIGHT;
			}
			else if(max_norm_index >= FREQ_PLAY_L && max_norm_index <= FREQ_PLAY_H){
				status=COMMAND_PLAY;
			}
			else if(max_norm_index >= FREQ_FASTER_L && max_norm_index <= FREQ_FASTER_H){
				status=COMMAND_FASTER;
			}
			else if(max_norm_index >= FREQ_SLOWER_L && max_norm_index <= FREQ_SLOWER_H){
				status=COMMAND_SLOWER;
			}
			break;

	case COMMAND_180DEG:
		stopCurrentMelody();
		dead_end();
		set_pause(FALSE);
		status=WAIT_COMMAND;
		break;

	case COMMAND_PAUSE:
		stopCurrentMelody();
		set_pause(TRUE);
		status=WAIT_COMMAND;
		break;

	case COMMAND_TURN_LEFT:
		stopCurrentMelody();
		right_angle_turn(TURN_LEFT);
		set_pause(FALSE);
		status=WAIT_COMMAND;
		break;

	case COMMAND_TURN_RIGHT:
		stopCurrentMelody();
		right_angle_turn(TURN_RIGHT);
		set_pause(FALSE);
		status=WAIT_COMMAND;
		break;

	case COMMAND_PLAY:
		stopCurrentMelody();
		set_pause(FALSE);
		set_ignore_junction(TRUE);
		status=WAIT_COMMAND;
		break;

	case COMMAND_FASTER:
		stopCurrentMelody();
		change_speed(FAST_SPEED);
		status=WAIT_COMMAND;
		break;

	case COMMAND_SLOWER:
		stopCurrentMelody();
		change_speed(SLOW_SPEED);
		status=WAIT_COMMAND;
		break;
	}
}

/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
void processAudioData(int16_t *data, uint16_t num_samples){

	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/

	static uint16_t nb_samples = 0;
	static uint8_t mustSend = 0;

	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4){
		//construct an array of complex numbers. Put 0 to the imaginary part
		micRight_cmplx_input[nb_samples] = (float)data[i + MIC_RIGHT];
		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];
		micBack_cmplx_input[nb_samples] = (float)data[i + MIC_BACK];
		micFront_cmplx_input[nb_samples] = (float)data[i + MIC_FRONT];

		nb_samples++;

		micRight_cmplx_input[nb_samples] = 0;
		micLeft_cmplx_input[nb_samples] = 0;
		micBack_cmplx_input[nb_samples] = 0;
		micFront_cmplx_input[nb_samples] = 0;

		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE)){
		/*	FFT proccessing
		*
		*	This FFT function stores the results in the input buffer given.
		*	This is an "In Place" function. 
		*/

		doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
		doFFT_optimized(FFT_SIZE, micFront_cmplx_input);
		doFFT_optimized(FFT_SIZE, micBack_cmplx_input);

		/*	Magnitude processing
		*
		*	Computes the magnitude of the complex numbers and
		*	stores them in a buffer of FFT_SIZE because it only contains
		*	real numbers.
		*
		*/
		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);
		arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);

		//sends only one FFT result over 10 for 1 mic to not flood the computer
		//sends to UART3
		if(mustSend > 8){
			//signals to send the result to the computer
			chBSemSignal(&sendToComputer_sem);
			mustSend = 0;
		}
		nb_samples = 0;
		mustSend++;

		sound_remote(micLeft_output);
	}
}

//--------------------------------------------Nos Fonctions----------------------------------//
//Permet de lancer la thread audio
void init_sound_thread(void){
	chThdCreateStatic(Mic_thread_wa, sizeof(Mic_thread_wa), NORMALPRIO, Mic_thread, NULL);
}
