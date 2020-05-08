#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H


#define FFT_SIZE 	1024

typedef enum {
	//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
	LEFT_CMPLX_INPUT = 0,
	RIGHT_CMPLX_INPUT,
	FRONT_CMPLX_INPUT,
	BACK_CMPLX_INPUT,
	//Arrays containing the computed magnitude of the complex numbers
	LEFT_OUTPUT,
	RIGHT_OUTPUT,
	FRONT_OUTPUT,
	BACK_OUTPUT
} BUFFER_NAME_t;

typedef enum{
	WAIT_COMMAND = 0,
	COMMAND_180DEG,
	COMMAND_PAUSE,
	COMMAND_PLAY,
	COMMAND_TURN_LEFT,
	COMMAND_TURN_RIGHT,
	COMMAND_FASTER,
	COMMAND_SLOWER,
} Commandstate;

void processAudioData(int16_t *data, uint16_t num_samples);

void init_sound_thread(void);

#endif /* AUDIO_PROCESSING_H */
