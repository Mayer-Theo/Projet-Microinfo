#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "audio/microphone.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "i2c_bus.h"
#include "leds.h"
#include "main.h"
#include "memory_protection.h"
#include <motors.h>
#include "sdio.h"
#include "spi_comm.h"
#include "usbcfg.h"
#include "uc_usage.h"
#include "sensors/proximity.h"
#include "audio/play_melody.h"
#include "audio/play_sound_file.h"
#include "audio/audio_thread.h"

//nos librairies
#include "control.h"

#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


static bool load_config(void)
{
    extern uint32_t _config_start;

    return config_load(&parameter_root, &_config_start);
}

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

int main(void)
{
    halInit();
    chSysInit();
    mpu_init();

    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    // Init the peripherals.
	clear_leds();
	set_body_led(0);
	set_front_led(0);
	usb_start();
	motors_init();
	proximity_start();
	spi_comm_start();
	VL53L0X_start();
	serial_start();
	sdio_start();
	playMelodyStart();
	playSoundFileStart();
	dac_start();

	//lancement du programme
	init_threads();

    /* Infinite loop. */
    while (1) {
        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
