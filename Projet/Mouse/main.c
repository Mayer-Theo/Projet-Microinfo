#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "chprintf.h"
#include "hal.h"
#include "shell.h"

//#include "aseba_vm/aseba_node.h"
//#include "aseba_vm/skel_user.h"
//#include "aseba_vm/aseba_can_interface.h"
//#include "aseba_vm/aseba_bridge.h"
//#include "audio/audio_thread.h"
//#include "audio/play_melody.h"
//#include "audio/play_sound_file.h"
#include "audio/microphone.h"
//#include "camera/po8030.h"
//#include "epuck1x/Asercom.h"
//#include "epuck1x/Asercom2.h"
//#include "epuck1x/a_d/advance_ad_scan/e_acc.h"
//#include "sensors/battery_level.h"
#include "sensors/imu.h"
#include "sensors/mpu9250.h"
#include "sensors/VL53L0X/VL53L0X.h"
//#include "cmd.h"
//#include "config_flash_storage.h"
//#include "exti.h"
#include "i2c_bus.h"
#include "ir_remote.h"
#include "leds.h"
#include "main.h"
#include "memory_protection.h"
#include <motors.h>
#include "sdio.h"
#include "selector.h"
#include "spi_comm.h"
#include "usbcfg.h"
#include "communication.h"
#include "uc_usage.h"
#include "sensors/proximity.h"

#include "IR_sensors.h"
#include "audio_processing.h"
#include "fft.h"
#include "tof.h"

#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

//parameter_namespace_t parameter_root, aseba_ns;

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

//    parameter_namespace_declare(&parameter_root, NULL, NULL);

    // Init the peripherals.
	clear_leds();
	set_body_led(0);
	set_front_led(0);
	usb_start();
//	dcmi_start();
//	po8030_start();
	motors_init();
	proximity_start();
//	battery_level_start();
//	dac_start();
//	exti_start();
	imu_start();
	ir_remote_start();
	spi_comm_start();
	VL53L0X_start();
	serial_start();
	mic_start(NULL);
	sdio_start();
//	playMelodyStart();
//	playSoundFileStart();

    //proximity_check();
    //sound_check();
	tof_scan();

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
