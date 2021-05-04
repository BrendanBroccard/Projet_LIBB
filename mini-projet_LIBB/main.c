#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include "i2c_bus.h"
#include "motors.h"
#include "sensors/imu.h"
#include "sensors/proximity.h"
#include "msgbus/messagebus.h"
#include <deplacement.h>
#include <controle.h>

messagebus_t bus;
MUTEX_DECL(bus_lock); // @suppress("Field cannot be resolved")
CONDVAR_DECL(bus_condvar);

/*
static thread_t *sensorsUpdateThd;
static thread_t *robotControlThd;
*/

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

static void timer11_start(void){
    //General Purpose Timer configuration
    //timer 11 is a 16 bit timer so we can measure time
    //to about 65ms with a 1Mhz counter
    static const GPTConfig gpt11cfg = {
        1000000,        /* 1MHz timer clock in order to measure uS.*/
        NULL,           /* Timer callback.*/
        0,
        0
    };

    gptStart(&GPTD11, &gpt11cfg);
    //let the timer count to max value
    gptStartContinuous(&GPTD11, 0xFFFF);
}

int main(void)
{
    halInit();
    chSysInit();
    mpu_init();

    timer11_start();		//starts the timer 11
    serial_start();			//starts the serial communication
    usb_start();			//starts the USB communication
    motors_init();			//inits the motors
    imu_start();			//inits the imu and the i2c communication
    proximity_start();		//inits the proximity sensors

    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    chThdCreateStatic(sensorsUpdateThd_wa, sizeof(sensorsUpdateThd_wa), NORMALPRIO, sensorsUpdateThd, NULL);
    chThdCreateStatic(robotControlThd_wa, sizeof(robotControlThd_wa), NORMALPRIO, robotControlThd, NULL);

    while(1) {
        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
