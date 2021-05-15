/*
 * main.c
 *
 *      Author: Brendan and Laetitia
 */
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

#include "motors.h"
#include "leds.h"
#include "sensors/imu.h"
#include "sensors/proximity.h"
#include "msgbus/messagebus.h"
#include <deplacement.h>
#include <controle.h>

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

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

    //Allume des LEDs témoins de l'initialisation
    set_led(LED3, ON);
    set_led(LED7, ON);

    serial_start();									//Démarre la communication serial
    usb_start();									//Démarre la communication USB
    motors_init();									//Initie les moteurs
    imu_start();									//Initie l'imu and la communication i2c
    proximity_start();								//Initie les capteurs IR de proximité

    messagebus_init(&bus, &bus_lock, &bus_condvar);	//Initie le bus de communication

    init_thread();									//Initie le thread défini dans controle.c

    //Boucle infinie du main
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
