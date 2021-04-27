#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <chprintf.h>
#include <messagebus.h>
#include <i2c_bus.h>
#include <imu.h>

void SendUint8ToComputer(uint8_t* data, uint16_t size) 
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
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
    i2c_start();

    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    //starts the serial communication
    serial_start();

    //start the USB communication
    usb_start();

    //inits the motors
    motors_init();

    //inits the imu
    imu_start();

    /*
    //starts the camera
    dcmi_start();
	po8030_start();
	*/

	/*
	//stars the threads for the pi regulator and the processing of the image
	pi_regulator_start();
	process_image_start();

    Infinite loop.
    while (1) {
    	//waits 1 second
        chThdSleepMilliseconds(1000);
     */
    }
}

void moveTowardsUp(void){
	float treshold = 0.2;
	int16_t position;

	get_acc_all(position);
	for(int i=0, i < 3, i++){

	}
}


#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
