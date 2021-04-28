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

    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    //starts the timer 11
    timer11_start();

    //starts the serial communication
    serial_start();

    //starts the USB communication
    usb_start();

    //starts the i2c communication
    i2c_start();

    //inits the motors
    motors_init();

    //inits the imu
    imu_start();

    while(1) {

    }

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

    } */
}

void moveTowardsUp(void){
	float treshold = 0.2;
	float acc_x;
	float acc_y;
	acc_x = get_acceleration(0);
	acc_y = get_acceleration(1);
	bool acc_x_pos;
	bool acc_x_neg;
	bool acc_y_pos;
	bool acc_y_neg;

	if(acc_x > treshold) {
		acc_x_pos = true;
	} else if(acc_x < -treshold) {
		acc_x_neg = true;
	}

	if(acc_y > treshold) {
		acc_y_pos = true;
	} else if(acc_y < -treshold) {
		acc_y_neg = true;
	}

	if(acc_y_pos || acc_y_neg) {
		if(acc_x_pos) {
			//Tourne à droite
		} else if(acc_x_neg) {
			//Tourne à gauche
		} else if(acc_y_pos) {
			//Va tout droit
		} else if(acc_y_neg) {
			//Demi-tour
		}
	} else {
		//Arrête tout
	}
}


#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
