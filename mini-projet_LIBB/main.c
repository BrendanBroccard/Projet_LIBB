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
//#include "sensors/mpu9250.h"
#include "sensors/proximity.h"
#include "msgbus/messagebus.h"
#include <deplacement.h>
#include <controle.h>

messagebus_t bus;
MUTEX_DECL(bus_lock); // @suppress("Field cannot be resolved")
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


//Initier les THREAD

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

    bool obstacle_left = false;
    bool obstacle_right = false;

    while(1) {

    	moveTowardsUp();

    	obstacle_right = obstacle_detection(CAPTEUR_IR_FRONTRIGHT, OBSTACLE);
    	obstacle_left = obstacle_detection(CAPTEUR_IR_FRONTLEFT, OBSTACLE);
    	if(obstacle_right) {
    		dodge_left();
    		obstacle_right = false;
    	} else if(obstacle_left) {
    		dodge_right();
    		obstacle_left = false;
    	}

        //chThdSleepMilliseconds(100);
    }
}

void moveTowardsUp(void) {

	messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
	imu_msg_t imu_values;

	calibrate_acc();
	messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

	//chprintf((BaseSequentialStream *)&SDU1, "acc_x : %4d, ", acc_x);

	if((abs(imu_values.acc_offset[0]) > TRESHOLD) || (abs(imu_values.acc_offset[1]) > TRESHOLD)) {
		if(imu_values.acc_offset[0] > TRESHOLD) {
			turn_right(MAX_SPEED/3);
		} else if(imu_values.acc_offset[0] < -TRESHOLD) {
			turn_left(MAX_SPEED/3);
		} else if(imu_values.acc_offset[1] > TRESHOLD) {
			go_forward();
		} else if(imu_values.acc_offset[1] < -TRESHOLD) {
			demi_tour();
		}
	} else {
		stop_motors();
	}
}

bool obstacle_detection(int capteur, int trigger) {
	bool obs = false;

	messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
	proximity_msg_t prox_values;
	//int16_t leftSpeed = 0, rightSpeed = 0;

	calibrate_ir();
	messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));

	if (get_calibrated_prox(capteur) > trigger) {
		obs = true;
	}

	return obs;
}

void dodge_left() {
	quart_de_tour_left();
	bool obs_right = true;
	go_forward();
	while(obs_right) {
		obs_right = obstacle_detection(CAPTEUR_IR_RIGHT, SIDE_OBSTACLE);
	}
	right_motor_set_pos(RESET_VALUE);
	while(abs(right_motor_get_pos()) < DODGE_OBSTACLE)  {		//Avance un dernier coup pour esquiver le coin de l'obstacle
	}
	stop_motors();
	right_motor_set_pos(RESET_VALUE);
	quart_de_tour_right();
}

void dodge_right() {
	quart_de_tour_right();
	bool obs_left = true;
	go_forward();
	while(obs_left) {
		obs_left = obstacle_detection(CAPTEUR_IR_LEFT, SIDE_OBSTACLE);
	}
	right_motor_set_pos(RESET_VALUE);
	while(abs(right_motor_get_pos()) < DODGE_OBSTACLE)  {		//Avance un dernier coup pour esquiver le coin de l'obstacle
	}
	stop_motors();
	right_motor_set_pos(RESET_VALUE);
	quart_de_tour_left();
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
