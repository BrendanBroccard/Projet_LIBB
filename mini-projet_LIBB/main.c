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
#include "sensors/mpu9250.h"
#include "sensors/proximity.h"
#include "msgbus/messagebus.h"

messagebus_t bus;
MUTEX_DECL(bus_lock); // @suppress("Field cannot be resolved")
CONDVAR_DECL(bus_condvar);

/*
void SendUint8ToComputer(uint8_t* data, uint16_t size) 
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}
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
    	/*
    	go_forward();

    	obstacle_right = obstacle_detection(CAPTEUR_IR_FRONTRIGHT);
    	//obstacle_left = obstacle_detection(CAPTEUR_IR_FRONTLEFT);
    	if(obstacle_right) {
    		dodge_left();
    		obstacle_right = false;
    	}

    	if(obstacle_left) {

    	}
		*/
    	moveTowardsUp();
        chThdSleepMilliseconds(100);
    }
}

void moveTowardsUp(void) {

	messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
	imu_msg_t imu_values;

	float treshold = 0.2;
	float acc_x = 0;
	float acc_y = 0;

	calibrate_acc();
	messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));
	acc_x = get_acceleration(0);
	acc_y = get_acceleration(1);
	bool acc_x_pos = false;
	bool acc_x_neg = false;
	bool acc_y_pos = false;
	bool acc_y_neg = false;

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

	if(acc_y_pos || acc_y_neg || acc_x_pos || acc_x_neg) {
		if(acc_x_pos) {
			turn_right();
		} else if(acc_x_neg) {
			turn_left();
		} else if(acc_y_pos) {
			go_forward();
		} else if(acc_y_neg) {
			demi_tour();
		}
	} else {
		stop_motors();
	}
}

bool obstacle_detection(int capteur) {
	bool obs = false;

	messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
	proximity_msg_t prox_values;
	//int16_t leftSpeed = 0, rightSpeed = 0;

	calibrate_ir();
	messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));

	if (get_calibrated_prox(capteur) > OBSTACLE) {
		obs = true;
	}

	return obs;
}

void dodge_left() {
	quart_de_tour_left();
	bool obs_right = true;
	while(obs_right) {
		obs_right = obstacle_detection(CAPTEUR_IR_RIGHT);
	}
	right_motor_set_pos(RESET_VALUE);
	go_forward();
	while(abs(right_motor_get_pos()) < DODGE_OBSTACLE)  {		//Avance un coup pour esquiver le coin de l'obstacle
	}
	stop_motors();
	right_motor_set_pos(RESET_VALUE);
	quart_de_tour_right();
}

void turn_right() {
	left_motor_set_speed(MAX_SPEED);
	right_motor_set_speed(- MAX_SPEED);
}

void turn_left() {
	left_motor_set_speed(- MAX_SPEED);
	right_motor_set_speed(MAX_SPEED);
}

void quart_de_tour_right(void) {
	right_motor_set_pos(RESET_VALUE);
	turn_right();
	while(abs(right_motor_get_pos()) < QUART_TOUR)  {
	}
	left_motor_set_speed(RESET_VALUE);
	right_motor_set_speed(RESET_VALUE);
	right_motor_set_pos(RESET_VALUE);
}

void quart_de_tour_left(void) {
	left_motor_set_pos(RESET_VALUE);
	turn_left();
	while(abs(left_motor_get_pos()) < QUART_TOUR)  {
		left_motor_set_speed(- MAX_SPEED);
		right_motor_set_speed(MAX_SPEED);
	}
	left_motor_set_speed(RESET_VALUE);
	right_motor_set_speed(RESET_VALUE);
	left_motor_set_pos(RESET_VALUE);
}

void demi_tour(void) {
	right_motor_set_pos(RESET_VALUE);
	turn_right();
	while(abs(right_motor_get_pos()) < 2*QUART_TOUR)  {
	}
	left_motor_set_speed(RESET_VALUE);
	right_motor_set_speed(RESET_VALUE);
	right_motor_set_pos(RESET_VALUE);
}

void go_forward(void) {
	left_motor_set_speed(MAX_SPEED);
	right_motor_set_speed(MAX_SPEED);
}

void stop_motors(void) {
	left_motor_set_speed(RESET_VALUE);
	right_motor_set_speed(RESET_VALUE);
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
