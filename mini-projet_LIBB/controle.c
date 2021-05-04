/*
 * controle.c
 *
 *  Created on: 4 mai 2021
 *      Author: Brendan
 */

//--------------------------------------------------------------------------------------------------------------------
//	controle.c définit les thread qui seront utilisés et par conséquent les fonctions qui vont contrôler le robot
//--------------------------------------------------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <chprintf.h>

#include "i2c_bus.h"
#include "motors.h"
#include "sensors/imu.h"
//#include "sensors/mpu9250.h"
#include "sensors/proximity.h"
#include "msgbus/messagebus.h"
#include "motors.h"
#include <main.h>
#include <deplacement.h>
#include <controle.h>

//Initier les Thread



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
