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
#include "sensors/proximity.h"
#include "msgbus/messagebus.h"
#include "motors.h"
#include <main.h>
#include <deplacement.h>
#include <controle.h>

static proximity_msg_t prox_values;
static imu_msg_t imu_values;

/* Ce thread va utiliser les variables globales, qui sont mises à jour par le second thread, pour dicter au robot ce qu'il doit faire */
static THD_WORKING_AREA(robotControlThd_wa, 2048);
static THD_FUNCTION(robotControlThd, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    systime_t time;

    while(1) {
    	time = chVTGetSystemTime();

    	moveTowardsUp();																// Trouve la direction du sommet

    	bool obstacle_right = false;
    	bool obstacle_left = false;
    	obstacle_right = obstacle_detection(CAPTEUR_IR_FRONTRIGHT, OBSTACLE);			// Détecte un éventuel obstacle
    	obstacle_left = obstacle_detection(CAPTEUR_IR_FRONTLEFT, OBSTACLE);
    	if(obstacle_right) {															// Esquive l'éventuel obstacle
    		dodge_left();
    		obstacle_right = false;
    	} else if(obstacle_left) {
    		dodge_right();
    		obstacle_left = false;
    	}
    	//chprintf((BaseSequentialStream *)&SDU1, "%4dEsquive obstacle OK, ", 0);

    	chThdSleepUntilWindowed(time, time + MS2ST(100));								//Reset à une fréquence de 10 Hz.
    }
}

/* Ce thread met à jour les valeurs des variables globales, qui concernent les valeurs mesurées par les capteurs IR et l'IMU */
static THD_WORKING_AREA(sensorsUpdateThd_wa, 2048);
static THD_FUNCTION(sensorsUpdateThd, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    systime_t time;

    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
    messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
    calibrate_ir();

    while(1) {
    	time = chVTGetSystemTime();

        calibrate_acc();
    	messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));
    	messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));

    	chThdSleepUntilWindowed(time, time + MS2ST(100));										//Reset à une fréquence de 10 Hz.
    }
}

void initThreads(void) {
	chThdCreateStatic(sensorsUpdateThd_wa, sizeof(sensorsUpdateThd_wa), NORMALPRIO, sensorsUpdateThd, NULL);
    chThdCreateStatic(robotControlThd_wa, sizeof(robotControlThd_wa), NORMALPRIO, robotControlThd, NULL);
}

void moveTowardsUp(void) {
	if((abs(imu_values.acc_offset[0]) > TRESHOLD) || (abs(imu_values.acc_offset[1]) > TRESHOLD)) {
		if(imu_values.acc_offset[0] > TRESHOLD) {
			turn_right(MAX_SPEED/2);
		} else if(imu_values.acc_offset[0] < -TRESHOLD) {
			turn_left(MAX_SPEED/2);
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
	if (get_calibrated_prox(capteur) > trigger) {
		obs = true;
	}

	return obs;
}

void dodge_left(void) {
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

void dodge_right(void) {
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
