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

static imu_msg_t imu_values;
static proximity_msg_t prox_values;
static bool lookingForDirection = true;
static bool atTheTop = false;

/* Ce thread va utiliser les valeurs mesurées par l'accéléromètre de l'IMU pour trouver la direction du sommet de la pente */
static THD_WORKING_AREA(findDirectionThd_wa, 2048);
static THD_FUNCTION(findDirectionThd, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    systime_t time;

    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
    calibrate_acc();
    messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
    calibrate_ir();

    while(1) {
    	time = chVTGetSystemTime();

    	messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));
    	moveTowardsUp();														// Trouve la direction du sommet de la pente

    	messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));

    	    		if(obstacle_detection(CAPTEUR_IR_FRONTRIGHT, OBSTACLE_TRIGGER)) {
    	    			dodge_left();
    	    			//chprintf((BaseSequentialStream *)&SDU1, "#DAB LEFT%4d# ", get_calibrated_prox(CAPTEUR_IR_FRONTRIGHT));
    	    		} else if(obstacle_detection(CAPTEUR_IR_FRONTLEFT, OBSTACLE_TRIGGER)) {
    	    			dodge_right();
    	    			//chprintf((BaseSequentialStream *)&SDU1, "#DAB RIGHT%4d# ", get_calibrated_prox(CAPTEUR_IR_FRONTLEFT));
    	    		} else if(obstacle_detection(CAPTEUR_IR_45DEGRIGHT, OBSTACLE_TRIGGER)) {
    	    			dodge_frontleft();
    	    			//chprintf((BaseSequentialStream *)&SDU1, "#DAB 45LEFT%4d# ", get_calibrated_prox(CAPTEUR_IR_FRONTLEFT));
    	    		} else if(obstacle_detection(CAPTEUR_IR_45DEGLEFT, OBSTACLE_TRIGGER)) {
    	    			dodge_frontright();
    	    			//chprintf((BaseSequentialStream *)&SDU1, "#DAB 45RIGHT%4d# ", get_calibrated_prox(CAPTEUR_IR_FRONTLEFT));
    	    		}

    	chThdSleepUntilWindowed(time, time + MS2ST(100));						//Reset à une fréquence de 10 Hz.
    }
}

/* Ce thread va utiliser les valeurs mesurées par les capteurs pour détecter et esquiver les éventuels obstacles */
static THD_WORKING_AREA(checkObstacleThd_wa, 2048);
static THD_FUNCTION(checkObstacleThd, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    systime_t time;

    messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
    calibrate_ir();

    while(1) {
    	time = chVTGetSystemTime();

    	//if(!atTheTop) {
    		messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));

    		if(obstacle_detection(CAPTEUR_IR_FRONTRIGHT, OBSTACLE_TRIGGER)) {
    			dodge_left();
    			//chprintf((BaseSequentialStream *)&SDU1, "#DAB LEFT%4d# ", get_calibrated_prox(CAPTEUR_IR_FRONTRIGHT));
    		} else if(obstacle_detection(CAPTEUR_IR_FRONTLEFT, OBSTACLE_TRIGGER)) {
    			dodge_right();
    			//chprintf((BaseSequentialStream *)&SDU1, "#DAB RIGHT%4d# ", get_calibrated_prox(CAPTEUR_IR_FRONTLEFT));
    		} else if(obstacle_detection(CAPTEUR_IR_45DEGRIGHT, OBSTACLE_TRIGGER)) {
    			dodge_frontleft();
    			//chprintf((BaseSequentialStream *)&SDU1, "#DAB 45LEFT%4d# ", get_calibrated_prox(CAPTEUR_IR_FRONTLEFT));
    		} else if(obstacle_detection(CAPTEUR_IR_45DEGLEFT, OBSTACLE_TRIGGER)) {
    			dodge_frontright();
    			//chprintf((BaseSequentialStream *)&SDU1, "#DAB 45RIGHT%4d# ", get_calibrated_prox(CAPTEUR_IR_FRONTLEFT));
    		}

    	//}

    	chThdSleepUntilWindowed(time, time + MS2ST(100));						//Reset à une fréquence de 10 Hz.
    }
}

void initThreads(void) {
	chThdCreateStatic(findDirectionThd_wa, sizeof(findDirectionThd_wa), NORMALPRIO, findDirectionThd, NULL);
    //chThdCreateStatic(checkObstacleThd_wa, sizeof(checkObstacleThd_wa), NORMALPRIO, checkObstacleThd, NULL);
}

void moveTowardsUp(void) {
	//chprintf((BaseSequentialStream *)&SDU1, "acc_offset:%4d ", imu_values.acc_offset[0]);
	//chprintf((BaseSequentialStream *)&SDU1, "acceleration:%4d ", imu_values.acceleration[0]);
	//chprintf((BaseSequentialStream *)&SDU1, "acc_raw:%4d ", imu_values.acc_raw[0]);

	int16_t acc_x = imu_values.acc_raw[0];//-imu_values.acc_offset[0];
	int16_t acc_y = imu_values.acc_raw[1];//-imu_values.acc_offset[1];

	//chprintf((BaseSequentialStream *)&SDU1, "|MOVE%4d |", acc_x);

	if((abs(acc_x) > TRESHOLD) || (abs(acc_y) > TRESHOLD)) {
		atTheTop = false;
		if(acc_x > TRESHOLD) {
			turn_right(MAX_SPEED/2);
		} else if(acc_x < -TRESHOLD) {
			turn_left(MAX_SPEED/2);
		} else if(acc_y > TRESHOLD) {
			lookingForDirection = false;
			go_forward();
		} else if(acc_y < -TRESHOLD) {
			demi_tour();
		}
	} else {
		atTheTop = true;
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

bool frontObstacleAnalysis(void) {
	if(obstacle_detection(CAPTEUR_IR_FRONTRIGHT, OBSTACLE_TRIGGER) || obstacle_detection(CAPTEUR_IR_FRONTLEFT, OBSTACLE_TRIGGER) ||
	   obstacle_detection(CAPTEUR_IR_45DEGRIGHT, OBSTACLE_TRIGGER) || obstacle_detection(CAPTEUR_IR_45DEGLEFT, OBSTACLE_TRIGGER)) {
		return true;
	} else {
		return false;
	}
}

void dodge_left(void) {
	quart_de_tour_left();
	go_forward();
	while(obstacle_detection(CAPTEUR_IR_RIGHT, SIDE_OBSTACLE_TRIGGER)) {
		if(frontObstacleAnalysis()) {
			demi_tour();												//Fais demi-tour s'il recontre un obstacle latéral
		}
		if(obstacle_detection(CAPTEUR_IR_RIGHT, OBSTACLE_TRIGGER*3)) {	//S'écarte s'il s'approche trop de l'obstacle en esquivant
			small_turn_left();
		}
		if(obstacle_detection(CAPTEUR_IR_LEFT, OBSTACLE_TRIGGER*3)) {	//S'écarte s'il s'approche trop de l'obstacle en esquivant
			small_turn_right();
		}
	}
	right_motor_set_pos(RESET_VALUE);
	while(abs(right_motor_get_pos()) < DODGE_OBSTACLE)  {				//Avance encore un peu pour éviter de toucher le coin de l'obstacle
	}
	stop_motors();
	right_motor_set_pos(RESET_VALUE);
	quart_de_tour_right();
}

void dodge_right(void) {
	quart_de_tour_right();
	go_forward();
	while(obstacle_detection(CAPTEUR_IR_LEFT, SIDE_OBSTACLE_TRIGGER)) {
		if(frontObstacleAnalysis()) {
			demi_tour();												//Fais demi-tour s'il recontre un obstacle latéral
		}
		if(obstacle_detection(CAPTEUR_IR_LEFT, OBSTACLE_TRIGGER*3)) {	//S'écarte s'il s'approche trop de l'obstacle en esquivant
			small_turn_right();
		}
		if(obstacle_detection(CAPTEUR_IR_RIGHT, OBSTACLE_TRIGGER*3)) {	//S'écarte s'il s'approche trop de l'obstacle en esquivant
			small_turn_left();
		}
	}
	right_motor_set_pos(RESET_VALUE);
	while(abs(right_motor_get_pos()) < DODGE_OBSTACLE)  {				//Avance encore un peu pour éviter de toucher le coin de l'obstacle
	}
	stop_motors();
	right_motor_set_pos(RESET_VALUE);
	quart_de_tour_left();
}

void dodge_frontleft(void) {
	huitieme_de_tour_left();
	go_forward();
	while(obstacle_detection(CAPTEUR_IR_RIGHT, SIDE_OBSTACLE_TRIGGER)) {
		if(frontObstacleAnalysis()) {
			demi_tour();												//Fais demi-tour s'il recontre un obstacle latéral
		}
		if(obstacle_detection(CAPTEUR_IR_RIGHT, OBSTACLE_TRIGGER*3)) {	//S'écarte s'il s'approche trop de l'obstacle en esquivant
			small_turn_left();
		}
		if(obstacle_detection(CAPTEUR_IR_LEFT, OBSTACLE_TRIGGER*3)) {	//S'écarte s'il s'approche trop de l'obstacle en esquivant
			small_turn_right();
		}
	}
	right_motor_set_pos(RESET_VALUE);
	while(abs(right_motor_get_pos()) < DODGE_OBSTACLE*sqrt(2))  {		//Avance encore un peu pour éviter de toucher le coin de l'obstacle
	}
	stop_motors();
	huitieme_de_tour_right();
}

void dodge_frontright(void) {
	huitieme_de_tour_right();
	go_forward();
	while(obstacle_detection(CAPTEUR_IR_LEFT, SIDE_OBSTACLE_TRIGGER)) {
		if(frontObstacleAnalysis()) {
			demi_tour();												//Fais demi-tour s'il recontre un obstacle latéral
		}
		if(obstacle_detection(CAPTEUR_IR_RIGHT, OBSTACLE_TRIGGER*3)) {	//S'écarte s'il s'approche trop de l'obstacle en esquivant
			small_turn_left();
		}
		if(obstacle_detection(CAPTEUR_IR_LEFT, OBSTACLE_TRIGGER*3)) {	//S'écarte s'il s'approche trop de l'obstacle en esquivant
			small_turn_right();
		}
	}
	right_motor_set_pos(RESET_VALUE);
	while(abs(right_motor_get_pos()) < DODGE_OBSTACLE*sqrt(2))  {		//Avance encore un peu pour éviter de toucher le coin de l'obstacle
	}
	stop_motors();
	right_motor_set_pos(RESET_VALUE);
	huitieme_de_tour_left();
}
