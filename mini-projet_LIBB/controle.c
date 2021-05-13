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
#include "leds.h"
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
static bool dodgingRightObstacle = false;
static bool dodgingLeftObstacle = false;

/* Ce thread va utiliser les valeurs mesurées par l'accéléromètre de l'IMU et les capteurs IR pour contrôler le robot */
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

    chThdSleepMilliseconds(1000);													//Laisse une sseconde de marge de stabilisation

    clear_leds();																	//Eteint les LEDs témoins de l'initialisation

    while(1) {
    	time = chVTGetSystemTime();

    	if(!dodgingRightObstacle && !dodgingLeftObstacle) {
    		lookingForDirection = true;
    		while(lookingForDirection) {											//Se focalise uniquement sur l'accéléromètre jusqu'à avoir la bonne direction à emprunter
    			messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));
    			move_towards_up();													//Cherche la direction du sommet de la pente
    		}
    	}

    	if(!atTheTop) {																//Empêche le robot d'entamer une esquive lorsqu'il est à l'arrêt sur un plat
    		messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));
    		obstacle_check();
    	}

    	chThdSleepUntilWindowed(time, time + MS2ST(100));							//Reset à une fréquence de 10 Hz.
    }
}

void init_thread(void) {
	chThdCreateStatic(findDirectionThd_wa, sizeof(findDirectionThd_wa), NORMALPRIO+1, findDirectionThd, NULL);
}

void move_towards_up(void)  {

	int16_t acc_x = get_acc_filtered(X_AXIS, FILTER_SIZE);
	int16_t acc_y = get_acc_filtered(Y_AXIS, FILTER_SIZE);

	//chprintf((BaseSequentialStream *)&SDU1, "%4d", acc_x);

	if((abs(acc_x) > TRESHOLD) || (abs(acc_y) > TRESHOLD)) {
		atTheTop = false;
		set_body_led(0);
		if(acc_x > TRESHOLD) {
			turn_right(MAX_SPEED/2);
		} else if(acc_x < -TRESHOLD) {
			turn_left(MAX_SPEED/2);
		} else if(acc_y > TRESHOLD) {
			lookingForDirection = false;
			go_forward();
		} else if(acc_y < -TRESHOLD) {
			turn_right_until(DEMI_TOUR);
		}
	} else {
		atTheTop = true;
		set_body_led(1);
		stop_motors();
	}
}

void obstacle_check(void) {
	if(!dodgingRightObstacle && !dodgingLeftObstacle) {
		if(obstacle_detection(CAPTEUR_IR_FRONTRIGHT, OBSTACLE_TRIGGER)) {
			dodgingRightObstacle = true;
			set_led(LED1, ON);
			set_led(LED3, ON);
			turn_left_until(QUART_TOUR);
			go_forward();
		} else if(obstacle_detection(CAPTEUR_IR_FRONTLEFT, OBSTACLE_TRIGGER)) {
			dodgingLeftObstacle = true;
			set_led(LED1, ON);
			set_led(LED7, ON);
			turn_right_until(QUART_TOUR);
			go_forward();
		} else if(obstacle_detection(CAPTEUR_IR_45DEGRIGHT, HIGH_OBSTACLE_TRIGGER)) {
			dodgingRightObstacle = true;
			set_led(LED3, ON);
			turn_left_until(HUITIEME_TOUR);
			go_forward();
		} else if(obstacle_detection(CAPTEUR_IR_45DEGLEFT, HIGH_OBSTACLE_TRIGGER)) {
			dodgingLeftObstacle = true;
			set_led(LED7, ON);
			turn_right_until(HUITIEME_TOUR);
			go_forward();
		} else {
			dodge_sidewall();														//S'écarte s'il s'approche trop d'un obstacle latéral
		}

	} else if(dodgingRightObstacle) {
		if(obstacle_detection(CAPTEUR_IR_RIGHT, SIDE_OBSTACLE_TRIGGER)) {
			if(front_obstacle_analysis()) {
				turn_left_until(DEMI_TOUR);											//Fait demi-tour s'il recontre un obstacle durant l'esquive
			}
			dodge_sidewall();														//S'écarte s'il s'approche trop de l'obstacle en esquivant
		} else {
			dodge_obstacle_edge();													//Avance encore un peu pour éviter de toucher le coin de l'obstacle
			dodgingRightObstacle = false;
			clear_leds();
		}
	} else if(dodgingLeftObstacle) {
		if(obstacle_detection(CAPTEUR_IR_LEFT, SIDE_OBSTACLE_TRIGGER)) {
			if(front_obstacle_analysis()) {
				turn_right_until(DEMI_TOUR);										//Fait demi-tour s'il recontre un obstacle durant l'esquive
			}
			dodge_sidewall();														//S'écarte s'il s'approche trop de l'obstacle en esquivant
		} else {
			dodge_obstacle_edge();													//Avance encore un peu pour éviter de toucher le coin de l'obstacle
			dodgingLeftObstacle = false;
			clear_leds();
		}
	}
}

bool obstacle_detection(sensor_ir_number capteur, int trigger) {
	bool obs = false;
	if (get_calibrated_prox(capteur) > trigger) {
		obs = true;
	}
	return obs;
}

bool front_obstacle_analysis(void) {
	if(obstacle_detection(CAPTEUR_IR_FRONTRIGHT, OBSTACLE_TRIGGER) || obstacle_detection(CAPTEUR_IR_FRONTLEFT, OBSTACLE_TRIGGER)
	   || obstacle_detection(CAPTEUR_IR_45DEGRIGHT, HIGH_OBSTACLE_TRIGGER) || obstacle_detection(CAPTEUR_IR_45DEGLEFT, HIGH_OBSTACLE_TRIGGER)	) {
		return true;
	} else {
		return false;
	}
}

void dodge_sidewall(void) {
	if(obstacle_detection(CAPTEUR_IR_RIGHT, HIGH_OBSTACLE_TRIGGER)) {
		turn_left_until(SMALL_TURN);
	}
	if(obstacle_detection(CAPTEUR_IR_LEFT, HIGH_OBSTACLE_TRIGGER)) {
		turn_right_until(SMALL_TURN);
	}
}

void dodge_obstacle_edge(void) {
	go_forward();
	right_motor_set_pos(RESET_VALUE);
	while(abs(right_motor_get_pos()) < DODGE_OBSTACLE)  {
	}
	stop_motors();
	right_motor_set_pos(RESET_VALUE);
}
