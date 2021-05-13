/*
 * controle.c
 *
 *  Created on: 4 mai 2021
 *      Author: Brendan
 */

//--------------------------------------------------------------------------------------------------------------------
//	controle.c d�finit les thread qui seront utilis�s et par cons�quent les fonctions qui vont contr�ler le robot
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

/* Ce thread va utiliser les valeurs mesur�es par l'acc�l�rom�tre de l'IMU et les capteurs IR pour contr�ler le robot */
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

    clear_leds();																	//Eteint les LEDs t�moins de l'initialisation

    while(1) {
    	time = chVTGetSystemTime();

    	lookingForDirection = true;
    	while(lookingForDirection) {												//Se focalise uniquement sur l'acc�l�rom�tre jusqu'� avoir la bonne direction � emprunter
    		messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));
    		moveTowardsUp();														//Cherche la direction du sommet de la pente
    		//chThdSleepMilliseconds(200);
    	}

    	if(!atTheTop) {																//Emp�che le robot d'entamer une esquive lorsqu'il est � l'arr�t sur un plat
    		messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));

    		//obstacleCheck();
    		if(obstacle_detection(CAPTEUR_IR_FRONTRIGHT, OBSTACLE_TRIGGER)) {
    			dodge_left();
    		} else if(obstacle_detection(CAPTEUR_IR_FRONTLEFT, OBSTACLE_TRIGGER)) {
    			dodge_right();
    		} else if(obstacle_detection(CAPTEUR_IR_45DEGRIGHT, OBSTACLE_TRIGGER)) {
    			dodge_frontleft();
    		} else if(obstacle_detection(CAPTEUR_IR_45DEGLEFT, OBSTACLE_TRIGGER)) {
    			dodge_frontright();
    		}
    	}

    	chThdSleepUntilWindowed(time, time + MS2ST(100));							//Reset � une fr�quence de 10 Hz.
    }
}

/* Ce thread va utiliser les valeurs mesur�es par les capteurs pour d�tecter et esquiver les �ventuels obstacles
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

    	chThdSleepUntilWindowed(time, time + MS2ST(100));						//Reset � une fr�quence de 10 Hz.
    }
}
*/

void init_thread(void) {
	chThdCreateStatic(findDirectionThd_wa, sizeof(findDirectionThd_wa), NORMALPRIO+1, findDirectionThd, NULL);
}

void moveTowardsUp(void)  {

	int16_t acc_x = get_acc_filtered(X_AXIS, FILTER_SIZE);	//imu_values.acc_raw[0]-imu_values.acc_offset[0];
	int16_t acc_y = get_acc_filtered(Y_AXIS, FILTER_SIZE);	//imu_values.acc_raw[1]-imu_values.acc_offset[1];

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

bool obstacle_detection(sensor_ir_number capteur, int trigger) {
	bool obs = false;
	if (get_calibrated_prox(capteur) > trigger) {
		obs = true;
	}
	return obs;
}

bool frontObstacleAnalysis(void) {
	if(obstacle_detection(CAPTEUR_IR_FRONTRIGHT, OBSTACLE_TRIGGER) || obstacle_detection(CAPTEUR_IR_FRONTLEFT, OBSTACLE_TRIGGER)) {
	   //|| obstacle_detection(CAPTEUR_IR_45DEGRIGHT, OBSTACLE_TRIGGER) || obstacle_detection(CAPTEUR_IR_45DEGLEFT, OBSTACLE_TRIGGER)) {
		return true;
	} else {
		return false;
	}
}

void dodge_left(void) {
	turn_left_until(QUART_TOUR);
	go_forward();
	while(obstacle_detection(CAPTEUR_IR_RIGHT, SIDE_OBSTACLE_TRIGGER)) {
		if(frontObstacleAnalysis()) {
			turn_left_until(DEMI_TOUR);												//Fais demi-tour s'il recontre un obstacle lat�ral
		}
		if(obstacle_detection(CAPTEUR_IR_RIGHT, HIGH_OBSTACLE_TRIGGER)) {	//S'�carte s'il s'approche trop de l'obstacle en esquivant
			turn_left_until(SMALL_TURN);
		}
		if(obstacle_detection(CAPTEUR_IR_LEFT, HIGH_OBSTACLE_TRIGGER)) {	//S'�carte s'il s'approche trop de l'obstacle en esquivant
			turn_right_until(SMALL_TURN);
		}
	}
	right_motor_set_pos(RESET_VALUE);
	while(abs(right_motor_get_pos()) < DODGE_OBSTACLE)  {				//Avance encore un peu pour �viter de toucher le coin de l'obstacle
	}
	stop_motors();
	right_motor_set_pos(RESET_VALUE);
	//quart_de_tour_right();
}

void dodge_right(void) {
	turn_right_until(QUART_TOUR);
	go_forward();
	while(obstacle_detection(CAPTEUR_IR_LEFT, SIDE_OBSTACLE_TRIGGER)) {
		if(frontObstacleAnalysis()) {
			turn_right_until(DEMI_TOUR);												//Fais demi-tour s'il recontre un obstacle lat�ral
		}
		if(obstacle_detection(CAPTEUR_IR_LEFT, HIGH_OBSTACLE_TRIGGER)) {	//S'�carte s'il s'approche trop de l'obstacle en esquivant
			turn_right_until(SMALL_TURN);
		}
		if(obstacle_detection(CAPTEUR_IR_RIGHT, HIGH_OBSTACLE_TRIGGER)) {	//S'�carte s'il s'approche trop de l'obstacle en esquivant
			turn_left_until(SMALL_TURN);
		}
	}
	right_motor_set_pos(RESET_VALUE);
	while(abs(right_motor_get_pos()) < DODGE_OBSTACLE)  {				//Avance encore un peu pour �viter de toucher le coin de l'obstacle
	}
	stop_motors();
	right_motor_set_pos(RESET_VALUE);
	//quart_de_tour_left();
}

void dodge_frontleft(void) {
	turn_left_until(HUITIEME_TOUR);
	go_forward();
	while(obstacle_detection(CAPTEUR_IR_RIGHT, SIDE_OBSTACLE_TRIGGER)) {
		if(frontObstacleAnalysis()) {
			turn_left_until(DEMI_TOUR);												//Fais demi-tour s'il recontre un obstacle lat�ral
		}
		if(obstacle_detection(CAPTEUR_IR_RIGHT, HIGH_OBSTACLE_TRIGGER)) {	//S'�carte s'il s'approche trop de l'obstacle en esquivant
			turn_left_until(SMALL_TURN);
		}
		if(obstacle_detection(CAPTEUR_IR_LEFT, HIGH_OBSTACLE_TRIGGER)) {	//S'�carte s'il s'approche trop de l'obstacle en esquivant
			turn_right_until(SMALL_TURN);
		}
	}
	right_motor_set_pos(RESET_VALUE);
	while(abs(right_motor_get_pos()) < DODGE_OBSTACLE)  {		//Avance encore un peu pour �viter de toucher le coin de l'obstacle
	}
	stop_motors();
	//huitieme_de_tour_right();
}

void dodge_frontright(void) {
	turn_right_until(HUITIEME_TOUR);
	go_forward();
	while(obstacle_detection(CAPTEUR_IR_LEFT, SIDE_OBSTACLE_TRIGGER)) {
		if(frontObstacleAnalysis()) {
			turn_right_until(DEMI_TOUR);												//Fais demi-tour s'il recontre un obstacle lat�ral
		}
		if(obstacle_detection(CAPTEUR_IR_RIGHT, HIGH_OBSTACLE_TRIGGER)) {	//S'�carte s'il s'approche trop de l'obstacle en esquivant
			turn_left_until(SMALL_TURN);
		}
		if(obstacle_detection(CAPTEUR_IR_LEFT, HIGH_OBSTACLE_TRIGGER)) {	//S'�carte s'il s'approche trop de l'obstacle en esquivant
			turn_right_until(SMALL_TURN);
		}
	}
	right_motor_set_pos(RESET_VALUE);
	while(abs(right_motor_get_pos()) < DODGE_OBSTACLE)  {		//Avance encore un peu pour �viter de toucher le coin de l'obstacle
	}
	stop_motors();
	right_motor_set_pos(RESET_VALUE);
	//huitieme_de_tour_left();
}
