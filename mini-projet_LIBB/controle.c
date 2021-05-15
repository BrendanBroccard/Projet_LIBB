/*
 * controle.c
 *
 *  Created on: 4 mai 2021
 *      Author: Brendan and Laetitia
 */

//-------------------------------------------------------------------------------------------------------------------------
//	controle.c définit le thread principal qui sera utilisé et par conséquent les fonctions qui vont contrôler le robot
//-------------------------------------------------------------------------------------------------------------------------

#include "leds.h"
#include "sensors/imu.h"
#include "sensors/proximity.h"
#include "msgbus/messagebus.h"
#include <main.h>
#include <deplacement.h>
#include <controle.h>


static imu_msg_t imu_values;														//Tableau qui contient les mesures de l'IMU
static proximity_msg_t prox_values;													//Tableau qui contient les mesures des capteurs IR
static bool lookingForDirection = true;												//Booléen qui indique si le robot est en train de chercher la bonne direction
static bool atTheTop = false;														//Booléen qui indique si le robot est sur un plat
static bool dodgingRightObstacle = false;											//Booléen qui indique si le robot est en train d'esquiver un obstacle sur sa droite
static bool dodgingLeftObstacle = false;											//Booléen qui indique si le robot est en train d'esquiver un obstacle sur sa gauche


/* Ce thread va utiliser les valeurs mesurées par l'accéléromètre de l'IMU et les capteurs IR pour contrôler le robot */
static THD_WORKING_AREA(robotControlThd_wa, 2048);
static THD_FUNCTION(robotControlThd, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    systime_t time;
    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
    calibrate_acc();
    messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
    calibrate_ir();

    chThdSleepMilliseconds(1000);													//Laisse une seconde de marge de stabilisation

    clear_leds();																	//Eteint les LEDs témoins de l'initialisation

    while(1) {
    	time = chVTGetSystemTime();

    	if(!dodgingRightObstacle && !dodgingLeftObstacle) {
    		lookingForDirection = true;
    		while(lookingForDirection) {											//Se focalise uniquement sur l'accéléromètre jusqu'à avoir la bonne direction
    			messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));
    			move_towards_up();													//Se tourne jusqu'à être dans la direction du sommet de la pente
    		}
    	}

    	if(!atTheTop) {																//Empêche le robot d'entamer une esquive lorsqu'il est à l'arrêt sur un plat
    		messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));
    		obstacle_check();
    		while(dodgingRightObstacle || dodgingLeftObstacle) {
    			chThdSetPriority(NORMALPRIO);										//Diminue la priorité du thread principal pour permettre aux capteurs IR d'être ajournés plus régulièrement
    			messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));
    			dodge_obstacle();
    		}
			chThdSetPriority(NORMALPRIO+1);

    	}

    	chThdSleepUntilWindowed(time, time + MS2ST(100));							//Reset à une fréquence de 10 Hz.
    }
}

void init_thread(void) {															//Est appelée dans le main et initialise le thread de contrôle du robot
	chThdCreateStatic(robotControlThd_wa, sizeof(robotControlThd_wa), NORMALPRIO+1, robotControlThd, NULL);
}

void move_towards_up(void)  {

	int16_t acc_x = get_acc_filtered(X_AXIS, FILTER_SIZE);							//Initialise la valeur moyenne des 50 dernières mesures de l'accéléromètre dans l'axe des X
	int16_t acc_y = get_acc_filtered(Y_AXIS, FILTER_SIZE);							//Initialise la valeur moyenne des 50 dernières mesures de l'accéléromètre dans l'axe des Y

	if((abs(acc_x) > TRESHOLD) || (abs(acc_y) > TRESHOLD)) {
		atTheTop = false;
		set_body_led(OFF);
		set_front_led(ON);
		if(acc_x > TRESHOLD) {
			turn_right(HALF_MAX_SPEED);
		} else if(acc_x < -TRESHOLD) {
			turn_left(HALF_MAX_SPEED);
		} else if(acc_y > TRESHOLD) {
			lookingForDirection = false;
			set_front_led(OFF);
			go_forward();
		} else if(acc_y < -TRESHOLD) {
			turn_right(HALF_MAX_SPEED);
		}
	} else {
		atTheTop = true;
		stop_motors();																//Le robot s'arrête d'une fois qu'il arrivé sur un plat
		set_body_led(ON);
		set_front_led(OFF);
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
	}
}

void dodge_obstacle(void) {
	if(dodgingRightObstacle) {
		if(obstacle_detection(CAPTEUR_IR_RIGHT, SIDE_OBSTACLE_TRIGGER)) {
			if(front_obstacle_analysis()) {
				set_led(LED3, OFF);
				set_led(LED7, ON);
				dodgingRightObstacle = false;
				dodgingLeftObstacle = true;
				turn_left_until(DEMI_TOUR);											//Fait demi-tour s'il recontre un obstacle durant l'esquive
				go_forward();
			}
			dodge_sidewall();														//S'écarte s'il s'approche trop de l'obstacle en l'esquivant
		} else {
			wide_turn_right();														//Prend un virage large pour éviter de toucher le coin de l'obstacle
			dodgingRightObstacle = false;
			clear_leds();
		}
	} else if(dodgingLeftObstacle) {
		if(obstacle_detection(CAPTEUR_IR_LEFT, SIDE_OBSTACLE_TRIGGER)) {
			if(front_obstacle_analysis()) {
				set_led(LED7, OFF);
				set_led(LED3, ON);
				dodgingLeftObstacle = false;
				dodgingRightObstacle = true;
				turn_right_until(DEMI_TOUR);										//Fait demi-tour s'il recontre un obstacle durant l'esquive
				go_forward();
			}
			dodge_sidewall();														//S'écarte s'il s'approche trop de l'obstacle en l'esquivant
		} else {
			wide_turn_left();														//Prend un virage large pour éviter de toucher le coin de l'obstacle
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
		go_forward();
	}
	if(obstacle_detection(CAPTEUR_IR_LEFT, HIGH_OBSTACLE_TRIGGER)) {
		turn_right_until(SMALL_TURN);
		go_forward();
	}
}
