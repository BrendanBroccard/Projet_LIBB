/*
 * deplacement.c
 *
 *  Created on: 4 mai 2021
 *      Author: Brendan and Laetitia
 */

//------------------------------------------------------------------------------------------------------------------------
//	deplacement.c contient toutes les fonctions spécifiques au déplacement du robot, liées à l'utilisation des moteurs
//------------------------------------------------------------------------------------------------------------------------

#include "motors.h"
#include <main.h>
#include <deplacement.h>
#include <controle.h>

void turn_right(int speed) {
	left_motor_set_speed(speed);
	right_motor_set_speed(-speed);
}

void turn_left(int speed) {
	left_motor_set_speed(-speed);
	right_motor_set_speed(speed);
}

void turn_right_until(int distance) {
	right_motor_set_pos(RESET_VALUE);
	turn_right(MAX_SPEED);
	while(abs(right_motor_get_pos()) < distance)  {
	}
	stop_motors();
	right_motor_set_pos(RESET_VALUE);
}

void turn_left_until(int distance) {
	left_motor_set_pos(RESET_VALUE);
	turn_left(MAX_SPEED);
	while(abs(left_motor_get_pos()) < distance)  {
	}
	stop_motors();
	left_motor_set_pos(RESET_VALUE);
}

void wide_turn_right(void) {
	left_motor_set_speed(MAX_SPEED);
	right_motor_set_speed(QUARTER_MAX_SPEED);
	right_motor_set_pos(RESET_VALUE);
	while(abs(right_motor_get_pos()) < WIDE_TURN)  {
	}
	right_motor_set_pos(RESET_VALUE);
}

void wide_turn_left(void) {
	right_motor_set_speed(MAX_SPEED);
	left_motor_set_speed(QUARTER_MAX_SPEED);
	left_motor_set_pos(RESET_VALUE);
	while(abs(left_motor_get_pos()) < WIDE_TURN)  {
	}
	left_motor_set_pos(RESET_VALUE);
}

void go_forward(void) {
	left_motor_set_speed(MAX_SPEED);
	right_motor_set_speed(MAX_SPEED);
}

void stop_motors(void) {
	left_motor_set_speed(RESET_VALUE);
	right_motor_set_speed(RESET_VALUE);
}
