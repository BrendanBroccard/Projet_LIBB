/*
 * deplacement.c
 *
 *  Created on: 4 mai 2021
 *      Author: Brendan
 */

//--------------------------------------------------------------------------------------------------------------------
//	deplacement.c contient toutes les fonctions particulières permettant le déplacement du robot
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

void quart_de_tour_right(void) {
	right_motor_set_pos(RESET_VALUE);
	turn_right(MAX_SPEED);
	while(abs(right_motor_get_pos()) < QUART_TOUR)  {
	}
	left_motor_set_speed(RESET_VALUE);
	right_motor_set_speed(RESET_VALUE);
	right_motor_set_pos(RESET_VALUE);
}

void quart_de_tour_left(void) {
	left_motor_set_pos(RESET_VALUE);
	turn_left(MAX_SPEED);
	while(abs(left_motor_get_pos()) < QUART_TOUR)  {
	}
	left_motor_set_speed(RESET_VALUE);
	right_motor_set_speed(RESET_VALUE);
	left_motor_set_pos(RESET_VALUE);
}

void huitieme_de_tour_right(void) {
	right_motor_set_pos(RESET_VALUE);
	turn_right(MAX_SPEED);
	while(abs(right_motor_get_pos()) < QUART_TOUR/2)  {
	}
	left_motor_set_speed(RESET_VALUE);
	right_motor_set_speed(RESET_VALUE);
	right_motor_set_pos(RESET_VALUE);
}

void huitieme_de_tour_left(void) {
	left_motor_set_pos(RESET_VALUE);
	turn_left(MAX_SPEED);
	while(abs(left_motor_get_pos()) < QUART_TOUR/2)  {
	}
	left_motor_set_speed(RESET_VALUE);
	right_motor_set_speed(RESET_VALUE);
	left_motor_set_pos(RESET_VALUE);
}

void demi_tour(void) {
	right_motor_set_pos(RESET_VALUE);
	turn_right(MAX_SPEED);
	while(abs(right_motor_get_pos()) < 2*QUART_TOUR)  {
	}
	left_motor_set_speed(RESET_VALUE);
	right_motor_set_speed(RESET_VALUE);
	right_motor_set_pos(RESET_VALUE);
}

void small_turn_right(void) {
	right_motor_set_pos(RESET_VALUE);
	turn_right(MAX_SPEED);
	while(abs(right_motor_get_pos()) < SMALL_TURN)  {
	}
	left_motor_set_speed(RESET_VALUE);
	right_motor_set_speed(RESET_VALUE);
	right_motor_set_pos(RESET_VALUE);
}

void small_turn_left(void) {
	left_motor_set_pos(RESET_VALUE);
	turn_left(MAX_SPEED);
	while(abs(left_motor_get_pos()) < SMALL_TURN)  {
	}
	left_motor_set_speed(RESET_VALUE);
	right_motor_set_speed(RESET_VALUE);
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
