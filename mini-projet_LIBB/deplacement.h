/*
 * deplacement.h
 *
 *  Created on: 4 mai 2021
 *      Author: Brendan and Laetitia
 */

#ifndef DEPLACEMENT_H_
#define DEPLACEMENT_H_

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_SPEED 				200		//Vitesse que nous avons déterminé bonne pour le déplacement général de notre robot
#define HALF_MAX_SPEED			100
#define QUARTER_MAX_SPEED		50
#define WIDE_TURN				200		//Nombre de steps pour faire un virage large de 90 degrés dans notre utilisation précise



void turn_right(int speed);
void turn_left(int speed);
void turn_right_until(int distance);
void turn_left_until(int distance);
void wide_turn_right(void);
void wide_turn_left(void);
void go_forward(void);
void stop_motors(void);


#endif /* DEPLACEMENT_H_ */
