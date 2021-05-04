/*
 * controle.h
 *
 *  Created on: 4 mai 2021
 *      Author: Brendan
 */

#ifndef CONTROLE_H_
#define CONTROLE_H_

#ifdef __cplusplus
extern "C" {
#endif

#define RESET_VALUE 			0
#define OBSTACLE 				50
#define SIDE_OBSTACLE			5
#define CAPTEUR_IR_FRONTRIGHT 	0
#define CAPTEUR_IR_FRONTLEFT	7
#define CAPTEUR_IR_RIGHT		2
#define CAPTEUR_IR_LEFT 		5
#define CAPTEUR_IR_RIGHTBACK	3
#define CAPTEUR_IR_BACKLEFT 	4
#define DODGE_OBSTACLE			400
#define TRESHOLD				1000

void moveTowardsUp(void);
bool obstacle_detection(int capteur, int trigger);
void dodge_right();
void dodge_left();

#endif /* CONTROLE_H_ */
