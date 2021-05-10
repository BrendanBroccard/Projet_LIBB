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
#define OBSTACLE_TRIGGER 		50
#define SIDE_OBSTACLE_TRIGGER	5
#define CAPTEUR_IR_FRONTRIGHT 	0
#define CAPTEUR_IR_FRONTLEFT	7
#define CAPTEUR_IR_45DEGRIGHT	1
#define CAPTEUR_IR_45DEGLEFT	6
#define CAPTEUR_IR_RIGHT		2
#define CAPTEUR_IR_LEFT 		5
#define CAPTEUR_IR_RIGHTBACK	3
#define CAPTEUR_IR_BACKLEFT 	4
#define DODGE_OBSTACLE			350
#define TRESHOLD				600
#define SMALL_TURN				50

void initThreads(void);
void moveTowardsUp(void);
bool obstacle_detection(int capteur, int trigger);
bool frontObstacleAnalysis(void);
void dodge_right(void);
void dodge_left(void);
void dodge_frontright(void);
void dodge_frontleft(void);

#endif /* CONTROLE_H_ */
