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
#define ON						1
#define OFF						0
#define RGB_MAX					100		//Intensité que l'on souhaite donner aux LED RGB

typedef enum {							//Les numéros des différents capteurs IR
	CAPTEUR_IR_FRONTRIGHT,
	CAPTEUR_IR_45DEGRIGHT,
	CAPTEUR_IR_RIGHT,
	CAPTEUR_IR_RIGHTBACK,
	CAPTEUR_IR_LEFTBACK,
	CAPTEUR_IR_LEFT,
	CAPTEUR_IR_45DEGLEFT,
	CAPTEUR_IR_FRONTLEFT,
	NB_CAPTEURS_IR,
} sensor_ir_number;

#define OBSTACLE_TRIGGER 		50
#define HIGH_OBSTACLE_TRIGGER	150
#define SIDE_OBSTACLE_TRIGGER	5
#define DODGE_OBSTACLE			350

#define TRESHOLD				600		//Le treshold choisi auquel on compare les valeurs d'accélérations mesurées par l'IMU
#define X_AXIS					0
#define Y_AXIS					1
#define FILTER_SIZE				50		//Nombre de mesures dont on calcule la moyenne pour obtenir l'accélération mesurée

#define SMALL_TURN				50		//Nombre de step que l'on a défini pour que le robot s'écarte d'une paroi latérale
#define QUART_TOUR				320		//Nombre de step que prend le robot pour faire un quart de tour
#define DEMI_TOUR				640		//Nombre de step que prend le robot pour faire un demi-tour
#define HUITIEME_TOUR			160		//Nombre de step que prend le robot pour faire un hutième de tour

void init_thread(void);
void moveTowardsUp(void);
bool obstacle_detection(sensor_ir_number capteur, int trigger);
bool frontObstacleAnalysis(void);
void dodge_right(void);
void dodge_left(void);
void dodge_frontright(void);
void dodge_frontleft(void);

#endif /* CONTROLE_H_ */
