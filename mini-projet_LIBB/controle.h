/*
 * controle.h
 *
 *  Created on: 4 mai 2021
 *      Author: Brendan and Laetitia
 */

#ifndef CONTROLE_H_
#define CONTROLE_H_

#ifdef __cplusplus
extern "C" {
#endif

#define RESET_VALUE 			0
#define ON						1
#define OFF						0

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

#define OBSTACLE_TRIGGER 		50		//Valeur comparée aux valeurs mesurées par les capteurs de proximité pour déceler la présence d'un obstacle
#define HIGH_OBSTACLE_TRIGGER	1000	//Valeur de comparaison beaucoup plus grande qui permet simplement d'éviter un choc imminent imprévu
#define SIDE_OBSTACLE_TRIGGER	10		//Valeur de comparaison plus faible utilisée pour determiner lorsqu'un obstacle a été complètement évité

#define TRESHOLD				500		//Le treshold choisi auquel on compare les valeurs d'accélération mesurées par l'IMU
#define X_AXIS					0
#define Y_AXIS					1
#define FILTER_SIZE				50		//Nombre de mesures dont on calcule la moyenne pour obtenir l'accélération mesurée

#define SMALL_TURN				50		//Nombre de steps que l'on a défini pour que le robot s'écarte d'une paroi latérale
#define QUART_TOUR				320		//Nombre de steps que prend le robot pour faire un quart de tour
#define DEMI_TOUR				640		//Nombre de steps que prend le robot pour faire un demi-tour
#define HUITIEME_TOUR			160		//Nombre de steps que prend le robot pour faire un huitième de tour


/*
 * Cette fonction permet d'initier le thread robotControlThd depuis le main.c
 */
void init_thread(void);

/*
 * Cette fonction fait tourner le robot jusqu'à ce que sa direction pointe vers le sommet de la pente
 */
void move_towards_up(void);

/*
 * Cette fonction utilise les mesures des capteurs IR pour déceler la présence d'un obstacle et le cas échéant, met le robot en état d'esquive
 */
void obstacle_check(void);

/*
 * Cette fonction utilise les mesures des capteurs IR pour permettre au robot d'esquiver un obstacle précédemment détécté
 */
void dodge_obstacle(void);

/*
 * Cette fonction compare la mesure la plus récente d'un capteur IR à une valeur de trigger défini
 *
 * Paramètre capteur : Le numéro du capteur dont on souhaite comparer la mesure
 * Paramètre trigger : La valeur de trigger à laquelle on veut comparer la mesure du capteur IR
 *
 * Renvoie : Le booléen true si la mesure du capteur est plus grande que le trigger, le booléen false sinon
 */
bool obstacle_detection(sensor_ir_number capteur, int trigger);

/*
 * Cette fontion analyse l'état des 4 capteurs IR avant pour savoir si un obstacle se trouve face au robot
 * Renvoie : Le booléen true si au moins un des 4 capteurs IR avant détecte un obstacle, le booléen false sinon
 */
bool front_obstacle_analysis(void);

/*
 * Cette fonction utilise les valeurs mesurées par les capteurs IR latéraux pour permettre au robot de s'écarter légèrement s'il est trop proche d'un obstacle latéral
 */
void dodge_sidewall(void);


#endif /* CONTROLE_H_ */
