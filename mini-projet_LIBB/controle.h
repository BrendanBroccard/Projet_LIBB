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

typedef enum {							//Les num�ros des diff�rents capteurs IR
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

#define OBSTACLE_TRIGGER 		50		//Valeur compar�e aux valeurs mesur�es par les capteurs de proximit� pour d�celer la pr�sence d'un obstacle
#define HIGH_OBSTACLE_TRIGGER	1000	//Valeur de comparaison beaucoup plus grande qui permet simplement d'�viter un choc imminent impr�vu
#define SIDE_OBSTACLE_TRIGGER	10		//Valeur de comparaison plus faible utilis�e pour determiner lorsqu'un obstacle a �t� compl�tement �vit�

#define TRESHOLD				500		//Le treshold choisi auquel on compare les valeurs d'acc�l�ration mesur�es par l'IMU
#define X_AXIS					0
#define Y_AXIS					1
#define FILTER_SIZE				50		//Nombre de mesures dont on calcule la moyenne pour obtenir l'acc�l�ration mesur�e

#define SMALL_TURN				50		//Nombre de steps que l'on a d�fini pour que le robot s'�carte d'une paroi lat�rale
#define QUART_TOUR				320		//Nombre de steps que prend le robot pour faire un quart de tour
#define DEMI_TOUR				640		//Nombre de steps que prend le robot pour faire un demi-tour
#define HUITIEME_TOUR			160		//Nombre de steps que prend le robot pour faire un huiti�me de tour


/*
 * Cette fonction permet d'initier le thread robotControlThd depuis le main.c
 */
void init_thread(void);

/*
 * Cette fonction fait tourner le robot jusqu'� ce que sa direction pointe vers le sommet de la pente
 */
void move_towards_up(void);

/*
 * Cette fonction utilise les mesures des capteurs IR pour d�celer la pr�sence d'un obstacle et le cas �ch�ant, met le robot en �tat d'esquive
 */
void obstacle_check(void);

/*
 * Cette fonction utilise les mesures des capteurs IR pour permettre au robot d'esquiver un obstacle pr�c�demment d�t�ct�
 */
void dodge_obstacle(void);

/*
 * Cette fonction compare la mesure la plus r�cente d'un capteur IR � une valeur de trigger d�fini
 *
 * Param�tre capteur : Le num�ro du capteur dont on souhaite comparer la mesure
 * Param�tre trigger : La valeur de trigger � laquelle on veut comparer la mesure du capteur IR
 *
 * Renvoie : Le bool�en true si la mesure du capteur est plus grande que le trigger, le bool�en false sinon
 */
bool obstacle_detection(sensor_ir_number capteur, int trigger);

/*
 * Cette fontion analyse l'�tat des 4 capteurs IR avant pour savoir si un obstacle se trouve face au robot
 * Renvoie : Le bool�en true si au moins un des 4 capteurs IR avant d�tecte un obstacle, le bool�en false sinon
 */
bool front_obstacle_analysis(void);

/*
 * Cette fonction utilise les valeurs mesur�es par les capteurs IR lat�raux pour permettre au robot de s'�carter l�g�rement s'il est trop proche d'un obstacle lat�ral
 */
void dodge_sidewall(void);


#endif /* CONTROLE_H_ */
