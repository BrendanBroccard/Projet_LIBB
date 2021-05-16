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

#define MAX_SPEED 				200		//[step/s] Vitesse que nous avons d�termin� bonne pour le d�placement g�n�ral de notre robot
#define HALF_MAX_SPEED			100		//[step/s]
#define QUARTER_MAX_SPEED		50		//[step/s]
#define WIDE_TURN				180		//Nombre de steps pour faire un virage large de 90 degr�s dans notre utilisation pr�cise


/*
 * Cette fonction active les moteurs � une certaine vitesse pour que le robot tourne sur lui m�me vers la droite (sens horaire)
 *
 * Param�tre speed : La vitesse � laquelle les moteurs vont tourner
 */
void turn_right(int speed);

/*
 * Cette fonction active les moteurs � une certaine vitesse pour que le robot tourne sur lui m�me vers la gauche (sens anti-horaire)
 *
 * Param�tre speed : La vitesse � laquelle les moteurs vont tourner
 */
void turn_left(int speed);

/*
 * Cette fonction active les moteurs pour que le robot tourne sur lui m�me vers la droite (sens horaire) d'une certaine distance
 *
 * Param�tre distance : Distance particuli�re que le robot doit parcourir en tournant
 */
void turn_right_until(int distance);

/*
 * Cette fonction active les moteurs pour que le robot tourne sur lui m�me vers la gauche (sens anti-horaire) d'une certaine distance
 *
 * Param�tre distance : Distance particuli�re que le robot doit parcourir en tournant
 */
void turn_left_until(int distance);

/*
 * Cette fonction permet au robot de faire un virage large de 90 degr�s vers la droite
 */
void wide_turn_right(void);

/*
 * Cette fonction permet au robot de faire un virage large de 90 degr�s vers la gauche
 */
void wide_turn_left(void);

/*
 * Cette fonction active les moteurs du robot pour qu'il avance tout droit � la vitesse d�finie exp�rimentalement comme bonne pour notre projet
 */
void go_forward(void);

/*
 * Cette fonction arr�te les moteurs du robot
 */
void stop_motors(void);


#endif /* DEPLACEMENT_H_ */
