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

#define MAX_SPEED 				200		//[step/s] Vitesse que nous avons déterminé bonne pour le déplacement général de notre robot
#define HALF_MAX_SPEED			100		//[step/s]
#define QUARTER_MAX_SPEED		50		//[step/s]
#define WIDE_TURN				180		//Nombre de steps pour faire un virage large de 90 degrés dans notre utilisation précise


/*
 * Cette fonction active les moteurs à une certaine vitesse pour que le robot tourne sur lui même vers la droite (sens horaire)
 *
 * Paramètre speed : La vitesse à laquelle les moteurs vont tourner
 */
void turn_right(int speed);

/*
 * Cette fonction active les moteurs à une certaine vitesse pour que le robot tourne sur lui même vers la gauche (sens anti-horaire)
 *
 * Paramètre speed : La vitesse à laquelle les moteurs vont tourner
 */
void turn_left(int speed);

/*
 * Cette fonction active les moteurs pour que le robot tourne sur lui même vers la droite (sens horaire) d'une certaine distance
 *
 * Paramètre distance : Distance particulière que le robot doit parcourir en tournant
 */
void turn_right_until(int distance);

/*
 * Cette fonction active les moteurs pour que le robot tourne sur lui même vers la gauche (sens anti-horaire) d'une certaine distance
 *
 * Paramètre distance : Distance particulière que le robot doit parcourir en tournant
 */
void turn_left_until(int distance);

/*
 * Cette fonction permet au robot de faire un virage large de 90 degrés vers la droite
 */
void wide_turn_right(void);

/*
 * Cette fonction permet au robot de faire un virage large de 90 degrés vers la gauche
 */
void wide_turn_left(void);

/*
 * Cette fonction active les moteurs du robot pour qu'il avance tout droit à la vitesse définie expérimentalement comme bonne pour notre projet
 */
void go_forward(void);

/*
 * Cette fonction arrête les moteurs du robot
 */
void stop_motors(void);


#endif /* DEPLACEMENT_H_ */
