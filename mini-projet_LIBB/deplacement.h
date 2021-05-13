/*
 * deplacement.h
 *
 *  Created on: 4 mai 2021
 *      Author: Brendan
 */

#ifndef DEPLACEMENT_H_
#define DEPLACEMENT_H_

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_SPEED 				200

void turn_right(int speed);
void turn_left(int speed);
void turn_right_until(int distance);
void turn_left_until(int distance);
/*
void quart_de_tour_right(void);
void quart_de_tour_left(void);
void huitieme_de_tour_right(void);
void huitieme_de_tour_left(void);
void demi_tour(void);
void small_turn_right(void);
void small_turn_left(void);
*/
void go_forward(void);
void stop_motors(void);


#endif /* DEPLACEMENT_H_ */
