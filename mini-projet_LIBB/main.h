#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


//constants for the differents parts of the project
#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			40
#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			2 
#define PXTOCM					1570.0f //experimental value
#define GOAL_DISTANCE 			10.0f
#define MAX_DISTANCE 			25.0f
#define ERROR_THRESHOLD			0.1f	//[cm] because of the noise of the camera
#define KP						800.0f
#define KI 						3.5f	//must not be zero
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)

#define RESET_VALUE 			0
#define MAX_SPEED 				100
#define OBSTACLE 				50
#define SIDE_OBSTACLE			5
#define QUART_TOUR  			320 //nombre de step qu'il prend pour faire un quart de tour
#define CAPTEUR_IR_FRONTRIGHT 	0
#define CAPTEUR_IR_FRONTLEFT	7
#define CAPTEUR_IR_RIGHT		2
#define CAPTEUR_IR_LEFT 		5
#define CAPTEUR_IR_RIGHTBACK	3
#define CAPTEUR_IR_BACKLEFT 	4
#define DODGE_OBSTACLE			400
#define TRESHOLD				1000


/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

//void SendUint8ToComputer(uint8_t* data, uint16_t size);
//void imu_compute_offset(messagebus_topic_t * imu_topic, uint16_t nb_samples);
void moveTowardsUp(void);
bool obstacle_detection(int capteur, int trigger);
void dodge_right();
void dodge_left();
void turn_right(void);
void turn_left(void);
void quart_de_tour_right(void);
void quart_de_tour_left(void);
void demi_tour(void);
void go_forward(void);
void stop_motors(void);

#ifdef __cplusplus
}
#endif

#endif
