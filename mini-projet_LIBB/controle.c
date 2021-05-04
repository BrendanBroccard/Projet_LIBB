/*
 * controle.c
 *
 *  Created on: 4 mai 2021
 *      Author: Brendan
 */

//--------------------------------------------------------------------------------------------------------------------
//	controle.c définit les thread qui seront utilisés et par conséquent les fonctions qui vont contrôler le robot
//--------------------------------------------------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <chprintf.h>

#include "i2c_bus.h"
#include "motors.h"
#include "sensors/imu.h"
//#include "sensors/mpu9250.h"
#include "sensors/proximity.h"
#include "msgbus/messagebus.h"
#include "motors.h"
#include <main.h>
#include <deplacement.h>
#include <controle.h>
