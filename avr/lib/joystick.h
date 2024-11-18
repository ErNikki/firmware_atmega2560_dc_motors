#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <avr/io.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "port.h"

struct JCoordinates{
  int x;
  int y;
};

void setupJoystick(uint8_t pinX, uint8_t pinY);
/*
imposta i pin x e y come OUTPUT
*/

void getJCoordinates(uint8_t pinX, uint8_t pinY, struct JCoordinates* coordinates);
/*
ritorna la posizione x e y del joystick
*/

#endif
