#include "joystick.h"

void setupJoystick(uint8_t pinX, uint8_t pinY){

  set_mode(pinX, INPUT);
  set_mode(pinX, INPUT);

}

void getJCoordinates(uint8_t pinX, uint8_t pinY,struct JCoordinates* coordinates){

  int x=analog_read(pinX);
  int y=analog_read(pinY);

  coordinates->x=x;
  coordinates->y=y;
}
