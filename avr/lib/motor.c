#include "motor.h"

int getSpeed(int jVal){

  if(jVal>524){
    return abs((jVal-524)*255/499);
  }
  else if(jVal<524){
    return abs(jVal*255/524);
  }

  return 0;

}

void setupMotor(uint8_t enA, uint8_t enB, uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4){

  set_mode(enA, OUTPUT);
  set_mode(enB, OUTPUT);
  set_mode(in1, OUTPUT);
  set_mode(in2, OUTPUT);
  set_mode(in3, OUTPUT);
  set_mode(in4, OUTPUT);


  //spegni i motori
  set_level(in1,LOW);
  set_level(in2,LOW);
  set_level(in3,LOW);
  set_level(in4,LOW);


}

void setupSpeed(uint8_t enAorB, uint8_t speed){

  set_pwm(enAorB, speed);

}

void setupDirection(uint8_t direction,uint8_t in1, uint8_t in2 ){

  switch (direction) {
    case FORWARD:
        set_level(in1, LOW);
        set_level(in2, HIGH);
        break;
    case BACKWARD:
        set_level(in1, HIGH);
        set_level(in2,LOW);
        break;
    case STATIONARY:
        set_level(in1, LOW);
        set_level(in2,LOW);
        break;

    }
}
