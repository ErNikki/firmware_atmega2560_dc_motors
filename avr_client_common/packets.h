#ifndef PACKETS_H
#define PACKETS_H

#include <stdint.h>

/*
Definisci qua le strutture per lo scambio di pacchetti tra avr e il pc
*/
#define MAX_DATA 27

struct data {
  uint32_t data_size;
  uint8_t data_type;
  uint8_t data[MAX_DATA];
}data;

// AVR -> PC

#define MOTORS 1
struct motors_packet{
    uint8_t in1;
    uint8_t in2;
    uint8_t in3;
    uint8_t in4;
    uint8_t enA;
    uint8_t enB;
    uint8_t speedMotorL;
    uint8_t speedMotorR;
}motors_packet;

#define JOYSTICK 2
struct joystick_packet{
  uint8_t pinX;
  uint8_t pinY;
  uint8_t Xval1;
  uint8_t Xval2;
  uint8_t Yval1;
  uint8_t Yval2;
}joystick_packet;

#define ENCODERL 3
struct encoder_packetL{

  uint8_t encL1;
  uint8_t encL2;

  uint8_t encPrevL1;
  uint8_t encPrevL2;

  uint8_t encSpeedL1;
  uint8_t encSpeedL2;

  uint8_t error_speedL1;
  uint8_t error_speedL2;

}encoder_packetL;

#define ENCODERR 4
struct encoder_packetR{

  uint8_t encR1;
  uint8_t encR2;

  uint8_t encPrevR1;
  uint8_t encPrevR2;

  uint8_t encSpeedR1;
  uint8_t encSpeedR2;

  uint8_t error_speedR1;
  uint8_t error_speedR2;

}encoder_packetR;

#define PID 5
struct PID_packet{

  uint8_t kp1;
  uint8_t kp2;
  uint8_t kp3;
  uint8_t kp4;

  uint8_t ki1;
  uint8_t ki2;
  uint8_t ki3;
  uint8_t ki4;

  uint8_t kd1;
  uint8_t kd2;
  uint8_t kd3;
  uint8_t kd4;

}PID_packet;

#define GET_DATA 6
#define ACK_MOTORS 7
#define ACK_JOYSTICK 8
#define ACK_PID 9
#define CLEAN_EEPROM 10
#define INIT_EEPROM 11
#define ACK_CLEAN_EEPROM 12
#define ACK_INIT_EEPROM 13

#endif
