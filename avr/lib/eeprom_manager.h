#ifndef EEPROM_MANAGER_H
#define EEPROM_MANAGER_H

#include <avr/eeprom.h>
#include <stdint.h>
#include <inttypes.h>

#endif

//48 byte
struct motor_eeprom_s{

  uint8_t in1;
  uint8_t in2;
  uint8_t in3;
  uint8_t in4;
  uint8_t enA;
  uint8_t enB;

}motor_eeprom;

//16 byte
struct joystick_eeprom_s{

  uint8_t pinX;
  uint8_t pinY;

}joystick_eeprom;

//96 byte
struct pid_eeprom_s{

  float kp;
  float kd;
  float ki;

}pid_eeprom_s;

void clean_eeprom(void);

void add_motor_to_eeprom(uint8_t in1,uint8_t in2,uint8_t in3,uint8_t in4,uint8_t enA,uint8_t enB);
void add_joystick_to_eeprom(uint8_t pinX,uint8_t pinY);
void add_pid_to_eeprom(float kp,float ki, float kd);
void get_motor_pins_from_eeprom(struct motor_eeprom_s* dest_motor);
void get_joystick_pins_from_eeprom(struct joystick_eeprom_s* dest_joystick);
void get_pid_values_from_eeprom(struct pid_eeprom_s* dest_pid);
