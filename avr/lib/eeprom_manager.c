#include "eeprom_manager.h"

uint16_t addr_motor_s=0;
const uint8_t offset_for_motor_s=48;

uint16_t addr_joystick_s=48;
const uint8_t offset_for_joystick_s=16;

uint16_t addr_pid_s=64;
const uint8_t offset_for_pid_s=96;




void clean_eeprom(void){

  unsigned char clean[160]={0};
  eeprom_write_block(clean,(void*)0,160);

}

void add_motor_to_eeprom(uint8_t in1,uint8_t in2,uint8_t in3,uint8_t in4,uint8_t enA,uint8_t enB){
  struct motor_eeprom_s new;
  new.in1=in1;
  new.in2=in2;
  new.in3=in3;
  new.in4=in4;
  new.enA=enA;
  new.enB=enB;
  eeprom_update_block(&new, (void *)addr_motor_s, sizeof(struct motor_eeprom_s));
}

void add_joystick_to_eeprom(uint8_t pinX,uint8_t pinY){
  struct joystick_eeprom_s new;
  new.pinX=pinX;
  new.pinY=pinY;
  eeprom_update_block(&new, (void *)addr_joystick_s, sizeof(struct joystick_eeprom_s));
}

void add_pid_to_eeprom(float kp,float ki, float kd){
  struct pid_eeprom_s new;
  new.kd=kd;
  new.ki=ki;
  new.kp=kp;
  eeprom_update_block(&new, (void *)addr_pid_s, sizeof(struct pid_eeprom_s));
}


void get_motor_pins_from_eeprom(struct motor_eeprom_s* dest_motor){

  eeprom_read_block(dest_motor, (void*) addr_motor_s,sizeof(struct motor_eeprom_s ));


}

void get_joystick_pins_from_eeprom(struct joystick_eeprom_s* dest_joystick){

  eeprom_read_block(dest_joystick, (void*) addr_joystick_s,sizeof(struct joystick_eeprom_s ));

}

void get_pid_values_from_eeprom(struct pid_eeprom_s* dest_pid){

  eeprom_read_block(dest_pid, (void*) addr_pid_s,sizeof(struct pid_eeprom_s ));

}
