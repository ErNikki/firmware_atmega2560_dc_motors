
#include <stdint.h>
#include "lib/uart.h"
#include "lib/port.h"
#include "lib/motor.h"
#include "lib/joystick.h"
#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>
#include "../avr_client_common/packets.h"
#include "lib/encoder.h"
#include "lib/eeprom_manager.h"
//100
//200
//NOTA I VALORI CENTRALI DI X=523 OR 522 QUELLO DI Y=518 OR 517
// HO SCAMBIATO X E Y SUL JOYSTICK COSI Y=0 SI TROVA SUI PIN
//PORTB [ bit-07 bit-06 bit-05 bit-04 bit-03 bit-02 bit-01 bit-00]
//BOARD [ pin-13 pin-12 pin-11 pin-10 pin-50 pin-51 pin-52 pin-53]
//timer 5: connesso a pin-44-oc5c pin-46-oc5a pin-45-oc5b
//uso oc5c

//variabili motori per i test
//in1=42 in2=40 in3=38 in4=36 enA=7 enB=6
//variabili del joystick per i test
//pinX=A1 pinY=A0
//varibili PID
//KP=0.5 KI=0 KD=0.2
//promemoria
//invertire pinx e pinY




//COMUNICAZIONE
struct UART * uart;
struct data d;
struct motors_packet m_packet;
struct joystick_packet j_packet;
struct encoder_packetL e_packetL;
struct encoder_packetR e_packetR;
struct PID_packet p_packet;

struct data aux_d;
uint8_t set=0;
//MOTORI
uint8_t in1=0;
uint8_t in2=0;
uint8_t in3=0;
uint8_t in4=0;
uint8_t enA=0;
uint8_t enB=0;
int speedMotorL=0;//ena
int speedMotorR=0;//enb
uint8_t maxSpeed=255;

float kp=0;
float kd=0;
float ki=0;


//JOYSTICK
uint8_t pinX=0;
uint8_t pinY=0;
int x;
int y;

//ENCODER 1 MOTORE L PIN GIALLO 53 PIN VERDE 52
uint16_t encL=0;
uint16_t encPrevL=0;
int16_t encSpeedL=0;
//derivara d
float error_preL=0;
//integrale i
float error_sumL=0;
int16_t error_speedL=0;
int16_t pwm_pulseL=0;
//DEBUG PIN
uint8_t pin53=0;
uint8_t pin52=0;

//ENCDER 2 MOTORE R
uint16_t encR=0;
uint16_t encPrevR=0;
int16_t encSpeedR=0;
//derivara d
float error_preR=0;
//integrale i
float error_sumR=0;
int16_t error_speedR=0;
int16_t pwm_pulseR=0;
//DEBUG PIN
//da vedere  pin
uint8_t pin51=0;
uint8_t pin50=0;

static inline int16_t clamp(int16_t v, int16_t value){
  if (v>value)
    return value;
  if (v<-value)
    return -value;
  return v;
}

static inline int16_t clamp_pwm(int16_t pwm){

  if(pwm<0){
    pwm=-pwm;
  }
  if(pwm <-255 || pwm>255){
    pwm=255;
  }

  return pwm;

}

void init_from_eeprom(void){

  struct motor_eeprom_s motor_pins;
  struct joystick_eeprom_s joystick_pins;
  struct pid_eeprom_s pid_values;

  get_motor_pins_from_eeprom(&motor_pins);
  get_joystick_pins_from_eeprom(&joystick_pins);
  get_pid_values_from_eeprom(&pid_values);

  in1=motor_pins.in1;
  in2=motor_pins.in2;
  in3=motor_pins.in3;
  in4=motor_pins.in4;
  enA=motor_pins.enA;
  enB=motor_pins.enB;

  pinX=joystick_pins.pinX;
  pinY=joystick_pins.pinY;

  kd=pid_values.kd;
  kp=pid_values.kp;
  ki=pid_values.ki;




}



//non posso usare i pin 44 45 46 come pwm
/*
void timer5_init(void){

  TCCR5A=0;

  //prescaler at 1024
  TCCR5B|= 1<<CS52| 1<<CS50;
  TCCR5B&=  ~(1<<CS51);
  //clear timer on compare set_mode
  TCCR5B|=1<<WGM52;
  //100=timer_duration_ms
  uint16_t ocrVal=(uint16_t)(15.625*1);
  OCR5A=ocrVal;

  //clear int
  cli();
  //abilità timer int
  TIMSK5 |= 1<< OCIE5A;
  sei();
}

ISR(TIMER5_COMPA_vect){

  Encoder_sample();

  encPrevL=encL;
  encL=Encoder_getValue(0);
  //encR=Encoder_getValue(1);

  if(encL>=encPrevL){
    //lo divido per quatro perchè ritengo abbia 4 sensori e quindi faccio 4 cambi di stato a rotazione
    //se faccio tot cambi di stato e li divido per 4 ottento il numero di giri totale
    //RPM
    encSpeedL=(60*(encL-encPrevL)/1100)/(0.001);
  }
  else if(encL<encPrevL){
    //RPM
    encSpeedL=-((60*(encL-encPrevL)/1100)/(0.001));
  }

}
*/
void send_pins(void){

  m_packet.in1=in1;
  m_packet.in2=in2;
  m_packet.in3=in3;
  m_packet.in4=in4;
  m_packet.enA=enA;
  m_packet.enB=enB;
  m_packet.speedMotorL=speedMotorL;
  m_packet.speedMotorR=speedMotorR;

  UART_putData(uart, (uint8_t *)&m_packet, sizeof(struct motors_packet), MOTORS);

  //Encoder_sample();
  j_packet.pinX=pinX;//Encoder_getValue(0);
  j_packet.pinY=pinY;
  j_packet.Xval1=x;
  j_packet.Xval2=x>>8;
  j_packet.Yval1=y;
  j_packet.Yval2=y>>8;

  UART_putData(uart, (uint8_t *)&j_packet, sizeof(struct joystick_packet), JOYSTICK);

  e_packetL.encL1=encL;
  e_packetL.encL2=encL>>8;
  e_packetL.encPrevL1=encPrevL;
  e_packetL.encPrevL2=encPrevL>>8;
  e_packetL.encSpeedL1=encSpeedL;
  e_packetL.encSpeedL2=encSpeedL>>8;
  e_packetL.error_speedL1=error_speedL;
  e_packetL.error_speedL2=error_speedL>>8;

  UART_putData(uart, (uint8_t *)&e_packetL, sizeof(struct encoder_packetL), ENCODERL);

  e_packetR.encR1=encR;
  e_packetR.encR2=encR>>8;
  e_packetR.encPrevR1=encPrevR;
  e_packetR.encPrevR2=encPrevR>>8;
  e_packetR.encSpeedR1=encSpeedR;
  e_packetR.encSpeedR2=encSpeedR>>8;
  e_packetR.error_speedR1=error_speedR;
  e_packetR.error_speedR2=error_speedR>>8;

  UART_putData(uart, (uint8_t *)&e_packetR, sizeof(struct encoder_packetR), ENCODERR);

  p_packet.kp1=((unsigned char*)&kp)[0];
  p_packet.kp2=((unsigned char*)&kp)[1];
  p_packet.kp3=((unsigned char*)&kp)[2];
  p_packet.kp4=((unsigned char*)&kp)[3];

  p_packet.ki1=((unsigned char*)&ki)[0];
  p_packet.ki2=((unsigned char*)&ki)[1];
  p_packet.ki3=((unsigned char*)&ki)[2];
  p_packet.ki4=((unsigned char*)&ki)[3];

  p_packet.kd1=((unsigned char*)&kd)[0];
  p_packet.kd2=((unsigned char*)&kd)[1];
  p_packet.kd3=((unsigned char*)&kd)[2];
  p_packet.kd4=((unsigned char*)&kd)[3];

  UART_putData(uart, (uint8_t *)&p_packet, sizeof(struct PID_packet), PID);




}
//il problema è che il client si impalla se non riceve un qualsiasi ack
void update_pins(void){
  if(UART_getData(uart,(uint8_t*)&d,sizeof(struct data))==1){
    unsigned char* aux;

    switch (d.data_type) {
      case MOTORS:
      in1=d.data[0];
      in2=d.data[1];
      in3=d.data[2];
      in4=d.data[3];
      enA=d.data[4];
      enB=d.data[5];
      add_motor_to_eeprom(in1,in2,in3,in4,enA,enB);
      UART_putData(uart,NULL ,0,ACK_MOTORS);
      break;

      case JOYSTICK:
      pinX=d.data[0];
      pinY=d.data[1];
      add_joystick_to_eeprom(pinX,pinY);
      UART_putData(uart,NULL ,0,ACK_JOYSTICK);
      break;

      case PID:
      aux=((unsigned char*)&kp);
      aux[0]= d.data[0];
      aux[1]= d.data[1];
      aux[2]= d.data[2];
      aux[3]= d.data[3];

      aux=((unsigned char*)&ki);
      aux[0]= d.data[4];
      aux[1]= d.data[5];
      aux[2]= d.data[6];
      aux[3]= d.data[7];

      aux=((unsigned char*)&kd);
      aux[0]= d.data[8];
      aux[1]= d.data[9];
      aux[2]= d.data[10];
      aux[3]= d.data[11];

      add_pid_to_eeprom(kp,ki,kd);
      UART_putData(uart,NULL ,0,ACK_PID);
      break;

      case GET_DATA:
      send_pins();
      break;

      case CLEAN_EEPROM:
      clean_eeprom();
      UART_putData(uart,NULL ,0,ACK_CLEAN_EEPROM);
      break;

      case INIT_EEPROM:
      init_from_eeprom();
      UART_putData(uart,NULL ,0,ACK_INIT_EEPROM);


    }
  }

}
//aspetta finchè i pin non vengono settati dall'utente
void wait_for_setup_pins(void){

  while(in1==0 || in2==0 || in3==0 || in4==0 || enA==0 || enB==0 || pinX==0 || pinY==0){
    update_pins();
  }

}
//almeno 150 se no non gira

int16_t pid_control1(uint16_t des_Speed){

  float mypwm_pulse;
  float myerror_speed=clamp(des_Speed-encSpeedL,10);

  error_speedL=(int16_t)myerror_speed;


  error_sumL += error_speedL; //sum of error
  error_sumL= clamp(error_sumL,255);

  mypwm_pulse=myerror_speed*kp + error_sumL*ki + (myerror_speed - error_preL)*kd;
  error_preL = error_speedL;  //save last (previous) error
  return (int16_t) mypwm_pulse;
}

int16_t pid_control2(uint16_t des_Speed){

  float mypwm_pulse;
  float myerror_speed=clamp(des_Speed-encSpeedR,10);

  error_speedR=(int16_t)myerror_speed;


  error_sumR += error_speedR; //sum of error
  error_sumR= clamp(error_sumR,255);

  mypwm_pulse=myerror_speed*kp + error_sumR*ki + (myerror_speed - error_preR)*kd;
  error_preR = error_speedR;  //save last (previous) error
  return (int16_t) mypwm_pulse;
}

int main(void){

    uart=UART_init();
    Encoder_init();
    //timer5_init();
    init_from_eeprom();
    wait_for_setup_pins();


    struct JCoordinates coordinates;
    setupJoystick(pinX,pinY);


    setupMotor(enA,enB,in1,in2,in3,in4);


    while(1){


      //leggo i valori del joystick
      getJCoordinates(pinX,pinY,&coordinates);
      //lo faccio in modo tale che y=1024 stia davanti ai pin
      y=coordinates.x;
      x=coordinates.y;

      //Al centro del grafico
      if(y>500 && y<540 && x>500 && x<540){

        setupDirection(STATIONARY,in2,in1);
        setupDirection(STATIONARY,in3,in4);
        error_speedL=0;
        error_speedR=0;
        pwm_pulseL=0;
        pwm_pulseR=0;

      }

      //allora ci troviamo nel quadrante superiore
      else if (y>=524 && y<=1023){

        setupDirection(FORWARD,in2,in1);
        setupDirection(FORWARD,in3,in4);
        //in rpm

        Encoder_sample();

        encPrevL=encL;
        encL=Encoder_getValue(0);
        encSpeedL=encL-encPrevL;

        encPrevR=encR;
        encR=Encoder_getValue(1);
        encSpeedR=encR-encPrevR;



        if(y<723){

          pwm_pulseL+=pid_control1(4);
          pwm_pulseL=clamp_pwm(pwm_pulseL);

          pwm_pulseR+=pid_control2(4);
          pwm_pulseR=clamp_pwm(pwm_pulseR);

          speedMotorL=(uint8_t)pwm_pulseL;
          speedMotorR=(uint8_t)pwm_pulseR;

          setupSpeed(enA,(uint8_t)pwm_pulseL);
          setupSpeed(enB,(uint8_t)pwm_pulseR);


        }

        else if(y<898){

          pwm_pulseL+=pid_control1(8);
          pwm_pulseL=clamp_pwm(pwm_pulseL);

          pwm_pulseR+=pid_control2(8);
          pwm_pulseR=clamp_pwm(pwm_pulseR);

          speedMotorL=(uint8_t)pwm_pulseL;
          speedMotorR=(uint8_t)pwm_pulseR;

          setupSpeed(enA,(uint8_t)pwm_pulseL);
          setupSpeed(enB,(uint8_t)pwm_pulseR);
        }

        else if(y<=1023){

          pwm_pulseL+=pid_control1(12);
          pwm_pulseL=clamp_pwm(pwm_pulseL);

          pwm_pulseR+=pid_control2(12);
          pwm_pulseR=clamp_pwm(pwm_pulseR);

          speedMotorL=(uint8_t)pwm_pulseL;
          speedMotorR=(uint8_t)pwm_pulseR;

          setupSpeed(enA,(uint8_t)pwm_pulseL);
          setupSpeed(enB,(uint8_t)pwm_pulseR);
        }



      }

      //allora ci troviamo nel quadrante inferiore
      else if(y>=0 && y<524){
        setupDirection(BACKWARD,in2,in1);
        setupDirection(BACKWARD,in3,in4);

        Encoder_sample();

        encPrevL=encL;
        encL=Encoder_getValue(0);
        encSpeedL=-(encL-encPrevL);

        encPrevR=encR;
        encR=Encoder_getValue(1);
        encSpeedR=-(encR-encPrevR);



        if(y>325){

          pwm_pulseL+=pid_control1(4);
          pwm_pulseL=clamp_pwm(pwm_pulseL);

          pwm_pulseR+=pid_control2(4);
          pwm_pulseR=clamp_pwm(pwm_pulseR);

          speedMotorL=(uint8_t)pwm_pulseL;
          speedMotorR=(uint8_t)pwm_pulseR;

          setupSpeed(enA,(uint8_t)pwm_pulseL);
          setupSpeed(enB,(uint8_t)pwm_pulseR);


        }

        else if(y>150){

          pwm_pulseL+=pid_control1(8);
          pwm_pulseL=clamp_pwm(pwm_pulseL);

          pwm_pulseR+=pid_control2(8);
          pwm_pulseR=clamp_pwm(pwm_pulseR);

          speedMotorL=(uint8_t)pwm_pulseL;
          speedMotorR=(uint8_t)pwm_pulseR;

          setupSpeed(enA,(uint8_t)pwm_pulseL);
          setupSpeed(enB,(uint8_t)pwm_pulseR);
        }

        else if(y>=0){

          pwm_pulseL+=pid_control1(12);
          pwm_pulseL=clamp_pwm(pwm_pulseL);

          pwm_pulseR+=pid_control2(12);
          pwm_pulseR=clamp_pwm(pwm_pulseR);

          speedMotorL=(uint8_t)pwm_pulseL;
          speedMotorR=(uint8_t)pwm_pulseR;

          setupSpeed(enA,(uint8_t)pwm_pulseL);
          setupSpeed(enB,(uint8_t)pwm_pulseR);
        }


      }

      pin53=get_level(53);
      pin52=get_level(52);
      pin51=get_level(51);
      pin50=get_level(50);


      update_pins();


  }

}
