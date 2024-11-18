#include "serial_linux.h"
#include <stdio.h>
#include <stdlib.h>
#include "../avr_client_common/packets.h"
#include <pthread.h>
#include <semaphore.h>
///#include "comunication.h"

int8_t used_digital_pin[54]={0};
int8_t used_analog_pin[16]={0};

// COMUNICAZIONE
struct data * d;
struct motors_packet* m_packet={0};
struct joystick_packet* j_packet={0};
struct encoder_packetL* e_packetL={0};
struct encoder_packetR* e_packetR={0};
struct PID_packet* p_packet= {0};
int fd;

uint8_t in1=0;
uint8_t in2=0;
uint8_t in3=0;
uint8_t in4=0;
uint8_t enA=0;
uint8_t enB=0;
uint8_t speedMotorL=0;
uint8_t speedMotorR=0;

uint8_t pinX=0;
uint8_t pinY=0;
uint Xval=0;
uint Yval=0;

//ENCODER 1 MOTORE L PIN GIALLO 53 PIN VERDE 52
uint16_t encL=0;
uint16_t encPrevL=0;
int16_t encSpeedL=0;
int16_t error_speedL=0;
//DEBUG PIN
uint8_t pin53=0;
uint8_t pin52=0;

//ENCDER 2 MOTORE R PIN GIALLO 51 PIN VERDE 50
uint16_t encR=0;
uint16_t encPrevR=0;
int16_t encSpeedR=0;
int16_t error_speedR=0;
//DEBUG PIN
//da vedere  pin
uint8_t pin51=0;
uint8_t pin50=0;



float kp;
float ki;
float kd;

void print_information(void){
  printf("MOTORS PIN:\n" );
  printf("in1:%d in2:%d in3:%d in4:%d enA:%d enB:%d \n",in1,in2,in3,in4,enA,enB);
  printf("speedMotorL:%d speedMotorR:%d\n\n",speedMotorL,speedMotorR );

  printf("JOYSTICK PINS:\n");
  printf("pinX:%d pinY:%d\n",pinX,pinY);
  printf("Xval:%d Yval:%d\n\n",Xval,Yval);

  printf("ENCODER LEFT VALUES:\n" );
  printf("encoder left value:%d \n", encL);
  printf("encoder left previousvalue:%d\n", encPrevL);
  printf("encoder left speed:%d\n", encSpeedL );
  printf("encoder left error:%d\n",error_speedL );
  printf("pin53:%d pin52:%d\n\n",pin53, pin52 );

  printf("ENCODER RIGHT VALUES:\n" );
  printf("encoder right value:%d \n", encR);
  printf("encoder right previous value:%d\n", encPrevR);
  printf("encoder right speed:%d\n", encSpeedR);
  printf("encoder right error:%d\n",error_speedR);
  printf("pin51:%d pin50:%d\n\n",pin51, pin50 );

  printf("PID VALUES\n");
  printf("kp:%f kd:%f ki:%f\n",kp,ki,kd );



}

void send_pid(float mykp,float myki, float mykd){
  unsigned char* aux;
  aux=((unsigned char*)&mykp);
  p_packet->kp1=aux[0];
  p_packet->kp2=aux[1];
  p_packet->kp3=aux[2];
  p_packet->kp4=aux[3];

  aux=((unsigned char*)&myki);
  p_packet->ki1=aux[0];
  p_packet->ki2=aux[1];
  p_packet->ki3=aux[2];
  p_packet->ki4=aux[3];

  aux=((unsigned char*)&mykd);
  p_packet->kd1=aux[0];
  p_packet->kd2=aux[1];
  p_packet->kd3=aux[2];
  p_packet->kd4=aux[3];

  serial_putData(fd,(uint8_t*)p_packet,sizeof(struct PID_packet),PID);
  serial_read(fd, (uint8_t*)d,sizeof(struct data));
  //parse_data(d,(uint8_t*)m_packet,sizeof(struct motors_packet));
  if(d->data_type!=ACK_PID){
    printf("potrebbe esserci stato un errore nell'invio dei dati\n");
    return;
  }
  printf("DATI INVIATI\n");
  /*
  kp=mykp;
  ki=myki;
  kd=mykd
  */
}

void send_motors_pins(uint8_t myin1,uint8_t myin2,uint8_t myin3,uint8_t myin4,uint8_t myenA,uint8_t myenB){

  m_packet->in1=myin1;
  m_packet->in2=myin2;
  m_packet->in3=myin3;
  m_packet->in4=myin4;
  m_packet->enA=myenA;
  m_packet->enB=myenB;

  serial_putData(fd,(uint8_t*)m_packet,sizeof(struct motors_packet),MOTORS);
  serial_read(fd, (uint8_t*)d,sizeof(struct data));
  //parse_data(d,(uint8_t*)m_packet,sizeof(struct motors_packet));
  if(d->data_type!=ACK_MOTORS){
    printf("potrebbe esserci stato un errore nell'invio dei dati\n");
    return;
  }
  printf("DATI INVIATI\n");

  /*
  in1=myin1;
  in2=myin2;
  in3=myin3;
  in4=myin4;
  enA=myenA;
  enB=myenB;
  */
}

void send_joystick_pins(uint8_t mypinX,uint8_t mypinY){
  j_packet->pinX=mypinX;
  j_packet->pinY=mypinY;

  serial_putData(fd,(uint8_t*)j_packet,sizeof(struct joystick_packet),JOYSTICK);
  serial_read(fd, (uint8_t*)d,sizeof(struct data));

  //parse_data(d,(uint8_t*)m_packet,sizeof(struct motors_packet));
  if(d->data_type!=ACK_JOYSTICK){
    printf("potrebbe esserci stato un errore nell'invio dei dati\n");
    return;
  }


  printf("DATI INVIATI\n");
  /*
  pinX=mypinX;
  pinY=mypinY;
  */


}

void get_motor_pin_from_input_and_send(void){
  struct motors_packet m_pins;

  printf("Inserisci 0 per tornare al menu precedente\n\n");
  printf("Inserisci il pin in1\n");
  printf("\n");
  scanf("%hhd",&(m_pins.in1));

  if(m_pins.in1==0){
    return;
  }

  printf("Inserisci 0 per tornare al menu precedente\n\n");
  printf("Inserisci il pin in2\n");
  printf("\n");
  scanf("%hhd",&(m_pins.in2));

  if(m_pins.in2==0){
    return;
  }

  printf("Inserisci 0 per tornare al menu precedente\n\n");
  printf("Inserisci il pin in3\n");
  printf("\n");
  scanf("%hhd",&(m_pins.in3));

  if(m_pins.in3==0){
    return;
  }

  printf("Inserisci 0 per tornare al menu precedente\n\n");
  printf("Inserisci il pin in4\n");
  printf("\n");
  scanf("%hhd",&(m_pins.in4));

  if(m_pins.in4==0){
    return;
  }

  printf("Inserisci 0 per tornare al menu precedente\n\n");
  printf("Inserisci il pin enA\n");
  printf("\n");
  scanf("%hhd",&(m_pins.enA));

  if(m_pins.enA==0){
    return;
  }

  printf("Inserisci 0 per tornare al menu precedente\n\n");
  printf("Inserisci il pin enB\n");
  printf("\n");
  scanf("%hhd",&(m_pins.enB));

  if(m_pins.enB==0){
    return;
  }

  send_motors_pins(m_pins.in1, m_pins.in2, m_pins.in3, m_pins.in4, m_pins.enA, m_pins.enB);
  //printf("DATA INVIATI\n");

  printf("\nPREMI INVIO PER TORNARE AL MENU PRINCIPALE\n");
      while(getchar()!='\n');
      getchar();



}

void get_joystick_pin_from_input_and_send(void){

  struct joystick_packet j_pins;

  printf("Inserisci 0 per tornare al menu precedente\n\n");
  printf("Inserisci il pin X\n");
  printf("\n");
  scanf("%hhd",&(j_pins.pinX));

  if(j_pins.pinX==0){
    return;
  }

  printf("Inserisci 0 per tornare al menu precedente\n\n");
  printf("Inserisci il pin Y\n");
  printf("\n");
  scanf("%hhd",&(j_pins.pinY));

  if(j_pins.pinY==0){
    return;
  }

  send_joystick_pins(j_pins.pinX,j_pins.pinY);
  //printf("DATA INVIATI\n");

  printf("\nPREMI INVIO PER TORNARE AL MENU PRINCIPALE\n");
      while(getchar()!='\n');
      getchar();


}

void get_pid_from_input_and_send(void){


  float temp1;
  float temp2;
  float temp3;



  printf("Inserisci 999 per tornare al menu precedente\n\n");
  printf("Inserisci il valore kp X\n");
  printf("\n");
  scanf("%f",&(temp1));
  if(temp1==999){
    return;
  }

  printf("Inserisci 999 per tornare al menu precedente\n\n");
  printf("Inserisci il valore ki X\n");
  printf("\n");
  scanf("%f",&(temp2));
  if(temp2==999){
    return;
  }


  printf("Inserisci 999 per tornare al menu precedente\n\n");
  printf("Inserisci il valore kd X\n");
  printf("\n");
  scanf("%f",&(temp3));
  if(temp3==999){
    return;
  }

  send_pid(temp1,temp2,temp3);

}

void get_and_update_data(void){

  serial_read(fd, (uint8_t*)d,sizeof(struct data));
  switch (d->data_type){
    case MOTORS:
    parse_data(d,(uint8_t*)m_packet,sizeof(struct motors_packet));
    in1=m_packet->in1;
    in2=m_packet->in2;
    in3=m_packet->in3;
    in4=m_packet->in4;
    enA=m_packet->enA;
    enB=m_packet->enB;
    speedMotorL=m_packet->speedMotorL;
    speedMotorR=m_packet->speedMotorR;
    break;

    case JOYSTICK:
    parse_data(d,(uint8_t*)j_packet,sizeof(struct joystick_packet));
    pinX=j_packet->pinX;
    pinY=j_packet->pinY;
    Xval=j_packet->Xval2 <<8 | j_packet->Xval1;
    Yval=j_packet->Yval2 <<8 | j_packet->Yval1;
    break;

    case ENCODERL:
    parse_data(d,(uint8_t*)e_packetL,sizeof(struct encoder_packetL));
    encL=e_packetL->encL2<<8 | e_packetL->encL1;
    encPrevL=e_packetL->encPrevL2<<8 | e_packetL->encPrevL1;
    encSpeedL=((int16_t)e_packetL->encSpeedL2)<<8 | (int16_t)e_packetL->encSpeedL1;
    error_speedL=((int16_t)e_packetL->error_speedL2)<<8 | (int16_t)e_packetL->error_speedL1;
    break;

    case ENCODERR:
    parse_data(d,(uint8_t*)e_packetR,sizeof(struct encoder_packetR));
    encR=e_packetR->encR2<<8 | e_packetR->encR1;
    encPrevR=e_packetR->encPrevR2<<8 | e_packetR->encPrevR1;
    encSpeedR=((int16_t)e_packetR->encSpeedR2)<<8 | (int16_t)e_packetR->encSpeedR1;
    error_speedR=((int16_t)e_packetR->error_speedR2)<<8 | (int16_t)e_packetR->error_speedR1;
    break;

    case PID:
    parse_data(d,(uint8_t*)p_packet,sizeof(struct PID_packet));

    unsigned char* aux;
    aux=((unsigned char*)&kp);
    aux[0]= p_packet->kp1;
    aux[1]= p_packet->kp2;
    aux[2]= p_packet->kp3;
    aux[3]= p_packet->kp4;

    aux=((unsigned char*)&kd);
    aux[0]= p_packet->kd1;
    aux[1]= p_packet->kd2;
    aux[2]= p_packet->kd3;
    aux[3]= p_packet->kd4;

    aux=((unsigned char*)&ki);
    aux[0]= p_packet->ki1;
    aux[1]= p_packet->ki2;
    aux[2]= p_packet->ki3;
    aux[3]= p_packet->ki4;





  }

}

void clean_eeprom(void){

  printf("sicuro di voler pulire la EEPROM?\n\n");
  printf("premi invio per continuare\n");
  printf("\n");
  if(getchar()!='\n'){
    return;
  }
  getchar();

  serial_putData(fd,NULL,0,CLEAN_EEPROM);
  serial_read(fd, (uint8_t*)d,sizeof(struct data));
  while(d->data_type!=ACK_CLEAN_EEPROM){
    printf("potrebbe esserci stato un errore nell'invio dei dati\n");
    return;
  }

  printf("EEPROM pulita riavvia arduino\n");
  printf("\nPREMI INVIO PER TORNARE AL MENU PRINCIPALE\n");
  while(getchar()!='\n');




}

void init_from_eeprom(void){
  printf("sicuro di voler resettare i pin arduino dalla EEPROM?\n\n");
  printf("premi invio per continuare\n");
  printf("\n");
  if(getchar()!='\n'){
    return;
  }
  getchar();

  serial_putData(fd,NULL,0,INIT_EEPROM);
  serial_read(fd, (uint8_t*)d,sizeof(struct data));
  while(d->data_type!=ACK_INIT_EEPROM){
    printf("potrebbe esserci stato un errore nell'invio dei dati\n");
    return;
  }

  printf("pin arduino resettati dalla eeprom\n");
  printf("\nPREMI INVIO PER TORNARE AL MENU PRINCIPALE\n");
  while(getchar()!='\n');
}

void info_handler(void){
  //chiedo i dati
  serial_putData(fd,NULL,0,GET_DATA);
  //ricevo pin motori
  get_and_update_data();
  //ricevo pin joystick
  get_and_update_data();
  //ricevo encoder L
  get_and_update_data();
  //ricevo encoder R
  get_and_update_data();
  //ricevo pid
  get_and_update_data();


  print_information();

      while(getchar()=='\n'){
        //POTREI FARE COSI PER MANDARE LA VELOCITÀ
        //chiedo i dati
        printf("\e[1;1H\e[2J");

        printf("\nPREMI INVIO PER AGGIORNARE I DATI\n");
        printf("\nPREMI UN TASTO PER TORNARE AL MENU PRINCIPALE\n\n");

        serial_putData(fd,NULL,0,GET_DATA);

        //ricevo pin motori
        get_and_update_data();

        //ricevo pin joystick
        get_and_update_data();

        //ricevo encoder L
        get_and_update_data();

        //ricevo encoder R
        get_and_update_data();

        //ricevo pid
        get_and_update_data();

        print_information();

      }
      //getchar();
}

int main(int argc, char *argv[]){

  fd = serial_open("/dev/ttyACM0");
  if(fd<0){
    printf("FanDuino non collegato sulla porta ttyACM0 \n");
    return -1;
  }

  serial_set_interface_attribs(fd, 57600, 0);
  serial_set_blocking(fd,1);
  sleep(1);
  //printf("ciao\n" );
  d = malloc(sizeof(struct data));
  m_packet=malloc(sizeof(struct motors_packet));
  j_packet=malloc(sizeof(struct joystick_packet));
  e_packetL=malloc(sizeof(struct encoder_packetL));
  e_packetR=malloc(sizeof(struct encoder_packetR));
  p_packet=malloc(sizeof(struct PID_packet));
  printf("Syncing...");
  //chiedo i dati
  serial_putData(fd,NULL,0,GET_DATA);
  //ricevo pin motori
  get_and_update_data();
  //ricevo pin joystick
  get_and_update_data();
  //ricevo encoder L
  get_and_update_data();
  //ricevo encoder R
  get_and_update_data();
  //ricevo pid
  get_and_update_data();
  for(int i = 0;i<50;i++){
    printf(".");

  }
  printf("\n");
  printf("\e[1;1H\e[2J");
  int choose;
  while (1) {

    printf("\e[1;1H\e[2J");
    printf("INFORMAZIONI\n");
    printf("L'ENCODER DEL MOTORE SINISTRO VA MESSO PIN GIALLO 53 PIN VERDE 52\n");
    printf("L'ENCODER DEL MOTORE DESTRI VA MESSO PIN GIALLO 51 PIN VERDE 50\n\n");
    printf("\tPremi 0 per visualizzare le informazioni\n\n");
    printf("JOYSTICK\n");
    printf("\tPremi 1 per aggiungere o reimpostare il Joystick\n");
    printf("MOTORI\n");
    printf("\tPremi 2 per aggiungere o reimpostare i motori\n");
    printf("PID\n");
    printf("\tPremi 3 per aggiungere o reimpostare i valori PID\n");
    printf("EEPROM\n");
    printf("\tPremi 4 per pulire la eeprom\n");
    //printf("\tPremi 5 per resettare i pin dalla eeprom\n");

    //serial_putData(fd,NULL,0,GET_DATA);
    scanf("%d",&choose);
    //serial_putData(fd,NULL,0,GET_DATA);
    printf("\e[1;1H\e[2J");

    switch (choose) {
      case 0:
        info_handler();
      break;
      case 1:
       get_joystick_pin_from_input_and_send();
      break;
      case 2:
        get_motor_pin_from_input_and_send();
      break;
      case 3:
        get_pid_from_input_and_send();
      break;
      case 4:
        clean_eeprom();
      break;
      //case 5:
      //  init_from_eeprom();
      //break;
      default:
        printf("Opzione non disponile!\n");
      break;

    }

  }
  close(fd);
  return 0;
}
