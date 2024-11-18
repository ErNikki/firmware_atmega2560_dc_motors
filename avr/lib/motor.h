#ifndef MOTOR_H
#define MOTOR_H

#include "joystick.h"
#include "port.h"
#include <avr/io.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define MAX_XY_COORDINATE 1023

#define FORWARD 0
#define BACKWARD 1
#define STATIONARY 2


int getSpeed(/*uint8_t val, uint8_t max_val, uint8_t max_out*/int jVal);
/*
stabilisce la velocità basandosi su due funzioni che hanno come ascissa
la posizione e come ordinata la velocita fino a 255
PRIMA FACEVO:
calcola x=(la percentuale di val rispetto a max_val) val:val_max=x:100
e riporta in uscita (y) il x% di max_out ---> y:max_out=x:100
*/


void setupMotor(uint8_t enA, uint8_t enB, uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4);
/*
la funzione setupuint8_ti pin in output e spegne i motori

MAGGIORI INFO:
i pin enA ed enB gestiscono in PWM la velocità dei motori rispettivamente sinitro e destro
i pin in1/in3 in2/in4 gestiscono la direzione dei motori
se In1 == HIGH e in2==LOW il motore sinistro avanza
se In1 == LOW e in2==HIGH il motore sinistro indietreggia
se In1 == LOW e in2==LOW il motore sinistro spento
stesso discorso vale per i pin In3 e In4
*/

void setupSpeed(uint8_t enAorB, uint8_t speed);
/*
setta la velocità del motore
*/


void setupDirection(uint8_t direction,uint8_t in1, uint8_t in2 );
/*
 imposta la direzione del motore che ha come ingressi in1 e in2
 */

#endif
