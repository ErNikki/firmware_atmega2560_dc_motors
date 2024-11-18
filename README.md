# Info
The code works on arduino mega2560 and can be taken as example to write arduino code in c.
For educational purpose i implented a lot of adrduino basic functions in c.

# Configuration of arduino    

### Encoders
The left enconder has to be insert in digitals pins 53-52                       
The right enconder has to be insert in digitals pins 51-50                      

### Controller                                                                               
The controller can be insert in every analogic pin and then configured throught  
client                                                                          

### Motors                                                                                
Every pwm pin and digital pin can be used to control motor and then configured  
throught client                                                                   

### Bridge                                                                             
I used lm-298n as bridge for motors                                             

# How to compile code

### Load code on arduino
In order to load the code in arduino, you have to: 
    1. open a terminal in folder "avr/"
    2. write "make load". i have set up a make file for you.

### Client
To open client:
    1. compile c code, in order to do this open a terminal in folder "client/" and just write "make".
    2. launch the client with "./pc"

Tested on 12V Motors.
