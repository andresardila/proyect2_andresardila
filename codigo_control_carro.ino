#include "PID.h"
#include <QTRSensors.h>
#define led1 9
#define led2 10
#define cant_sensores 6// Numero de sensores de la regleta
#define tiempo 5 // tiempo de respuesta del sensor,lo que se demora en sensar y dar la respuesta(en us)
#define pin_emisor 8 //pin led on
QTRSensorsAnalog qtra((unsigned char[]) { 2, 3, 4, 5,6,7}, //conexion pin 0 a 5 de arduino con los seis sensores
cant_sensores,tiempo, pin_emisor);
unsigned int sensorValues[cant_sensores];//valores de sensores y posición
int posicion=0;//incializar posicion
int derivativo=0,proporcional=0,integral=0;//inicializa las variables del control PID
int  salida_pwm=0, proporcional_pasado=0;
int velocidad=120;//modifica la velocidad con un maximo de 255
float Kp=0.18, Kd=4, Ki=0.001;  //prueba constantes adecuadas para el PID, esatas se controlan con el circuito y potenciometros
int linea=0;//0 linea blanca,1 linea negra
void setup() {
  
  delay(700);
  
 for (int i = 0; i < 40; i++)//calibracion de los sensores durante 2.5 segundos
{ 
 digitalWrite(led1, HIGH); 
            delay(30);
  qtra.calibrate();    //funcion para calibrar sensores   
  digitalWrite(led1, LOW);//parpadeo que indica que estaq calibarando   
                delay(30);
 }
 digitalWrite(led1, LOW); //se apagan los sensores para indicar fin de calibracion  
 delay(400); 
       digitalWrite(led2,HIGH); //encender led 2 para indicar la espera  de pulsacion de boton  
       int boton = 11;
 int val = 0;
 pinMode(led1, OUTPUT);
 pinMode(led2, OUTPUT);
 pinMode(boton,INPUT);
  val = digitalRead(boton);
 while (val == HIGH)
 {
 digitalWrite(led1, HIGH);
 digitalWrite(led2, HIGH);
 val = digitalRead(boton);
 };
 if (val == LOW)
 {
 digitalWrite(led1, LOW);
 digitalWrite(led2, LOW); 
};
  }

void loop() {
  // put your main code here, to run repeatedly:

}
