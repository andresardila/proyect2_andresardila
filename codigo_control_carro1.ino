#include <QTRSensors.h>
#define NUM_SENSORS 6 // Numero de sensores que usa
#define NUM_SAMPLES_PER_SENSOR 5 // Muestras por sensor
#define EMITTER_PIN 3 //pin led on
//PINES ARDUINO
#define led1              7
#define led2              8
#define mot_izquierdo     5
#define mot_derecho       4
#define sensores          6  //pin para boton
#define pin_pwm_izquierdo         12
#define pin_pwm_derecho         11
QTRSensorsAnalog qtra((unsigned char[]) { A0, A1, A2, A3, A4, A5}, //conexion pin 0 a 5 de arduino con los seis sensores
 NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];//valores de sensores y posición
int posicion=0;//incializar posicion
int derivativo=0,proporcional=0,integral=0;//inicializa las variables del control PID
int  salida_pwm=0, proporcional_anterior=0;
int velocidad=120;//modifica la velocidad con un maximo de 255
float Kp=0.18, Kd=4, Ki=0.001;  //prueba constantes adecuadas para el PID, esatas se controlan con el circuito y pulsador.
int linea=0;//linea negra.
void setup() {
   delay(700);
   int boton = 10;
 int val = 0;
  pinMode(mot_izquierdo, OUTPUT);//pin de direccion motor izquierdo
 pinMode(mot_derecho, OUTPUT);
 pinMode(led1, OUTPUT);
 pinMode(led2, OUTPUT);
 pinMode(boton,INPUT);
 pinMode(pin_pwm_izquierdo,OUTPUT);
 pinMode(pin_pwm_derecho,OUTPUT);
  
  for (int i = 0; i < 40; i++)//calibracion de los sensores durante 2.5 segundos
{ 
 digitalWrite(led1,HIGH); 
            delay(30);
  qtra.calibrate();    //funcion para calibrar sensores   
  digitalWrite(led1,LOW);//parpadeo que indica que estaq calibarando   
                delay(30);
 }
 digitalWrite(led1,LOW); //se apagan el led1 para indicar fin de calibracion  
 delay(400); 
       digitalWrite(led2,HIGH); //encender led 2 para indicar la espera  de pulsacion de boton  
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

 qtra.readLine(sensorValues);
  if (sensorValues[0]<=45 && sensorValues[1]<=45 && sensorValues[2]<=45 && sensorValues[3]<=45 && sensorValues[4]<=45 && sensorValues[5]<=45) //si se salio de la linea
 { 
  control_motores(0,0);
 }
}

  //funcion control de motores
void control_motores(int motor_i,int motor_d)
{
  
  if ( motor_i >= 0 )  //motor izquierdo
 {
  digitalWrite(mot_izquierdo,HIGH); // con high avanza
  analogWrite(pin_pwm_izquierdo,255-motor_i); //escribe el valor del control en el motor izquierdo
 }
 else
 {
  digitalWrite(mot_izquierdo,LOW); //con low retrocede
  motor_i = motor_i*(-1); //cambio de signo
  analogWrite(pin_pwm_izquierdo,motor_i); 
 }


  if ( motor_d >= 0 ) //motor derecho
 {
  digitalWrite(mot_derecho,HIGH);
  analogWrite(pin_pwm_derecho,255-motor_d);
 }
 else
 {
  digitalWrite(mot_derecho,LOW);
  motor_d= motor_d*(-1);
  analogWrite(pin_pwm_derecho,motor_d);
 }
}
void control_pid(int linea, int velocidad, float Kp, float Ki, float Kd)
{
  posicion= qtra.readLine(sensorValues, QTR_EMITTERS_ON, linea); //0 para linea 
                                                //negra, 1 para linea blanca
  proporcional = (posicion) - 3500; // set point de 3500
  integral=integral + proporcional_anterior; 
  derivativo = (proporcional - proporcional_anterior);
  if (integral>1000) integral=1000; //limite integral
  if (integral<-1000) integral=-1000;
  salida_pwm =( proporcional * Kp ) + ( derivativo * Kd )+(integral*Ki);
  //valores correctos de velocidad
  if (  salida_pwm > velocidad )  
  salida_pwm = velocidad; 
  if ( salida_pwm < -velocidad )  
  salida_pwm = -velocidad;
  
  if (salida_pwm < 0)
 {
  control_motores(velocidad+salida_pwm, velocidad);
 }
 if (salida_pwm >0)
 {
  control_motores(velocidad, velocidad-salida_pwm);
 }

 proporcional_anterior = proporcional;  
}
//funcion frenado


void loop() 
{
  control_pid(linea,velocidad,Kp,Ki,Kd);
 //flanco de comparacion de 0 a 1000
}

