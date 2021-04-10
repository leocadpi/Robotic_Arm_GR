#include "MatrixMath.h"
#include <Math.h>
#include "MeMegaPi.h"
#include <Servo.h>

MePort limitSwitch(PORT_7);
Servo svs[1] = {Servo()};
MeStepperOnBoard steppers[3] = {MeStepperOnBoard(PORT_1),MeStepperOnBoard(PORT_2),MeStepperOnBoard(PORT_3)}; 

float qlimit_0[2] = {0.0,0.0}; //ToDo
float qlimit_1[2] = {0.0,0.0}; //ToDo
float qlimit_2[2] = {0.0,0.0}; //ToDo

struct Vector3
{
  double x;
  double y; 
  double z;
};

float lastPositions[3] = {0,0,0};
const double RADS = PI / 180.0;
const double DEGS = 180.0 / PI;
const int STEPS = 2;
const int GEAR_1 = 9; //Motor base
const int GEAR_2 = 7; //Otros dos motores

const double L1 = 150.0; 
const double L2 = 155.0; 
const double L3 = 200.0;
const double Tz = -45.0;
const double Tx = 65.0;

String buffer = "";
bool endMoving = true;
int sensor1, sensor2;

Vector3 vectorToAngles(float x,float y,float z);
void setSpeed(float speed);

int testSpeed = 200;
int testAcceleration = 300;
int currentSpeed = 400;
int maxSpeed = 500;
int currentAcceleration = 1000;
int pin1, pin2;

void setup() {
  
  //Puerto serie
  Serial.begin(115200);

  //Configuración motores paso a paso
  setSpeedConfiguration(currentSpeed,maxSpeed,currentAcceleration);
  
  for(int i=0;i<3;i++){
    steppers[i].setMicroStep(STEPS);
    steppers[i].enableOutputs();
  }

  //Sensores finales de carrera
  pin1 = limitSwitch.pin1();
  pin2 = limitSwitch.pin2();
  pinMode(pin1,INPUT_PULLUP);
  pinMode(pin2,INPUT_PULLUP);
  
  //Pinza. Configuración servo 
  svs[0].attach(A8);
  open_grip();
  delay(1000);
  close_grip();
  
}

void loop() {

  //Lectura del puerto serie y filtrado del comando
  if (Serial.available()) {
     char c = Serial.read();
     if (c == '\n') {
      parseBuffer();
    } else {
      buffer += c;
    }
  }

  //Visualización finales de carrera
  sensor1 = digitalRead(pin1);
  Serial.println(sensor1);
  sensor2 = digitalRead(pin2);
  Serial.println(sensor2);

  //Permito corriente a los motores en un movimiento
  long isMoving = 0;
  for(int i=0;i<3;i++){
      isMoving += abs(steppers[i].distanceToGo()); //Mide la distancia
      steppers[i].run(); //Activa el motor
  }
  if(isMoving>0){ //Si la distancia es mayor que 0
      endMoving = true;
  }
  else{
      if(endMoving){
         endMoving = false;
         steppers[0].setCurrentPosition(lastPositions[0]);
         steppers[1].setCurrentPosition(lastPositions[1]);
         steppers[2].setCurrentPosition(lastPositions[2]);
      }
  }
}

void parseBuffer() {
  
  buffer =" "+buffer+" ";
  buffer.toLowerCase();

  int count = 0;
  int startIndex = 0;
  int endIndex = 0;
  int len = buffer.length();
  if (len < 1) {
    return;
  }
  String tmp;
  String values[6];

  bool openEnable = false;
  bool closeEnable = false;
  
  //Filtrado del mensaje por el puerto serie
  while (true) {
    startIndex = buffer.indexOf(" ", endIndex);
    endIndex = buffer.indexOf(" ", startIndex + 1);
    tmp = buffer.substring(startIndex + 1, endIndex);
   
    if(tmp.indexOf("q1",0)>-1){
      values[0] = tmp.substring(2, tmp.length());
      Serial.println(values[0]);
      move_q1(stringToFloat(values[0]));
    }
    else if(tmp.indexOf("open",0)>-1){
       openEnable = true;
    }
    else if(tmp.indexOf("close",0)>-1){
      closeEnable = true;
    }
    count++;
    
    if (endIndex == len - 1) break;
  }

  //Acciones a realizar tras el filtrado del puerto serie
  if(closeEnable){
     close_grip();
  }
  else if(openEnable){
     open_grip();
  }

  Serial.println("OK"); //Está filtrado
  buffer = "";
}

//Establecer velocidad de los motores
void setSpeedConfiguration(float c_speed, float max_speed, float accel){
    for(int i=0;i<3;i++){
        steppers[i].setSpeed(c_speed);
        steppers[i].setMaxSpeed(max_speed);
        steppers[i].setAcceleration(accel);
    }
}

//Apertura y cierre de la pinza
void runServo(int index,int angle){
  svs[index].write(angle);
}

void close_grip(){
 runServo(0,120);
 delay(100);
}

void open_grip(){
 runServo(0,0);
 delay(100);
}

//Transformación de String a float
float stringToFloat(String s){
  float f = atof(s.c_str());
  return f;
}

//******FUNCIONES A IMPLEMENTAR POR EL GRUPO DE ALUMNOS******//

//Busqueda de límites del robot

//comento a fondo la función del q1, pero las de q2 y q3 son practicamente 
//iguales asi que no doy mucho detalle salvo un par de diferencias pequeñas

//Límites eje 1 - establecerlo en -90,90
void reset_stepper0(){
  //Variables para contar los pasos que da el motor
  int cuenta_pasos1 = 0;
  int cuenta_pasos2 = 0;
  
  //Lo mismo pero pasado a grados 
  float cuenta_pasos1_grad = 0;
  float cuenta_pasos2_grad = 0;
  bool final_1 = true;
  bool final_2 = true;
  steppers[0].setSpeed(testSpeed);
  
  while(final_1){ //mientras no se llegue a los 90 grados que pide
    if(cuenta_pasos1_grad < 90.0*GEAR_1*STEPS){ //comprueba los grados pero añadiendo la fórmula de conversion al engranaje dentado
      steppers[0].step(); //damos un paso
      delay(10);
      cuenta_pasos1++; //lo contamos
      cuenta_pasos1_grad = (float)cuenta_pasos1 * 1.8; //lo pasamos a grados
    }
    else{ //cuando llegamos al limite
      qlimit_0[0] = cuenta_pasos1_grad/(GEAR_1*STEPS); //rellenamos el vector del primer limite de la variable
      Serial.println(qlimit_0[0]);
      final_1=false;
    }
  }
  
  steppers[0].setSpeed(-testSpeed); //velocidad de prueba pero negativa para movernos en el sentido contrario
  delay(2000);
  
  while(final_2){ //misma condicion de antes pero ahora con -90
    if(cuenta_pasos2_grad < 90.0*GEAR_1*STEPS){
      steppers[0].step();
      delay(10);
      cuenta_pasos2++;
      cuenta_pasos2_grad = (float)cuenta_pasos2 * 1.8;
    }
    else{
      qlimit_0[1] = -(cuenta_pasos2_grad - cuenta_pasos1_grad)/(GEAR_1*STEPS); //hacemos la resta de cuenta pasos 1 porque hay que tener
      //en cuenta que partimos desde la posicion de 90 grados, asi que si no lo hiciermos el motor solo iria hasta la posicion inicial
      Serial.println(qlimit_0[1]);
      final_2=false;
    }
  }

  steppers[0].setCurrentPosition(-(cuenta_pasos2-cuenta_pasos1)); //para saber donde estamos
  delay(1000);
  move_q1(0.0); //nos movemos a la posicion inicial 0.0
  delay(1000);
}

//Límites eje 2 - ejemplo
void reset_stepper1(){
  
  int count_steps1 = 0;
  int count_steps2 = 0;
  steppers[1].setSpeed(testSpeed);
  bool exit1=true;
  bool exit2=true;
  
  while(exit1){ //en este caso ponemos el limite cuando pulsemos el final de carrera
    //con lo cual la condición es distinta, aparte de eso, el esquema de la funcion es igual al de q1 y q3
    //cambiando los indices de los vectores asociado de steppers o qlimits
    if(digitalRead(pin1)==LOW){
      steppers[1].step();
      delay(10);
      count_steps1++;
    }
    else{
      qlimit_1[0] = count_steps1*1.8/(GEAR_2*STEPS);
      Serial.println(qlimit_1[0]);
      exit1=false;
    }
  }
  
  steppers[1].setSpeed(-testSpeed);
  delay(2000);
  
  while(exit2){
    if(digitalRead(pin1)==LOW){
      steppers[1].step();
      delay(10);
      count_steps2++;
    }
    else{
      qlimit_1[1] = -(count_steps2-count_steps1)*1.8/(GEAR_2*STEPS);
      Serial.println(qlimit_1[1]);
      exit2=false;
    }
  }

  steppers[1].setCurrentPosition(-(count_steps2-count_steps1));
  delay(1000);
  move_q2(0.0);
  delay(1000);
}

//Límites eje 3
void reset_stepper2(){
  int count_steps1 = 0;
  int count_steps2 = 0;
  steppers[2].setSpeed(testSpeed);
  bool exit1=true;
  bool exit2=true;
  
  while(exit1){
    if(digitalRead(pin2)==LOW){
      steppers[2].step();
      delay(10);
      count_steps1++;
    }
    else{
      qlimit_2[0] = count_steps1*1.8/(GEAR_2*STEPS);
      Serial.println(qlimit_2[0]);
      exit1=false;
    }
  }
  
  steppers[2].setSpeed(-testSpeed);
  delay(2000);
  
  while(exit2){
    if(digitalRead(pin2)==LOW){
      steppers[2].step();
      delay(10);
      count_steps2++;
    }
    else{
      qlimit_2[1] = -(count_steps2-count_steps1)*1.8/(GEAR_2*STEPS);
      Serial.println(qlimit_2[1]);
      exit2=false;
    }
  }

  steppers[2].setCurrentPosition(-(count_steps2-count_steps1));
  delay(1000);
  move_q3(0.0);
  delay(1000);
}

//Punto inicial
void setHome(){
 
}

//Vuelta a la posición de home
void goHome(){

}

//******Cinemática directa. Movimiento en q1,q2,q3******//

//Función comprueba si estamos dentro de los límites
//Si al sumarle la posición seguimos dentro de los límites devolverá true
bool dentroLimites(float q){
 float posActual = steppers[0].currentPosition() *1.8;
 return q + posActual < qlimit_0[0] && q + posActual > qlimit_0[1];
}

//Las funciones que mueven los ejes del robot
void move_q1(float q1){
  //Comprobar que q1 está en los límites establecidos
	//Paso de grados q1 a pasos
	//Uso de la función steppers[n].moveTo(steps) para mover el eje
	//Actualizar el vector lastPositions con los pasos calculados
  
  int q_pasos;
  bool in;
 
  //Comprobar q1 está en los límites establecidos
  in = dentroLimites(q1);
  if(in == true){
    //paso de grados a pasos
    q_pasos = q1 / 1.8;
    //uso de la función steppers[n].moveTo(steps) para mover el eje
    steppers[0].moveTo(q_pasos);
    //actualizar el vector lastPosition con los pasos calculados
    lastPositions[0] += q_pasos;
    steppers[0].setCurrentPosition(lastPositions[0]);
  }
  else{
    Serial.println("Out of limits");
  }
}

void move_q2(float q2){
  
}

void move_q3(float q3){
  
}

void moveToAngles(float q1, float q2, float q3){
  move_q1(q1);
  move_q2(q2);
  move_q3(q3);
}

//Función que devuelve la matriz de transformación T entre Si-1 y Si
void denavit(float q, float d, float a, float alfa)
{

}

//Función que utiliza la función denavit para calcular 0T3 (de la base al extremo 3)
Vector3 forwardKinematics (float q1, float q2, float q3){
  
}

//******Cinemática inversa. Movimiento en x,y,z******//

void moveToPoint(float x,float y,float z){
 
}

Vector3 inverseKinematics(float x,float y,float z){
 
}

//Trayectoria
void trajectory (float q1, float q2, float q3, float t){
  
}

//Tarea Pick&Place (p&p)
void pick_and_place (){

}
