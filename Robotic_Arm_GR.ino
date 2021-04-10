#include "MeMegaPi.h"
#include <Servo.h>
// Leo gayyyyyyyyy
//vicky marica
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
const int GEAR_1 = 9;
const int GEAR_2 = 7;

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
int currentSpeed = 350;
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
      isMoving += abs(steppers[i].distanceToGo());
      steppers[i].run();
  }
  if(isMoving>0){
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
  bool reset1Enable = false;
  
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
    } else if(tmp.indexOf("reset1",0)>-1){
     reset1Enable = true;
    }
    count++;
    
    if (endIndex == len - 1) break;
  }


  //Acciones a realizar tras el filtrado del puerto serie
  if(closeEnable){
     close_grip();
  }else if(openEnable){
     open_grip();
  } else if(reset1Enable){
    reset_stepper0();
  }

  Serial.println("OK");
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


//******FUNCIONES A IMPLEMENTAR POR EL GRUPO DE ALUMNOS****//

//Busqueda de límites del robot

//Límites eje 1 - establecerlo en -90,90
void reset_stepper0(){
  int cuenta_pasos1 = 0;
  int cuenta_pasos2 = 0;
   float cuenta_pasos1_grad = 0;
  float cuenta_pasos2_grad = 0;
  bool final_1 = true;
  bool final_2 = true;
  steppers[0].setSpeed(testSpeed);
  
  while(final_1){
    if(cuenta_pasos1 < 500){
       steppers[0].step();
      delay(10);
      cuenta_pasos1++;
      cuenta_pasos1_grad = (float)cuenta_pasos1 * 1.8;
    }
    else{
      qlimit_0[0] = cuenta_pasos1_grad/(GEAR_1*STEPS);
      Serial.println(qlimit_0[0]);
      final_1=false;
    }
  }
  steppers[0].setSpeed(-testSpeed);
  delay(2000);
  
  while(final_2){
    if(cuenta_pasos2 < 500){
       steppers[0].step();
      delay(10);
      cuenta_pasos2++;
      cuenta_pasos2_grad = (float)cuenta_pasos2 * 1.8;
    }
    else{
      qlimit_0[1] = cuenta_pasos2_grad/(GEAR_1*STEPS);
      Serial.println(qlimit_0[1]);
      final_2=false;
    }
  }

  steppers[0].setCurrentPosition(-(cuenta_pasos2-cuenta_pasos1));
  delay(1000);
  move_q1(0.0);
  delay(1000);
}

//Límites eje 2 - ejemplo
void reset_stepper1(){
  
  int count_steps1 = 0;
  int count_steps2 = 0;
  steppers[1].setSpeed(testSpeed);
  bool exit1=true;
  bool exit2=true;
  
  while(exit1){
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



//Punto inicial y vuelta a la posición de home
void setHome(){
 
}

void goHome(){

}


//Cinemática directa. Movimiento en q1,q2,q3
//Cinemática directa. Movimiento en q1,q2,q3
void move_q1(float q1){
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
 

}

//Función comprueba si estamos dentro de los límites
//Si al sumarle la posición seguimos dentro de los límites devolverá true
bool dentroLimites(float q){
 float posActual = steppers[0].currentPosition() *1.8;
 
  return q + posActual < qlimit_0[0] && q + posActual > qlimit_0[1];
}

void move_q2(float q2){
  
}

void move_q3(float q3){
  
}

void moveToAngles(float q1, float q2, float q3){
  
}

Vector3 forwardKinematics (float q1, float q2, float q3){
  
}


//Cinemática inversa. Movimiento en x,y,z
void moveToPoint(float x,float y,float z){
 
}

Vector3 inverseKinematics(float x,float y,float z){
 
}


//Trayectoria y tarea p&p
void trajectory (float q1, float q2, float q3, float t){
  
}

void pick_and_place (){

}
