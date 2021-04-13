// #include "MatrixMath.h"
// #include <Math.h>

#include <MatrixMath.h>
#include "MeMegaPi.h"
#include <Servo.h>

MePort limitSwitch(PORT_7);
Servo svs[1] = {Servo()};
MeStepperOnBoard steppers[3] = {MeStepperOnBoard(PORT_1), MeStepperOnBoard(PORT_2), MeStepperOnBoard(PORT_3)}; 

float qlimit_0[2] = {0.0, 0.0};
float qlimit_1[2] = {0.0, 0.0};
float qlimit_2[2] = {0.0, 0.0};

struct Vector3 {
  double x;
  double y; 
  double z;
};

float lastPositions[3] = {0,0,0};
const double RADS = PI / 180.0;
const double DEGS = 180.0 / PI;
const int STEPS = 2; // Micropasos/paso ???
const int GEAR_1 = 9; // Motor base
const int GEAR_2 = 7; // Otros dos motores

const double L1 = 150.0; 
const double L2 = 155.0; 
const double L3 = 200.0;
const double Tz = -45.0;
const double Tx = 65.0;

float T_0_1[4][4];
float T_1_2[4][4];
float T_2_3[4][4];

String buffer = "";
bool endMoving = true;
int sensor1, sensor2;

Vector3 vectorToAngles(float x, float y, float z);
void setSpeed(float speed);

int testSpeed = 200;
int testAcceleration = 300;
int currentSpeed = 400;
int maxSpeed = 500;
int currentAcceleration = 1000;
int pin1, pin2;

void setup() {
  
  // Puerto serie
  Serial.begin(115200);

  // Configuración motores paso a paso
  setSpeedConfiguration(currentSpeed,maxSpeed,currentAcceleration);
  
  for (int i = 0; i < 3; i++) {
    steppers[i].setMicroStep(STEPS);
    steppers[i].enableOutputs();
  }

  // Sensores finales de carrera
  pin1 = limitSwitch.pin1();
  pin2 = limitSwitch.pin2();
  pinMode(pin1, INPUT_PULLUP);
  pinMode(pin2, INPUT_PULLUP);
  
  // Pinza. Configuración servo 
  svs[0].attach(A8);
  open_grip();
  delay(1000);
  close_grip();

}

void loop() {

  // Lectura del puerto serie y filtrado del comando
  if (Serial.available()) {
    char c = Serial.read();
      if (c == '\n') {
        parseBuffer();
      }
      else {
        buffer += c;
      }
  }

  // Visualización finales de carrera
  sensor1 = digitalRead(pin1);
  Serial.println(sensor1);
  sensor2 = digitalRead(pin2);
  Serial.println(sensor2);

  // Permito corriente a los motores en un movimiento
  long isMoving = 0;

  for (int i = 0; i < 3; i++) {
      isMoving += abs(steppers[i].distanceToGo()); // Mide la distancia
      steppers[i].run(); // Activa el motor
  }

  if (isMoving > 0) { // Si la distancia es mayor que 0
      endMoving = true;
  }
  else {
      if (endMoving) {
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
  
  // Filtrado del mensaje por el puerto serie
  while (true) {
    startIndex = buffer.indexOf(" ", endIndex);
    endIndex = buffer.indexOf(" ", startIndex + 1);
    tmp = buffer.substring(startIndex + 1, endIndex);
   
    if (tmp.indexOf("q1", 0) > -1) {
      values[0] = tmp.substring(2, tmp.length());
      Serial.println(values[0]);
      move_q1(stringToFloat(values[0]));
    }
    else if (tmp.indexOf("open", 0) > -1) {
      openEnable = true;
    }
    else if (tmp.indexOf("close", 0) > -1) {
      closeEnable = true;
    }
    count++;
    
    if (endIndex == len - 1) 
      break;
  }

  // Acciones a realizar tras el filtrado del puerto serie
  if (closeEnable) {
    close_grip();
  }
  else if (openEnable) {
    open_grip();
  }

  Serial.println("OK"); // Está filtrado
  buffer = "";
}

// Establecer velocidad de los motores
void setSpeedConfiguration(float c_speed, float max_speed, float accel) {
    for (int i=0;i<3;i++) {
      steppers[i].setSpeed(c_speed);
      steppers[i].setMaxSpeed(max_speed);
      steppers[i].setAcceleration(accel);
    }
}

// Apertura y cierre de la pinza
void runServo(int index, int angle) {
  svs[index].write(angle);
}

void close_grip() {
  runServo(0, 120);
  delay(100);
}

void open_grip() {
  runServo(0, 0);
  delay(100);
}

// Transformación de String a float
float stringToFloat(String s) {
  float f = atof(s.c_str());
  return f;
}


//**************** FUNCIONES A IMPLEMENTAR POR EL GRUPO DE ALUMNOS ****************//

// Límites eje 1 - [-90, 90]
void reset_stepper0() {

  /* Comento a fondo la función del q1, pero las de q2 y q3 son practicamente 
  iguales asi que no doy mucho detalle salvo un par de diferencias pequeñas */

  // Contadores de pasos del motor
  int cuenta_pasos1 = 0;
  int cuenta_pasos2 = 0;
  
  // Contadores de pasos pasados a grados 
  float cuenta_pasos1_grad = 0;
  float cuenta_pasos2_grad = 0;

  // Variables boooleanas
  bool final_1 = true;
  bool final_2 = true;

  // Establecemos la velocidad
  steppers[0].setSpeed(testSpeed);
  
  // Mientras no se llegue a los 90 grados que pide
  while (final_1) { 
    // Comprueba los grados pero añadiendo la fórmula de conversion al engranaje dentado
    if (cuenta_pasos1_grad < 90.0 * GEAR_1 * STEPS) {
      steppers[0].step();                                   // Da un paso de motor
      delay(10);                                            // Espera de 10ms entre cada paso
      cuenta_pasos1++;                                      // Cuenta los pasos
      cuenta_pasos1_grad = (float) cuenta_pasos1 * 1.8;     // Los pasos contados se pasan a grados
    }
    // Cuando llegamos al limite
    else {
      qlimit_0[0] = cuenta_pasos1_grad / (GEAR_1 * STEPS);  // Rellenamos el vector del primer limite de la variable
      Serial.println(qlimit_0[0]);
      final_1 = false;                                      // Paramos el bucle
    }
  }
  
  // Velocidad de prueba pero negativa para movernos en el sentido contrario
  steppers[0].setSpeed(-testSpeed);
  delay(2000);
  
  // La misma condición de antes pero ahora con -90 grados
  while (final_2) {
    if (cuenta_pasos2_grad < 90.0 * GEAR_1 * STEPS) {
      steppers[0].step();
      delay(10);
      cuenta_pasos2++;
      cuenta_pasos2_grad = (float) cuenta_pasos2 * 1.8;
    }
    else {
      qlimit_0[1] = -(cuenta_pasos2_grad - cuenta_pasos1_grad) / (GEAR_1 * STEPS);
      
      /* Hacemos la resta de cuenta pasos 1 porque hay que tener en cuenta que
      partimos desde la posicion de 90 grados, asi que si no lo hicieramos el
      motor solo iría hasta la posicion inicial */

      Serial.println(qlimit_0[1]);
      final_2 = false;
    }
  }

  // Establecemos la posición actual del motor para saber donde estamos
  steppers[0].setCurrentPosition(-(cuenta_pasos2 - cuenta_pasos1));
  delay(1000);

  // Nos movemos a la posicion inicial 0.0
  move_q1(0.0);
  delay(1000);
}

// Límites eje 2 - ejemplo
void reset_stepper1() {
  
  // Contadores de pasos
  int count_steps1 = 0;
  int count_steps2 = 0;

  // Establecemos la velocidad
  steppers[1].setSpeed(testSpeed);

  // Variables boooleanas
  bool exit1 = true;
  bool exit2 = true;
  
  /* En este caso ponemos el limite cuando pulsemos el final de carrera con lo cual
  la condición es distinta, aparte de eso, el esquema de la funcion es igual al
  de q1 y q3 cambiando los indices de los vectores asociado de steppers o qlimits */

  while (exit1) {
    //  Si el sensor de final de carrera está apagado
    if (digitalRead(pin1) == LOW) {
      steppers[1].step();                                   // Da un paso de motor
      delay(10);                                            // Espera de 10ms entre cada paso
      count_steps1++;                                       // Cuenta los pasos
    }
    // Si el sensor de final de carrera se enciende
    else {
      qlimit_1[0] = count_steps1 * 1.8 / (GEAR_2 * STEPS);
      Serial.println(qlimit_1[0]);
      exit1 = false;                                        // Paramos el bucle
    }
  }
  
  // Establecemos la velocidad en el sentido contrario
  steppers[1].setSpeed(-testSpeed);
  delay(2000);
  
  while (exit2) {
    if (digitalRead(pin1) == LOW) {
      steppers[1].step();
      delay(10);
      count_steps2++;
    }
    else {
      qlimit_1[1] = -(count_steps2 - count_steps1) * 1.8 / (GEAR_2 * STEPS);
      Serial.println(qlimit_1[1]);
      exit2 = false;
    }
  }

  // Establecemos la posición actual del motor
  steppers[1].setCurrentPosition(-(count_steps2 - count_steps1));
  delay(1000);

  // Nos movemos a la posicion inicial 0.0
  move_q2(0.0);
  delay(1000);
}

// Límites eje 3
void reset_stepper2() {

  int count_steps1 = 0;
  int count_steps2 = 0;

  steppers[2].setSpeed(testSpeed);
  
  bool exit1 = true;
  bool exit2 = true;
  
  while (exit1) {
    if (digitalRead(pin2) == LOW) {
      steppers[2].step();
      delay(10);
      count_steps1++;
    }
    else {
      qlimit_2[0] = count_steps1 * 1.8 / (GEAR_2 * STEPS);
      Serial.println(qlimit_2[0]);
      exit1 = false;
    }
  }
  
  steppers[2].setSpeed(-testSpeed);
  delay(2000);
  
  while (exit2) {
    if (digitalRead(pin2) == LOW) {
      steppers[2].step();
      delay(10);
      count_steps2++;
    }
    else {
      qlimit_2[1] = -(count_steps2 - count_steps1) * 1.8 / (GEAR_2 * STEPS);
      Serial.println(qlimit_2[1]);
      exit2 = false;
    }
  }

  steppers[2].setCurrentPosition(-(count_steps2 - count_steps1));
  delay(1000);

  move_q3(0.0);
  delay(1000);
}

// Punto inicial
void setHome() {

  String cadena_leida = "";

  // Leemos la cadena del monitor
  if (Serial.available() > 0) {                                     // Comprobamos si en el buffer hay datos
    do {
      char caracter_leido = Serial.read();                          // Lee cada carácter uno por uno y se almacena en una variable
      cadena_leida += caracter_leido;                               // Introduce el caracter leído en la cadena 
      delay(5);
    } while (Serial.available() > 0);
  }
  
  // Sacamos los valores de ángulos
  float angulos[3] = {0.0, 0.0, 0.0};                               // Array para guardar los valores de los ángulos
  char separador = ' ';                                                  // Espacio como separador
  int index;                                                        // Para la posición de los espacios en el String

  for (int i = 0; i < cadena_leida.length(); i++) {
    index = cadena_leida.indexOf(separador);                        // Localizamos espacios dentro de la cadena (desde el pricipio)
    angulos[i] = stringToFloat(cadena_leida.substring(0, index));   // Sacamos parte de la cadena desde 0 hasta el espacio encontrado NO INCLUIDO
    cadena_leida = cadena_leida.substring(index + 1);               // Quitamos el espacio junto con la parte antes del mismo
  }

  for (int i = 0; i < 3; i++) {
    steppers[i].setCurrentPosition(angulos[i] / 1.8);
  }

}

// Vuelta a la posición de home
void goHome() {

}


//***************** Cinemática directa. Movimiento en q1, q2, q3 *****************//

// Funciones que comprueban si estamos dentro de los límites
bool dentroLimites1(float q1) {

  // Variable de la posición actual
  float posActual = steppers[0].currentPosition() * 1.8;
  // Si al sumarle la posición seguimos dentro de los límites devolverá true
  return q1 + posActual < qlimit_0[0] && q1 + posActual > qlimit_0[1];

}

bool dentroLimites2(float q2) {

  // Variable de la posición actual
  float posActual = steppers[1].currentPosition() * 1.8;
  // Si al sumarle la posición seguimos dentro de los límites devolverá true
  return q2 + posActual < qlimit_1[0] && q2 + posActual > qlimit_1[1];

}

bool dentroLimites3(float q3) {

  // Variable de la posición actual
  float posActual = steppers[2].currentPosition() * 1.8;
  // Si al sumarle la posición seguimos dentro de los límites devolverá true
  return q3 + posActual < qlimit_2[0] && q3 + posActual > qlimit_2[1];

}

// Funciones que mueven los ejes del robot
void move_q1(float q1) {

  int q_pasos;
  bool in;
 
  // Comprobamos si q1 está en los límites establecidos
  in = dentroLimites1(q1);

  // Si está dentro
  if (in == true) {
    steppers[0].setSpeed(currentSpeed);                 // Establecemos la velocidad
    steppers[0].setMaxSpeed(maxSpeed);                  // ???
    steppers[0].setAcceleration(currentAcceleration);   // ???
    q_pasos = q1 / 1.8;                                 // Paso de grados q1 a pasos
    steppers[0].moveTo(q_pasos);                        // Movemos el eje
    lastPositions[0] += q_pasos;                        // Actualizamos el vector lastPosition con los pasos calculados
    steppers[0].setCurrentPosition(lastPositions[0]);   // Establecemos la posición actual del motor
  }
  // Si está fuera
  else {
    Serial.println("Out of limits");
  }
}

void move_q2(float q2) {
  
  int q_pasos;
  bool in;
 
  // Comprobamos si q1 está en los límites establecidos
  in = dentroLimites2(q2);

  // Si está dentro
  if (in == true) {
    steppers[1].setSpeed(currentSpeed);                 // Establecemos la velocidad
    steppers[1].setMaxSpeed(maxSpeed);                  // ???
    steppers[1].setAcceleration(currentAcceleration);   // ???
    q_pasos = q2 / 1.8;                                 // Paso de grados q1 a pasos
    steppers[1].moveTo(q_pasos);                        // Movemos el eje
    lastPositions[1] += q_pasos;                        // Actualizamos el vector lastPosition con los pasos calculados
    steppers[1].setCurrentPosition(lastPositions[1]);   // Establecemos la posición actual del motor
  }
  // Si está fuera
  else {
    Serial.println("Out of limits");
  }
}

void move_q3(float q3) {

  int q_pasos;
  bool in;
 
  // Comprobamos si q1 está en los límites establecidos
  in = dentroLimites3(q3);

  // Si está dentro
  if (in == true) {
    steppers[2].setSpeed(currentSpeed);               // Establecemos la velocidad
    steppers[2].setMaxSpeed(maxSpeed);                  // ???
    steppers[2].setAcceleration(currentAcceleration);   // ???
    q_pasos = q3 / 1.8;                               // Paso de grados q1 a pasos
    steppers[2].moveTo(q_pasos);                      // Movemos el eje
    lastPositions[2] += q_pasos;                      // Actualizamos el vector lastPosition con los pasos calculados
    steppers[2].setCurrentPosition(lastPositions[2]); // Establecemos la posición actual del motor
  }
  // Si está fuera
  else {
    Serial.println("Out of limits");
  }
}

void moveToAngles(float q1, float q2, float q3) {
  move_q1(q1);
  move_q2(q2);
  move_q3(q3);
}

// Función que devuelve la matriz de transformación T entre Si-1 y Si
void denavit(float q, float d, float a, float alfa, float t[][4]) {

  t[4][4] = { {cos(q), -cos(alfa)*sin(q) ,sin(alfa)*sin(q), a*cos(q)},
              {sin(q), cos(alfa)*cos(q), -sin(alfa)*cos(q), a*sin(q)},
              {0.0, sin(alfa), cos(alfa), d},
              {0.0, 0.0, 0.0, 1.0} };
  
}

// No se si va, de momento la función lo que hace es recibir la matriz 4x4
// T1, 2, o 3 junto con los parametros que se calcularon de la tabla denavit
/*
float denavit(float q, float d, float a, float alfa) {

  float T[][4] = { {cos(q), -cos(alfa)*sin(q) ,sin(alfa)*sin(q), a*cos(q)},
                   {sin(q), cos(alfa)*cos(q), -sin(alfa)*cos(q), a*sin(q)},
                   {0.0, sin(alfa), cos(alfa), d},
                   {0.0, 0.0, 0.0, 1.0} };

  return T;

}
*/

// Función que utiliza la función denavit para calcular 0T3 (de la base al extremo 3)
Vector3 forwardKinematics (float q1, float q2, float q3) {
  Vector3 elpepe;
  float q[3] ={q1, q2-90, q2-q3};                                             // Vector donde estan las variables q denavit
  float d[3]= {L1, 0, 0};                                                     // Vector de variables d
  float a[3] = {0, L2, L3};                                                   // Vector de variables a
  float alf[3] = {-90, 0, 0};                                                 // Vector de variables alfa

  // Llamamos a la funcion denavit pasandole cada matriz para que la modifique las matrices
  denavit(q[0], d[0], a[0], alf[0], T_0_1);
  denavit(q[1], d[1], a[1], alf[1], T_1_2);
  denavit(q[2], d[2], a[2], alf[2], T_2_3);
  float T_0_3[4][4];                                                          // Esta es la matriz que queremos
  float T_0_2[4][4];                                                          // Esta es para el resultado de la multiplicacion intermedia
  
  Matrix.Multiply((double*)T_0_1, (double*)T_1_2, 4, 4, 4, (double*)T_0_2);   // Primera pultiplicacion de t1 y t2
  Matrix.Multiply((double*)T_0_2, (double*)T_2_3, 4, 4, 4, (double*)T_0_3);   // Segundo multiplicacion del resultado anterior por t3
  
  elpepe.x = T_0_3[0][3];                                                     // Igualamos las tres variables internas 
  elpepe.y = T_0_3[1][3];                                                     // del struct al resultado del vector de traslación de nuestra matriz 0T3
  elpepe.z = T_0_3[2][3];

  return elpepe;                                                              // Devolvemos el struct de tipo vector3
}


//***************** Cinemática inversa. Movimiento en x, y, z *****************//

// Lleva el extremo a una posición cartesiana x, y, z determinada
void moveToPoint(float x, float y, float z) {
 
}

// Devuelve los valores articulares del robot
Vector3 inverseKinematics(float x, float y, float z) {
 
}

// Trayectoria
// Mueve el robot a una posición articular (q1, q2, q3)
// El movimiento de los ejes es síncrono, es decir, que todos los ejes lleguen a su destino
// en el mismo tiempo
void trajectory (float q1, float q2, float q3, float t) {
  
}

// Tarea p&p
// Realiza una tarea de coger y mover un objeto mediante las funciones desarrolladas
// en los apartados anteriores
void pick_and_place () {

}
