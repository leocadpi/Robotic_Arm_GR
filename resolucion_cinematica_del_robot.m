% PRÁCTICA 2

clc
clear

%-------------- Cálculo de la cinemática directa e inversa --------------%

% Longitudes de los eslabones
L1 = 1.50;
L2 = 1.55;
L3 = 2.00;

% Conversión de grados a radianes
k = pi/180;

% Definición de las articulaciones
L(1) = Link([0 L1 0 -90*k]);
L(2) = Link([-90*k 0 L2 0]);
L(3) = Link([90*k 0 L3 0]);

% Base del objeto robot
robot = SerialLink(L,'name','BrazoRobot')

% Coordenadas articulares
q1 = 0;
q2 = 45;
q3 = 90;
q3f = q2-q3;

q = [q1*k q2*k q3f*k]

% Cinemática directa
T = robot.fkine(q)

% Cinemática inversa
q_inv = robot.ikunc(T)                                      % En radianes
q_inv_grad = [q_inv(1)/k q_inv(2)/k (q_inv(2)-q_inv(3))/k]  % En grados

% Dibujo del robot
% robot.plot(q)

%------------------------ Herramienta tipo pinza ------------------------%

% Coordenadas de la pinza respecto al extremo del robot
z = -0.45;
x = 0.65;

% Matriza de transformación homogénea
T_pinza = transl(x,0,z)

% Base del objeto robot
robot = SerialLink(L,'name','BrazoRobot','tool',T_pinza)

% Cinemática directa
T = robot.fkine(q)

% Cinemática inversa
q_inv = robot.ikunc(T)                                      % En radianes
q_inv_grad = [q_inv(1)/k q_inv(2)/k (q_inv(2)-q_inv(3))/k]  % En grados

% Dibujo del robot
robot.plot(q)

%------------------------------------------------------------------------%

robot.teach()
