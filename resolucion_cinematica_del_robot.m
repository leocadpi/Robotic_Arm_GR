% PRÁCTICA 2

% Cálculo de la cinemática directa e inversa

% ???
% Para la herramienta (matriz tool de la RT) tipo pinza, considerar
% una transformación de Z=-45mm, X=65mm respecto del extremo del robot

clear
clc

% Longitudes de los eslabones
L1 = 1.50;
L2 = 1.55;
L3 = 2.00;

% Conversión de grados a radianes
k = pi/180;

% Definición de las articulaciones (q_3f = q_2 - q_3)
A1 = Link([0 L1 0 -90*k]);
A2 = Link([0 0 L2 0]);
A3 = Link([0 0 L3 0]);

% Vector de Links
L = [A1 A2 A3];

% Información sobre el robot (base del objeto robot)
robot = SerialLink(L, 'name', 'BrazoRobot')

q1 = 0*k;
q2 = 45*k;
q3 = 30*k;
q3f = q2 - q3;

% Coordenadas articulares
q = [q1 q2-90 q3f+90]

% NOTA: Hay que poner las coordenadas finales guardando la relación que
% tienen las articulaciones q2, q3 y q3f !!!

% Cinemática directa 1
T = robot.fkine(q)

% Cinemática inversa
q_inversa = robot.ikunc(T)      % Coincide con la q principal

% Dibujo del robot
robot.plot(q)
