% PRÁCTICA 2

% Cálculo de la cinemática directa e inversa

% ???
% Para la herramienta (matriz tool de la RT) tipo pinza, considerar
% una transformación de Z=-45mm, X=65mm respecto del extremo del robot

clear
clc

% Longitudes de los eslabones
L1 = 150/100;
L2 = 155/100;
L3 = 200/100;

% Conversión de grados a radianes
k = pi/180;

% Definición de las articulaciones (q_3f = q_2 - q_3)
A1 = Link([0 L1 0 -90*k]);
A2 = Link([0 0 L2 0]);
A3 = Link([0 0 L3 0]);

% Vector de Links
L = [A1 A2 A2];

% Información sobre el robot (base del objeto robot)
robot = SerialLink(L, 'name', 'BrazoRobot')

% Coordenadas articulares
q = [0 -90*k 0]

% NOTA: Hay que poner las coordenadas finales guardando la relación que
% tienen las articulaciones q2, q3 y q3f !!!

% Cinemática directa 1
T = robot.fkine(q)

% Cinemática inversa
q_inversa = robot.ikunc(T)      % Coincide con la q principal

% Dibujo del robot
robot.plot(q)