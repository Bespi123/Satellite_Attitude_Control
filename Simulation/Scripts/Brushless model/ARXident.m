%**************************************************************************
% AUTHOR: Danny Turpo 02/02/2021
% DESCRIPTION: 
%
% 
% *************************************************************************

%% INICIALIZACION:
clear all; clc;
close all

YU = dlmread('Step.txt');  % Lectura del archivo
u = [0;0;YU(1:300, 1)];  % Separacion de la entrada u(PWMvalue)
w = [0;0;YU(1:300, 2)];  % Velocidad angular (RPM)
ia = [0;0;YU(1:300, 3)]; % Corriente RW (volts)
t=length(w);

%% Tratamiento
U = u';
W = w'*2*pi/60;
I = (2500-ia')/0.185;
U = U.';
W = W.';
I = I.';
%% GRAFICAS DE “U”, “W” y “I” USANDO DATOS TXT:
plot(U, '-r', 'LineWidth', 1.5); 
hold on;
plot(W, '-b', 'LineWidth', 1.5);
plot(I, '-g', 'LineWidth', 1.5);

%% Formacion de Y y F:

% velocidad angular
Y1 = W(3:t);
F1 = [W(2:t-1), W(1:t-2), U(2:t-1), U(1:t-2)];

% corriente
Y2 = I(3:t);
F2= [I(2:t-1), I(1:t-2), U(2:t-1), U(1:t-2)];
%% Calculo de parametros ARX por ecuacion normal:
% velocidad angular
TETA1 = inv(F1'*F1)*F1'*Y1;
TETA1
% 1.0097   -0.1489    5.7263   -0.7180

% corriente
TETA2 = inv(F2'*F2)*F2'*Y2;
TETA2
% 0.4854    0.2924    0.0054   -0.0052

%% Funcion transferencia velocidad angular
%N1 = [5.7263 -0.7180]; % numerador
%D1 = [1, -1.0097 0.1489]; % denominador (se cambia de signo)
N1 = TETA1(3:4)' % numerador
D1 = [1,-TETA1(1:2)'] % denominador (se cambia de signo)

%% Calcular la respuesta "W":
[W, x] = dlsim(N1, D1, U);

%% Desplegar los datos:

figure(1);
plot(W, '-k', 'LineWidth', 1.5);
title('Respuesta ARX identificado de motor');
