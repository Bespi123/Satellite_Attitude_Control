%**************************************************************************
% AUTHOR: Brayan Espinoza 1/10/2020
% DESCRIPTION: 
% This program perform the simulation for reaction wheels that will be used
% in the Cubesat reaction wheels.
% IMPORTANT: This a simplification that not takes into account the fase
% change dynamics.
% 
% *************************************************************************

% Clear old variables and close all windows and graphs 
clc
clear
close all;

% Simulation control parameters
n=50000;            %Number of iterations
Tf=20;              %Simulation time
t=linspace(0,Tf,n);  %Vector time

%Constant values
kt=10;
J=0.5;
B=0.2;
Kc=0.5;
L=100*10^(-6);
R=0.4;
Ke=10;
Tcte=0;

%Input signal
uArr=linspace(0,12,n);

%Initial conditions;
x=[0,0]';


%Vectors to store information
xArr=[x];

    for i=1:n-2
        u=uArr(i);
        dt=t(2)-t(1);
        %Empezamos en RK 4 de cuarto orden
            g1=dt*BrushelessModel(kt,J,B,Kc,L,R,Ke,Tcte,u,x);
            g2=dt*BrushelessModel(kt,J,B,Kc,L,R,Ke,Tcte,u,x+0.5.*g1);
            g3=dt*BrushelessModel(kt,J,B,Kc,L,R,Ke,Tcte,u,x+0.5.*g2);
            g4=dt*BrushelessModel(kt,J,B,Kc,L,R,Ke,Tcte,u,x+0.5.*g3);
            x=x+(1/6).*(g1+2.*g2+2.*g3+g4);
        %Final del RK 4
        %Store information
        xArr=[xArr,x];
    end
    
% Figures
    plot(t(1:n-1),xArr); grid on;
    legend('Angular rate (rad/s)','Fase current (A)');

    