%**************************************************************************
% AUTHOR: Brayan Espinoza 22/10/2020
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
Kc=0.1;
L=100*10^(-6);
R=0.4;
Ke=10;
Tcte=0;

%Input signal
%uArr=linspace(0,12,n);
uArr=[linspace(0,12,round(n/5)),12*ones(1,round(n/5)),linspace(12,0,round(n/5)),zeros(1,round(n/5)),linspace(0,12,round(n/5)),12*ones(1,round(n/5)),linspace(12,0,round(n/5)),zeros(1,round(n/5))];
%Initial conditions;
x=[0,0]';


%Vectors to store information
xArr=[x];

    for i=1:n-2
        u=uArr(i);
        dt=t(2)-t(1);
        %Empezamos en RK 4 de cuarto orden
            g1=dt*BrushelessModel(t(i),x,u,kt,J,B,Kc,L,R,Ke);
            g2=dt*BrushelessModel(t(i),x+0.5.*g1,u,kt,J,B,Kc,L,R,Ke);
            g3=dt*BrushelessModel(t(i),x+0.5.*g2,u,kt,J,B,Kc,L,R,Ke);
            g4=dt*BrushelessModel(t(i),x+0.5.*g3,u,kt,J,B,Kc,L,R,Ke);
            x=x+(1/6).*(g1+2.*g2+2.*g3+g4);
        %Final del RK 4
        %Store information
        xArr=[xArr,x];
    end
    
% Figures
    plot(t(1:n-1),[xArr;uArr(1:n-1)]); grid on;
    legend('Angular rate (rad/s)','Fase current (A)','Voltaje input(V)');

    