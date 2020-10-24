%**************************************************************************
% AUTHOR: Brayan Espinoza 3/10/2020
% DESCRIPTION: 
% This program contains the steady-state model for a brushless motor
% developed by José A. Becerra-Vargas in the article "Estimation parameters 
% and black box model of a brushless DC motor".
% IMPORTANT: 
%
% *************************************************************************

function [x_dot,y] = BrushelessModel(t,x,u,kt,J,B,Kc,L,R,Ke,varargin)
%Define inputs
%u:     Vpwm (Vmean)
%kt:    Torque constant (N*m/A)
%ke:    Constante contraelectromotriz (V/(Rad/s))
%kc:    Constante de friccion de Coulomb (N*m*s)
%J:     Rotor inertia (kg*m^2)
%B:     Constante de friccion viscosa (N*m*s)
%L:     Inductancia (H)
%R:     Resistencia (R)
%Tcte:  Torque de la carga
Tcte=0;
w=x(1);   %Reaction wheel angular rate
i=x(2);   %Reaction wheel current face

%Perform ecuations
w_dot=1/J*(kt*i-B*w-Kc*sign(w)-Tcte);
i_dot=1/L*(u-R*i-Ke*w);

%x_dot vector
x_dot=[w_dot,i_dot]';

%Output equation (Angular rate)
y=[x(1);x(2)];

end

