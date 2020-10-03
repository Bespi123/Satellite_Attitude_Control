%**************************************************************************
% AUTHOR: Brayan Espinoza 1/10/2020
% DESCRIPTION: 
% This program contains the steady-state model for a brushless motor
% developed by José A. Becerra-Vargas in the article "Estimation parameters 
% and black box model of a brushless DC motor".
% IMPORTANT: 
%
% *************************************************************************

function [x_dot] = BrushelessModel(x,u)
%Define inputs
x1=x(1);   %Reaction wheel angular rate
x2=x(2);   %Reaction wheel acceleration 
x3=x(3);

x1_dot=x2;
x2_dot=kt/J*x3-B/J*x1-K/J*sign(x1);
x3_dot=u/L-R/L*x3-ke/L*x1;

x_dot=[x1_dot,x2_dot,x3_dot]';

end

