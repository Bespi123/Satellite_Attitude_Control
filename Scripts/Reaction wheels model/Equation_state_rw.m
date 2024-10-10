function [x_dot] = Equation_state_rw(Td,Irw_per,Irw_par,J_tilde,U,x)
%----------------------Definiendo variables--------------------------------
%   Td: Torque de(disturbios)
%   U: Torque de entrada (torque de ruedas de reaccion)
%   Irw_par:    Inercia paralela a las reaction wheels               
%   Irw_per:    Inercia perpendicular a las reaction wheels   
%   J_tilde:    Matriz de inercias del rigid body
%   q:  Cuaternio de actitud
%   w:  Velocidades angulares del cuerpo [wx,wy,wz]
%   Wrw: Velocidades angulares de ruedas de reacción

%-----------------------------Calculos-------------------------------------
q=x(1:4);
w=x(5:7);
Wrw=x(8:10);
J_tilde=[J_tilde(1),J_tilde(4),J_tilde(5);
         J_tilde(4),J_tilde(2),J_tilde(6);
         J_tilde(5),J_tilde(6),J_tilde(3)];
R=diag(Irw_par*ones(1,3));             %Matriz de inercias paralelas
J=J_tilde+diag(2*Irw_per*ones(1,3));   %Tensor de inercia general

%------------------------Ecuaciones cinematica y dinamica------------------
%Libro: Fundamentals of Spacecraft Attitude Determination and Control
%Autor: F. Landis Markley & John L. Crassidis
%Cinemática de cuaternos..Ecuación (2.88 modificada)
% Xi=[-q3,-q2,-q1;             
%     q2,-q3,q0;               
%     -q1,q0,q3;               
%     q0,-q1,-q2]; 
%Cinemática de cuaternos..Ecuación (2.88)
Xi=[-q(2),-q(3),-q(4);
     q(1),-q(4),q(3);
     q(4),q(1),-q(2);
    -q(3),q(2),q(1)]; 
q_dot=1/2*Xi*w;                                   %Ecuacion cinematica (3.21)
w_dot=J\(Td-U-cross(w,J*w+R*(w+Wrw)));      %Ecuación dinámica (3.147)
Wrw_dot=(R\U)-w_dot;                   %Ecuación de Ruedas de reacción (Rw)
%-----------------Vector x_dot---------------------------------------------
x_dot=[q_dot;w_dot;Wrw_dot];
end

