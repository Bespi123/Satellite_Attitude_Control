function [x_dot] = Equation_state_rw(entr)
%----------------------Definiendo variables--------------------------------
Td=[entr(1),entr(2),entr(3)]';   %Torque de(disturbios)
U=[entr(4),entr(5),entr(6)]';    %Torque de entrada (torque de ruedas de reaccion)
Irw_per=entr(7);                 %Inercia paralela a las reaction wheels               
Irw_par=entr(8);                 %Inercia perpendicular a las reaction wheels   
J_tilde=[entr(9),entr(12),entr(13); %Matriz de inercias del rigid body
         entr(12),entr(10),entr(14);
         entr(13),entr(14),entr(11)];
R=diag(Irw_par*ones(1,3));             %Matriz de inercias paralelas
J=J_tilde+diag(2*Irw_per*ones(1,3));   %Tensor de inercia general
%-------------------------Vectores de estado-------------------------------
q=[entr(15),entr(16),entr(17),entr(18)]';
w=[entr(19),entr(20),entr(21)]';   %Velocidades angulares del cuerpo [wx,wy,wz]
Wrw=[entr(22),entr(23),entr(24)]';   %Velocidades angulares de ruedas de reacción
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
x1_dot=1/2*Xi*w;                                   %Ecuacion cinematica (3.21)
x2_dot=inv(J)*(Td-U-cross(w,J*w+R*(w+Wrw)));      %Ecuación dinámica (3.147)
x3_dot=(inv(R)*U)-x2_dot;                   %Ecuación de Ruedas de reacción (Rw)
%-----------------Vector x_dot---------------------------------------------
x_dot=[x1_dot;x2_dot;x3_dot];
end

