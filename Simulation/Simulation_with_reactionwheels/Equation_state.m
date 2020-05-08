function [x_dot] = Equation_state(Td_x,Td_y,Td_z,U_x,U_y,U_z,Irw_per,Irw_par,Ix,Iy,Iz,q0,q1,q2,q3,x2_x,x2_y,x2_z,x3_x,x3_y,x3_z)
%----------------------Definiendo variables--------------------------------
Td=[Td_x,Td_y,Td_z]';   %Torque de(disturbios)
U=[U_x,U_y,U_z]';       %Torque de entrada (torque de ruedas de reaccion)
R=diag([Irw_par,Irw_par,Irw_par]);                  %Inercia paralela de las ruedas de reaccion
J=diag([Ix+2*Irw_per,Iy+2*Irw_per,Iz+2*Irw_per]);   %Tensor de inercia del cuerpo
%-------------------------Vectores de estado-------------------------------
x2=[x2_x,x2_y,x2_z]';   %Velocidades angulares del cuerpo [wx,wy,wz]
x3=[x3_x,x3_y,x3_z]';   %Velocidades angulares de ruedas de reacción
%------------------------Ecuaciones cinematica y dinamica------------------
%Libro: Fundamentals of Spacecraft Attitude Determination and Control
%Autor: F. Landis Markley & John L. Crassidis
%Cinemática de cuaternos..Ecuación (2.88 modificada)
% Xi=[-q3,-q2,-q1;             
%     q2,-q3,q0;               
%     -q1,q0,q3;               
%     q0,-q1,-q2]; 
%Cinemática de cuaternos..Ecuación (2.88)
Xi=[-q1,-q2,-q3;
    q0,-q3,q2;
    q3,q0,-q1;
    -q2,q1,q0]; 
x1_dot=1/2*Xi*x2;                                   %Ecuacion cinematica (3.21)
x2_dot=inv(J)*(Td-U-cross(x2,J*x2+R*(x2+x3)));      %Ecuación dinámica (3.147)
x3_dot=(inv(R)*U)-x2_dot;                   %Ecuación de Ruedas de reacción (Rw)
%-----------------Vector x_dot---------------------------------------------
x_dot=[x1_dot;x2_dot;x3_dot];
end

