function [x_dot] = Equation_state_Euler(Td,U,Irw_per,Irw_par,J_tilde,x)
%----------------------Definiendo variables--------------------------------
%   Td: Torque de(disturbios)
%   U: Torque de entrada (torque de ruedas de reaccion)
%   Irw_par:    Inercia paralela a las reaction wheels               
%   Irw_per:    Inercia perpendicular a las reaction wheels   
%   J_tilde:    Matriz de inercias del rigid body
%   Euler:  	Angulos de Euler
%   w:  Velocidades angulares del cuerpo [wx,wy,wz]
%   Wrw: Velocidades angulares de ruedas de reacción

%-----------------------------Calculos-------------------------------------
Td=[Td(1),Td(2),Td(3)]';
U=[U(1),U(2),U(3)]';
Euler=[x(1),x(2),x(3)]';
w=[x(4),x(5),x(6)]';
Wrw=[x(7),x(8),x(8)]';
J_tilde=[J_tilde(1),J_tilde(4),J_tilde(5);
         J_tilde(4),J_tilde(2),J_tilde(6);
         J_tilde(5),J_tilde(6),J_tilde(3)];
R=diag(Irw_par*ones(1,3));             %Matriz de inercias paralelas
J=J_tilde+diag(2*Irw_per*ones(1,3));   %Tensor de inercia general

%-----------------Ecuación cinematica---------------------------------
%Libro: Fundamentals of Spacecraft Attitude Determination and Control
%Autor: F. Landis Markley & John L. Crassidis
%Singularidad en pitch=90° (tomando x1_x->yaw,x1_y->pitch,x1_z->roll)
% B=[0,sec(x1_y)*sin(x1_z),sec(x1_y)*cos(x1_z);
%     0,cos(x1_z),-sin(x1_z);
%     1,tan(x1_y)*sin(x1_z),tan(x1_y)*cos(x1_z)];   %Ecuacion 3.44
%https://la.mathworks.com/help/aeroblks/6dofeulerangles.html#mw_2f302a65-767b-4836-81d3-8d9423421b84
%Space_craft 3->2->1 (tomando x1_x->roll,x1_y->pitch,x1_z->yaw)
B=[1,sin(Euler(1))*tan(Euler(2)),cos(Euler(1))*tan(Euler(2));
    0,cos(Euler(1)),-sin(Euler(1));
    0,sin(Euler(1))/cos(Euler(2)),cos(Euler(1))/cos(Euler(2))];
Euler_dot=B*w;            %Ecuación Cinemática (3.38)
w_dot=inv(J)*(Td-U-cross(w,J*w+R*(w+Wrw)));      %Ecuación dinámica (3.147)
Wrw_dot=(inv(R)*U)-w_dot;                   %Ecuación de Ruedas de reacción (Rw)
%-----------------Vector x_dot---------------------------------------------
x_dot=[Euler_dot;w_dot;Wrw_dot];             %Vector de estados
end

