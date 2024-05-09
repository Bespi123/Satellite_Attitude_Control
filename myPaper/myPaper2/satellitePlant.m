function x_dot = satellitePlant(entr)
    %----------------------Definiendo variables--------------------------------
Td  =  [entr(1),entr(2),entr(3)]';    %Torque de(disturbios)
U   =  [entr(4),entr(5),entr(6)]';    %Torque de entrada (torque de ruedas de reaccion)
Jrw =  diag(entr(7)*ones(1,3));       %Inercia perpendicular a las reaction wheels 
b   =  diag(entr(8)*ones(1,3));       %Viscous friction
c   =  diag(entr(9)*ones(1,3));       %Coulomb friction  
J   = [entr(10),entr(13),entr(14); %Matriz de inercias del rigid body
       entr(13),entr(11),entr(15);
       entr(14),entr(15),entr(12)];
%-------------------------Vectores de estado-------------------------------
q   = [entr(16),entr(17),entr(18),entr(19)]'; %Quaternions
w   = [entr(20),entr(21),entr(22)]';     %Velocidades angulares del cuerpo [wx,wy,wz]
Wrw = [entr(23),entr(24),entr(25)]';   %Velocidades angulares de ruedas de reacción

%%%Saturate Wrw =
%Wrw = mySaturate(Wrw,457.6253298729);

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
x1_dot=1/2*Xi*w;                                %Ecuacion cinematica (3.21)
%x2_dot=J\(Td-U-cross(w,J*w+Jrw*(w+Wrw)));      %Ecuación dinámica (3.147)
x3_dot=Jrw\(U-b*Wrw+c*sign(Wrw));               %Ecuación de Ruedas de reacción (Rw)
%x2_dot=J\(Td-Jrw*x3_dot-cross(w,J*w+Jrw*(w+Wrw)));       %Ecuación dinámica (3.147)
x2_dot=J\(Td-U-cross(w,J*w+Jrw*(w+Wrw)));       %Ecuación dinámica (3.147)
%-----------------Vector x_dot---------------------------------------------
x_dot=[x1_dot;x2_dot;x3_dot];
end