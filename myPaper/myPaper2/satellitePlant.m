function x_dot = satellitePlant(t,x,u,J_11,J_22,J_33,J_12,J_13,J_23,)
    %----------------------Definiendo variables--------------------------------
Td  =  [entr(1),entr(2),entr(3)]';    %Torque de(disturbios)
U   =  [entr(4),entr(5),entr(6)]';    %Torque de entrada (torque de ruedas de reaccion)
J_sat   =  [J_11,J_12,J_13; %Matriz de inercias del rigid body
            J_12,J_22,J_23;
            J_13,J_23,J_33];
cg = [cg_x,cg_y,cg_z];
%-------------------------Vectores de estado-------------------------------
q   = [x(1),x(2),x(3),x(4)]'; %Quaternions
w   = [x(5),x(6),x(7)]';      %Velocidades angulares del cuerpo [wx,wy,wz]

%-------------------------Friccion coef-----------------------------------
aplha = entr(20);

%%%Saturate Wrw =
%Wrw = mySaturate(Wrw,457.6253298729);

%------------------------Ecuaciones cinematica y dinamica------------------
%Libro: Fundamentals of Spacecraft Attitude Determination and Control
%Autor: F. Landis Markley & John L. Crassidis
%Cinemática de cuaternos..Ecuación (2.88 modificada)
%Cinemática de cuaternos..Ecuación (2.88)
Xi=[-q(2),-q(3),-q(4);
     q(1),-q(4),q(3);
     q(4),q(1),-q(2);
    -q(3),q(2),q(1)]; 
q_dot=1/2*Xi*w;                   %Ecuacion cinematica (3.21)
w_dot=J\(Td+U-cross(w,J*w))-aplha*w;      %Ecuación dinámica (3.147)

%-----------------Vector x_dot---------------------------------------------
x_dot=[q_dot;w_dot];
end


function [ Tgg ] = T_disturbances(entr)
    Ms    = entr(1);
    R_cm  = [entr(2),entr(3),entr(4)]';
    q     = [entr(5),entr(6),entr(7),entr(8)]';
    g     = [0,0,-9.81]';
    Gb    = quatRotation(q,g);
    Tgg = cross(R_cm,Ms*Gb);
end

function rotX = quatRotation(q,x)
    qx = [0,x(1),x(2),x(3)]';
    qrotX = quatmultiply(quatmultiply(q', qx'), quatconj(q'));
    rotX = qrotX(2:4);
end