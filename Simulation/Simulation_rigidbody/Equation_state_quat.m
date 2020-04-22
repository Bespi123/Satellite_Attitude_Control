function [x_dot] = Equation_state_quat(Ux,Uy,Uz,Ix,Iy,Iz,q0,q1,q2,q3,x2_x,x2_y,x2_z)
%x1= vectores de actitud
x2=[x2_x,x2_y,x2_z]';   %Velocidades angulares [wx,wy,wz]';
U=[Ux,Uy,Uz]';          %Torque de entrada
J=diag([Ix,Iy,Iz]);     %Tensor de inercia diagonal
%-----------------Ecuación cinematica y dinamica--------------------
%spacecraft 
%Cinemática de cuaternos
% Xi=[q4,-q3,-q2;
%     -q3,q4,q1;
%     q2,-q1,q4;
%     -q1,-q2,-q3];            %Ecuación (2.88)
Xi=[-q1,-q2,-q3;
    q0,-q3,q2;
    q3,q0,-q1;
    -q2,q1,q0];            
x1_dot=1/2*Xi*x2;            %Ecuación cinemática (3.21)
x2_dot=inv(J)*(U-cross(x2,J*x2));   %Ecuacion dinamica (3.81)
x_dot=[x1_dot;x2_dot];              %Vector de estados
end

