function [x_dot] = Equation_state_quat(Ux,Uy,Uz,Ixx,Iyy,Izz,Ixy,Ixz,Iyz,q0,q1,q2,q3,x2_x,x2_y,x2_z)
%q0,q1,q2,q3 cuaternos de actitud
x2=[x2_x,x2_y,x2_z]';   %Velocidades angulares [wx,wy,wz]';
U=[Ux,Uy,Uz]';          %Torque de entrada
J=[Ixx,Ixy,Ixz;
   Ixy,Iyy,Iyz;
   Ixz,Iyz,Izz];     %Tensor de inercia
%-----------------Ecuación cinematica y dinamica--------------------
%Libro: Fundamentals of Spacecraft Attitude Determination and Control
%Autor: F. Landis Markley & John L. Crassidis
%Cinemática de cuaternos..Ecuación (2.88)
Xi=[-q1,-q2,-q3;
    q0,-q3,q2;
    q3,q0,-q1;
    -q2,q1,q0];           
%Ecuacion 2.88 modificada ya que el libro considera roll->z,pitch->y,yaw->x
%Se modificó para considerar roll->x,pitch->z,yaw->z
% Xi=[-q3,-q2,-q1;             
%     q2,-q3,q0;               
%     -q1,q0,q3;               
%     q0,-q1,-q2];             
x1_dot=1/2*Xi*x2;            %Ecuación cinemática (3.21)
x2_dot=inv(J)*(U-cross(x2,J*x2));   %Ecuacion dinamica (3.81)
x_dot=[x1_dot;x2_dot];              %Vector de estados
end

