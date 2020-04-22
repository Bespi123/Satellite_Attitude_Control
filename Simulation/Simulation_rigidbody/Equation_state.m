function [x_dot] = Equation_state(Ux,Uy,Uz,Ix,Iy,Iz,x1_x,x1_y,x1_z,x2_x,x2_y,x2_z)

x2=[x2_x,x2_y,x2_z]';
%spacecraft 3->2->1
%Singularidad en pitch=90°
B=[0,sec(x1_y)*sin(x1_z),sec(x1_y)*cos(x1_z);
     0,cos(x1_z),-sin(x1_z);
     1,tan(x1_y)*sin(x1_z),tan(x1_y)*cos(x1_z)];   %Ecuacion 3.44

U=[Ux,Uy,Uz]';          %Torque de entrada
J=diag([Ix,Iy,Iz]);     %Tensor de inercia diagonal
x1_dot=B*x2;       %Ecuación Cinemática (3.3)
x2_dot=inv(J)*(U-cross(x2,J*x2)); %...Ecuacion dinamica (3.81)
x_dot=[x1_dot;x2_dot];
end

