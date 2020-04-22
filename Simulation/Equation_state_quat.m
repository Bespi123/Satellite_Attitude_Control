function [x_dot] = Equation_state_quat(Ux,Uy,Uz,Ix,Iy,Iz,x1_x,x1_y,x1_z,x2_x,x2_y,x2_z)
%x1= vectores de actitud
x2=[x2_x,x2_y,x2_z]';   %Velocidades angulares [wx,wy,wz]';
U=[Ux,Uy,Uz]';          %Torque de entrada
J=diag([Ix,Iy,Iz]);     %Tensor de inercia diagonal
%-----------------Ecuación cinematica---------------------------------
%spacecraft 3->2->1 
%Singularidad en pitch=90° (tomando x1_x->yaw,x1_y->pitch,x1_z->roll)
% B=[0,sec(x1_y)*sin(x1_z),sec(x1_y)*cos(x1_z);
%     0,cos(x1_z),-sin(x1_z);
%     1,tan(x1_y)*sin(x1_z),tan(x1_y)*cos(x1_z)];   %Ecuacion 3.44
%https://la.mathworks.com/help/aeroblks/6dofeulerangles.html#mw_2f302a65-767b-4836-81d3-8d9423421b84
%Space_craft 3->2->1 (tomando x1_x->roll,x1_y->pitch,x1_z->yaw)
B=[1,sin(x1_x)*tan(x1_y),cos(x1_x)*tan(x1_y);
    0,cos(x1_x),-sin(x1_x);
    0,sin(x1_x)/cos(x1_y),cos(x1_x)/cos(x1_y)];
x1_dot=B*x2;            %Ecuación Cinemática (3.38)
x2_dot=inv(J)*(U-cross(x2,J*x2));   %Ecuacion dinamica (3.81)
x_dot=[x1_dot;x2_dot];              %Vector de estados
end

