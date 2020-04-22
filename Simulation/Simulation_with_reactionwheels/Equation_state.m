function [x_dot] = Equation_state(Ux,Uy,Uz,Tdx,Tdy,Tdz,Ix,Iy,Iz,Irw,Km,phi,theta,psi,x2x,x2y,x2z,x3x,x3y,x3z)
x2=[x2x,x2y,x2z]';
x3=[x3x,x3y,x3z]';
%Segun libro Attitude Kinematicas Equations of motion for nonspining
%spacecraft 3->2->1
%Singularidad en pitch=90°
A=[1,0,-sin(theta);
     0,cos(phi),cos(theta)*sin(phi);
     0,-sin(phi),cos(theta)*cos(phi)];
%Torque de disturbios
Td=[Tdx,Tdy,Tdz]';
U=[Ux,Uy,Uz]';
Km=diag(Km*ones(1,3));
Irw=diag([Irw,Irw,Irw]);
I=diag([Ix,Iy,Iz]);
x1_dot=inv(A)*x2;
x2_dot=inv(I)*(Td-2*Km*U-cross(x2,I*x2+Irw*x3));
x3_dot=inv(Irw)*(Km*U);
% x2_dot=inv(I)*(Td-Km*U-Irw*x3_dot-cross(x2,I*x2+Irw*x3));
x_dot=[x1_dot;x2_dot;x3_dot];
end

