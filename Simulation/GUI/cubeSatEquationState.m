function [x_dot] = cubeSatEquationState(Td,I,U,x)
%LS2125204: Brayan Espinoza
% Inputs
%   Td: Disturbances
%   U: Input torque  
%   I: Rigid body inertia torque
%   q: Attitude Quaternion
%   w: CubeSat Angular rates[wx,wy,wz]
%   x: [q;w]

%Read inputs
q=x(1:4);
w=x(5:7);

%Kinematics and Dynamic Equations
%Book: Fundamentals of Spacecraft Attitude Determination and Control
%Author: F. Landis Markley & John L. Crassidis
%Quaternions Kinematics..Ecuación (2.88)
Xi=[-q(2),-q(3),-q(4);
     q(1),-q(4),q(3);
     q(4),q(1),-q(2);
    -q(3),q(2),q(1)]; 
q_dot=1/2*Xi*w;                        %Kynematics Equation(3.21)
w_dot=inv(I)*(Td+U-cross(w,I*w));      %Dynamics Equatión (3.147)

%x_dot Vector
x_dot=[q_dot;w_dot];
end

