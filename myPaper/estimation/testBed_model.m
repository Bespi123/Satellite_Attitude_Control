function [x_dot] = testBed_model(t, u, x, K)
%%%Inertia Tensor Values
J_11=K(1);
J_22=K(2);
J_33=K(3);
J_12=K(4);
J_13=K(5);
J_23=K(6);
% %%%Gravity center offset
% cg= K(7:9)';
% %%%Satellite mass
% m_sat = K(10);
% %%%airbearing Friction Coef
% aplha = K(11);
% %%%Unmodeled friction coef
% A = K(12);

%%%%----------------------Definiendo variables--------------------------------
J   =  [J_11,J_12,J_13; %Matriz de inercias del rigid body incluido semiesfera y Láser
        J_12,J_22,J_23;
        J_13,J_23,J_33];
%%%Matrix de inercia alineado al centro geometrico(es la que se usa)
%%%J = J_sat+m_sat*skew(cg)*skew(cg)';

%-------------------------Vectores de estado-------------------------------
q   = [x(1),x(2),x(3),x(4)]'; %Quaternions
w   = [x(5),x(6),x(7)]';     %Velocidades angulares del cuerpo [wx,wy,wz]

% % % % % %%%------------------------AirBearing undesired Torque---------------------
% % % % % t_air_inertial = [0,0,A]'; %%% Inertial frame
% % % % % t_air_body = quatRotation(quatconj(q'),t_air_inertial);
% % % % % 
% % % % % %%%------------------------AirBearing friction Torque----------------------
% % % % % coef= [0,0,aplha];  %%%Inertial Frame
% % % % % t_air_friction = -diag(quatRotation(quatconj(q'),coef))*w;
% % % % % %%%------------------------Gravity Gradient Torque-------------------------
% % % % % t_gg_body = T_disturbances(m_sat,cg,q);  %%Body Frame
% % % % % 
% % % % % Td_body = t_gg_body+t_air_friction+t_air_body;
% % % % % tdArr = [t_gg_body;t_air_friction;t_air_body];
% % % % % td = [td,tdArr];
%------------------------Ecuaciones cinematica y dinamica------------------
%Libro: Fundamentals of Spacecraft Attitude Determination and Control
%Autor: F. Landis Markley & John L. Crassidis
%Cinemática de cuaternos..Ecuación (2.88 modificada)
%Cinemática de cuaternos..Ecuación (2.88)
Xi=[-q(2),-q(3),-q(4);
     q(1),-q(4),q(3);
     q(4),q(1),-q(2);
    -q(3),q(2),q(1)]; 
x1_dot=1/2*Xi*w;                   %Ecuacion cinematica (3.21)
%x2_dot=J\(Td+U-cross(w,J*w));      %Ecuación dinámica (3.147)
x2_dot=J\(u-cross(w,J*w));      %Ecuación dinámica (3.147)

%-----------------Vector x_dot---------------------------------------------
x_dot=[x1_dot;x2_dot];
end

% function [ Tgg ] = T_disturbances(Ms,R_cm,q)
%     g_inertial     = [0,0,-9.81]';
%     g_body         = quatRotation(quatconj(q'),g_inertial);
%     Tgg = cross(R_cm,Ms*g_body);
% end
% 
% function rotX = quatRotation(q,x)
%     qx = [0,    x(1), x(2), x(3)];
%     q  = [q(1), q(2), q(3), q(4)];
%     qrotX = quatmultiply(quatmultiply(q, qx), quatconj(q));
%     rotX = [qrotX(2);qrotX(3);qrotX(4)];
% end