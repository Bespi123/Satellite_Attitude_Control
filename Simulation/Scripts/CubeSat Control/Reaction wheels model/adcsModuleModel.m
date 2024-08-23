function [dx,y] = adcsModuleModel(t,x,u,J_11,J_22,J_33,J_12,J_23,J_13,cx,cy,cz,m_sat,alpha,miss_x,miss_y,miss_z,varargin)
    %Define inputs
    % u: control torques in the inertial frame
    % J_11:  Inertia tensor diagonal (kg⋅m²)
    % J_22:  Inertia tensor diagonal (kg⋅m²) 
    % J_33:  Inertia tensor diagonal (kg⋅m²)
    % J_12:  Inertia tensor residual (kg⋅m²)
    % J_13:  Inertia tensor residual (kg⋅m²)
    % J_23:  Inertia tensor residual (kg⋅m²)
    % cx  : CM and GM offset x component (m)
    % cy  : CM and GM offset y component (m)
    % cz  : CM and GM offset z component (m)
    % m_sat:    Stribeck angular rate (Kg)
    % alpha : friction coeff
    % miss_x: misscelaneous torques in x
    % miss_y: misscelaneous torques in y
    % miss_z: misscelaneous torques in z
    
    %%Gravity center offset
    cg= [cx,cy,cz]';
    %%Unmodeled friction coef
    miscellaneous = [miss_x,miss_y,miss_z]';
    
    %%% State variables
    q  = x(1:4);
    w  = x(5:7);   %Reaction wheel current face
    
    %%% Matriz de inercias del rigid body incluido semiesfera y Láser
    J_body   =  [J_11,J_12,J_13; 
                 J_12,J_22,J_23;
                 J_13,J_23,J_33];
    
    % Get disturbances in body frame
    w_inertia = quatRotation(q,w); %get w in the inertial frame
    T_air_inertia = [0,0,alpha*w_inertia(3)]';
    T_air_body = quatRotation(quatconj(q'),T_air_inertia);
    T_gg_body  = T_disturbances(m_sat,cg,q);
    Td_body = T_gg_body-T_air_body+miscellaneous-u'; 
    %Td_body = T_gg_body-T_air_body-u'; 


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
    x2_dot=J_body\(Td_body-cross(w,J_body*w));      %Ecuación dinámica (3.147)

    %x_dot vector
    dx=[x1_dot;x2_dot];

    %Output equation (Angular rate)
    y = x(5:7);
end

function [ Tgg ] = T_disturbances(Ms,R_cm,q)
     g_inertial     = [0,0,-9.81]';
     g_body         = quatRotation(quatconj(q'),g_inertial);
     Tgg = cross(R_cm,Ms*g_body);
end