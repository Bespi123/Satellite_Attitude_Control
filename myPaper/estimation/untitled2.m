function [dx,y] = adcsModuleModel(t,x,u,J_11,J_22,J_33,J_12,J_23,J_13,cx,cy,cz,m_sat,alpha,miss_x,miss_y,miss_z,varargin)
    %Define inputs
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
    q  = x(1:4);   %Sat quaternions
    w  = x(5:7);   %Reaction wheel current face
    
    %%%%----------------------Definiendo variables--------------------------------
    J   =  [J_11,J_12,J_13; %Matriz de inercias del rigid body incluido semiesfera y Láser
            J_12,J_22,J_23;
            J_13,J_23,J_33];

    
    w_dot=1/J*(kt*i-B*w-Kc*sign(w));
    i_dot=1/L*(u-R*i-Ke*w);
    
    %x_dot vector
    dx=[w_dot;i_dot];
    
    %Output equation (Angular rate)
    y=[x(1);
       x(2)];
end