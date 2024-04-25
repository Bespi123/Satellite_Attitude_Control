%function [dx,y] = BrushelessModel(t,x,u,kt,J,B,Kc,L,R,Ke,d,ws,varargin)
function [dx,y] = BrushelessModel(t,x,u,kt,J,B,Kc,L,R,Ke,varargin)
    %Define inputs
    %u:     Vpwm (Vmean)
    %kt:    Torque constant (N*m/A)
    %J:     Rotor inertia (kg*m^2)
    %B:     Constante de friccion viscosa (N*m*s)
    %kc:    Constante de friccion de Coulomb (N*m*s)
    %L:     Inductancia (H)
    %R:     Resistencia (R)
    %ke:    Constante contraelectromotriz (V/(Rad/s))
    %d:     Starting Torque (N*m)   
    %ws:    Stribeck angular rate (rad/s)
    
    %kt=25.5E-3;
    %L=0.56E-3;
    %R=1.2;
    %Ke=0.85;
    %d =0;
    %ws = 1;

    %%% State variables
    w  = x(1);   %Reaction wheel angular rate
    i  = x(2);   %Reaction wheel current face

    %Perform ecuations
    %w_dot=1/J*(kt*i-B*w-sign(w)*(Kc+d*exp(w^2/ws)));
    % if(isnan(x(1))||isnan(x(2)))
    %     dx=[NaN,NaN]';
    %     y = [NaN,NaN]';
    %     return;
    % end

    %w_dot=1/J*(kt*i-B*w);
    w_dot=1/J*(kt*i-B*w-Kc*sign(w));
    i_dot=1/L*(u-R*i-Ke*w);
    
    %x_dot vector
    dx=[w_dot,i_dot]';
    
    %Output equation (Angular rate)
    y=[x(1);
       x(2)
        ];
end