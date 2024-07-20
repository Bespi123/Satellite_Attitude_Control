function dx = BrushelessModel_simu(entr)

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
   
    %%% State variables
    w  = entr(1);   %Reaction wheel angular rate
    i  = entr(2);   %Reaction wheel current face
    u  = entr(3);
    kt = entr(4);
    J  = entr(5);
    B  = entr(6);
    Kc = entr(7);
    L  = entr(8);
    R  = entr(9);
    Ke = entr(10);

    w_dot=1/J*(kt*i-B*w-Kc*sign(w));
    i_dot=1/L*(u-R*i-Ke*w);
    
    %x_dot vector
    dx=[w_dot;i_dot];
end