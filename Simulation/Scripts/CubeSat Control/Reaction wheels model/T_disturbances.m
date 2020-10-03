function [ Tgg ] = T_disturbances(rc,J_tilde,Irw_per,q)
%----------------------------Definiendo varibles-------------------------
    mu=3.986*10^(14);   %Coeficiente gravitacional de la tierra
    %rc:                 Distania desde el centro de la tierra
    %Irw_per:            Inercia perpendicular a las reaction wheels  
    %q:                  Cuaterno de actitud
%---------------------------- Calculos ---------------------------------- 
   J_tilde=[J_tilde(1),J_tilde(4),J_tilde(5);
            J_tilde(4),J_tilde(2),J_tilde(6);
            J_tilde(5),J_tilde(6),J_tilde(3)];
   J=J_tilde+diag(2*Irw_per*ones(1,3));   %Tensor de inercia general
   wo2=mu/rc^3;
   c3=[2*(q(2)*q(4)-q(3)*q(1));
       2*(q(3)*q(4)+q(2)*q(1));
       1-2*(q(2)^2+q(3)^2)];
%----------------------Torque por gradiente de gravedad------------------    
   Tgg=cross(3*wo2*c3,J*c3);
end

