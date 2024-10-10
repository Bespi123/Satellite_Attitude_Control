function [ Tgg ] = T_disturbances(entr)
mu=3.986*10^(14); %coeficiente gravitacional de la tierra
rc=entr(11);      %Distancia desde el centro de la tierra
Irw_per=entr(12);
J_tilde=[entr(1),entr(4),entr(5); %Matriz de inercias del rigid body
         entr(4),entr(2),entr(6);
         entr(5),entr(6),entr(3)];
J=J_tilde+diag(2*Irw_per*ones(1,3));   %Tensor de inercia general
%q=[entr(7),entr(8),entr(9),entr(10)]';
wo2=mu/rc^3;
c3=[2*(entr(8)*entr(10)-entr(9)*entr(7));
    2*(entr(9)*entr(10)+entr(8)*entr(7));
    1-2*(entr(8)^2+entr(9)^2)];
Tgg=cross(3*wo2*c3,J*c3);
end

