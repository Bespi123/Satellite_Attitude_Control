function [Out] = Measures (entr)
%Definiendo entradas
 q=[entr(1),entr(2),entr(3),entr(4)]';
 u=[entr(5),entr(6),entr(7)]';
%Modulo de torque al cuadrado
umodpow2=entr(5)^2+entr(6)^2+entr(7)^2;
%Ecuación de angulo euler
    Euler_ang=2*acos(q(1));
%Ecuacion de salida
    Out=[umodpow2,Euler_ang]';
end

