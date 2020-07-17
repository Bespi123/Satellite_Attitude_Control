function [Out] = Measures (entr)
%Definiendo entradas
 q=[entr(1),entr(2),entr(3),entr(4)]';
 w=[entr(5),entr(6),entr(7)]';
 J=[entr(8),entr(11),entr(12);
    entr(11),entr(9),entr(13);
    entr(12),entr(13),entr(10)];     %Tensor de inercia calculado
%Ecuacion de energia cinetica    
    K=w'*J*w;
%Ecuación de angulo euler
    Euler_ang=2*acos(q(1));
%Ecuacion de salida
    Out=[K,Euler_ang]';
end

