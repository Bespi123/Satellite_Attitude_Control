function [M] = funcion_M(x)
%Funcion M definida para un vector de tres componentes generico
M=[x(1),0,0,0,x(3),x(2);
    0,x(2),0,x(3),0,x(1);
    0,0,x(3),x(2),x(1),0];
end

