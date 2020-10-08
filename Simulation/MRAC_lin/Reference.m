function xm_dot = Reference(x1,x2,r)
%Esta función representa a una funcion de transferencia lineal de segundo
%orden que nos dará el comportamiento del sistema deseado
z=1;       %Amortiguamiento
wn=5;      %Frecuencia natural
%Ecuación es espacio de estados
xm_dot=[x2;
    wn^2*r-2*z*wn*x2-wn^2*x1];
end
