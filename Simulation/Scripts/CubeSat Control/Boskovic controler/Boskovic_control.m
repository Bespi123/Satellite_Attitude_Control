function [U] = Boskovic_control(w,wd,dq,delta,k,Umax)
%Entradas
%    w:     Velocidad Angular del satelite;
%    wd:    Velocidad Angular deseada;
%    dq:    Cuaterno de error;
%    delta: Ganancia del cuaterno;
%    k:     Ganancia adaptativa;
%    Umax:  Torque de saturación;
%Calculos
dq0=dq(1);
dq13=dq(2:4);
R=quat2rotm([dq0,dq13']);
we=w-R*wd;
%Superficie deslizante
S=we+k^2*dq13;
%Señal de control
    for i=1:3
        U(i)=-Umax*S(i)/(abs(S(i))+k^2*delta);
    end
    U=U';
end

