function [ k_dot ] = Gain_estimator_bosk(w,wd,dq,delta,gamma,k,Umax)
%Señales de entrada
%   w:  velocidad angular del satelite;
%   wd: velocidad angular deseada';
%   dq: Quaterno error;
%   delta: Ganancia
%   gamma: Learning rate
%   k: ganancia adaptativa
%   Umax: Torque de saturación
%Calculos
dq0=dq(1);
dq13=dq(2:4);
R=quat2rotm([dq0,dq13']);
we=w-R*wd;
%Variables de iniciación
    sum=0;
%Superficie deslizante
    S=we+k^2*dq13;
%Sumatoria
    for i=1:3
        sum=sum+(we(i)*dq13(i))/(abs(S(i))+k^2*delta)-(abs(we(i))*(1+delta))/(abs(we(i))+k^2*(1+delta));
    end
%Estimator
k_dot=gamma*k/(1+4*gamma*(1-dq0))*(Umax*(sum)*we'*dq13-k^2*(dq13')*dq13);
end

