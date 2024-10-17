function [ k_dot ] = Gain_estimator(Entr)
%Señales de entrada
    w=[Entr(1),Entr(2),Entr(3)]';
    wd=[Entr(4),Entr(5),Entr(6)]';
    dq0=Entr(7);
    dq13=[Entr(8),Entr(9),Entr(10)]';
    delta=Entr(11);
    gamma=Entr(12);
    k=Entr(13);
    Umax=Entr(14);
%Calculos
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

