function [U] = Boskovic_control(entr)
%Entradas
    w=[entr(1),entr(2),entr(3)]';
    wd=[entr(4),entr(5),entr(6)]';
    dq0=entr(7);
    dq13=[entr(8),entr(9),entr(10)]';
    delta=entr(11);
    k=entr(12);
    Umax=entr(13);
%Calculos
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

