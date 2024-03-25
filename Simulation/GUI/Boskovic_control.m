function [U] = Boskovic_control(w,wd,dq,delta,k,Umax)
%LS2125204: Brayan Espinoza
%Inputs
%    w:     Satellite angular rate;
%    wd:    Desired angular rate;
%    dq:    Error quaternion;
%    delta: quaternion gain;
%    k:     Adaptive gain;
%    Umax:  Saturation torque;
%Calculus
dq0=dq(1);
dq13=dq(2:4);
R=quat2rotm([dq0,dq13']);
we=w-R*wd;
%Sliding surface
S=we+k^2*dq13;
%Señal de control
U = zeros(3,1);
    for i=1:3
        U(i)=-Umax*S(i)/(abs(S(i))+k^2*delta);
    end
end