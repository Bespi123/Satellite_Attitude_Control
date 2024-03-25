function [ k_dot ] = Gain_estimator_bosk(w,wd,dq,delta,gamma,k,Umax)
%LS2125204: Brayan Espinoza
%Input signals
%   w:  Satellite angular rates;
%   wd: Desired angular rate;
%   dq: Error quaternion;
%   delta: Gain;
%   gamma: Learning rate
%   k: Adaptive gain
%   Umax: Saturation torque
%Calculus
dq0  = dq(1);
dq13 = dq(2:4);
R=quat2rotm([dq0,dq13']);
we=w-R*wd;
%Initial variables
    sum=0;
%Sliding surface
    S=we+k^2*dq13;
%Sum
    for i=1:3
        sum=sum+(we(i)*dq13(i))/(abs(S(i))+k^2*delta)-(abs(we(i))*(1+delta))/(abs(we(i))+k^2*(1+delta));
    end
%Estimator
k_dot=gamma*k/(1+4*gamma*(1-dq0))*(Umax*(sum)*we'*dq13-k^2*(dq13')*dq13);
end

