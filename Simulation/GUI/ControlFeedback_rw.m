function [U] = ControlFeedback_rw(I,x,dq,Wr,Wr_dot,P,K)
%LS2125204: Brayan Espinoza
%Control law carried out by Lyapunov with control law
%by Feedback 
%Define Variables
%   U: Input torque
%   I: Inertia tensor (rigid body)
%   q: Attitude quaternion
%   w: Angular rate Body [wx,wy,wz]
%   P,K: Control gains

%Calculate inputs
W=x(5:7);
dq13=[dq(2),dq(3),dq(4)]';

%Calculate control law
dW=W-Wr;                               %Angular rate error
U= -P*dW - K*dq13 + skew(W)*I*W + I*(Wr_dot-skew(W)*Wr);
end


