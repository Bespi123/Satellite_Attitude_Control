function [U] = ControlFeedback(Ixx,Iyy,Izz,Ixy,Ixz,Iyz,wrx,wry,wrz,wrd_x,wrd_y,wrd_z,wx,wy,wz,q0,q1,q2,q3,P,K)
%Ley de control realizada mediante Lyapunov con ley de control
%por Feedback
J=[Ixx,Ixy,Ixz;
   Ixy,Iyy,Iyz;
   Ixz,Iyz,Izz];     %Tensor de inercia
P=diag([P,P,P]);
K=diag([K,K,K]);
Wr_dot=[wrd_x,wrd_y,wrd_z]';
W=[wx,wy,wz]';
Wr=[wrx,wry,wrz]';
dW=W-Wr;
e=[q1,q2,q3]';
U=-P*dW + skew(W)*J*W + J*Wr_dot - J*skew(W)*Wr - K*e;
end

