function [U] = ControlFeedback(Ix,Iy,Iz,wrx,wry,wrz,wrd_x,wrd_y,wrd_z,wx,wy,wz,q0,q1,q2,q3)
%Ley de control realizada mediante Lyapunov con ley de control
%por Feedback
J=diag([Ix,Iy,Iz]);
Wr_dot=[wrd_x,wrd_y,wrd_z]';
W=[wx,wy,wz]';
Wr=[wrx,wry,wrz]';
dW=W-Wr;
e=[q1,q2,q3]';
U=-dW + skew(W)*J*W + J*Wr_dot - J*skew(W)*Wr - e;
end

