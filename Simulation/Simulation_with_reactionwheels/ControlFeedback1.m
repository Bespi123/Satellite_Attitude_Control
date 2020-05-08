function [U] = ControlFeedback1(Ix,Iy,Iz,Irw_per,Irw_par,wrx,wry,wrz,wrd_x,wrd_y,wrd_z,wx,wy,wz,q0,q1,q2,q3,wrw_x,wrw_y,wrw_z)
%Ley de control realizada mediante Lyapunov con ley de control
%por Feedback
J=diag([Ix+2*Irw_per,Iy+2*Irw_per,Iz+2*Irw_per]);   %Tensor de inercia del cuerpo
Wr_dot=[wrd_x,wrd_y,wrd_z]';                        
W=[wx,wy,wz]';
Wr=[wrx,wry,wrz]';
Wrw=[wrw_x,wrw_y,wrw_z]';
dW=W-Wr;
R=diag([Irw_par,Irw_par,Irw_par]);
e=[q1,q2,q3]';
U=dW+e-skew(W)*(J*W+R*(W+Wrw))- J*(Wr_dot-skew(W)*Wr);
end

