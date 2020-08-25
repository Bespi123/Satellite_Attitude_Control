function [U] = ControlFeedback_rw(J_tilde,Irw_per,Irw_par,x,dq,Wr,Wr_dot,P,K)
%Ley de control realizada mediante Lyapunov con ley de control
%por Feedback
%---------------------definiendo variables-------------------------
%   Td: Torque de(disturbios)
%   U: Torque de entrada (torque de ruedas de reaccion)
%   Irw_par:    Inercia paralela a las reaction wheels               
%   Irw_per:    Inercia perpendicular a las reaction wheels   
%   J_tilde:    Matriz de inercias del rigid body
%   q:  Cuaternio de actitud
%   w:  Velocidades angulares del cuerpo [wx,wy,wz]
%   Wrw: Velocidades angulares de ruedas de reacción
%   P,K: Ganancias de control
W=x(5:7);
Wrw=x(8:10);
dq13=[dq(2),dq(3),dq(4)]';
J_tilde=[J_tilde(1),J_tilde(4),J_tilde(5);
         J_tilde(4),J_tilde(2),J_tilde(6);
         J_tilde(5),J_tilde(6),J_tilde(3)];        %Tensor de inercia del rigid body

%---------------------cálculos-------------------------------------
J=J_tilde+diag(2*Irw_per*ones(1,3));   %Tensor de inercia general
dW=W-Wr;                               %Calculo de We
R=diag(Irw_par*ones(1,3));             %Matriz de inercias paralelas
U= P*dW+K*dq13-skew(W)*(J*W+R*(W+Wrw))- J*(Wr_dot-skew(W)*Wr);
end

