function [U] = ControlFeedback_rw(entr)
%Ley de control realizada mediante Lyapunov con ley de control
%por Feedback
%---------------------definiendo variables-------------------------
J_tilde=[entr(1),entr(4),entr(5);
         entr(4),entr(2),entr(6);
         entr(5),entr(6),entr(3)];        %Tensor de inercia del rigid body
Irw_per=entr(7);
Irw_par=entr(8);
Wr=[entr(9),entr(10),entr(11)]';
Wr_dot=[entr(12),entr(13),entr(14)]';                        
W=[entr(15),entr(16),entr(17)]';
dq13=[entr(19),entr(20),entr(21)]';
Wrw=[entr(22),entr(23),entr(24)]';
P=entr(25);
K=entr(26);
%---------------------cálculos-------------------------------------
J=J_tilde+diag(2*Irw_per*ones(1,3));   %Tensor de inercia general
dW=W-Wr;                               %Calculo de We
R=diag(Irw_par*ones(1,3));             %Matriz de inercias paralelas
U= P*dW+K*dq13-skew(W)*(J*W+R*(W+Wrw))- J*(Wr_dot-skew(W)*Wr);
end

