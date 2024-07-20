function T = feedBackControl(entr)
P   = diag([entr(1),entr(2),entr(3)]);
K   = diag([entr(4),entr(5),entr(6)]);
J   = [entr(7),entr(10),entr(11); %Matriz de inercias del rigid body
       entr(10),entr(8),entr(12);
       entr(11),entr(12),entr(9)];
dw    = [entr(13),entr(14),entr(15)]';
dBeta = [entr(17),entr(18),entr(19)]';

W      = [entr(20),entr(21),entr(22)]';
wd_dot = [entr(23),entr(24),entr(25)]';
wd     = [entr(26),entr(27),entr(28)]';

%%%Controller
T = P*dw+K*dBeta-skew(W)*J*W-J*(wd_dot-skew(W)*wd);
end