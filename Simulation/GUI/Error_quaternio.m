function [q_err]=Error_quaternio(qd,q)
%LS2125204: Brayan Espinoza
%Vector definition
%q=[q0,q1,q2,q3]';
%qd=[qd0,qd1,qd2,qd3]';
Xi=[-qd(2),-qd(3),-qd(4);
     qd(1),-qd(4),qd(3);
     qd(4),qd(1),-qd(2);
    -qd(3),qd(2),qd(1)];
%Cuaternios de error
q13_err=Xi'*q;
q0_err=qd'*q;
q_err=[q0_err;q13_err];
end

