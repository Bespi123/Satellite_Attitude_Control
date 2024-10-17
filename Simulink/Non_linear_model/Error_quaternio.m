function [q_err]=Error_quaternio(qm0,qm1,qm2,qm3,q0,q1,q2,q3)
%Definicion de vectores
q=[q0,q1,q2,q3]';
qm=[qm0,qm1,qm2,qm3]';
Xi=[-qm1,-qm2,-qm3;
     qm0,-qm3,qm2;
     qm3,qm0,-qm1;
    -qm2,qm1,qm0];
%Cuaternios de error
q13_err=Xi'*q;
q0_err=qm'*q;
q_err=[q0_err;q13_err];
end

