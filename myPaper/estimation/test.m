syms q0 q1 q2 q3 g msat rx ry rz


r = -1*[rx ry rz];
q_1 = [q0, -q1, -q2, -q3];
q_2 = [0, 0, 0, -1*g];
gg = [0, 0, -1*g];

G = quatRotation(q_1, gg)

Tgg = cross(r,msat*G)


%qqq=msat*g*[2*rz*(q0*q1 - q2*q3) + ry*(q0^2 - q1^2 - q2^2 + q3^2);2*rz*(q0*q2 + q1*q3) - rx*(q0^2 - q1^2 - q2^2 + q3^2); -2*(rx*(q0*q1 - q2*q3) + ry*(q0*q2 + q1*q3))]
qqq=msat*g*[-2*rz*(q0*q1 + q2*q3) + ry*(q0^2 - q1^2 - q2^2 + q3^2);-2*rz*(q0*q2 - q1*q3) - rx*(q0^2 - q1^2 - q2^2 + q3^2); 2*(rx*(q0*q1 + q2*q3) + ry*(q0*q2 - q1*q3))]


Tgg_111 = -1*cross([1 2 3],1*quatRotation([0.3 -0.2 -0.1 -0.3], [0, 0, -1*9.5]))

Tgg_num = subs(Tgg, {q0, q1, q2, q3, g, msat, rx, ry, rz}, {0.3 0.2 0.1 0.3 9.5 1 1 2 3});
% Evaluar el resultado
Tgg_num = double(Tgg_num)

qqq_num = subs(qqq, {q0, q1, q2, q3, g, msat, rx, ry, rz}, {0.3 0.2 0.1 0.3 9.5 1 1 2 3});
% Evaluar el resultado
qqq_num = double(qqq_num)


function rotX = quatRotation(q,x)
     qx = [0,    x(1), x(2), x(3)]
     q  = [q(1), q(2), q(3), q(4)]
     quatmultiplication(q, qx)

     [q(1), -q(2), -q(3), -q(4)]

     qrotX = quatmultiplication(quatmultiplication(q, qx), [q(1), -q(2), -q(3), -q(4)]);
     rotX = [qrotX(2);qrotX(3);qrotX(4)];
end

function q_mult = quatmultiplication(q1,q2)
w1 = q1(1);
x1 = q1(2);
y1 = q1(3);
z1 = q1(4); 

w2 = q2(1);
x2 = q2(2);
y2 = q2(3);
z2 = q2(4);

q_mult = [w1*w2 - x1*x2 - y1*y2 - z1*z2, ...
          w1*x2 + x1*w2 + y1*z2 - z1*y2, ...
          w1*y2 - x1*z2 + y1*w2 + z1*x2, ...
          w1*z2 + x1*y2 - y1*x2 + z1*w2];
end