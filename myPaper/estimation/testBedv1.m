function [w_dot] = testBedv1(t, Td_body, w, K)
%%%Inertia Tensor Values (body Frame)
J_11=K(1);
J_22=K(2);
J_33=K(3);
J_12=K(4);
J_13=K(5);
J_23=K(6);

%%%%----------------------Definiendo variables--------------------------------
J   =  [J_11,J_12,J_13; %Matriz de inercias del rigid body incluido semiesfera y Láser
        J_12,J_22,J_23;
        J_13,J_23,J_33];

%------------------------Ecuaciones cinematica y dinamica------------------
%%%Devido a que las angular rates son bajas, la friccion del airbearing es
%%%despreciada
w_dot=J\(Td_body-cross(w,J*w));      %Ecuación dinámica (3.147)
end