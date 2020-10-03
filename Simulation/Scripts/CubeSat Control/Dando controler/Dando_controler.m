function [u] = Dando_controler(wd_dot,wd,w,dq,dq_dot,J_tilde,theta,lambda,Kd)
%Tensor de inercia calculado
    J=[J_tilde(1),J_tilde(4),J_tilde(5);
       J_tilde(4),J_tilde(2),J_tilde(6);
       J_tilde(5),J_tilde(6),J_tilde(3)];%Tensor de inercia del rigid body
%Variables de entrada 
%    wd:       Velocidad angular deseada
%    wd_dot:   Derivada de velocidad angular 
%    w:        Velocidad angular del satelite
%    dq:       Cuaterno de error
%    dq_dot:   Derivada del cuaterno de error
%    theta:    Error de tensor de inercia calculado

%Calculos
dq0=dq(1);
dq13=dq(2:4);
dq13_dot=dq_dot(2:4);
R=quat2rotm([dq0,dq13']);
wr=R*wd-lambda*sign(dq0)*dq13;
wc_tilde=w-R*wd;
S=w-wr;
alpha_r=R*wd_dot-skew(wc_tilde)*R*wd-lambda*dq13_dot;
phi=-(funcion_M(alpha_r)+skew(wr)*funcion_M(w));
tilde_u=phi*theta;
u=-Kd*S+J*alpha_r+skew(wr)*J*w+tilde_u;
end

