function [theta_dot] = Ley_adaptacion_dando(wd_dot,wd,w,dq,dq_dot,lambda,gamma)
%Ley de adaptacion
%Variables
    Gamma=gamma;
%Orden de vectores
%    wd:        Velocidad angular deseada;
%    wd_dot:    Velocidad angular derivada deseada; 
%    w:         Velocidad angular de spacecraft;
%    dq:        Cuaterno de error;
%    dq_dot:    Derivada del cuaterno de error;
%Calculos
dq0=dq(1);
dq13=dq(2:4);
dq13_dot=dq_dot(2:4);
R=quat2rotm(dq');
wr=R*wd-lambda*sign(dq0)*dq13;
wc_tilde=w-R*wd;
%wc_tilde=R*wd-w;
S=w-wr;
alpha_r=R*wd_dot-skew(wc_tilde)*R*wd-lambda*dq13_dot;
phi=-(funcion_M(alpha_r)+skew(wr)*funcion_M(w))';
theta_dot=-Gamma*phi*S;
end

