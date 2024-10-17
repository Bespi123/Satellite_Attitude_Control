function [theta_dot] = Ley_adaptacion(wd1_dot,wd2_dot,wd3_dot,wd1,wd2,wd3,w1,w2,w3,dq0,dq1,dq2,dq3,dq0_dot,dq1_dot,dq2_dot,dq3_dot,lambda,gamma)
%Ley de adaptacion
%Variables
    Gamma=gamma;
    %Gamma=diag([gamma,gamma,gamma])
%Orden de vectores
    wd=[wd1,wd2,wd3]';      %Velocidad angular deseada
    wd_dot=[wd1_dot,wd2_dot,wd3_dot]'; 
    w=[w1,w2,w3]';
    dq13=[dq1,dq2,dq3]';
    dq13_dot=[dq1_dot,dq2_dot,dq3_dot]';
%Calculos
R=quat2rotm([dq0,dq13']);
wr=R*wd-lambda*sign(dq0)*dq13;
wc_tilde=w-R*wd;
%wc_tilde=R*wd-w;
S=w-wr;
alpha_r=R*wd_dot-skew(wc_tilde)*R*wd-lambda*dq13_dot;
phi=-(funcion_M(alpha_r)+skew(wr)*funcion_M(w))';
theta_dot=-Gamma*phi*S;
end

