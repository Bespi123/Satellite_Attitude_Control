function [u] = Adaptive_controler(wd1_dot,wd2_dot,wd3_dot,wd1,wd2,wd3,w1,w2,w3,dq0,dq1,dq2,dq3,dq0_dot,dq1_dot,dq2_dot,dq3_dot,Jcxx,Jcyy,Jczz,Jcxy,Jcxz,Jcyz,J11,J22,J33,J23,J13,J12,lambda,Kd)
%Tensor de inercia calculado
    J=[Jcxx,Jcxy,Jcxz;
       Jcxy,Jcyy,Jcyz;
       Jcxz,Jcyz,Jczz];     %Tensor de inercia calculado
%Variables de entrada 
    wd=[wd1,wd2,wd3]';      %Velocidad angular deseada
    wd_dot=[wd1_dot,wd2_dot,wd3_dot]'; 
    w=[w1,w2,w3]';
    dq13=[dq1,dq2,dq3]';
    dq13_dot=[dq1_dot,dq2_dot,dq3_dot]';
    theta=[J11,J22,J33,J23,J13,J12]'; %Error de tensor de inercia calculado1
%Calculos
R=quat2rotm([dq0,dq13']);
wr=R*wd-lambda*sign(dq0)*dq13;
wc_tilde=w-R*wd;
%wc_tilde=R*wd-w;
S=w-wr;
alpha_r=R*wd_dot-skew(wc_tilde)*R*wd-lambda*dq13_dot;
phi=-(funcion_M(alpha_r)+skew(wr)*funcion_M(w));
tilde_u=phi*theta;
u=-Kd*S+J*alpha_r+skew(wr)*J*w+tilde_u;
end

