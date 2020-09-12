function [U] = Cheng_controller(J_tilde,dq,dq_dot,w,wd,wd_dot,Beta_dot,c,zeta,alpha1,alpha2,ko,tao1,tao2,rho,k3,k4,gamma1,n,gamma)
%Entradas
%    J_tilde:   Tensor de inercia calculado    
%    w:         Velocidad Angular del satelite;
%    wd:        Velocidad Angular deseada;
%    c:         Parametros estimados
%    dq:        Cuaterno de error;
%    alpha1:        Ganancia;
%    alpha2:        Ganancia;
%    Umax:  Torque de saturación;

%Calculos
%Tensor de inercia calculado
    Jo=[J_tilde(1),J_tilde(4),J_tilde(5);
       J_tilde(4),J_tilde(2),J_tilde(6);
       J_tilde(5),J_tilde(6),J_tilde(3)];
%Cuaterno de error
    dq0=dq(1);
    dq13=dq(2:4);
    dq13_dot=dq_dot(2:4);
%Matriz de rotacion de I a B
    C=quat2rotm([dq0,dq13']);
    we=w-C*wd;
%Calculos
    F=-skew(we+C*wd)*Jo*(we+C*wd)+Jo*(skew(we)*C*wd-C*wd_dot);
%Superficie deslizante
    s=we+alpha1*dq13+alpha2*Beta(dq13,n,gamma);
%Control compensativo
    som_u=c(1)+c(2)*norm(we)+c(3)*norm(we)^2;
    e=ko/(1+som_u);
    for i=1:3
        ur(i)=-tao1*s(i)-tao2*sign(s(i))*abs(s(i))^rho;
        un(i)=-k4*sign(s(i))*abs(s(i))^gamma1;
    end
    ur=ur';
    un=un';
    ua=-som_u*(s/(norm(s)+e));
%Señal de control
    U=-F-alpha1*Jo*dq13_dot-alpha2*Jo*Beta_dot-k3*zeta-1/2*s+ur+un+ua;
end

