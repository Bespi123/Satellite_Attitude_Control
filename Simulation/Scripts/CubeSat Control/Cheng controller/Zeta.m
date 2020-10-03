function [zeta_dot] = Zeta(dq,w,wd,u,zeta,alpha1,alpha2,n,gamma,gamma1,zeta_o,k1,k2,Umax,a)
%Auxiliarity system to compensate for the actuator saturation
%Cuaterno de error
    dq0=dq(1);
    dq13=dq(2:4);
%Calculos
    C=quat2rotm([dq0,dq13']);
    we=w-C*wd;   
    s=we+alpha1*dq13+alpha2*Beta(dq13,n,gamma);
    delta_u=sat(Umax,a,u)-u;
    if norm(zeta) <= zeta_o
        zeta_dot=[0,0,0]';
    else
        zeta_dot=-k1*zeta-k2*[sign(zeta(1))*abs(zeta(1))^gamma1,...
                     sign(zeta(2))*abs(zeta(2))^gamma1,...
                     sign(zeta(3))*abs(zeta(3))^gamma1]'...
                -(norm(s'*delta_u,1)+0.5*delta_u'*delta_u)/...
                (norm(zeta)^2)*zeta + delta_u;
    end
    zeta=zeta';
end

