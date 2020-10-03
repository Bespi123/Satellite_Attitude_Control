function [c_dot] = Cheng_stimator(dq,w,wd,alpha1,alpha2,n,gamma,p,x,c)
% cheng stimator that stimates the parameters c1,c2,c3.
%Cuaterno de error
    dq0=dq(1);
    dq13=dq(2:4);
%Matriz de rotacion de I a B
    C=quat2rotm([dq0,dq13']);
    we=w-C*wd;
    s=we+alpha1*dq13+alpha2*Beta(dq13,n,gamma);
for i=1:3
    c_dot(i)=p(i)*(norm(s)*norm(we)^i-x(i)*c(i));
end
end

