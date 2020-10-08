function [ xdot ] = Model(x1,x2,u)
%Linear transfer function for each axis
%B=(2.5+rand()*(3.3-2.5))*10^4;
%M=(5.8+rand()*(15.6-5.8))*10^7;
%B=1*10^4;
%M=2*10^7;
I=8.46*10^(-3);        %Body Inertia
xdot=[x2;
    u/I+0*x1];
end

