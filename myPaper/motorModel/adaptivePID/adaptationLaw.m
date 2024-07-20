function theta_dot = adaptationLaw(entr)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
n     = entr(1);
alpha = entr(2);
beta  = entr(3);
s     = entr(4);
s_dot = entr(5);
e     = entr(6); 
myPi  = [entr(7);entr(8);entr(9)];
theta_dot = n*myPi*(s_dot+alpha*s+beta*tanh(s/e));
end