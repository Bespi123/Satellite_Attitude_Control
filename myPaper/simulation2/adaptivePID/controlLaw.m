function u = controlLaw(entr)
theta = [entr(1);entr(2);entr(3)];
myPi  = [entr(4);entr(5);entr(6)];

u = theta'*myPi; 
end