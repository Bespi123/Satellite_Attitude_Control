function xm_dot = Reference(entr)
%**************************************************************************
% AUTHOR: Brayan Espinoza 13/10/2020
% DESCRIPTION: This function represents a LTI MIMO second orden system that
% will give us the desired tracking.
%
% IMPORTANT: 
%
% *************************************************************************

%+++++++++++++++++++Variables++++++++++++++++++++++++++++++
x1=entr(1);         %Roll
x3=entr(2);         %Pitch
x5=entr(3);         %Yaw

x2=entr(4);         %Angular rate x
x4=entr(5);         %Angular rate y
x6=entr(6);         %Angular rate z

r1=entr(7);         %x reference    
r2=entr(8);         %y reference
r3=entr(9);         %z reference

%+++++++++++++++Second orden parameters++++++++++++++++++++
z=1;       %Amortiguamiento
wn=5;      %Frecuencia natural

%Steady state equations
xm_dot=[x2;
        x4;
        x6;
        wn^2*r1-2*z*wn*x2-wn^2*x1;
        wn^2*r2-2*z*wn*x4-wn^2*x3;
        wn^2*r3-2*z*wn*x6-wn^2*x5];
end
