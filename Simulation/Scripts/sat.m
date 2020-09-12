function [Sat] = sat(Umax,a,u)
%Funcion saturacion vectorial
    for i=1:3
        if u(i)>= (Umax-a)
            Sat(i)=(Umax-a)+a*tanh((u(i)-Umax+a)/a);
        elseif (a-Umax) < u(i)
            Sat(i)=u(i);
        else
            Sat(i)=(a-Umax)+a*tanh((u(i)+Umax-a)/a);
        end
    end
    Sat=Sat';
end

