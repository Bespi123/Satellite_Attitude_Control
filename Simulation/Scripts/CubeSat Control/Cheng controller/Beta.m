function [B] = Beta(dq13,n,gamma)
B=[];
r1=(2-gamma)*n^(gamma-1);
r2=(gamma-1)*n^(gamma-2);
    for i=1:3
        if abs(dq13(i))>n
            B(i)=sign(dq13(i))*abs(dq13(i))^gamma;
        else
            B(i)=r1*dq13(i)+r2*sign(dq13(i))*dq13(i)^2;
        end
    end
    B=B';
end

