function [overshoot] = calculateOvershoot(y,sp)
    overshoot = zeros(1,length(sp));
    for i = 1:length(sp)
            if i == 1
                optimalRange = abs(1-sp(i)); %%%Considering initial conditions are [0,0,0]
            else
                optimalRange = abs(sp(i)); %%%Considering initial conditions are [0,0,0]
            end
            signalRange = max(y(:,i))-min(y(:,i));
            overshoot(i) = signalRange-optimalRange;
            overshoot(i) = abs(overshoot(i));
    end 
end