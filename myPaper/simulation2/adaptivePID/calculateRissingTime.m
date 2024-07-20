function tr = calculateRissingTime(e, t, rising)
    %%%Rising time
    boundsris = [-rising rising];
    %%%find index out the frange
    [x,~] = find((e<=boundsris(2) & e>=boundsris(1)));
    tr = t(min(x));
end