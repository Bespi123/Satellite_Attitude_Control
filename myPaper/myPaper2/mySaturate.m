function xsat = mySaturate(x, alpha)
    xsat = min(max(x, -alpha), alpha);
end