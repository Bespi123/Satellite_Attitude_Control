function rotX = quatRotation(q,x)
    qx = [0,    x(1), x(2), x(3)];
    q  = [q(1), q(2), q(3), q(4)];
    qrotX = quatmultiply(quatmultiply(q, qx), quatconj(q));
    rotX = [qrotX(2);qrotX(3);qrotX(4)];
end