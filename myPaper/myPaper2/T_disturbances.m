function [ Tgg ] = T_disturbances(entr)
    Ms    = entr(1);
    R_cm  = [entr(2),entr(3),entr(4)]';
    q     = [entr(5),entr(6),entr(7),entr(8)]';
    g     = [0,0,-9.81]';
    Gb    = quatRotation(q,g);
    Tgg = cross(R_cm,Ms*Gb);
end

function rotX = quatRotation(q,x)
    qx = [0,x(1),x(2),x(3)]';
    qrotX = quatmultiply(quatmultiply(q', qx'), quatconj(q'));
    rotX = qrotX(2:4);
end
