motor.Jrw = 2.55e-05;
motor.b   = 1.00023e-05;
motor.c   = 0.000435786;

sat.J     = [3.084,3.132,3.540,0.082,-0.054,0.016];


init.q    = [1,0,0,0];
init.w    = [0,0,0];
init.W_rw = [0,0,0];

setPoint.angd = [1,2,3]';
setPoint.wd   = [0,0,0]';

feedback.P = [2.5,2.5,3.5]*1E1;
feedback.K = [2.5,2.5,2.5]*1E1;