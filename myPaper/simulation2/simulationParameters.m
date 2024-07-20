%%Nominal motor values (estimated from real Data)
motor.kt = 0.000872234;  %kt 
motor.Jrw  = 2.55E-05;   %J
motor.b  = 1.00023E-05;  %B
motor.c = 0.000435786;   %kc
motor.L  = 0.000176564;  %H  
motor.R  = 1.23424;      %R    
motor.ke = 0.00966471;   %ke

%%Initial conditions
init.w_rw = 0;
init.current = 0; 

sat.J     = [3.084,3.132,3.540,0.082,-0.054,0.016];


init.q    = [1,0,0,0];
init.w    = [0,0,0];
init.W_rw = [0,0,0];

setPoint.angd = [5,0,0]';
setPoint.wd   = [0,0,0]';

feedback.P = [15.4318650394591 16.7482612383003 7.91545474858242];    
feedback.K = [3.09672280852135 5.29871200300818 1.22819168358399];

%%%Adaptation Law gains
% x.lambda = 10;
% x.n = 10;
% x.alpha = 10;
% x.beta = 10;
% x.epsilon = 10;
x.lambda = 5;
x.n = 20;
x.alpha = 3;
x.beta = 5;
x.epsilon = 0.01;

%%%Initial Conditions
% x.kd_init = 3.096305095968147e-05;
% x.ki_init = 0.001083608847320;
% x.kp_init = 7.034383411703327e-04;
x.kd_init = 1E-6;
x.ki_init = 1E-6;
x.kp_init = 1E-6;