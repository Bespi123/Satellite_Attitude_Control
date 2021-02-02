subplot(4,1,1);
plot(out.t,[[0.007*ones(6,1),zeros(6,1),zeros(6,1)];out.Torque]);
title('NON-LINEAR AND LINEAR MODEL COMPARISON');
ylabel('Input Torque (N.m)')
xlabel('time(s)')
legend('X','Y','Z')
grid on
subplot(4,1,2);
plot(out.t,[out.Euler,out.nonLinearEuler]);
ylabel('Euler Angles(deg)')
xlabel('time(s)')
legend('Roll','Pitch','Yaw')
grid on
subplot(4,1,3);
plot(out.t,[out.angularRate,out.nonLinearRates]);
ylabel('Angular rates(RPM)')
xlabel('time(s)')
legend('X','Y','Z')
grid on
subplot(4,1,4);
plot(out.t,[out.RwRate,out.nonLinearRwRates]);
ylabel('Reaction wheel rates(RPM)')
xlabel('time(s)')
legend('X','Y','Z')
grid on