%**************************************************************************
% AUTHOR: Brayan Espinoza 23/10/2020
% DESCRIPTION: 
% This program process the data obtained by the reaction wheel test. Also 
% applies a discrete filter to clean the data. 
%
% IMPORTANT:
% Do you need to import 'RawDataObtained' file as Numeric Matrix to the 
% workSpace.
% 
% *************************************************************************

%%OBTAINING DATA
V=RawDataObtained(:,1)'*20/255; %Applied voltaje
W=60*RawDataObtained(:,2)'/(6*0.2)*(2*pi/60); %Reaction wheel angular rate
i=RawDataObtained(:,3)'; %Brushless motor current 
t=0.2*[1:1:length(i)];   %Time vector

%%Media filtro de media movil
windowSize = 15; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
i_m = filter(b,a,i);

%%Perform input and outputs data
y=W';
u=V';

%%Plot data
figure();
subplot(3,1,1); plot(t,V); grid on;
    xlabel('time(s)'); ylabel('voltaje (v)');
    title('REACTION WHEEL DATA'); 
subplot(3,1,2); plot(t,W); grid on;
    xlabel('time(s)'); ylabel('Angular rate (rad/s)');
subplot(3,1,3); plot(t,[i;i_m]); grid on;
    xlabel('time(s)'); ylabel('Current (mA)');
    legend('Non-filtered signal','filtered signal');
