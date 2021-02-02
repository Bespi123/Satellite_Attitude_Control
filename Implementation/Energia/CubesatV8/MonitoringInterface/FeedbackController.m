function [] = FeedbackController(samplesNumber, portName)
%*************************************************************************
% MATLAB + TivaC Serial Comunication
% Author: Bespi123 
% Note: This code is adapted from the code developed by Mario Perez Esteao
%************************************************************************
%
%
 close all;
 clc;
 
 %Variables
 portStatus = 0;    % Port Status
 y=zeros(1,1000);    %   Data buffer
 
% Check if selected port is available
 list = serialportlist("available");
 listSize = size(list);
 for i=1:1:listSize(2)
     if list(i)== portName
         portStatus = 1;
     end
 end
 if portStatus
     serialPort=serialport(portName,9600);
     read(serialPort,5,"uint8")
     
 else
     %warning('off','MATLAB:serial:fscanf:unsuccessfullRead');
     disp('Error, Port not available');
 end
 
% Initialize Serial Port
% delete(instrfind({'Port'},{'COM5'}));

 
% serialPort=serialport(portName,9600);
% serialPort.BaudRate=9600;
% warning('off','MATLAB:serial:fscanf:unsuccessfullRead');
% 
% % Open Serial Port
% fopen(serialPort);
% 
% % Initialize counter
% samplesCounter = 1;
% 
% % Open a graphics window
% figure('Name','Serial comunication.');
% title('Serial comunication Matlab');
% xlabel('Samples number');
% ylabel('Sample');
% grid on;
% hold on;
% 
% % Bucle while to get samples
%     while (samplesCounter <= samplesNumber)
%         ylim([0 5.1]);
%         xlim([contadorMuestras-20 contadorMuestras+5]);
%         valor=fscanf(serialPort,'%d');
%         y(samplesCounter)=valor;
%         plot(samplesCounter,y(samplesCounter),'X-r');
%         drawnow
%         samplesCounter=samplesCounter+1;
%     end
% 
% % Close serialPort
% fclose(serialPort);
% delete(serialPort);
% clear;
end

