clear
clc
close all
d1=0.01;
d2=0.01;
d3=0.01;
d4=0.01;
t=[0:0.056:2];
L(1)=Link([0 d1 0 -pi/2 0 pi/2]);
L(2)=Link([0 0 d2 0 0 -pi/2]);
L(3)=Link([0 0 0 -pi/2 0]);
L(4)=Link([0 d3+d4 0 0 0 pi/2]);
rob=SerialLink(L,'name','robot1','comment','unsa');
%syms th1 th2 th3 th4
%cin_dir=rob.fkine([th1 th2 th3 th4])
%figure
rob.teach()
%Ti=transl(0,10,31);%Punto inicial
%rob.plot(Ti)
%s2 va de 0.3 a 1 es decir de 54 a 180
%Ti=transl(0,28.7,20.5);%Punto inicial
%Tf=transl(10,28.7,0);%Punto final
%T=ctraj(Ti,Tf,length(t));
%M=[1 0 1 0 0 0];
%q=rob.ikine(T,[0 0 0 0],M);
%rob.plot(q)
%q1=q(length(q/2),1)*180/(2*pi)
%q2=q(length(q/2),2)*180/(2*pi)
%q3=q(length(q/2),3)*180/(2*pi)
%q1=(q1+90)/180;
%q2=(q2+90)/180;
%q3=(q3+90)/180;