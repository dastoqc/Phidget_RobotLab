clear all;close all;
dat1=dlmread('2015-11-17-1654.csv',';');

maxVel=42;%7500*2*pi/60/24; %7900 rpm with 24 gearbox ratio

u1=dat1(:,2);%/100*maxVel;

figure('Name','Filter on velocity','NumberTitle','off');
subplot(2,1,1);plot(dat1(:,1),dat1(:,7),'k');hold on;plot(dat1(:,1),dat1(:,5),'r');xlabel('time(s)');ylabel('Velocity(rad/s)');
subplot(2,1,2);plot(dat1(:,1),dat1(:,8),'k');hold on;plot(dat1(:,1),dat1(:,6),'r');xlabel('time(s)');ylabel('Velocity(rad/s)');

figure('Name','Target velocity','NumberTitle','off');
subplot(2,1,1);plot(dat1(:,1),dat1(:,7),'k');hold on;plot(dat1(:,1),u1,'r');xlabel('time(s)');ylabel('Velocity(rad/s)');
subplot(2,1,2);plot(dat1(:,1),dat1(:,8),'k');hold on;plot(dat1(:,1),u1,'r');xlabel('time(s)');ylabel('Velocity(rad/s)');