clear all;close all;
dat1=dlmread('2015-10-15-1614.csv',';');

maxVel=41.8;%7500*2*pi/60/24; %7900 rpm with 24 gearbox ratio

u1=dat1(:,2)/100*maxVel;

figure;plot(dat1(:,1),dat1(:,7),'k');hold on;plot(dat1(:,1),dat1(:,5),'r');xlabel('time(s)');ylabel('Velocity(rad/s)');title('Filter on velocity');
figure;plot(dat1(:,1),dat1(:,7),'k');hold on;plot(dat1(:,1),u1,'r');xlabel('time(s)');ylabel('Velocity(rad/s)');title('Target velocity');

figure;plot(dat1(:,1),dat1(:,8),'k');hold on;plot(dat1(:,1),dat1(:,6),'r');xlabel('time(s)');ylabel('Velocity(rad/s)');title('Filter on velocity');
figure;plot(dat1(:,1),dat1(:,8),'k');hold on;plot(dat1(:,1),u1,'r');xlabel('time(s)');ylabel('Velocity(rad/s)');title('Target velocity');