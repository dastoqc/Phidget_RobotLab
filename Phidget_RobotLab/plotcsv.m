clear all;close all;
dat1=dlmread('2015-10-14-1809.csv',';');
dat2=dlmread('2015-10-14-1718.csv',';');

maxVel=40;%7500*2*pi/60/24; %7900 rpm with 24 gearbox ratio
minCom=29;

u1=dat1(:,2)/100*maxVel;
% for i=1:length(u1)
%     if abs(u1(i))<minCom
%         u1(i)=sign(u1(i))*minCom;
%     end
%     u1(i)=(u1(i)-sign(u1(i))*minCom)/(100-minCom)*maxVel;
% end
plot(dat1(:,1),dat1(:,7),'k');hold on;plot(dat1(:,1),u1,'r');plot(dat1(:,1),dat1(:,5),'b');

u2=dat2(:,2)/100*maxVel;
% for i=1:length(u2)
%     if abs(u2(i))<minCom
%         u2(i)=sign(u2(i))*minCom;
%     end
%     u2(i)=(u2(i)-sign(u2(i))*minCom)/(100-minCom)*maxVel;
% end
figure;
plot(dat2(:,1),dat2(:,7),'k');hold on;plot(dat2(:,1),u2,'r');plot(dat2(:,1),dat2(:,5),'b');