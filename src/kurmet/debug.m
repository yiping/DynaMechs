close all

data = load('sim_data.txt');

ax=[];

figure()
a =subplot(3,3,1);
ax = [ax a];
plot(data(:,1),data(:,2));
title('Torso p_x ICS');

a = subplot(3,3,2);
ax = [ax a];
plot(data(:,1),data(:,27));
title('q[1]');


a = subplot(3,3,3);
ax = [ax a];
plot(data(:,1),data(:,26));
title('boom 2 torque');

a = subplot(3,3, 4);
ax = [ax a];
plot(data(:,1),data(:,12));
title('Joint torque Rknee');

a = subplot(3,3, 5);
ax = [ax a];
hold on
plot(data(:,1),data(:,16)); % Rfx
plot(data(:,1),data(:,17),'r'); % Lfx
plot(data(:,1),data(:,15),'m'); % Zmpx
hold off
title('lfx, rfx, ZMPx');

a = subplot(3,3, 6);
ax = [ax a];
hold on
% plot(data(:,1),data(:,18)); 
% plot(data(:,1),data(:,19),'r'); 
% plot(data(:,1),data(:,20),'m'); 
% plot(data(:,1),data(:,21),'-.'); 
% plot(data(:,1),data(:,22),'.k'); 
% plot(data(:,1),data(:,23),'-g');

plot(data(:,1), data(:,11)); %5
plot(data(:,1), data(:,12),'r'); %6


plot(data(:,1), data(:,13), '--'); %7
plot(data(:,1), data(:,14), 'r--'); %8
hold off
title('joint torques');

a = subplot(3,3, 7);
ax = [ax a];
hold on
plot(data(:,1),data(:,24)); 
plot(data(:,1),data(:,18),'r.-');

hold off
title('\theta_{torso}');

a = subplot(3,3, 8);
ax = [ax a];
hold on
plot(data(:,1),data(:,25)); 
plot(data(:,1),data(:,19),'r.-');
hold off
title('\dot{\theta}_{torso}');


a = subplot(3,3, 9);
ax = [ax a];
hold on
plot(data(:,1),data(:,8)); 

hold off
title('torso acc');
 linkaxes(ax,'x');
 
 

