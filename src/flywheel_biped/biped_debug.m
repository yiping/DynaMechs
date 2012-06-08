% (1) sim_time
% (2) q[2]
% (3) qd[9]
% (4) JointTorque[9]
% (5) CoMx_ICS
% (6) p_Rf_ICS[0]
% (7) p_Lf_ICS[0]
% (8) actual_ZMPx_ICS
% (9) p_tx_ICS

close all
clear all

data = load('biped_sim_data.txt');

ax=[];

figure()
a = gca;
ax = [ax a];
hold on

plot(data(:,1),data(:,4),'r'); % flywheel torque
hold off
title('flywheel torque');


figure()
a = gca;
ax = [ax a];
hold on
plot(data(:,1),data(:,3)); % flywheel velocity
hold off
title('flywheel angular velocity');

 
 
figure()
a = gca;
ax = [ax a];
hold on
plot(data(:,1),data(:,5),'k'); % CoMx_ICS
plot(data(:,1),data(:,6),'r.'); % p_Rf_ICS_x
plot(data(:,1),data(:,7),'b.'); % p_Lf_ICS_x
plot(data(:,1),data(:,8),'m.'); % actual_ZMPx_ICS
plot(data(:,1),data(:,9),'k--'); % p_tx_ICS
hold off
title('CoMx, Rfx, Lfx, actual ZMPx in ICS');

linkaxes(ax,'x');

% sim_time
