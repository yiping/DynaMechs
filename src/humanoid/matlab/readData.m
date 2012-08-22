clear all
close all
setDataDir;
fid = fopen(strcat(dataDir,'/recentData.dat'));
filename = fscanf(fid,'%s');
fclose(fid);
filename
LKNEE = 19;
RKNEE = 12;

[cols, display, names, data, commands] = readColData(filename);

%t = data(:,1);
%data = data(t<2.821,:);

for i=1:cols
    eval(char(commands{i}));
end
prevState = [state(1) state(1:(end-1))];
transitionTimes = t(state ~= prevState);
transitionStates = state(state ~= prevState);

a=[];
figs=[];
figPos = [176          36        1104         671];


%%%%%%%%%%%%%
% CoM
%%%%%%%%%%%%%
f = figure('Name','CoM Position','Position',figPos); figs = [figs f];
clf
title('CoM Position');
a=[a subplot(131)];
hold on
plot(t,pCom(1,:),'r');
plot(t,pComDes(1,:),'r--');
drawTransitions(transitionTimes,transitionStates);
title('CoM X'); xlabel('Time (s)'); ylabel('X Position (m)');

a=[a subplot(132)];
hold on
plot(t,pCom(2,:),'g');
plot(t,pComDes(2,:),'g--');
drawTransitions(transitionTimes,transitionStates);
title('CoM Y'); xlabel('Time (s)'); ylabel('Y Position (m)');

a=[a subplot(133)];
hold on
plot(t,pCom(3,:),'b');
plot(t,pComDes(3,:),'b--');
drawTransitions(transitionTimes,transitionStates);
title('CoM Z'); xlabel('Time (s)'); ylabel('Z Position (m)');


f = figure('Name','Pelvis Position','Position',figPos); figs = [figs f];
clf
title('CoM Position');
a=[a subplot(131)];
hold on
plot(t,q(7,:),'r');
drawTransitions(transitionTimes,transitionStates);
title('Hip Z'); xlabel('Time (s)'); ylabel('Z Position (m)');


f = figure('Name','Knee Torques','Position',figPos); figs = [figs f];
clf
hold on
plot(t,tau(4,:),'r');
plot(t,tau(10,:),'b');
drawTransitions(transitionTimes,transitionStates);
xlabel('Time (s)'); ylabel('Knee Torques (Nm)');



f = figure('Name','Ang Momentum Tracking','Position',figPos); figs = [figs f];
clf
a=[a subplot(311)];
hold on
plot(t,hCom(1,:),'r');
plot(t,hComDes(1,:),'r--');
drawTransitions(transitionTimes,transitionStates);
xlabel('Time (s)'); ylabel('X Ang Mom');

a=[a subplot(312)];
hold on
plot(t,hCom(2,:),'g');
plot(t,hComDes(2,:),'g--');
drawTransitions(transitionTimes,transitionStates);
xlabel('Time (s)'); ylabel('Y Ang Mom');

a=[a subplot(313)];
hold on
plot(t,hCom(3,:),'b');
plot(t,hComDes(3,:),'b--');
drawTransitions(transitionTimes,transitionStates);
xlabel('Time (s)'); ylabel('Z Ang Mom (m)');



f = figure('Name','hDot','Position',figPos); figs = [figs f];
clf
a=[a subplot(311)];
hold on
plot(t,hDotOpt(1,:),'r');
plot(t,hDotDes(1,:),'r--');
drawTransitions(transitionTimes,transitionStates);
title('CoM X'); xlabel('Time (s)'); ylabel('X Position (m)');

a=[a subplot(312)];
hold on
plot(t,hDotOpt(2,:),'g');
plot(t,hDotDes(2,:),'g--');
drawTransitions(transitionTimes,transitionStates);
title('CoM Y'); xlabel('Time (s)'); ylabel('Y Position (m)');

a=[a subplot(313)];
hold on
plot(t,hDotOpt(3,:),'b');
plot(t,hDotDes(3,:),'b--');
drawTransitions(transitionTimes,transitionStates);
title('CoM Z'); xlabel('Time (s)'); ylabel('Z Position (m)');



f = figure('Name','CoM Trajectory','Position',figPos); figs = [figs f];
clf
hold on
plot(pCom(1,:),pCom(3,:))
plot(pComDes(1,:),pComDes(3,:),'b--')
axis square
axis([2,2.25 0 .5])
r = norm(pComDes(:,1)-[2 pComDes(2,1) 0]');
theta = 0:pi/40:pi/2;
plot(2+r*sin(theta),cos(theta)*r,'r.');
xlabel('X Position (m)'); ylabel('Z Position (m)')

%%%%%%%%%
% Feet
%%%%%%%%%

f=figure('Name','Left Foot Info','Position',figPos); figs = [figs f];
% clf
% a=[a subplot(311)];
% hold on
% plot(t,lFootPos(2,:),'r');
% plot(t,lFootPosDes(2,:),'r--');
% xlabel('Time (s)')
% ylabel('Y Position (m)')
% drawTransitions(transitionTimes,transitionStates);

a=[a subplot(311)];
hold on
plot(t,lFootPos(3,:),'b');
plot(t,lFootPosDes(3,:),'b--');
drawTransitions(transitionTimes,transitionStates);
xlabel('Time (s)')
ylabel('Z Position (m)')

a=[a subplot(312)];
hold on
plot(t,lFootVel(6,:),'b');
drawTransitions(transitionTimes,transitionStates);
xlabel('Time (s)')
ylabel('Z Velocity (m/s)')

a=[a subplot(313)];
hold on
plot(t,lCopForce(3,:),'b');
plot(t,lWrenchOpt(6,:),'b--');
drawTransitions(transitionTimes,transitionStates);
xlabel('Time (s)')
ylabel('L CoP Normal Force (N)')

f=figure('Name','Right Foot Info','Position',figPos); figs = [figs f];
% clf
% a=[a subplot(311)];
% hold on
% plot(t,lFootPos(2,:),'r');
% plot(t,lFootPosDes(2,:),'r--');
% xlabel('Time (s)')
% ylabel('Y Position (m)')
% drawTransitions(transitionTimes,transitionStates);

a=[a subplot(311)];
hold on
plot(t,rFootPos(3,:),'r');
plot(t,rFootPosDes(3,:),'r--');
drawTransitions(transitionTimes,transitionStates);
xlabel('Time (s)')
ylabel('Z Position (m)')

a=[a subplot(312)];
hold on
plot(t,rFootVel(6,:),'r');
drawTransitions(transitionTimes,transitionStates);
xlabel('Time (s)')
ylabel('Z Velocity (m/s)')

a=[a subplot(313)];
hold on
plot(t,rCopForce(3,:),'r');
plot(t,rWrenchOpt(6,:),'r--');
drawTransitions(transitionTimes,transitionStates);
xlabel('Time (s)')
ylabel('L CoP Normal Force (N)')



f=figure('Name','Contact States','Position',figPos); figs = [figs f];
a=[a subplot(211)];
hold on
plot(t,lContState,'r');
plot(t,rContState+.1,'b');
xlabel('Time (s)');
ylabel('Contact State');

a=[a subplot(212)];
hold on
plot(t,lSlideState,'r');
plot(t,rSlideState,'b');
xlabel('Time (s)');
ylabel('Slide State');

f=figure('Name','ZMP Force','Position',figPos); figs = [figs f];
a=[a subplot(311)]
hold on
plot(t,zmpForce(1,:),'r');
plot(t,zmpWrenchOpt(4,:),'r--');
xlabel('Time (s)'); ylabel('ZMP Force X');

a=[a subplot(312)]
hold on
plot(t,zmpForce(2,:),'g');
plot(t,zmpWrenchOpt(5,:),'g--');
xlabel('Time (s)'); ylabel('ZMP Force Y');

a=[a subplot(313)]
hold on
plot(t,zmpForce(3,:),'b');
plot(t,zmpWrenchOpt(6,:),'b--');
xlabel('Time (s)'); ylabel('ZMP Force Z');

f=figure('Name','ZMP Pos','Position',figPos); figs = [figs f];
a=[a subplot(311)]
hold on
plot(t,zmpPos(1,:),'r');
plot(t,zmpPosOpt(1,:),'r--');
xlabel('Time (s)'); ylabel('ZMP Pos X');

a=[a subplot(312)]
hold on
plot(t,zmpPos(2,:),'g');
plot(t,zmpPosOpt(2,:),'g--');
xlabel('Time (s)'); ylabel('ZMP Pos Y');

a=[a subplot(313)]
hold on
plot(t,zmpNz,'b');
plot(t,zmpWrenchOpt(3,:),'b--');
xlabel('Time (s)'); ylabel('ZMP N Z');



f = figure('Name','Foot Trajectories','Position',figPos); figs = [figs f];
clf
hold on
plot(lFootPos(1,:),lFootPos(3,:),'r')
plot(lFootPosDes(1,:),lFootPosDes(3,:),'r--')

plot(rFootPos(1,:),rFootPos(3,:),'b')
plot(rFootPosDes(1,:),rFootPosDes(3,:),'b--')
xlabel('X Position (m)');
ylabel('Z Position (m)');


f = figure('Name','Right Foot Trajectories','Position',figPos); figs = [figs f];
a=[a subplot(321)]
hold on
plot(t,rFootPos(1,:),'r');
plot(t,rFootPosDes(1,:),'r--');
xlabel('Time (s)'); ylabel('Pos X');
drawTransitions(transitionTimes,transitionStates);

a=[a subplot(323)]
hold on
plot(t,rFootPos(2,:),'g');
plot(t,rFootPosDes(2,:),'g--');
xlabel('Time (s)'); ylabel('Pos Y');
drawTransitions(transitionTimes,transitionStates);

a=[a subplot(325)]
hold on
plot(t,rFootPos(3,:),'b');
plot(t,rFootPosDes(3,:),'b--');
xlabel('Time (s)'); ylabel('Pos Z');
drawTransitions(transitionTimes,transitionStates);

a=[a subplot(322)]
hold on
plot(t,rFootVel(4,:),'r');
plot(t,rFootVelDes(4,:),'r--');
xlabel('Time (s)'); ylabel('Vel X');
drawTransitions(transitionTimes,transitionStates);

a=[a subplot(324)]
hold on
plot(t,rFootVel(5,:),'g');
plot(t,rFootVelDes(5,:),'g--');
xlabel('Time (s)'); ylabel('Vel Y');
drawTransitions(transitionTimes,transitionStates);

a=[a subplot(326)]
hold on
plot(t,rFootVel(6,:),'b');
plot(t,rFootVelDes(6,:),'b--');
xlabel('Time (s)'); ylabel('Vel Z');
drawTransitions(transitionTimes,transitionStates);



%%%%%%%%%
% Joints
%%%%%%%%%

f = figure('Name','Knee Angles','Position',figPos); figs = [figs f];
a=[a axes];
hold on
plot(t,q(19,:),'r');
plot(t,q(12,:),'b');
xlabel('Time (s)');
ylabel('Angle (rad)');

drawTransitions(transitionTimes,transitionStates);

f = figure('Name','Elbow Angles','Position',figPos); figs = [figs f];
a=[a axes];
hold on
plot(t,q(26,:),'r');
plot(t,q(31,:),'b');
xlabel('Time (s)');
ylabel('Angle (rad)');

drawTransitions(transitionTimes,transitionStates);


f = figure('Name','Shoulder Angles','Position',figPos); figs = [figs f];
a=[a axes];
hold on
plot(t,sum(q(22:24,:).^2),'r');
plot(t,sum(q(27:29,:).^2),'b');
xlabel('Time (s)');
ylabel('Angle (rad)');

drawTransitions(transitionTimes,transitionStates);



% f = figure('Name','Force Diff','Position',figPos); figs = [figs f];
% a=[a axes];
% hold on
% plot(t,lCopForce(1,:)-rCopForce(1,:),'r');
% plot(t,lCopForce(2,:)-rCopForce(2,:),'g');
% plot(t,lCopForce(3,:)-rCopForce(3,:),'b');
% 
% xlabel('Time (s)');
% ylabel('Angle (rad)');
% 
% 
% f = figure('Name','Foot Pos Diff','Position',figPos); figs = [figs f];
% a=[a axes];
% hold on
% plot(t,lFootPos(1,:)-rFootPos(1,:),'r');
% plot(t,lFootPos(2,:)-.18-rFootPos(2,:),'g');
% plot(t,lFootPos(3,:)-rFootPos(3,:),'b');
% 
% 
% f = figure('Name','Foot Vel Diff','Position',figPos); figs = [figs f];
% a=[a axes];
% hold on
% plot(t,lFootVel(1,:)-rFootVel(1,:),'r');
% plot(t,lFootVel(2,:)-rFootVel(2,:),'g');
% plot(t,lFootVel(3,:)-rFootVel(3,:),'b');
% 
% plot(t,lFootVel(4,:)-rFootVel(4,:),'r--');
% plot(t,lFootVel(5,:)-rFootVel(5,:),'g--');
% plot(t,lFootVel(6,:)-rFootVel(6,:),'b--');




%f = figure('Name','Foot Force','Position',figPos); figs = [figs f];







% f = figure('Name','Mu Des','Position',figPos); figs = [figs f];
% z=hDotDes(6,:)+9.81*19.12;
% x=hDotDes(4,:);
% muDes = x./z;
% a=[a axes()];
% plot(t,muDes);
% xlabel('Time (s)')
% ylabel('Mu Des');
% linkaxes(a,'x');

% f = figure('Name','Dynamic Discreptancy','Position',figPos); figs = [figs f];
% A = qddOpt-qddAct;
% a=[a axes()];
% hold on
% plot(t,sum(A.^2).^2);
% drawTransitions(transitionTimes,transitionStates);
% xlabel('Time (s)')
% ylabel('Discreptancy');



linkaxes(a,'x');


f = figure
pos = [0,0,175,700];
set(f,'MenuBar','none','Position',pos);
l = uicontrol('style','Listbox','Position',pos);
names = cell(1,length(figs));
for i=1:length(figs)
   names{i} = char(strcat('(',num2str(i),')', {' '},get(figs(i),'Name'))); 
end
set(l,'FontSize',12,'String',names,'Value',1,'Callback',@figSwitch);

as = findall(0,'type','axes');
for i=1:length(as)
    a=as(i);
   xlab = get(a,'XLabel');
   ylab = get(a,'YLabel');
   tlab = get(a,'Title');
   set(xlab,'FontSize',12);
   set(ylab,'FontSize',12);
   set(tlab,'FontSize',14);
end
'done'

