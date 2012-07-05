clear all
close all
setDataDir;
fid = fopen(strcat(dataDir,'/recentData.dat'));
filename = fscanf(fid,'%s');
fclose(fid);
filename

[cols, display, names, data, commands] = readColData(filename);


for i=1:cols
    eval(char(commands{i}));
end
prevState = [state(1) state(1:(end-1))];
transitionTimes = t(state ~= prevState);
transitionStates = state(state ~= prevState);

a=[];

figure('Name','CoM Position')
clf
title('CoM Position');
a=[a subplot(131)];
hold on
plot(t,pCom(1,:),'r');
plot(t,pComDes(1,:),'r--');
drawTransitions(transitionTimes,transitionStates);

a=[a subplot(132)];
hold on
plot(t,pCom(2,:),'g');
plot(t,pComDes(2,:),'g--');
drawTransitions(transitionTimes,transitionStates);

a=[a subplot(133)];
hold on
plot(t,pCom(3,:),'b');
plot(t,pComDes(3,:),'b--');
drawTransitions(transitionTimes,transitionStates);

figure('Name','Foot Positions')
clf
a=[a subplot(121)];
hold on
plot(t,lFootPos(2,:),'r');
plot(t,lFootPosDes(2,:),'r--');
drawTransitions(transitionTimes,transitionStates);

a=[a subplot(122)];
hold on
plot(t,lFootPos(3,:),'b');
plot(t,lFootPosDes(3,:),'b--');
drawTransitions(transitionTimes,transitionStates);

figure('Name','CoM Trajectory')
clf
plot(pCom(1,:),pCom(2,:))

figure('Name','Foot Trajectories')
clf
hold on
plot(lFootPos(2,:),lFootPos(3,:),'r')
plot(lFootPosDes(2,:),lFootPosDes(3,:),'r--')

plot(rFootPos(2,:),rFootPos(3,:),'b')
plot(rFootPosDes(2,:),rFootPosDes(3,:),'b--')


linkaxes(a,'x');
