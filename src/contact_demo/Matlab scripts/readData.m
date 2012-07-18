clear all
close all
setDataDir;


fid = fopen(strcat(dataDir,'/recentData.dat'));
filename = fscanf(fid,'%s');
fclose(fid);
filename


[cols, display, names, data, commands] = readColData(filename);

%t = data(:,1);
%data = data(t<2.821,:);

for i=1:cols
    eval(char(commands{i}));
end


% constants
mu_static = 0.9;
mu_slip = 0.86;


a=[];
figs=[];
figPos = [176          336        900         600];


f = figure('Name','1 ','Position',figPos); figs = [figs f];
clf
title('ground contact force');
a=[a subplot(311)];
hold on
plot(t, cf(4,:));
title(' Contact Force X component (body coordinate)'); 
xlabel('Time (s)'); 
ylabel(' X force (N) ');



a=[a subplot(312)];
hold on
plot(t, cf(5,:));
title('Contact Force Y component ');
xlabel('Time (s)'); 
ylabel('Y Force (N) ');



a=[a subplot(313)];
hold on
plot(t, cf(6,:));
title('Contact Force Z component  '); 
xlabel('Time (s)'); 
ylabel('Z Force (N)');
ylim([0,150]);

f = figure('Name','2 ','Position',figPos); figs = [figs f];
clf
title('external force');
a=[a subplot(311)];
hold on
plot(t, ext_f(4,:));
title(' Ext Force X component (body coordinate)'); 
xlabel('Time (s)'); 
ylabel(' X force (N) ');



a=[a subplot(312)];
hold on
plot(t, ext_f(5,:));
title('Ext Force Y component ');
xlabel('Time (s)'); 
ylabel('Y Force (N) ');



a=[a subplot(313)];
hold on
plot(t, ext_f(6,:));
title('Ext Force Z component  '); 
xlabel('Time (s)'); 
ylabel('Z Force (N)');


f = figure('Name','3 ','Position',figPos); figs = [figs f];
a=[a gca];
hold on
plot(t, cf(6,:)*mu_static );
plot(t, cf(6,:)*mu_slip,'m');
plot(t, -cf(4,:),'r');
plot(t, ext_f(4,:),'k');
plot(t, 4*547*box_vel(4,:),'g');
plot(t, -cf_pd(4,:),'y');
ylim([0,110]);


% f = figure('Name','4 ','Position',figPos); figs = [figs f];
% clf
% title('Box Velocity');
% a=[a subplot(311)];
% hold on
% plot(t, box_vel(4,:)*4);
% title(' Box Velocity X component (Body Coord.)'); 
% xlabel('Time (s)'); 
% ylabel(' X Vel (m/s) ');
% 
% 
% 
% a=[a subplot(312)];
% hold on
% plot(t, box_vel(5,:)*4);
% title('Box Velocity Y component  ');
% xlabel('Time (s)'); 
% ylabel('Y Vel (m/s) ');
% 
% 
% 
% a=[a subplot(313)];
% hold on
% plot(t, box_vel(6,:)*4);
% title('Box Velocity Z component  '); 
% xlabel('Time (s)'); 
% ylabel('Z Vel (m/s)');


% f = figure('Name','5 ','Position',figPos); figs = [figs f];
% clf
% title('Contact state');
% a=[a subplot(411)];
% hold on
% plot(t, cflags(1,:));
% title(' Conact state 1 '); 
% xlabel('Time (s)'); 
% ylabel(' C ');
% 
% 
% 
% a=[a subplot(412)];
% hold on
% plot(t, cflags(2,:));
% title('Conact state 2');
% xlabel('Time (s)'); 
% ylabel('C ');
% 
% 
% 
% a=[a subplot(413)];
% hold on
% plot(t, cflags(3,:));
% title('Conact state 3 '); 
% xlabel('Time (s)'); 
% ylabel('C');
% 
% a=[a subplot(414)];
% hold on
% plot(t, cflags(4,:));
% title('Conact state 4 '); 
% xlabel('Time (s)'); 
% ylabel('C');



% f = figure('Name','6 ','Position',figPos); figs = [figs f];
% clf
% title('Positions');
% a=[a subplot(311)];
% hold on
% title(' Contact 0 pos / Anchor 0 pos X components ICS'); 
% %plot(t, anc0(1,:));
% plot(t, box_pos(1,:)-0.2-anc0(1,:),'r');
% 
% xlabel('Time (s)'); 
% ylabel(' Pos X (m/s) ');
% 
% 
% 
% a=[a subplot(312)];
% hold on
% title('Contact 0 pos / Anchor 0 pos Y components ICS  ');
% %plot(t, anc0(2,:));
% plot(t, box_pos(2,:)-0.2-anc0(2,:),'r');
% xlabel('Time (s)'); 
% ylabel('Pos Y (m/s) ');
% 
% 
% 
% a=[a subplot(313)];
% hold on
% title('Contact 0 pos / Anchor 0 pos Z components ICS '); 
% plot(t, anc0(3,:));
% 
% plot(t, box_pos(3,:)-0.1,'r');
% xlabel('Time (s)'); 
% ylabel('Pos Z (m/s)');




linkaxes(a,'x');



% ---------------------------------------------------
% figure selection panel
f = figure
pos1 = [0,336,175,400];
pos2 = [0,0,175,400];
set(f,'MenuBar','none','Position',pos1);
% creates a user interface control in the current figure window
l = uicontrol('style','Listbox','Position',pos2);
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

