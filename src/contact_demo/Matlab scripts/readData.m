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
mu_static = 0.16;
mu_slip = 0.15;


a=[];
figs=[];
figPos = [176          336        900         600];


f = figure('Name','1 ','Position',figPos); figs = [figs f];
clf
title('ground contact force');
a=[a subplot(311)];
hold on
plot(t, fe_terrain(1,:));
title(' fe_{terrain} X in local patch coordinates'); 
xlabel('Time (s)'); 
ylabel(' X force (N) ');



a=[a subplot(312)];
hold on
plot(t, fe_terrain(2,:));
title(' fe_{terrain} Y in local patch coordinates');
xlabel('Time (s)'); 
ylabel('Y Force (N) ');



a=[a subplot(313)];
hold on
plot(t, fe_terrain(3,:));
title(' fe_{terrain} Z in local patch coordinates'); 
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
a=[a subplot(212)];
hold on
%plot(t, fe_terrain(3,:)*mu_static );
%plot(t, fe_terrain(3,:)*mu_slip,'m');
%plot(t, ext_f(4,:),'k');
plot(t, -fe_terrain(1,:),'r');
%plot(t, 4*547*box_vel(4,:),'g');
plot(t, -fe_pd_terrain(1,:),'Color',[0 0.498039215803146 0]);

ylim([0,110]);
xlabel('Time (s)','FontSize',12);
ylabel('Tangential force (N) (in local terrain patch CS) ');


a=[a subplot(211)];
plot(t, fe_terrain(3,:));
xlabel('Time (s)');
ylabel('normal force (N) (in local terrain patch CS)');
ylim([0,110]);

%plot(t, box_vel(4,:));
%xlabel('Time (s)','FontSize',12);
%ylabel('velocity (m/s)','FontSize',12);


f = figure('Name','4 ','Position',figPos); figs = [figs f];
a=[a subplot(311)];
plot(t, box_pos(1,:));
xlabel('Time (s)');
ylabel('contact point position X');

a=[a subplot(312)];
plot(t, box_pos(3,:));
xlabel('Time (s)');
ylabel('contact point position Z');


f = figure('Name','5 ','Position',figPos); figs = [figs f];
a=[a subplot(211)];
plot(t, n_penetration);
xlabel('Time (s)');
ylabel('normal penetration');

a=[a subplot(212)];
plot(t, n_velocity);
xlabel('Time (s)');
ylabel('normal velocity');




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

