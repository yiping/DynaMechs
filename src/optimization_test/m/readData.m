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




%%

a=[];
figs=[];
figPos = [176          336        900         600];


f = figure('Name','1 ','Position',figPos); figs = [figs f];
clf
title('lDot vs. lDotDes');
a=[a subplot(311)];
hold on
plot(t, hComDotOpt(4,:));
plot(t, hComDotDes(4,:),'r');
title('l Com Dot x in ICS'); 
xlabel('Time (s)'); 
ylabel('  ');



a=[a subplot(312)];
hold on
plot(t, hComDotOpt(5,:));
plot(t, hComDotDes(5,:),'r');
title(' l Com Dot y in ICS'); 
xlabel('Time (s)'); 
ylabel('  ');


a=[a subplot(313)];
hold on
plot(t, hComDotOpt(6,:));
plot(t, hComDotDes(6,:),'r');
title(' l Com Dot z in ICS' ); 
xlabel('Time (s)'); 
ylabel('  ');


f = figure('Name','2 ','Position',figPos); figs = [figs f];
clf
title('l vs. lDes');
a=[a subplot(311)];
hold on
plot(t, hCom(4,:));
plot(t, hComDes(4,:),'r');
title(' l_{c,x} in ICS'); 
xlabel('Time (s)'); 
ylabel('  ');



a=[a subplot(312)];
hold on
plot(t, hCom(5,:));
plot(t, hComDes(5,:),'r');
title(' l_{c,y} in ICS'); 
xlabel('Time (s)'); 
ylabel('  ');



a=[a subplot(313)];
hold on
plot(t, hCom(6,:));
plot(t, hComDes(6,:),'r');
title(' l_{c,z} in ICS'); 
xlabel('Time (s)'); 
ylabel('  ');














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

