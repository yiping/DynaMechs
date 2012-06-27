clear all
setDataDir;
fid = fopen(strcat(dataDir,'/recentData.dat'));
filename = fscanf(fid,'%s');
fclose(fid);


[cols, display, names, data, commands] = readColData(filename);


for i=1:cols
    eval(char(commands{i}));
end


figure(1)
clf
subplot(131)
hold on
plot(t,pCom(1,:),'r');
plot(t,pComDes(1,:),'r--');
subplot(132)
hold on
plot(t,pCom(2,:),'g');
plot(t,pComDes(2,:),'g--');
subplot(133)
hold on
plot(t,pCom(3,:),'b');
plot(t,pComDes(3,:),'b--');