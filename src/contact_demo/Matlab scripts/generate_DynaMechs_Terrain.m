% A script to automatically generate the .dat terrain file

close all
clear all


xDim = 20;
yDim = 20;
xRes = 0.5;
yRes = xRes;

%x = 0:xRes:xDim; 
%y = 0:yRes:yDim;

x = 0:1:xDim-1; 
y = 0:1:yDim-1;
x = x * xRes;
y = y * yRes;

[XI,YI] = meshgrid(x,y);
%%ZI = 2*(XI./2.5-xDim/5).*exp(-(XI./2.5-xDim/5).^2-(YI./2.5-yDim/5).^2);

%ZI = 2*(XI./2.5-(xDim*xRes)/5).*exp(-(XI./2.5-(xDim*xRes)/5).^2-(YI./2.5-(yDim*yRes)/5).^2);

ZI = 0.4*cos(YI./2);

mesh(XI,YI,ZI);
hold
	
%z = x.*exp(-x.^2-y.^2);
%plot3(x,y,z,'o'), hold off

hold off

%	rand('seed',0)	
%    x = rand(100,1)*4-2; y = rand(100,1)*4-2;
%    z = x.*exp(-x.^2-y.^2);







outputDir = '.';

fid = fopen(strcat(outputDir,'/terrain3.dat'),'w');
[nr, nc] =size(ZI);
fprintf(fid, '%d  %d  %2.2f\n',xDim, yDim, xRes); 
for i=1:nr
    fprintf(fid, '%f   ',ZI(i,:));
    fprintf(fid, '\n');
end


fclose(fid);

