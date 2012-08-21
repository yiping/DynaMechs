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


ZI = 0.4*cos(YI./0.5) - 0.4;

% zero out and shift around to make it look more complecated
[row col] =size(ZI);
ZI(8:col, :) = 0 ;
ZI(2:col, 1:floor(row/2))  = ZI(1:col-1, 1:floor(row/2)) ;

temp = ZI(1:col-4, :);
ZI(1:4, :) = 0;
ZI(5:col, :) = temp;

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

