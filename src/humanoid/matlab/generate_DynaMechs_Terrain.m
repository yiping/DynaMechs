% A script to automatically generate the .dat terrain file

close all
clear all


xDim = 35;
yDim = 35;
xRes = 0.1;
yRes = xRes;

%x = 0:xRes:xDim; 
%y = 0:yRes:yDim;

x = 0:xRes:4; 
y = 0:yRes:4;
xDim = length(x);
yDim = length(y);
%x = x * xRes;
%y = y * yRes;

[XI,YI] = meshgrid(x,y);


ZI = - 0.3 + (0.15*cos((XI+.28)./0.4).^2 + .2*sin((YI-.05)./.3).^2 ) .* exp(- (XI-2.5).^2 - (YI-2).^2);
%ZI = - 0.3 + (0.15*cos((XI+.28)./0.4).^2 + .2*sin((YI-.09)./.2).^2 ) .* exp(- (XI-2.5).^2 - (YI-2).^2);


for i=1:xDim
    for j=1:yDim
        if XI(i,j) < 2.25
            ZI(i,j) = 0;
            
        end
        
    end 
end




% zero out and shift around to make it look more complecated
%[row col] =size(ZI);
%ZI(8:col, :) = 0 ;
% ZI(2:col, 1:floor(row/2))  = ZI(1:col-1, 1:floor(row/2)) ;
% 
% temp = ZI(1:col-4, :);
% ZI(1:4, :) = 0;
% ZI(5:col, :) = temp;

mesh(XI,YI,ZI);
hold
xlabel('x');
ylabel('y');
%z = x.*exp(-x.^2-y.^2);
%plot3(x,y,z,'o'), hold off

hold off


%	rand('seed',0)	
%    x = rand(100,1)*4-2; y = rand(100,1)*4-2;
%    z = x.*exp(-x.^2-y.^2);







outputDir = '../config';

fid = fopen(strcat(outputDir,'/terrain3.dat'),'w');
[nr, nc] =size(ZI);
fprintf(fid, '%d  %d  %2.2f\n',yDim, xDim, xRes); 
for i=1:nr
    fprintf(fid, '%f   ',ZI(:,i)');
    fprintf(fid, '\n');
end


fclose(fid);

