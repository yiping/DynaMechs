close all
clear all

[X,Y] = meshgrid(0:.1:3,0:.1:3);
Z= (X -0.5).^2 + (Y-2).^2;

contour(X,Y,Z, 40);
axis equal
axis tight
hold on

x = [0:0.1:3];
plot(x, x);
