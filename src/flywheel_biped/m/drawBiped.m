figure(1)
clf;
plot3(0,0,0,'o');
%view(166,34)
%view(84,14)
%view(90,90)
%view(180,0)
view(48,24) %az el
hold on
axis square
grid on
x = robot(1).pi_p_i(1);
y = robot(1).pi_p_i(2);
z = robot(1).pi_p_i(3);
axis([x-3 x+3 y-3 y+3 z-1 z+1])
daspect([1 1 1]) % set relative scaling of data units along x-, y-, z-axes
xlabel('x');
ylabel('y');
zlabel('z');
% Setup all neccecaries for robot structs

if robot(1).fb == 0
    plotRows([robot(1).T0*[0 0 0 1]' [0 0 0 1]'],'--r');
end
for i=1:N_ext

    j  = robot(i);
    pred = robot(i).parent;
    if j.parent ~= 0
        plotRows([robot(i).T0*[0 0 0 1]' robot(pred).T0*[0 0 0 1]']);
    end 
    
    if i<=N
        % Plot Masses
        scatterRows(robot(i).T0*[robot(i).i_p_com ;1]);
        %[V D] = eig(robot(i).I_com);
    end
end



