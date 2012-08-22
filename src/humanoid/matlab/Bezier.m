ps =[ 0,0,0; 
      -2, 0 ,0;
      -2, .8 , 4,
      -2, -.2 , 4,
      -6, 0 ,-3,
       
      8,0,-3
      2,0,2
      2,0,2
      1.8,0,1.8
       2,0,1 ]'*.555;
   
   ps =[ 0,1,0; 
      0,1,0;
      -10,1,0;
      14,1,0;
      3,1,0;
      1,1,0;
      1,1,0]';
   
   
  
  
[M N]=size(ps);

T = 0:.01:1;
Pos = zeros(M,length(T));

clf
figure(1)
h = plot3(Pos(1,1),Pos(2,1),Pos(3,1));
hold on
axis([-2.0000         4   -1.0000    1.0000         0    1.4000])
axis square
h2 = plot3(Pos(1,1),Pos(2,1),Pos(3,1),'ko');

scatter3(ps(1,:),ps(2,:),ps(3,:))

for tsub = 1:length(T)
    t = T(tsub);
    for i=0:(N-1)
       Pos(:,tsub) = Pos(:,tsub)+nchoosek(N-1,i)*(1-t)^(N-1-i)*t^i*ps(:,i+1);
    end
    
    set(h,'Xdata',Pos(1,1:tsub));
    set(h,'Ydata',Pos(2,1:tsub));
    set(h,'Zdata',Pos(3,1:tsub));
    
    set(h2,'Xdata',Pos(1,tsub),'Ydata',Pos(2,tsub),'Zdata',Pos(3,tsub));
    
    
    pause(.001);
end

figure(2)
clf
subplot(121)
plot(T,Pos(1,:));
subplot(122)
plot(T,Pos(2,:));
