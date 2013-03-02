N = 6;
tStance = 0.168595;
tFlight = 0.142859;
theta = .28;

ts = [0 tFlight tFlight+tStance tFlight*2+tStance];

ts = [0 tFlight/2 tFlight tFlight+tStance/2 tFlight+tStance tFlight*3/2+tStance tFlight*2+tStance tFlight*2+tStance];

%pts = pts7
pts = [-0.268065, -0.580000, -0.600000, -0.319159, 0.144500, 0.420000, 0.268065, 0.000000, 0.152683, 0.350000, 0.305924, 0.183641, 0.049529, 0.000000];
pts = reshape(pts,7,2);

%pts = [pts7 ; pts7(end,1)-3.5*.1
M = 2;



yref = (pts(1,2) + pts(end,2))/2;
xref = (pts(1,1) + pts(end,1))/2;

pts(:,1) = pts(:,1) - xref;
pts(:,2) = pts(:,2) - yref;

pts(1,2) = 0;
pts(end,2) = 0;

yMax = max(pts(:,2));

stepWidth = pts(end,1) - pts(1,1);
scale = (2*.97*sin(theta)) / stepWidth;

pts(:,1) = pts(:,1)*scale
pts(:,2) = pts(:,2)*.45/yMax;


%%%%%
% M=2;
% N=2;
% pts = [0 0;0 1 ; 1 1];
% vInit = [0 0]';
% vFinal = [0 0]';
% ts=[0 1 2];


[numPts, dimPts] = size(pts);

slimPts = reshape(pts,numPts*dimPts,1);
fprintf(1,'%f, ',slimPts) 


A = zeros(4*N);
b = zeros(4*N,1);
vInit = [-3.5 -.7]';
vFinal = [-3.5 .7]';

j=1;

for i=1:N
   t0 = ts(i);
   tf = ts(i+1);
   A(2*i-1,(4*i-3):(4*i)) = [t0^3 t0^2 t0 1];
   A(2*i,(4*i-3):(4*i)) = [tf^3 tf^2 tf 1];

   
   if i~=1
       A(2*N + 2*(i-1) -1  ,(4*i-3):(4*i)) = -[3*t0^2 2*t0 1 0];   
       A(2*N + 2*(i-1) ,(4*i-3):4*i) = -[6*t0 2 0 0];
   else
      A(4*N-2+1 ,(4*i-3):(4*i)) = [3*t0^2 2*t0 1 0];   
   end
   
   if i~=N
        A(2*N + 2*(i)-1,(4*i-3):(4*i)) = [3*tf^2 2*tf 1 0];
        A(2*N + 2*(i),(4*i-3):(4*i)) = [6*tf   2    0 0];
   else
        A(4*N,(4*i-3):(4*i)) = [3*tf^2 2*tf 1 0];
   end

end

C = zeros(4,N,M);
size(b)

A
for j=1:M
    fprintf(1,'Dim %d\n',j);
    b
    
    for i=1:N
       b(2*i-1)  =  pts(i,j);
       b(2*i) = pts(i+1,j);
    end
    
    b(4*N-1) = vInit(j);
    b(4*N)   = vFinal(j);
    x = A\b;
    
    for i=1:N
       C(:,i,j) = x((4*i-3):4*i);
       
       T = ts(i):(ts(i+1)-ts(i)):ts(i+1);
       tMat = [T.^3 ; T.^2 ; T ; 0*T+1];
       Pos = C(:,i,j)'*tMat;
       
       tMat = [3*T.^2 ; 2*T ; 0*T+1 ; 0*T];
       vel = C(:,i,j)'*tMat;
       
       %fprintf(1,'Spline %d (%f,%f), %f %f -> %f %f\n',i, ts(i), ts(i+1), Pos(1), vel(1), Pos(end), vel(end));
       

    end
    
end

tf = ts(end);
t0 = ts(1);

T = t0:.001:tf;
tMat = [T.^3 ; T.^2 ; T ; 0*T+1];

Trajs = zeros(M, length(T));
for j=1:M
   for i=1:N
      Trajs(j,:) = Trajs(j,:)+(C(:,i,j)'*tMat).*(T>ts(i)).*(T<=ts(i+1)); 
   end
end

figure(3)
clf
hold on
plot(Trajs(1,2:end), Trajs(2,2:end))
scatter(pts(:,1),pts(:,2))

figure(4)
subplot(211)
plot(T,Trajs(1,:))
subplot(212)
plot(T,Trajs(2,:))



