tDes = [0,.1,.2,.3,.4,.5,.6,.7,.8,.9,1]
pDes = [0,-.1,-.3,-.4,-.5,-.1,.3,.25,.22,.2,.2];

length(tDes)
length(pDes)
figure(3)
clf
hold on
plot(tDes,pDes)

order = 5;
n=order;
A = zeros(length(tDes),order+1);

for tsub = 1:length(tDes)
   t = tDes(tsub);
   for i=0:n   
    A(tsub,i+1) = nchoosek(n,i)*(1-t)^(n-i)*t^i;
   end 
end


Acons = [eye(2) zeros(2,order+1-2) ;zeros(2,order+1-2) eye(2)];
bcons = [pDes(1) pDes(1) pDes(end) pDes(end)]';

Ql = [ A'*A Acons' ; Acons zeros(4,4)];
bl = [ A'*pDes' ;bcons];

a = pinv(Ql)*bl;

Ps = a(1:(order+1));




%Ps = pinv(A)*pDes';

T = 0:.01:1;

Pos = 0*T;

for tsub = 1:length(T)
    t = T(tsub);
    for i=0:n
       Pos(tsub) = Pos(tsub)+nchoosek(n,i)*(1-t)^(n-i)*t^i*Ps(i+1);
    end
end
plot(T,Pos,'r')

