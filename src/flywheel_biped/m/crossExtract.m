function x = crossExtract(M)
   x(1) = M(3,2);
   x(2) = M(1,3);
   x(3) = M(2,1);
   
   %M = [0 -x(3) x(2) ; x(3) 0 -x(1) ; -x(2) x(1) 0]; 
end