function plotRows(A,c)
   if nargin == 2
    plot3(A(1,:), A(2,:), A(3,:),c);
   else
    plot3(A(1,:), A(2,:), A(3,:));   
   end
end


