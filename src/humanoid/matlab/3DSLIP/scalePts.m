yref = (pts(1,2) + pts(end,2))/2;
xref = (pts(1,1) + pts(end,1))/2;

pts(:,1) = pts(:,1) - xref;
pts(:,2) = pts(:,2) - yref;

pts(1,2) = 0;
pts(end,2) = 0;

stepWidth = pts(end,1) - pts(1,1);
scale = (2*.97*sin(0.436461)) / stepWidth;

pts = pts*scale


