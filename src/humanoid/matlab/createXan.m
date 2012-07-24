function createXan( path, link)

fid = fopen(path,'w');

fprintf(fid,' 0.000 0.000 0.000\n');
fprintf(fid,' 0.700 0.700 0.700\n');
fprintf(fid,' 0.700 0.700 0.700\n');
fprintf(fid,' 0.700 0.700 0.700\n');
fprintf(fid,' 10.000\n');
fprintf(fid,' 1.000\n\n');
fprintf(fid,' 1 1 1\n\n');
fprintf(fid,' 8 6 \n\n');

offset = link.offset;
scale  = link.scale;

verts =     [0 0 0 ;
             1 0 0 ;
             1 1 0 ;
             0 1 0 ;
             0 0 1 ;
             1 0 1 ;
             1 1 1 ;
             0 1 1];
verts = verts * diag(scale);
verts = verts + ones(8,1) * offset;

for i=1:8
    fprintf(fid,'%.6f\t%.6f\t%.6f\n',verts(i,1),verts(i,2),verts(i,3));
end

fprintf(fid,'\n');

fprintf(fid,'4 4 4 4 4 4\n\n');

faces = [0 1 2 3 ;
         1 5 6 2 ;
         2 6 7 3 ;
         3 7 4 0 ;
         0 4 5 1 ;
         5 4 7 6 ];
for i=1:6
    fprintf(fid,'%d\t%d\t%d\t%d\n',faces(i,1),faces(i,2),faces(i,3),faces(i,4));     
end

fprintf(fid,'\n');
fclose(fid);

end

