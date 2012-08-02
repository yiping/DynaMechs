function createXanTorso( path, link)

fid = fopen(path,'w');

fprintf(fid,' 0.000 0.000 0.000\n');
fprintf(fid,' 0.700 0.700 0.700\n');
fprintf(fid,' 0.700 0.700 0.700\n');
fprintf(fid,' 0.700 0.700 0.700\n');
fprintf(fid,' 10.000\n');
fprintf(fid,' 1.000\n\n');
fprintf(fid,' 1 1 1\n\n');
fprintf(fid,' 24 18 \n\n');

offset = link.offset;
scale  = link.scale;

vertsInit =     [0 0 0 ;
             1 0 0 ;
             1 1 0 ;
             0 1 0 ;
             0 0 1 ;
             1 0 1 ;
             1 1 1 ;
             0 1 1];
         
verts = vertsInit * diag(scale);
verts = verts + ones(8,1) * offset;
allVerts = [verts];

verts = vertsInit * diag(link.neckScale);
verts = verts + ones(8,1) * link.neckOffset;
allVerts = [allVerts ; verts];

verts = vertsInit * diag(link.headScale);
verts = verts + ones(8,1) * link.headOffset;
allVerts = [allVerts ; verts];




for i=1:24
    fprintf(fid,'%.6f\t%.6f\t%.6f\n',allVerts(i,1),allVerts(i,2),allVerts(i,3));
end

fprintf(fid,'\n');

fprintf(fid,'4 4 4 4 4 4 4 4 4 4 4 4 4 4 4 4 4 4\n\n');

faces = [0 1 2 3 ;
         1 5 6 2 ;
         2 6 7 3 ;
         3 7 4 0 ;
         0 4 5 1 ;
         5 4 7 6 ];
allFaces = [faces ; faces+8 ; faces+16];

     
     
for i=1:18
    fprintf(fid,'%d\t%d\t%d\t%d\n',allFaces(i,1),allFaces(i,2),allFaces(i,3),allFaces(i,4));     
end

fprintf(fid,'\n');
fclose(fid);

end