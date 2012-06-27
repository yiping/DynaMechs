function  [ncols,diplayLabels,names,data,commands] = readColData(fname)

%  open file for input, include error handling
fin = fopen(fname,'r');
if fin < 0
   error(['Could not open ',fname,' for input']);
end

%  Preliminary reading of titles to determine number of columns
%  needed in the labels matrix.  This allows for an arbitrary number
%  of column titles with unequal (string) lengths.  We cannot simply
%  append to the labels matrix as new labels are read because the first
%  label might not be the longest.  The number of columns in the labels
%  matrix (= maxlen) needs to be set properly from the start.

maxlen = 0;
ncols = 0;
for i=1:2
   buffer = fgetl(fin);          %  get next line as a string
   for j=1:20000
      [next,buffer] = strtok(buffer,';');       %  parse next column label
      next = strtrim(next);
      maxlen = max(maxlen,length(next));   %  find the longest so far
      ncols = ncols + 1;
      if (length(buffer) == 0), break, end;
   end
end
ncols = ncols/2;

%  Set the number of columns in the labels matrix equal to the length
%  of the longest column title.  A complete preallocation (including
%  rows) of the label matrix is not possible since there is no string
%  equivalent of the ones() or zeros() command.  The blank() command
%  only creates a string row vector not a matrix.
labels = blanks(maxlen);

frewind(fin);    %  rewind in preparation for actual reading of labels and data


%  Read titles for keeps this time
for i=1:2

   buffer = fgetl(fin);          %  get next line as a string
   for j=1:ncols
      [next,buffer] = strtok(buffer,';');     %  parse next column label
      next = strtrim(next);
      n = j + (i-1)*ncols;                %  pointer into the label array for next label
      labels(n,1:length(next)) = next;    %  append to the labels matrix
   end
end
diplayLabels = labels(1:ncols,:);
names        = labels((ncols+1):end,:);

%  Read in the x-y data.  Use the vetorized fscanf function to load all
%  numerical values into one vector.  Then reshape this vector into a
%  matrix before copying it into the x and y matrices for return.

commands = cell(1,ncols);
for i=1:ncols
    tok = regexp(names(i,:),'(.*)\((.*?)\)','tokens');
    if length(tok) > 0
       name = strcat(tok{1}(1),'(',tok{1}(2),',:)'); 
    else
       name = names(i,:); 
    end
    commands{i} = strcat(name,'=data(:,',num2str(i),')'';'); 
end

data = fscanf(fin,'%f');  %  Load the numerical values into one long vector
%size(data)
nd = length(data);        %  total number of data points
nr = nd/ncols;            %  number of rows; check (next statement) to make sure
if nr ~= round(nd/ncols)
   fprintf(1,'\ndata: nrow = %f\tncol = %d\n',nr,ncols);
   fprintf(1,'number of data points = %d does not equal nrow*ncol\n',nd);
   error('data is not rectangular')
end

fclose(fin);
data = reshape(data,ncols,nr)';   %  notice the transpose operator

%  end of readColData.m

